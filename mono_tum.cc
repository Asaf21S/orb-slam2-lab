/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

// includes:
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <unistd.h>	
#include <opencv2/core/core.hpp>
#include <System.h>
#include <Converter.h>	
#include <thread>
#include "ctello.h"
#include <istream>
#include <string>
#include <vector>
#include <math.h>
#include <cmath>

// usings:
using namespace std;
using ctello::Tello;
using cv::CAP_FFMPEG;
using cv::imshow;
using cv::VideoCapture;
using cv::waitKey;

// function declarations:
void GetFrames();
void ScanRoom();
cv::Mat rot2euler(const cv::Mat & rotationMatrix);
void StoreMap(ORB_SLAM2::System &SLAM);
void NavigateToDoor();
vector<std::string> readCSVRow(const std::string&);
vector<vector<double>> readCSV(std::istream&);
double findDoor(std::istream&);

// global variables:
const char* const TELLO_STREAM_URL{"udp://0.0.0.0:11111"};
Tello tello{};
cv::Mat frame, euler;
std::atomic<bool> isScanning, beginFrame, streamOn, moving, startRotating;
// isScanning - for transfering the images to the orbslam only while scanning the room.
// beginFrame - to make sure orbslam is initialized only after we begin scanning and images are available.
// streamOn - to make sure to land only after the stream is off.

int main(int argc, char **argv)
{
    if(argc != 3)
    {
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings" << endl;
        return 1;
    }
    if (!tello.Bind())
    {
        return 0;
    }
    isScanning = true;
    beginFrame = false;
    streamOn = true;
    moving = true;
    startRotating = false;
    cv::Mat mat;

    thread th1 (GetFrames);
    thread th2 (ScanRoom);
    while (!beginFrame){ sleep(0.1); }

    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    while (isScanning)
    {
        if(!frame.empty()){
            mat = SLAM.TrackMonocular(frame, 0.1);
            if (!mat.empty())
            {
                startRotating = true;
                mat = mat.clone().inv();
                euler = rot2euler(mat);
            }
        }
        else
            cout << "no frame" << endl;
    }
    
    StoreMap(SLAM);
    SLAM.Shutdown();
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    cout << euler << endl;
    return 0;
}

// responsible for capturing the video from the drone
void GetFrames()
{
    tello.SendCommand("streamon");
    while (!tello.ReceiveResponse()) { sleep(0.2);}
    VideoCapture capture{TELLO_STREAM_URL, CAP_FFMPEG};
    if(!capture.isOpened())
        cout << "Error opening video stream or file" << endl;

    while (isScanning)
    {
        capture >> frame;
        beginFrame = true;
        sleep(0.1);
    }
    
    tello.SendCommand("streamoff");
    while (!tello.ReceiveResponse()) { sleep(0.2);}

    streamOn = false;
}

// responsible for spinning the drone
void ScanRoom()
{
    int degree = 24;
    while (!beginFrame){ sleep(0.2);}
    
    tello.SendCommand("takeoff");
    while (!tello.ReceiveResponse()) { sleep(0.2);}
    sleep(5);

    while (!startRotating){ sleep(0.2);}
    for (int i = degree; i <= 360; i+=degree)
    {
        // cout << "rotatingforward" << endl;
        tello.SendCommand("cw 24");
        while (!tello.ReceiveResponse()) { sleep(0.2);}
        sleep(0.5);
        tello.SendCommand("up 20");
        while (!tello.ReceiveResponse()) { sleep(0.2);}
        sleep(0.5);
        tello.SendCommand("down 20");
        while (!tello.ReceiveResponse()) { sleep(0.2);}
        sleep(0.5);
    }
    isScanning = false;

    while(streamOn){ sleep(0.2);}
    while(moving){ sleep(0.2);}
    tello.SendCommand("land");
    while (!tello.ReceiveResponse()) { sleep(0.2);}
}

// Converts a given Rotation Matrix to Euler angles
cv::Mat rot2euler(const cv::Mat & rotationMatrix)
{
    cv::Mat euler(3,1,CV_64F);

    double m00 = (double)rotationMatrix.at<float>(0,0);
    double m02 = (double)rotationMatrix.at<float>(0,2);
    double m10 = (double)rotationMatrix.at<float>(1,0);
    double m11 = (double)rotationMatrix.at<float>(1,1);
    double m12 = (double)rotationMatrix.at<float>(1,2);
    double m20 = (double)rotationMatrix.at<float>(2,0);
    double m22 = (double)rotationMatrix.at<float>(2,2);

    double x, y, z;

    // Assuming the angles are in radians.
    if (m10 > 0.998) { // singularity at north pole
        x = 0;
        y = CV_PI/2;
        z = atan2(m02,m22);
    }
    else if (m10 < -0.998) { // singularity at south pole
        x = 0;
        y = -CV_PI/2;
        z = atan2(m02,m22);
    }
    else
    {
        x = atan2(-m12,m11);
        y = asin(m10);
        z = atan2(-m20,m00);
    }

    euler.at<double>(0) = (double)((x/CV_PI) * 180);
    euler.at<double>(1) = (double)((y/CV_PI) * 180);
    euler.at<double>(2) = (double)((z/CV_PI) * 180);

    return euler;
}

// responsible for saving the map
void StoreMap(ORB_SLAM2::System &SLAM){
    std::vector<ORB_SLAM2::MapPoint*> mapPoints = SLAM.GetMap()->GetAllMapPoints();
    std::ofstream pointData;
    pointData.open("/tmp/pointData.csv");
    for(auto p : mapPoints) {
        if (p != NULL)
        {
            auto point = p->GetWorldPos();
            Eigen::Matrix<double, 3, 1> v = ORB_SLAM2::Converter::toVector3d(point);
            pointData << v.x() << "," << v.y() << "," << v.z()<<  std::endl;
        }
    }
    pointData.close();
    NavigateToDoor();
}

// responsible for navigating the drone through the door
void NavigateToDoor()
{
    ifstream is("/tmp/pointData.csv", ifstream::binary);
    float degToDoor = findDoor(is);
    cout << "degree to the door: " << degToDoor << endl;

    tello.SendCommand("cw " + std::to_string(degToDoor - euler.at<double>(2)));
    while (!tello.ReceiveResponse()) { sleep(0.2);}
    sleep(0.5);
    tello.SendCommand("forward 100");
    while (!tello.ReceiveResponse()) { sleep(0.2);}
    sleep(0.5);
    moving = false;
}

#define _USE_MATH_DEFINES

using std::vector;

/*****************************
        Read CSV file
*****************************/
enum class CSVState {
    UnquotedField,
    QuotedField,
    QuotedQuote
};

vector<std::string> readCSVRow(const std::string& row) {
    CSVState state = CSVState::UnquotedField;
    vector<std::string> fields{ "" };
    size_t i = 0; // index of the current field
    for (char c : row) {
        switch (state) {
        case CSVState::UnquotedField:
            switch (c) {
            case ',': // end of field
                fields.push_back(""); i++;
                break;
            case '"': state = CSVState::QuotedField;
                break;
            default:  fields[i].push_back(c);
                break;
            }
            break;
        case CSVState::QuotedField:
            switch (c) {
            case '"': state = CSVState::QuotedQuote;
                break;
            default:  fields[i].push_back(c);
                break;
            }
            break;
        case CSVState::QuotedQuote:
            switch (c) {
            case ',': // , after closing quote
                fields.push_back(""); i++;
                state = CSVState::UnquotedField;
                break;
            case '"': // "" -> "
                fields[i].push_back('"');
                state = CSVState::QuotedField;
                break;
            default:  // end of quote
                state = CSVState::UnquotedField;
                break;
            }
            break;
        }
    }
    return fields;
}

/// Read CSV file, Excel dialect. Accept "quoted fields ""with quotes"""
vector<vector<double>> readCSV(std::istream& in) {
    vector<vector<std::string>> table;
    std::string row;
    while (!in.eof()) {
        std::getline(in, row);
        if (in.bad() || in.fail()) {
            break;
        }
        auto fields = readCSVRow(row);
        table.push_back(fields);
    }
    // convert from an vector of strings to vector of doubles
    vector<vector<double>> points;
    for (auto& row : table)
    {
        double x = std::stod(row[0]);
        double y = std::stod(row[1]);
        double z = std::stod(row[2]);
        vector<double> values = { x, y, z };
        points.push_back(values);
        //double dist = sqrt(x * x + y * y + z * z);
        //std::cout << x << " " << y << " " << z << " " << dist << std::endl;
    }
    return points;
}

/*****************************
    Find door algorithem
*****************************/

double findDoor(std::istream& file)
{
    // constants that can be optimaized
    int numof_slices = 60;
    int overlapping = 5;
    // read the points in the CSV file
    auto points = readCSV(file);
    // we will look at the points "from above", ignoring the "y" value
    // calculate degrees and normalize them to be between 0 and 1
    vector<double> alpha;
    for (auto & row : points)
    {
        double a = atan2(row[0], row[2]);
        a = (a + M_PI) / (2 * M_PI);
        alpha.push_back(a);
    }
    // find the degree that gives the max standard deviation
    vector<double> sums(numof_slices, 0);       // the sum of the distences
    vector<double> sums2(numof_slices, 0);      // the sum of the dis
    vector<int> counts(numof_slices, 0);        // the number of points in the slice
    int num_points = points.size();
    for (int i = 0; i < num_points; i++)
    {
        int cell = int(alpha[i] * numof_slices);
        double x = points[i][0];
        x *= x;
        double z = points[i][2];
        z *= z;
        double dist = sqrt(x + z);
        counts[cell] += 1;
        sums[cell] += dist;
        sums2[cell] += dist * dist;
    }
    // calculate standard deviation for each cell
    vector<double> stds(numof_slices, 0);
    for (int i = 0; i < numof_slices; i++)
    {
        if (counts[i] != 0)
        {
            double avg = sums[i] / counts[i];
            stds[i] = sqrt(sums2[i] / counts[i] - avg * avg);
        }
    }
    // "smooth" the values to find destinct maximum
    vector<double> stds2(numof_slices, 0);
    for (int i = 0; i < numof_slices; i++)
    {
        stds2[i] = stds[i];
        for (int r = 0; r < overlapping; r++)
        {
            stds2[i] += (stds[(i + r) % numof_slices] + stds[(i - r + numof_slices) % numof_slices]);
        }
    }
    // find the place of the maximum standard deviation
    int max = 0;
    for (int i = 1; i < numof_slices; i++)
    {
        if (stds2[max] < stds2[i])
            max = i;
    }
    // revert changes made to the degree
    double teta = (double(max) / numof_slices) * (2 * M_PI) - M_PI;
    // convert from radians to degrees
    teta = teta * 180 / M_PI;
    return teta;
}