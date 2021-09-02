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
void StoreMap(ORB_SLAM2::System &SLAM);

// global variables:
const char* const TELLO_STREAM_URL{"udp://0.0.0.0:11111"};
Tello tello{};
cv::Mat frame;
std::atomic<bool> isScanning, beginFrame, streamOn;
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

    thread th1 (GetFrames);
    thread th2 (ScanRoom);
    while (!beginFrame){ sleep(0.1); }

    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    while (isScanning)
    {
        if(!frame.empty())
            SLAM.TrackMonocular(frame, 0.1);
    }
    
    StoreMap(SLAM);
    SLAM.Shutdown();
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
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
    int degree = 20;
    while (!beginFrame){}
    
    tello.SendCommand("takeoff");
    while (!tello.ReceiveResponse()) { sleep(0.2);}
    sleep(5);
    for (int i = degree; i <= 360; i+=degree)
    {
        // cout << "rotatingforward" << endl;
        tello.SendCommand("cw 20");
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

    while(streamOn){}
    tello.SendCommand("land");
    while (!tello.ReceiveResponse()) { sleep(0.2);}
}

// responsible for spinning the drone
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
}
