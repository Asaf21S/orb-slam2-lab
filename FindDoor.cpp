#define _USE_MATH_DEFINES

#include <istream>
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <math.h>
#include <cmath>
#include <algorithm>

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

int main()
{
    std::ifstream is("./data/data/pointData0.csv", std::ifstream::binary);
    std::cout << findDoor(is);
    return 0;
}