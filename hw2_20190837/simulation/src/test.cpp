#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include <mymat.hpp>
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp; 


// Function to read and process a file
void processFile(const std::string& filename, std::vector<double>& firstElements) {
    // Open the file
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Unable to open file " << filename << std::endl;
        return;
    }

    // Read the first line
    std::string line;
    while(std::getline(file, line)) {
        // Create a stringstream from the line
        std::stringstream ss(line);
        
        // Temporary variable to store the first value
        double firstValue;
        
        // Read the first value from the stringstream
        if (ss >> firstValue) {
            // Push the first value into the vector
            firstElements.push_back(firstValue);
        }
    }

    // Close the file
    file.close();
}


int main()
{
    std::vector<double> csvFirstElements;
    std::vector<double> csvSecondElements;

    processFile("w_results_cpp.csv", csvFirstElements);
    processFile("w_results_cpp2.csv", csvSecondElements);
    
    //time vector:
    // Define the vector to store values
    std::vector<double> time;

    // Loop from 0 to 10000 with increments of 0.1
    for (double i = 0; i <= 1000; i += 0.1) {
        // Push the value into the vector
        time.push_back(i);
    }

    // time.resize(3000);
    // csvFirstElements.resize(3000);
    // csvSecondElements.resize(3000);

    // Set x-axis ticks at increments of 10
    std::vector<double> xticks;
    for (int i = 0; i <= 1000; i += 50) {
        xticks.push_back(i);
    }

    // //plot (reconsider using matplotlibbcpp as it requires python)
    plt::title("x-component of angular velocity");
    plt::plot(time, csvFirstElements, "b", {{"label", "cpp x-axis"}});
    plt::plot(time, csvSecondElements, "r", {{"label", "cpp 10Hz x-axis"}});
    plt::xticks(xticks);
    plt::xlabel("time(s)");
    plt::ylabel("W_x(rad/s)");
    plt::legend();

    plt::show();
    
    return 0;
}