#include <filesystem>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cstring>

#include "ukf_gps_imu.hpp"

int main(int argc, char** argv) {
    // Check if the required arguments are provided
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <input_csv> <output_csv>" << std::endl;
        std::cerr << "Example: " << argv[0] << " data.csv output.csv" << std::endl;
        return 1;
    }

    std::string input_file = argv[1];
    std::string output_file = argv[2];

    // Open the input file
    std::ifstream infile(input_file);
    if (!infile.is_open()) {
        std::cerr << "Error opening input file: " << input_file << std::endl;
        return 1;
    }

    // Open the output file
    std::ofstream outfile(output_file);
    if (!outfile.is_open()) {
        std::cerr << "Error creating output file: " << output_file << std::endl;
        return 1;
    }

    std::string line;
    // Read the file line by line
    while (std::getline(infile, line)) {
        std::stringstream ss(line);
        std::string cell;
        bool first = true;
        
        // Split the line into cells using the comma delimiter
        while (std::getline(ss, cell, ',')) {
            if (!first) {
                outfile << ","; // Add comma between cells
            }
            outfile << cell; // Write each cell to output file
            first = false;
        }
        outfile << std::endl; // Move to the next line for the next row
    }

    infile.close();
    outfile.close();
    
    std::cout << "Successfully processed: " << input_file << std::endl;
    std::cout << "Output saved to: " << output_file << std::endl;
    
    return 0;
}

void SPEW_THE_ERROR(char const* str) {
#if (SYSTEM_IMPLEMENTATION == SYSTEM_IMPLEMENTATION_PC)
  cout << (str) << endl;
#elif (SYSTEM_IMPLEMENTATION == SYSTEM_IMPLEMENTATION_EMBEDDED_ARDUINO)
  Serial.println(str);
#else
  /* Silent function */
#endif
  while (1)
    ;
}