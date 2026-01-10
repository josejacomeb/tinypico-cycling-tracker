#include <filesystem>
#include <format>
#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "ukf_gps_imu.hpp"
#include "utils.hpp"

double altitude;
double vNorth, vEast;
int16_t accNorth, accEast;
unsigned long timestamp;

UKFGPSIMU sf_lat, sf_lng;
LatLonDeg GPSLatLon;
LatLonDeg ZeroPoint, PointEast, PointNorthEast;

const std::string date_time_format = "%Y%m%d_%H%M%S";

std::time_t stringToTime(const std::string &timeString, const std::string &format)
{
  std::tm tm = {}; // Initialize to zeros to prevent undefined behavior
  std::istringstream ss(timeString);
  ss >> std::get_time(&tm, format.c_str());

  if (ss.fail())
  {
    throw std::runtime_error("Failed to parse time string");
  }

  // Convert the tm structure to time_t value (local time)
  std::time_t time_stamp = std::mktime(&tm);
  if (time_stamp == -1)
  {
    throw std::runtime_error("mktime failed to convert time");
  }
  return time_stamp;
}

int main(int argc, char **argv)
{
  // Check if the required arguments are provided
  if (argc < 3)
  {
    std::cerr << "Usage: " << argv[0] << " <input_csv> <output_csv>" << std::endl;
    std::cerr << "Example: " << argv[0] << " data.csv output.csv" << std::endl;
    return 1;
  }

  std::filesystem::path input_file = std::filesystem::path(argv[1]);
  std::filesystem::path output_file = std::filesystem::path(argv[2]);
  std::cout << "Input file: " << input_file.stem() << std::endl;
  std::time_t time_stamp = stringToTime(input_file.stem().string(), date_time_format);
  // Convert from GMT-5 to UTC by subtracting 5 hours
  time_stamp -= 5 * 3600;
  auto sys_tp = std::chrono::system_clock::from_time_t(time_stamp);
  std::cout << "Parsed timestamp (UTC): " << std::asctime(std::localtime(&time_stamp));
  auto milliseconds_dt = std::chrono::duration_cast<std::chrono::milliseconds>(sys_tp.time_since_epoch());
  unsigned long epoch_millis = milliseconds_dt.count();
  std::cout << "Milliseconds since epoch: " << milliseconds_dt.count() << std::endl;

  // Open the input file
  std::ifstream infile(input_file);
  if (!infile.is_open())
  {
    std::cerr << "Error opening input file: " << input_file << std::endl;
    return 1;
  }

  // Open the output file
  std::ofstream outfile(output_file);
  if (!outfile.is_open())
  {
    std::cerr << "Error creating output file: " << output_file << std::endl;
    return 1;
  }

  std::string line;
  // Read the file line by line
  bool first = true;
  unsigned long initial_timestamp = 0, final_timestamp = 0;
  while (std::getline(infile, line))
  {
    if (first)
    {
      // Write header row
      outfile << "time,alt,lat,lng,vNorth,vEast,accNorth,accEast" << std::endl;
      first = false;
      continue;
    }

    std::stringstream ss(line);
    std::string cell;
    int col_index = 0;
    // Split the line into cells using the comma delimiter
    while (std::getline(ss, cell, ','))
    {
      switch (col_index)
      {
      case 0:
        timestamp = std::stoul(cell, nullptr, 0);
        break;
      case 1:
        altitude = std::stod(cell);
        break;
      case 2:
        GPSLatLon.lat = std::stod(cell);
        break;
      case 3:
        GPSLatLon.lon = std::stod(cell);
        break;
      case 4:
        vNorth = std::stod(cell);
        break;
      case 5:
        vEast = std::stod(cell);
        break;
      case 6:
        accNorth = std::stoi(cell);
        break;
      case 7:
        accEast = std::stoi(cell);
        break;
      default:
        break;
      }
      col_index++;
    }
    // Get first value as initial timestamp
    if (initial_timestamp == 0)
    {
      std::cout << "Initializing Sensor Fusion UKF..." << std::endl;
      initial_timestamp = timestamp;
      // Initialize the UKF instances for latitude and longitude
      sf_lat.init(GPSAxis::LATITUDE, GPSLatLon, vNorth);
      sf_lng.init(GPSAxis::LONGITUDE, GPSLatLon, vEast);
    }

    float accNorth_f = static_cast<float>(accNorth / LBM_2_M_S2);
    float accEast_f = static_cast<float>(accEast / LBM_2_M_S2);
    sf_lat.add_imu_acceleration(accNorth_f);
    sf_lng.add_imu_acceleration(accEast_f);
    if(GPSLatLon.lat != 0.0 && GPSLatLon.lon != 0.0) {
      sf_lat.add_gps_position_velocity(GPSLatLon, vNorth);
      sf_lng.add_gps_position_velocity(GPSLatLon, vEast);
  
      // Calculation of the final position
      PointEast = getPointAhead(ZeroPoint, sf_lat.get_predicted_position_meters(), 270);
      PointNorthEast = getPointAhead(PointEast, sf_lng.get_predicted_position_meters(), 180);
    }

    std::cout << timestamp << " Predicted Position - Lat: " << std::format("{:.6f}", PointNorthEast.lat) << ", Lon: " << std::format("{:.6f}", PointNorthEast.lon) << "\t";
    std::cout << "|| Sensor Lat: " << std::format("{:.6f}", GPSLatLon.lat) << ", Long: " << std::format("{:.6f}", GPSLatLon.lon) << " accNorth_f: " << accNorth_f << std::endl;
    final_timestamp = epoch_millis + (timestamp - initial_timestamp);
    outfile << final_timestamp << "," << altitude << "," << PointNorthEast.lat << "," << PointNorthEast.lon;
    outfile << "," << vNorth << "," << vEast << "," << accNorth << "," << accEast << std::endl;
  }

  infile.close();
  outfile.close();

  std::cout << "Successfully processed: " << input_file << std::endl;
  std::cout << "Output saved to: " << output_file << std::endl;

  return 0;
}

void SPEW_THE_ERROR(char const *str)
{
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