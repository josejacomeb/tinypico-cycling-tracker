#ifndef WRITTER_HPP
#define WRITTER_HPP

#include <SD.h>
#include <TinyGPS++.h>

// Define a buffer large enough for the filename.
// Format: /YYYYMMDD_HHMMSS.gpx (1 + 4 + 2 + 2 + 1 + 2 + 2 + 2 + 4 = 20 characters + null terminator = 21)
extern char GPX_file_path[21];
extern const char* header;
extern const char* gpx_header;
extern const char* trk_start_tag;
extern const char* trk_end_tag;
extern const char* trk_start_segm; 
extern const char* trk_end_segm;
extern File sd_gpx_file;
void write_gpx(TinyGPSLocation& Pos1, double& elevation, TinyGPSDate& date, TinyGPSTime& time);
void write_header(TinyGPSPlus& gps);
void write_file(const char* message);
const char* return_time_utc(TinyGPSDate& date, TinyGPSTime& time);

#endif