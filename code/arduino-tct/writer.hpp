#ifndef WRITER_HPP
#define WRITER_HPP

#include <SD.h>
#include <TinyGPS++.h>

#include "data_types.hpp"

// Define a buffer large enough for the filename.
// Format: /YYYYMMDD_HHMMSS.ext (1 + 8 + 1 + 6 + 1 + 3 = 20 characters + null terminator = 21)
extern char GPX_file_path[21];
extern const char* xml_header;
extern const char* gpx_header;
extern const char* trk_start_tag;
extern const char* trk_end_tag;
extern const char* trk_start_segm;
extern const char* trk_end_segm;
extern File sd_file;

// ---- Session lifecycle ----
// Call open_log_file() once when recording starts, close_log_file() when it stops.
// Do NOT call write_file() before open_log_file() or after close_log_file().
void open_log_file();
void close_log_file();

// ---- File naming ----
void set_file_name(TinyGPSPlus& gps, const char* extension);

// ---- Header / footer ----
// write_header() calls set_file_name() and open_log_file() internally.
// write_footer() closes the GPX structure and calls close_log_file() internally.
void write_header(TinyGPSPlus& gps);
void write_footer();

// ---- Low-level write ----
void write_file(const char* message);

// ---- GPX track point ----
void write_gpx(LatLonDeg& pos, double& elevation, TinyGPSDate& date, TinyGPSTime& time);

// ---- Time helpers ----
// Returns a pointer to a static buffer — copy immediately if you need to hold the value.
const char* return_time_utc(TinyGPSDate& date, TinyGPSTime& time);

#endif  // WRITER_HPP