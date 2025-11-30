#include "writter.hpp"

const char* header = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>";
const char* gpx_header = "<gpx xmlns=\"http://www.topografix.com/GPX/1/1\" "
                         "xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" "
                         "xmlns:gpxtpx=\"http://www.garmin.com/xmlschemas/TrackPointExtension/v1\" "
                         "version=\"1.1\" creator=\"josejacomeb\" "
                         "xsi:schemaLocation=\"http://www.topografix.com/GPX/1/1 http://www.topografix.com/GPX/1/1/gpx.xsd "
                         "http://www.garmin.com/xmlschemas/TrackPointExtension/v1 http://www.garmin.com/xmlschemas/TrackPointExtensionv1.xsd\">";
const char* trk_start_tag = "<trk>";
const char* trk_end_tag = "</trk>";
const char* trk_start_segm = "<trkseg>";
const char* trk_end_segm = "</trkseg>";
File sd_gpx_file;
char GPX_file_path[21];

void write_gpx(double& latitude, double& lng, double& elevation, TinyGPSDate& date, TinyGPSTime& time) {
  String trkpt = String("<trkpt lat=\"" + String(latitude, 6) + "\" lon=\"" + String(lng, 6) + "\">");
  write_file(trkpt.c_str());
  String elept = String("<ele>" + String(elevation, 3) + "</ele>");
  write_file(elept.c_str());
  String timept = String("<time>" + String(return_time_utc(date, time)) + "</time>");
  write_file(timept.c_str());
  write_file("</trkpt>");
}

void write_file(const char* message) {
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  sd_gpx_file = SD.open(GPX_file_path, FILE_APPEND);
  // if the file opened okay, write to it:
  if (sd_gpx_file) {
    sd_gpx_file.println(message);
    sd_gpx_file.close();  // close the file:
#if PRINT
    Serial.printf("Writing to %s", GPX_file_path);
    Serial.print(message);
    Serial.println(" completed...");
#endif
  } else {
#if PRINT
    Serial.print("error opening file ");
    Serial.println(GPX_file_path);
#endif
  }
}

const char* return_time_utc(TinyGPSDate& date, TinyGPSTime& time) {
  // Define a static buffer to hold the formatted string.
  // The format is YYYY-MM-DDT_HH:MM:SSZ (20 characters + null terminator = 21).
  static char time_buffer[21];

  // Use snprintf for safe, formatted string creation.
  // %04u: 4-digit unsigned integer, zero-padded (for year)
  // %02u: 2-digit unsigned integer, zero-padded (for month, day, hour, minute, second)
  snprintf(
    time_buffer,
    sizeof(time_buffer),
    "%04u-%02u-%02uT%02u:%02u:%02uZ",
    date.year(),
    date.month(),
    date.day(),
    time.hour(),
    time.minute(),
    time.second());

  // Return the pointer to the static buffer.
  return time_buffer;
}

void write_header(TinyGPSPlus& gps) {
  // Use snprintf to format and create the entire filename string safely.
  // %04u: 4-digit unsigned integer, zero-padded (for year)
  // %02u: 2-digit unsigned integer, zero-padded (for month, day, hour, minute, second)
  snprintf(
    GPX_file_path,
    sizeof(GPX_file_path),
    "/%04u%02u%02u_%02u%02u%02u.gpx",
    gps.date.year(),
    gps.date.month(),
    gps.date.day(),
    gps.time.hour(),
    gps.time.minute(),
    gps.time.second());
  write_file(header);
  write_file(gpx_header);
  write_file(String("<metadate>" + String(return_time_utc(gps.date, gps.time)) + "</metadate>").c_str());
  write_file(trk_start_tag);
  write_file(String("<name>" + String(GPX_file_path) + "</name>").c_str());
  write_file(trk_start_segm);
}