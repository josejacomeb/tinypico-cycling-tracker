#include "writer.hpp"

// ---- GPX File Constants ----

const char* xml_header = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>";
const char* gpx_header = "<gpx xmlns=\"http://www.topografix.com/GPX/1/1\" "
                         "xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" "
                         "xmlns:gpxtpx=\"http://www.garmin.com/xmlschemas/TrackPointExtension/v1\" "
                         "version=\"1.1\" creator=\"josejacomeb\" "
                         "xsi:schemaLocation=\"http://www.topografix.com/GPX/1/1 "
                         "http://www.topografix.com/GPX/1/1/gpx.xsd "
                         "http://www.garmin.com/xmlschemas/TrackPointExtension/v1 "
                         "http://www.garmin.com/xmlschemas/TrackPointExtensionv1.xsd\">";
const char* trk_start_tag = "<trk>";
const char* trk_end_tag = "</trk>";
const char* trk_start_segm = "<trkseg>";
const char* trk_end_segm = "</trkseg>";

File sd_file;
char GPX_file_path[21];

// ---- GPX Writing Methods ----

void write_gpx(LatLonDeg& pos, double& elevation, TinyGPSDate& date, TinyGPSTime& time) {
  String trkpt = "<trkpt lat=\"" + String(pos.lat, 6) + "\" lon=\"" + String(pos.lon, 6) + "\">";
  String elept = "<ele>" + String(elevation, 3) + "</ele>";
  String timept = "<time>" + String(return_time_utc(date, time)) + "</time>";

  write_file(trkpt.c_str());
  write_file(elept.c_str());
  write_file(timept.c_str());
  write_file("</trkpt>");
}

// Now: call open_log_file() once when recording starts, close_log_file() when it stops.

void open_log_file() {
  sd_file = SD.open(GPX_file_path, FILE_APPEND);
  if (!sd_file) {
    Serial.print("error opening file ");
    Serial.println(GPX_file_path);
  }
}

void close_log_file() {
  if (sd_file) {
    sd_file.close();
  }
}

void write_file(const char* message) {
  if (sd_file) {
    sd_file.println(message);
#if PRINT
    Serial.printf("Writing to %s: %s\n", GPX_file_path, message);
#endif
  } else {
    Serial.print("File not open: ");
    Serial.println(GPX_file_path);
  }
}

// ---- Time Conversion Methods ----

const char* return_time_utc(TinyGPSDate& date, TinyGPSTime& time) {
  // Format: YYYY-MM-DDTHH:MM:SSZ (20 chars + null = 21)
  static char time_buffer[21];
  snprintf(
    time_buffer, sizeof(time_buffer),
    "%04u-%02u-%02uT%02u:%02u:%02uZ",
    date.year(), date.month(), date.day(),
    time.hour(), time.minute(), time.second());
  return time_buffer;
}

void set_file_name(TinyGPSPlus& gps, const char* extension) {
  snprintf(
    GPX_file_path, sizeof(GPX_file_path),
    "/%04u%02u%02u_%02u%02u%02u.%03s",
    gps.date.year(), gps.date.month(), gps.date.day(),
    gps.time.hour(), gps.time.minute(), gps.time.second(),
    extension);
}

// ---- Header Writing Methods ----

void write_header(TinyGPSPlus& gps) {
  set_file_name(gps, "gpx");
  open_log_file();  // open once for the whole GPX session
  write_file(xml_header);
  write_file(gpx_header);
  write_file(String("<metadata><time>" + String(return_time_utc(gps.date, gps.time)) + "</time></metadata>").c_str());
  write_file(trk_start_tag);
  write_file(String("<name>" + String(GPX_file_path) + "</name>").c_str());
  write_file(trk_start_segm);
}

void write_footer() {
  write_file(trk_end_segm);
  write_file(trk_end_tag);
  write_file("</gpx>");
  close_log_file();  // close once at the end
}
