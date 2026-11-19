#ifndef TINY_GPS_PLUS_H
#define TINY_GPS_PLUS_H

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

struct RawDegrees {
    uint16_t deg;
    uint32_t billionths;
    bool negative;
};

struct TinyGPSLocation {
    bool valid;
    bool updated;
    double _lat;
    double _lng;

    TinyGPSLocation() : valid(false), updated(false), _lat(0.0), _lng(0.0) {}
    bool isValid() const { return valid; }
    bool isUpdated() const { return updated; }
    double lat() const { return _lat; }
    double lng() const { return _lng; }
};

struct TinyGPSCourse {
    bool valid;
    double _deg;
    TinyGPSCourse() : valid(false), _deg(0.0) {}
    bool isValid() const { return valid; }
    double deg() const { return _deg; }
};

struct TinyGPSSpeed {
    bool valid;
    double _knots;
    TinyGPSSpeed() : valid(false), _knots(0.0) {}
    bool isValid() const { return valid; }
    double knots() const { return _knots; }
    double mps() const { return _knots * 0.514444; }
    double kmph() const { return _knots * 1.852; }
};

class TinyGPSPlus {
private:
    char line_buf[128];
    uint8_t line_idx;

public:
    TinyGPSLocation location;
    TinyGPSCourse course;
    TinyGPSSpeed speed;

    TinyGPSPlus() : line_idx(0) {}

    bool encode(char c) {
        if (c == '\r' || c == '\n') {
            if (line_idx > 0) {
                line_buf[line_idx] = '\0';
                parse_sentence(line_buf);
                line_idx = 0;
                return true;
            }
            return false;
        }
        if (line_idx < 127) {
            line_buf[line_idx++] = c;
        }
        return false;
    }

private:
    void parse_sentence(const char* s) {
        // Simple NMEA parse for $GPRMC,hhmmss,A,lat,N,lon,E,speed,course...
        if (strstr(s, "$GPRMC") != nullptr) {
            char temp[128];
            strncpy(temp, s, 127);
            temp[127] = '\0';
            char* token = strtok(temp, ",");
            int field = 0;
            char* val_lat = nullptr;
            char* dir_lat = nullptr;
            char* val_lon = nullptr;
            char* dir_lon = nullptr;
            char* val_spd = nullptr;
            char* val_crs = nullptr;

            while (token != nullptr) {
                if (field == 3) val_lat = token;
                else if (field == 4) dir_lat = token;
                else if (field == 5) val_lon = token;
                else if (field == 6) dir_lon = token;
                else if (field == 7) val_spd = token;
                else if (field == 8) val_crs = token;
                token = strtok(nullptr, ",");
                field++;
            }

            if (val_lat && dir_lat && val_lon && dir_lon) {
                double raw_lat = atof(val_lat);
                int deg_lat = (int)(raw_lat / 100);
                double min_lat = raw_lat - (deg_lat * 100);
                double dec_lat = deg_lat + (min_lat / 60.0);
                if (dir_lat[0] == 'S') dec_lat = -dec_lat;

                double raw_lon = atof(val_lon);
                int deg_lon = (int)(raw_lon / 100);
                double min_lon = raw_lon - (deg_lon * 100);
                double dec_lon = deg_lon + (min_lon / 60.0);
                if (dir_lon[0] == 'W') dec_lon = -dec_lon;

                location._lat = dec_lat;
                location._lng = dec_lon;
                location.valid = true;
                location.updated = true;
            }

            if (val_spd) {
                speed._knots = atof(val_spd);
                speed.valid = true;
            }
            if (val_crs) {
                course._deg = atof(val_crs);
                course.valid = true;
            }
        }
    }
};

#endif // TINY_GPS_PLUS_H

