#ifndef UTM_CONVERTER_HPP_
#define UTM_CONVERTER_HPP_

#include <cmath>
#include <string>
#include <iostream>

namespace utm_converter
{

// WGS84 Parameters
const double WGS84_A = 6378137.0; // Major semiaxis
const double WGS84_E = 0.081819190842622; // First eccentricity

struct UTMCoords
{
    double easting;
    double northing;
    int zone;
};

// Function to convert latitude and longitude to UTM coordinates
inline UTMCoords toUTM(const double lat, const double lon)
{
    UTMCoords utm;

    if (lat < -80.0 || lat > 84.0) {
        // UTM is not defined for polar regions
        utm.easting = 0;
        utm.northing = 0;
        utm.zone = 0;
        return utm;
    }

    int zone = static_cast<int>((lon + 180.0) / 6) + 1;
    double lon_rad = lon * M_PI / 180.0;
    double lat_rad = lat * M_PI / 180.0;
    
    double zone_lon_rad = (zone * 6 - 183) * M_PI / 180.0;

    double e2 = WGS84_E * WGS84_E;
    double e4 = e2 * e2;
    double e6 = e4 * e2;

    double N = WGS84_A / sqrt(1 - e2 * sin(lat_rad) * sin(lat_rad));
    double T = tan(lat_rad) * tan(lat_rad);
    double C = e2 / (1 - e2) * cos(lat_rad) * cos(lat_rad);
    double A = (lon_rad - zone_lon_rad) * cos(lat_rad);

    double M = WGS84_A * ((1 - e2 / 4 - 3 * e4 / 64 - 5 * e6 / 256) * lat_rad
               - (3 * e2 / 8 + 3 * e4 / 32 + 45 * e6 / 1024) * sin(2 * lat_rad)
               + (15 * e4 / 256 + 45 * e6 / 1024) * sin(4 * lat_rad)
               - (35 * e6 / 3072) * sin(6 * lat_rad));

    utm.easting = 0.9996 * N * (A + (1 - T + C) * pow(A, 3) / 6 + (5 - 18 * T + T * T + 72 * C - 58 * e2) * pow(A, 5) / 120) + 500000.0;
    utm.northing = 0.9996 * (M + N * tan(lat_rad) * (A * A / 2 + (5 - T + 9 * C + 4 * C * C) * pow(A, 4) / 24 + (61 - 58 * T + T * T + 600 * C - 330 * e2) * pow(A, 6) / 720));

    if (lat < 0) {
        utm.northing += 10000000.0; // For southern hemisphere
    }
    
    utm.zone = zone;

    return utm;
}

} // namespace utm_converter

#endif // UTM_CONVERTER_HPP_
