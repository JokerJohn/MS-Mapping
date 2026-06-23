#ifndef MS_MAPPING_UTM_TO_MGRS_CONVERTER_H
#define MS_MAPPING_UTM_TO_MGRS_CONVERTER_H

#include <string>

enum class MGRSPrecision {
    _1_METER = 5,
    _100MICRO_METER = 9,
};

enum class CoordinateSystem {
    UTM = 0,
    MGRS = 1,
};

struct GNSSStat {
    GNSSStat()
        : coordinate_system(CoordinateSystem::MGRS),
          northup(true),
          zone(0),
          mgrs_zone(""),
          x(0),
          y(0),
          z(0),
          latitude(0),
          longitude(0),
          altitude(0) {}

    CoordinateSystem coordinate_system;
    bool northup;
    int zone;
    std::string mgrs_zone;
    double x;
    double y;
    double z;
    double latitude;
    double longitude;
    double altitude;
};

GNSSStat convertUTM2MGRS(GNSSStat gnss_stat_utm, const MGRSPrecision precision);

bool convertPCDToMGRS(const std::string& input_file_path, const std::string& output_file_path, 
                      double map_origin_northing, double map_origin_easting, double map_origin_height);

#endif // MS_MAPPING_UTM_TO_MGRS_CONVERTER_H
