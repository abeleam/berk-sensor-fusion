#pragma once
#include <string>

// one row from the CSV lives here
// not every field is filled, depends on which sensor it came from
struct Measurement {
    double time;        // timestamp in seconds
    std::string sensor; // "low_g_imu", "high_g_imu", "gyro", "barometer"

    // acceleration, only low_g and high_g fill these
    double ax, ay, az;

    // angular rate, low_g, high_g, and gyro fill these
    double wx, wy, wz;

    // altitude, only barometer fills this
    double altitude;

    // true means low_g sensor hit its limit and is lying
    bool low_g_saturated;

    // flags so we know which fields above are actually valid
    bool has_accel; // ax ay az are real numbers
    bool has_gyro;  // wx wy wz are real numbers
    bool has_baro;  // altitude is a real number
};