#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <iomanip>
#include "sensor_types.h"
#include "kalman.h"

// reads sensor_data.csv and returns every row as a Measurement struct
std::vector<Measurement> loadCSV(const std::string& path) {
    std::vector<Measurement> data;
    std::ifstream file(path);

    // bail out if file didnt open
    if (!file.is_open()) {
        std::cerr << "ERROR: could not open " << path << "\n";
        return data;
    }

    std::string line;
    std::getline(file, line); // first line is the header, skip it

    while (std::getline(file, line)) {
        // skip blank lines
        if (line.empty()) continue;

        std::stringstream ss(line);
        std::string token;
        Measurement m = {}; // zero everything out first

        // column 0: timestamp
        std::getline(ss, token, ',');
        m.time = std::stod(token);

        // column 1: sensor name
        std::getline(ss, token, ',');
        m.sensor = token;

        // column 2: ax. empty means this sensor doesnt report accel
        std::getline(ss, token, ',');
        m.has_accel = !token.empty();
        m.ax = token.empty() ? 0.0 : std::stod(token);

        // column 3: ay
        std::getline(ss, token, ',');
        m.ay = token.empty() ? 0.0 : std::stod(token);

        // column 4: az
        std::getline(ss, token, ',');
        m.az = token.empty() ? 0.0 : std::stod(token);

        // column 5: wx. empty means this sensor doesnt report angular rate
        std::getline(ss, token, ',');
        m.has_gyro = !token.empty();
        m.wx = token.empty() ? 0.0 : std::stod(token);

        // column 6: wy
        std::getline(ss, token, ',');
        m.wy = token.empty() ? 0.0 : std::stod(token);

        // column 7: wz
        std::getline(ss, token, ',');
        m.wz = token.empty() ? 0.0 : std::stod(token);

        // column 8: altitude. empty means this sensor doesnt report altitude
        std::getline(ss, token, ',');
        m.has_baro = !token.empty();
        m.altitude = token.empty() ? 0.0 : std::stod(token);

        // column 9: low_g_saturated flag. only matters for low_g_imu rows
        std::getline(ss, token, ',');
        m.low_g_saturated = (token == "True");

        data.push_back(m);
    }

    return data;
}

int main() {
    // load all sensor data into memory
    std::vector<Measurement> measurements = loadCSV("data/sensor_data.csv");

    if (measurements.empty()) {
        std::cerr << "ERROR: no data loaded, check your CSV path\n";
        return 1;
    }

    std::cout << "Loaded " << measurements.size() << " measurements\n";

    // create the filter
    KalmanFilter kf;

    // set starting altitude from the first barometer reading
    // dont just leave it at zero because the launch pad is at real altitude
    for (auto& m : measurements) {
        if (m.sensor == "barometer") {
            kf.x(6) = m.altitude;
            std::cout << "Starting altitude set to " << m.altitude << " m\n";
            break;
        }
    }

    // open output file
    std::ofstream out("output_dev.csv");
    if (!out.is_open()) {
        std::cerr << "ERROR: could not create output_dev.csv\n";
        return 1;
    }

    // write CSV header
    out << "t_s,ax_mps2,ay_mps2,az_mps2,wx_radps,wy_radps,wz_radps,altitude_m\n";
    out << std::fixed << std::setprecision(6);

    double prev_time = 0.0;  // time of last measurement
    double t_output = 0.0;   // next output timestamp we need to write
    double dt_output = 0.01; // 100 Hz output = one row every 0.01 seconds

    // loop through every measurement in time order
    for (auto& m : measurements) {
        // how many seconds since last measurement
        double dt = m.time - prev_time;

        // predict step: let uncertainty grow since last update
        if (dt > 0.0) {
            kf.predict(dt);
        }

        prev_time = m.time;

        // update step: feed measurement into filter based on sensor type
        if (m.sensor == "low_g_imu") {
            if (!m.low_g_saturated) {
                // low-g is good, use it, it is the most accurate sensor
                kf.updateAccelGyro(m.ax, m.ay, m.az,
                                   m.wx, m.wy, m.wz, false);
            }
            // if saturated, do nothing, the high-g will cover this moment
        }
        else if (m.sensor == "high_g_imu") {
            // high-g is always available, just noisier
            // filter will weight it less because R_highg is bigger
            kf.updateAccelGyro(m.ax, m.ay, m.az,
                               m.wx, m.wy, m.wz, true);
        }
        else if (m.sensor == "gyro") {
            // standalone gyro, only gives angular rate
            kf.updateGyroOnly(m.wx, m.wy, m.wz);
        }
        else if (m.sensor == "barometer") {
            // barometer, only gives altitude
            kf.updateBaro(m.altitude);
        }

        // write output rows at exactly 100 Hz up to current time
        // this covers any gap between measurements with the last known state
        while (t_output <= m.time + 1e-9) {
            out << t_output       << ","
                << kf.x(0)       << ","  // ax
                << kf.x(1)       << ","  // ay
                << kf.x(2)       << ","  // az
                << kf.x(3)       << ","  // wx
                << kf.x(4)       << ","  // wy
                << kf.x(5)       << ","  // wz
                << kf.x(6)       << "\n"; // altitude
            t_output += dt_output;
        }
    }

    out.close();
    std::cout << "Done. Output written to output_dev.csv\n";
    return 0;
}