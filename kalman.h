#pragma once
#include <Eigen/Dense>

// the filter. holds state and does math.
// state vector x has 7 values: ax, ay, az, wx, wy, wz, altitude
class KalmanFilter {
public:
    KalmanFilter(); // sets up starting values

    // call this between measurements, dt is seconds since last update
    void predict(double dt);

    // call this when low_g or high_g gives us accel + gyro together
    // isHighG = true means use the noisier high-g sensor noise values
    void updateAccelGyro(double ax, double ay, double az,
                         double wx, double wy, double wz,
                         bool isHighG);

    // call this when the standalone gyro sends a reading
    void updateGyroOnly(double wx, double wy, double wz);

    // call this when barometer sends altitude
    void updateBaro(double altitude);

    // these are public so main.cpp can read and write the output CSV
    Eigen::VectorXd x; // the 7 state values we are estimating
    Eigen::MatrixXd P; // 7x7 matrix, tracks how uncertain we are

private:
    // Q: how much we expect the state to drift on its own between updates
    Eigen::MatrixXd Q;

    // R matrices: how noisy each sensor is
    // bigger R = we trust that sensor less
    Eigen::MatrixXd R_lowg;  // low-g accel + gyro noise (6x6)
    Eigen::MatrixXd R_highg; // high-g accel + gyro noise (6x6), noisier
    Eigen::MatrixXd R_gyro;  // standalone gyro noise (3x3)
    Eigen::MatrixXd R_baro;  // barometer noise (1x1)
};