#include "kalman.h"

KalmanFilter::KalmanFilter() {
    // state vector: 7 values, start everything at zero
    // order: ax, ay, az, wx, wy, wz, altitude
    x = Eigen::VectorXd::Zero(7);

    // covariance: start with high uncertainty, filter will settle
    // identity * 10 means we are pretty unsure at the start
    P = Eigen::MatrixXd::Identity(7, 7) * 10.0;

    // process noise Q: how much each state drifts between measurements
    // bigger number = we expect that state to change a lot on its own
    Q = Eigen::MatrixXd::Zero(7, 7);
    Q(0, 0) = 0.5;  // ax can change a lot between steps
    Q(1, 1) = 0.5;  // ay
    Q(2, 2) = 0.5;  // az
    Q(3, 3) = 0.01; // wx changes slowly
    Q(4, 4) = 0.01; // wy
    Q(5, 5) = 0.01; // wz
    Q(6, 6) = 0.1;  // altitude drifts a little

    // low-g sensor noise: this sensor is accurate so R values are small
    // 6x6 because it reports 3 accel + 3 gyro
    R_lowg = Eigen::MatrixXd::Zero(6, 6);
    R_lowg(0, 0) = 0.01; // ax noise
    R_lowg(1, 1) = 0.01; // ay noise
    R_lowg(2, 2) = 0.01; // az noise
    R_lowg(3, 3) = 0.05; // wx noise
    R_lowg(4, 4) = 0.05; // wy noise
    R_lowg(5, 5) = 0.05; // wz noise

    // high-g sensor noise: bigger range but noisier, so R values are larger
    // same shape as low-g, 6x6
    R_highg = Eigen::MatrixXd::Zero(6, 6);
    R_highg(0, 0) = 1.0; // ax noise, much worse than low-g
    R_highg(1, 1) = 1.0; // ay noise
    R_highg(2, 2) = 1.0; // az noise
    R_highg(3, 3) = 0.1; // wx noise
    R_highg(4, 4) = 0.1; // wy noise
    R_highg(5, 5) = 0.1; // wz noise

    // standalone gyro noise: 3x3 because it only reports wx wy wz
    R_gyro = Eigen::MatrixXd::Zero(3, 3);
    R_gyro(0, 0) = 0.05; // wx noise
    R_gyro(1, 1) = 0.05; // wy noise
    R_gyro(2, 2) = 0.05; // wz noise

    // barometer noise: 1x1 because it only reports altitude
    // the README says 1.2m RMS noise so we set this a bit above that
    R_baro = Eigen::MatrixXd::Zero(1, 1);
    R_baro(0, 0) = 0.5;
}

void KalmanFilter::predict(double dt) {
    // we assume state doesnt change on its own (random walk model)
    // so x stays the same, but uncertainty P grows with time
    // the longer since the last update, the less sure we are
    P = P + Q * dt;
}

void KalmanFilter::updateAccelGyro(double ax, double ay, double az,
                                    double wx, double wy, double wz,
                                    bool isHighG) {
    // H is the observation matrix
    // it maps state vector (7 values) to what this sensor measures (6 values)
    // first 6 states map directly 1 to 1, altitude (index 6) is not observed here
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(6, 7);
    H(0, 0) = 1.0; // sensor ax comes from state ax
    H(1, 1) = 1.0; // sensor ay comes from state ay
    H(2, 2) = 1.0; // sensor az comes from state az
    H(3, 3) = 1.0; // sensor wx comes from state wx
    H(4, 4) = 1.0; // sensor wy comes from state wy
    H(5, 5) = 1.0; // sensor wz comes from state wz

    // z is the actual measurement we just received
    Eigen::VectorXd z(6);
    z << ax, ay, az, wx, wy, wz;

    // pick the right noise matrix based on which sensor this came from
    Eigen::MatrixXd& R = isHighG ? R_highg : R_lowg;

    // S = innovation covariance, how uncertain is our predicted measurement
    Eigen::MatrixXd S = H * P * H.transpose() + R;

    // K = Kalman gain, how much to trust this measurement vs our prediction
    // high K = trust sensor more, low K = trust our prediction more
    Eigen::MatrixXd K = P * H.transpose() * S.inverse();

    // update state: pull x toward what the sensor measured, scaled by K
    x = x + K * (z - H * x);

    // update covariance: we are more certain now that we got a measurement
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(7, 7);
    P = (I - K * H) * P;
}

void KalmanFilter::updateGyroOnly(double wx, double wy, double wz) {
    // H maps state (7 values) to what gyro measures (3 values: wx wy wz)
    // only states 3, 4, 5 are observed here
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 7);
    H(0, 3) = 1.0; // sensor wx comes from state wx
    H(1, 4) = 1.0; // sensor wy comes from state wy
    H(2, 5) = 1.0; // sensor wz comes from state wz

    // z is the gyro measurement
    Eigen::VectorXd z(3);
    z << wx, wy, wz;

    // same Kalman math as above, just smaller matrices
    Eigen::MatrixXd S = H * P * H.transpose() + R_gyro;
    Eigen::MatrixXd K = P * H.transpose() * S.inverse();
    x = x + K * (z - H * x);
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(7, 7);
    P = (I - K * H) * P;
}

void KalmanFilter::updateBaro(double altitude) {
    // H maps state (7 values) to what baro measures (1 value: altitude)
    // only state 6 is observed here
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, 7);
    H(0, 6) = 1.0; // sensor altitude comes from state altitude

    // z is the barometer reading
    Eigen::VectorXd z(1);
    z << altitude;

    // same Kalman math again, 1x1 matrices this time
    Eigen::MatrixXd S = H * P * H.transpose() + R_baro;
    Eigen::MatrixXd K = P * H.transpose() * S.inverse();
    x = x + K * (z - H * x);
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(7, 7);
    P = (I - K * H) * P;
}