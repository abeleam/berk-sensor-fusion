# berk-sensor-fusion
# Sensor Fusion with Kalman Filter
# Berk Aerospace Systems Take-Home Exercise
# Abel Zelalem

---

## How to Build and Run

Requirements: g++ with C++17 support, Eigen (header-only, included in repo)

Compile:
g++ -std=c++17 -O2 -I. -o fusion main.cpp kalman.cpp

Run:
.\fusion

Output is written to output_dev.csv.

Plot:
python plot.py

Requires pandas and matplotlib. Install with: pip install pandas matplotlib

---

## State Vector

I used 7 states: ax, ay, az, wx, wy, wz, and altitude.

The three acceleration components and three angular rate components are what
the IMUs and gyro measure directly, so those were the obvious choices. Altitude
goes in because the barometer gives an independent measurement of it.

I left velocity and position out. To estimate those you have to integrate
acceleration over time, and without proper IMU preintegration that error
builds up fast. Given the time limit I kept it to what the sensors actually
measure directly and got those right instead.

I also left gyro bias out of the state. It adds another 3 states and the
math to go with it. I handled it by giving the standalone gyro a slightly
higher noise value in R so the filter just trusts it a little less.

---

## Process Model

I used a random walk. The assumption is the state doesn't change on its own
between measurements and any change is just noise.

x_new = x_old
P_new = P_old + Q * dt

It's the simplest model that works here. The sensors update often enough that
the state doesn't drift far before getting corrected. The tradeoff is it
doesn't predict real vehicle dynamics, so during any gap between updates the
estimate just holds its last known value.

Process noise Q is diagonal. Acceleration states get 0.5 because they can
change quickly. Angular rate states get 0.01 because they change more slowly.
Altitude gets 0.1 because the barometer updates often enough to keep it in
check.

---

## Measurement Models

Each sensor sees a slice of the state vector directly. The H matrix connects
state to measurement for each update step.

Low-G IMU (100 Hz, +/-8g range): sees ax, ay, az, wx, wy, wz. This is the
most trusted sensor under normal conditions so R is small, 0.01 for accel
channels and 0.05 for gyro channels. Skipped when low_g_saturated is True.

High-G IMU (20 Hz, +/-32g range): also sees ax, ay, az, wx, wy, wz. Always
used regardless of saturation. R is larger here, 1.0 for accel and 0.1 for
gyro, because it's noisier than the low-G under normal conditions. During a
saturation event it becomes the only accel source and the filter relies on it
more naturally because the low-G update just stops happening.

Standalone MEMS Gyro (100 Hz): sees wx, wy, wz only. Has a known measurement
bias that I didn't model as a state, so R is set to 0.05 to reduce how much
the filter leans on it.

Barometer (roughly 10 Hz, asynchronous): sees altitude only. The data README
notes about 1.2m RMS noise plus a small offset. R is set to 0.5. The timestamp
jitter is handled naturally since the filter just processes rows in time order
regardless of which sensor they came from.

---

## Saturation Handling

The dataset has a low_g_saturated column that flags when the low-G sensor
hits its range limit. When that flag is True, I skip the measurement entirely
and do no update from it.

The high-G IMU keeps running through those events since it has a +/-32g range
and doesn't saturate. The transition is smooth because there's no hard switch
happening. The low-G update just stops and the high-G update keeps going with
its own noise weighting. The two saturation events in the data show up as
spikes at roughly 8s and 20s, which matches what the README described.

---

## Assumptions

Gravity is already excluded from accelerometer readings per the data README,
so I don't subtract 9.81 from az.

The body frame stays fixed throughout. The README says the vehicle stays near
vertical so I didn't need to estimate orientation.

All sensors share one clock. Timing jitter is handled by processing rows in
timestamp order, which is how the CSV is already sorted.

The flight recording is ascent only. Altitude goes from 103m to around 480m
over 30 seconds with no descent in the data.

A test flight CSV was not included in the files provided to me, so the
development flight output is my primary submission.

---

## Known Limitations

The random walk process model doesn't predict vehicle dynamics. During any gap
between sensor updates the estimate just holds its last value.

Gyro bias is not in the state vector. It accumulates over time and adds some
error to the angular rate estimate, probably more noticeable late in the flight.

Q and R values were tuned by hand based on looking at output plots. They
weren't derived from sensor calibration data or Allan variance analysis. With
more time I'd do that properly.

Altitude tracks the barometer well but would drift without it. There's no
independent check on it.

The filter uses Eigen's built-in matrix inverse. For 7 states that's fine,
but it wouldn't hold up for much larger state vectors without switching to
something like a square root filter.

No velocity or position is estimated. I left those out because of the time
limit and the integration error problem mentioned above.
