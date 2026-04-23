/*
 * REI Drone - State Estimator Implementation
 * Rootcastle Engineering & Innovation
 *
 * Implements a Complementary Filter for attitude estimation.
 * Fuses high-frequency gyroscope data with low-frequency accelerometer
 * data to calculate stable roll and pitch. Yaw is integrated from the gyro
 * and optionally corrected by the magnetometer.
 *
 * Author: Batuhan Ayribas, M.Sc.
 */

#include "estimator.h"
#include <math.h>

#define ALPHA 0.98f /* Complementary filter weight (favors gyro for short term) */

static struct EstimatorState current_state;

void estimator_init(void) {
    current_state.roll = 0.0f;
    current_state.pitch = 0.0f;
    current_state.yaw = 0.0f;
    
    current_state.roll_rate = 0.0f;
    current_state.pitch_rate = 0.0f;
    current_state.yaw_rate = 0.0f;
    
    current_state.altitude = 0.0f;
    current_state.vertical_vel = 0.0f;
}

void estimator_update(const struct IMUData* imu, 
                      const struct BaroData* baro, 
                      const struct MagData* mag, 
                      const struct GPSData* gps,
                      float dt) {
    
    if (!imu) return;

    /* 1. Update angular rates directly from Gyroscope */
    current_state.roll_rate = imu->gyro_x;
    current_state.pitch_rate = imu->gyro_y;
    current_state.yaw_rate = imu->gyro_z;

    /* 2. Integrate Gyroscope for short-term attitude */
    float gyro_roll = current_state.roll + (current_state.roll_rate * dt);
    float gyro_pitch = current_state.pitch + (current_state.pitch_rate * dt);
    current_state.yaw = current_state.yaw + (current_state.yaw_rate * dt);

    /* 3. Calculate long-term attitude from Accelerometer 
       Assuming NED coordinate system (Z points down, gravity is negative)
    */
    float accel_roll = atan2f(imu->accel_y, sqrtf(imu->accel_x * imu->accel_x + imu->accel_z * imu->accel_z));
    float accel_pitch = atan2f(-imu->accel_x, sqrtf(imu->accel_y * imu->accel_y + imu->accel_z * imu->accel_z));

    /* 4. Complementary Filter Fusion */
    current_state.roll = ALPHA * gyro_roll + (1.0f - ALPHA) * accel_roll;
    current_state.pitch = ALPHA * gyro_pitch + (1.0f - ALPHA) * accel_pitch;

    /* Note: Yaw would normally be fused with magnetometer data here */
    if (mag) {
        /* Simple Mag yaw correction stub */
        /* float mag_yaw = atan2f(-mag->mag_y, mag->mag_x); */
        /* current_state.yaw = ALPHA_YAW * current_state.yaw + (1.0f - ALPHA_YAW) * mag_yaw; */
    }

    /* 5. Altitude Estimation (Basic Barometer low-pass) */
    if (baro) {
        float alt_alpha = 0.1f;
        float prev_alt = current_state.altitude;
        current_state.altitude = (1.0f - alt_alpha) * current_state.altitude + (alt_alpha * baro->altitude);
        current_state.vertical_vel = (current_state.altitude - prev_alt) / dt;
    }
}

void estimator_get_state(struct EstimatorState* state) {
    if (state) {
        *state = current_state;
    }
}
