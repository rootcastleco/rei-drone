/*
 * REI Drone - State Estimator Interface
 * Rootcastle Engineering & Innovation
 *
 * Provides sensor fusion algorithms (Complementary Filter / EKF stub)
 * to estimate the vehicle's attitude (roll, pitch, yaw) and position.
 * 
 * Author: Batuhan Ayribas, M.Sc.
 */

#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include "sensors.h"

struct EstimatorState {
    float roll;         /* Estimated roll angle (radians) */
    float pitch;        /* Estimated pitch angle (radians) */
    float yaw;          /* Estimated yaw angle (radians) */
    
    float roll_rate;    /* Body roll rate (rad/s) */
    float pitch_rate;   /* Body pitch rate (rad/s) */
    float yaw_rate;     /* Body yaw rate (rad/s) */
    
    float altitude;     /* Estimated altitude above ground (m) */
    float vertical_vel; /* Estimated vertical velocity (m/s) */
};

/* Initialize the state estimator */
void estimator_init(void);

/* Update the state estimator with new sensor data.
 * dt: time since last update in seconds.
 */
void estimator_update(const struct IMUData* imu, 
                      const struct BaroData* baro, 
                      const struct MagData* mag, 
                      const struct GPSData* gps,
                      float dt);

/* Get the current estimated state */
void estimator_get_state(struct EstimatorState* state);

#endif /* ESTIMATOR_H */
