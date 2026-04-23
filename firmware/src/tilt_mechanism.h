/*
 * REI Drone - Tilt Mechanism Control
 * Rootcastle Engineering & Innovation
 *
 * Controls the servo-driven tilt mechanisms that rotate the motors
 * between vertical (hover) and horizontal (cruise) orientations.
 * Implements a smooth S-curve transition profile to prevent
 * abrupt attitude disturbances during the changeover.
 *
 * The REI Drone features 4 independent tilt mechanisms (one per motor)
 * driven by standard PWM servos. During hover, the motors point upward.
 * As forward speed builds, the tilt angle gradually increases until
 * the motors are fully horizontal for cruise flight.
 *
 * Author: Batuhan Ayribas, M.Sc.
 */

#ifndef TILT_MECHANISM_H
#define TILT_MECHANISM_H

#include "../config/vehicle_params.h"

enum TiltState {
    TILT_HOVER = 0,         /* Motors vertical, full hover */
    TILT_TRANSITION_FWD,    /* Transitioning from hover to cruise */
    TILT_CRUISE,            /* Motors horizontal, full cruise */
    TILT_TRANSITION_REV     /* Transitioning from cruise to hover */
};

struct TiltController {
    enum TiltState state;
    float current_angle;        /* Current tilt angle in degrees */
    float target_angle;         /* Target tilt angle in degrees */
    float transition_progress;  /* 0.0 to 1.0 */
    float min_transition_speed; /* Minimum airspeed to begin transition (m/s) */
    float full_transition_speed;/* Airspeed for full cruise tilt (m/s) */
};

/* Initialize the tilt controller */
void tilt_init(struct TiltController *tilt);

/* Update the tilt controller based on current airspeed and commands.
 * airspeed_ms: current forward airspeed in m/s
 * manual_tilt: pilot-commanded tilt override (-1 to ignore)
 * dt: time step in seconds */
void tilt_update(struct TiltController *tilt, float airspeed_ms,
                 float manual_tilt, float dt);

/* Get the current tilt angle for a specific motor (degrees) */
float tilt_get_angle(const struct TiltController *tilt, int motor_index);

/* Convert tilt angle to servo PWM microseconds */
int tilt_angle_to_pwm(float angle_deg);

/* Command an immediate transition to hover (for failsafe) */
void tilt_emergency_hover(struct TiltController *tilt);

/* Check whether the vehicle is in a safe state for transition */
int tilt_can_transition(const struct TiltController *tilt, float airspeed_ms,
                        float altitude_m);

#endif /* TILT_MECHANISM_H */
