/*
 * REI Drone - Flight Controller Core
 * Rootcastle Engineering & Innovation
 *
 * Implements the cascaded control architecture for the REI Drone
 * VTOL tilt-rotor UAV. The controller operates at multiple rates:
 *   - Rate control:     1000 Hz (inner loop)
 *   - Attitude control:  250 Hz (outer loop)
 *   - Altitude control:   50 Hz
 *   - Position control:   10 Hz
 *
 * Author: Batuhan Ayribas, M.Sc.
 */

#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include "pid.h"
#include "../config/vehicle_params.h"

/* Flight modes */
enum FlightMode {
    MODE_DISARMED = 0,
    MODE_STABILIZE,
    MODE_ALT_HOLD,
    MODE_LOITER,
    MODE_AUTO,
    MODE_RTL,
    MODE_LAND,
    MODE_TRANSITION
};

/* Vehicle state passed from sensor fusion */
struct VehicleState {
    /* Attitude (radians) */
    float roll;
    float pitch;
    float yaw;

    /* Angular rates (rad/s) */
    float roll_rate;
    float pitch_rate;
    float yaw_rate;

    /* Position NED (meters) */
    float pos_north;
    float pos_east;
    float pos_down;

    /* Velocity NED (m/s) */
    float vel_north;
    float vel_east;
    float vel_down;

    /* Airspeed (m/s) */
    float airspeed;

    /* GPS quality */
    int gps_fix_type;
    int gps_num_sats;

    /* Battery */
    float battery_voltage;
    float battery_current;
    float battery_remaining;

    /* Tilt angle (degrees) */
    float tilt_angle;
};

/* Control commands from pilot or navigation */
struct ControlCommand {
    float roll_cmd;        /* Desired roll angle or rate */
    float pitch_cmd;       /* Desired pitch angle or rate */
    float yaw_cmd;         /* Desired yaw angle or rate */
    float throttle_cmd;    /* Desired throttle (0-1) or altitude */
    float tilt_cmd;        /* Desired tilt angle (degrees) */
};

/* Motor outputs (0.0 to 1.0 normalized) */
struct MotorOutput {
    float motor[NUM_MOTORS];
    float tilt_servo[NUM_MOTORS];
};

/* Flight controller context */
struct FlightController {
    enum FlightMode mode;
    enum FlightMode prev_mode;

    /* PID controllers for each axis */
    struct PIDConfig rate_roll_cfg;
    struct PIDState  rate_roll_state;

    struct PIDConfig rate_pitch_cfg;
    struct PIDState  rate_pitch_state;

    struct PIDConfig rate_yaw_cfg;
    struct PIDState  rate_yaw_state;

    struct PIDConfig att_roll_cfg;
    struct PIDState  att_roll_state;

    struct PIDConfig att_pitch_cfg;
    struct PIDState  att_pitch_state;

    struct PIDConfig att_yaw_cfg;
    struct PIDState  att_yaw_state;

    struct PIDConfig alt_cfg;
    struct PIDState  alt_state;

    struct PIDConfig pos_north_cfg;
    struct PIDState  pos_north_state;

    struct PIDConfig pos_east_cfg;
    struct PIDState  pos_east_state;

    /* Target setpoints */
    float target_altitude;
    float target_heading;
    float target_north;
    float target_east;

    /* Transition state */
    float current_tilt_angle;
    int transition_active;

    /* Timing */
    unsigned long last_rate_update_us;
    unsigned long last_att_update_us;
    unsigned long last_alt_update_us;
    unsigned long last_pos_update_us;

    /* Motor output */
    struct MotorOutput output;

    /* Armed state */
    int armed;
};

/* Initialize the flight controller with default gains */
void fc_init(struct FlightController *fc);

/* Update the rate controller (call at 1000 Hz) */
void fc_update_rate(struct FlightController *fc,
                    const struct VehicleState *state,
                    const struct ControlCommand *cmd,
                    float dt);

/* Update the attitude controller (call at 250 Hz) */
void fc_update_attitude(struct FlightController *fc,
                        const struct VehicleState *state,
                        const struct ControlCommand *cmd,
                        float dt);

/* Update the altitude controller (call at 50 Hz) */
void fc_update_altitude(struct FlightController *fc,
                        const struct VehicleState *state,
                        float dt);

/* Update the position controller (call at 10 Hz) */
void fc_update_position(struct FlightController *fc,
                        const struct VehicleState *state,
                        float dt);

/* Set the flight mode */
void fc_set_mode(struct FlightController *fc, enum FlightMode mode);

/* Arm or disarm the vehicle */
void fc_set_armed(struct FlightController *fc, int armed);

/* Get the current motor outputs */
const struct MotorOutput *fc_get_output(const struct FlightController *fc);

#endif /* FLIGHT_CONTROLLER_H */
