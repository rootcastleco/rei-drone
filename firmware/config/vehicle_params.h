/*
 * REI Drone - Vehicle Configuration Parameters
 * Rootcastle Engineering & Innovation
 *
 * Central configuration file for the REI Drone VTOL tilt-rotor UAV.
 * All physical constants, tuning parameters, and hardware-specific
 * values are defined here. Modify this file to adapt the firmware
 * to different airframe configurations.
 *
 * Author: Batuhan Ayribas, M.Sc.
 */

#ifndef VEHICLE_PARAMS_H
#define VEHICLE_PARAMS_H

/* ---------------------------------------------------------------
 * Airframe Geometry
 * --------------------------------------------------------------- */
#define VEHICLE_MASS_KG             2.0f
#define ARM_LENGTH_M                0.25f
#define WINGSPAN_M                  1.0f
#define FUSELAGE_LENGTH_M           0.65f

/* ---------------------------------------------------------------
 * Motor Configuration
 * 4x 2216 880KV Brushless with 10x4.5" 3-blade propellers
 * --------------------------------------------------------------- */
#define NUM_MOTORS                  4
#define MOTOR_KV                    880
#define PROP_DIAMETER_INCH          10.0f
#define PROP_PITCH_INCH             4.5f
#define NUM_BLADES                  3

/* Motor positions relative to CG (body frame, meters)
 * Motor 1: Front Left  (+x, -y)
 * Motor 2: Front Right (+x, +y)
 * Motor 3: Rear Left   (-x, -y)
 * Motor 4: Rear Right  (-x, +y) */
#define MOTOR_X_OFFSET_M            0.18f
#define MOTOR_Y_OFFSET_M            0.18f

/* Thrust limits */
#define THRUST_MAX_PER_MOTOR_N      8.2f
#define THRUST_MIN_N                0.0f
#define IDLE_THROTTLE               0.05f

/* ---------------------------------------------------------------
 * Tilt Mechanism
 * Servo-driven motor tilting for VTOL-to-cruise transition
 * --------------------------------------------------------------- */
#define TILT_ENABLED                1
#define TILT_ANGLE_HOVER_DEG        0.0f
#define TILT_ANGLE_CRUISE_DEG       90.0f
#define TILT_RATE_DEG_PER_SEC       18.0f
#define TILT_SERVO_MIN_US           1000
#define TILT_SERVO_MAX_US           2000
#define TRANSITION_AIRSPEED_MS      12.0f

/* ---------------------------------------------------------------
 * Battery (6S LiPo, 5000 mAh)
 * --------------------------------------------------------------- */
#define BATTERY_CELLS               6
#define BATTERY_CAPACITY_MAH        5000
#define BATTERY_VOLTAGE_FULL        25.2f
#define BATTERY_VOLTAGE_NOMINAL     22.2f
#define BATTERY_VOLTAGE_CUTOFF      21.0f
#define BATTERY_CELL_CUTOFF         3.5f
#define BATTERY_CELL_WARNING        3.6f

/* ADC calibration for voltage and current sensing */
#define BATT_VOLTAGE_DIVIDER        11.0f
#define BATT_CURRENT_SCALE          17.0f

/* ---------------------------------------------------------------
 * Flight Controller PID Gains
 * Tuned for the REI Drone airframe
 * --------------------------------------------------------------- */

/* Rate controller (inner loop, 1000 Hz) */
#define RATE_ROLL_KP                6.0f
#define RATE_ROLL_KI                1.0f
#define RATE_ROLL_KD                0.15f

#define RATE_PITCH_KP               6.0f
#define RATE_PITCH_KI               1.0f
#define RATE_PITCH_KD               0.15f

#define RATE_YAW_KP                 4.0f
#define RATE_YAW_KI                 0.5f
#define RATE_YAW_KD                 0.1f

/* Attitude controller (outer loop, 250 Hz) */
#define ATT_ROLL_KP                 5.0f
#define ATT_PITCH_KP                5.0f
#define ATT_YAW_KP                  3.5f

/* Altitude controller (50 Hz) */
#define ALT_KP                      3.0f
#define ALT_KI                      0.5f
#define ALT_KD                      1.5f

/* Position controller (10 Hz) */
#define POS_KP                      1.0f
#define POS_KI                      0.1f
#define POS_KD                      0.5f

/* ---------------------------------------------------------------
 * Moments of Inertia (kg.m^2)
 * Estimated for X-frame quad-tilt-rotor layout
 * --------------------------------------------------------------- */
#define INERTIA_XX                  0.015f
#define INERTIA_YY                  0.015f
#define INERTIA_ZZ                  0.025f

/* ---------------------------------------------------------------
 * Sensor Configuration
 * --------------------------------------------------------------- */
#define IMU_SAMPLE_RATE_HZ          1000
#define BARO_SAMPLE_RATE_HZ         50
#define GPS_SAMPLE_RATE_HZ          10
#define MAG_SAMPLE_RATE_HZ          100

/* IMU orientation (rotation from sensor frame to body frame) */
#define IMU_ROTATION_NONE           0

/* ---------------------------------------------------------------
 * Telemetry (915 MHz MAVLink)
 * --------------------------------------------------------------- */
#define TELEMETRY_BAUD_RATE         57600
#define TELEMETRY_UPDATE_RATE_HZ    10
#define MAVLINK_SYSTEM_ID           1
#define MAVLINK_COMPONENT_ID        1

/* ---------------------------------------------------------------
 * RC Input
 * --------------------------------------------------------------- */
#define RC_CHANNELS                 8
#define RC_MIN_US                   1000
#define RC_MAX_US                   2000
#define RC_MID_US                   1500
#define RC_DEADZONE_US              20

/* Channel mapping */
#define RC_CH_ROLL                  0
#define RC_CH_PITCH                 1
#define RC_CH_THROTTLE              2
#define RC_CH_YAW                   3
#define RC_CH_MODE                  4
#define RC_CH_TILT                  5
#define RC_CH_AUX1                  6
#define RC_CH_AUX2                  7

/* ---------------------------------------------------------------
 * Failsafe Thresholds
 * --------------------------------------------------------------- */
#define FAILSAFE_VOLTAGE_WARN       (BATTERY_CELL_WARNING * BATTERY_CELLS)
#define FAILSAFE_VOLTAGE_CRITICAL   (BATTERY_CELL_CUTOFF * BATTERY_CELLS)
#define FAILSAFE_RC_TIMEOUT_MS      1000
#define FAILSAFE_GPS_TIMEOUT_MS     3000
#define FAILSAFE_ALTITUDE_MAX_M     120.0f
#define FAILSAFE_GEOFENCE_RADIUS_M  500.0f

/* ---------------------------------------------------------------
 * Navigation
 * --------------------------------------------------------------- */
#define NAV_WAYPOINT_RADIUS_M       3.0f
#define NAV_LOITER_RADIUS_M         30.0f
#define NAV_RTL_ALTITUDE_M          30.0f
#define NAV_MAX_SPEED_MS            28.0f
#define NAV_CRUISE_SPEED_MS         15.0f

/* ---------------------------------------------------------------
 * Physical Constants
 * --------------------------------------------------------------- */
#define GRAVITY_MS2                 9.81f
#define AIR_DENSITY_KGM3            1.225f
#define DEG_TO_RAD                  0.01745329252f
#define RAD_TO_DEG                  57.2957795131f

#endif /* VEHICLE_PARAMS_H */
