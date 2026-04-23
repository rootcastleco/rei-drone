/*
 * REI Drone - Flight Controller Implementation
 * Rootcastle Engineering & Innovation
 *
 * Author: Batuhan Ayribas, M.Sc.
 */

#include "flight_controller.h"
#include <math.h>

/* Helper to constrain a value within bounds */
static float constrain_float(float val, float min_val, float max_val)
{
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

/* Wrap an angle to the range -PI to PI */
static float wrap_pi(float angle)
{
    while (angle > 3.14159265f) angle -= 2.0f * 3.14159265f;
    while (angle < -3.14159265f) angle += 2.0f * 3.14159265f;
    return angle;
}

/* Configure a PID with given gains and limits */
static void setup_pid(struct PIDConfig *cfg, struct PIDState *st,
                       float kp, float ki, float kd,
                       float out_min, float out_max, float i_max)
{
    cfg->kp = kp;
    cfg->ki = ki;
    cfg->kd = kd;
    cfg->output_min = out_min;
    cfg->output_max = out_max;
    cfg->integral_max = i_max;
    cfg->d_filter_alpha = 0.5f;
    pid_init(st);
}

void fc_init(struct FlightController *fc)
{
    fc->mode = MODE_DISARMED;
    fc->prev_mode = MODE_DISARMED;
    fc->armed = 0;
    fc->current_tilt_angle = TILT_ANGLE_HOVER_DEG;
    fc->transition_active = 0;

    fc->target_altitude = 0.0f;
    fc->target_heading = 0.0f;
    fc->target_north = 0.0f;
    fc->target_east = 0.0f;

    /* Rate controllers (inner loop) */
    setup_pid(&fc->rate_roll_cfg,  &fc->rate_roll_state,
              RATE_ROLL_KP, RATE_ROLL_KI, RATE_ROLL_KD,
              -1.0f, 1.0f, 0.5f);

    setup_pid(&fc->rate_pitch_cfg, &fc->rate_pitch_state,
              RATE_PITCH_KP, RATE_PITCH_KI, RATE_PITCH_KD,
              -1.0f, 1.0f, 0.5f);

    setup_pid(&fc->rate_yaw_cfg,   &fc->rate_yaw_state,
              RATE_YAW_KP, RATE_YAW_KI, RATE_YAW_KD,
              -1.0f, 1.0f, 0.3f);

    /* Attitude controllers (outer loop) */
    setup_pid(&fc->att_roll_cfg,   &fc->att_roll_state,
              ATT_ROLL_KP, 0.0f, 0.0f,
              -3.14159f, 3.14159f, 1.0f);

    setup_pid(&fc->att_pitch_cfg,  &fc->att_pitch_state,
              ATT_PITCH_KP, 0.0f, 0.0f,
              -3.14159f, 3.14159f, 1.0f);

    setup_pid(&fc->att_yaw_cfg,    &fc->att_yaw_state,
              ATT_YAW_KP, 0.0f, 0.0f,
              -3.14159f, 3.14159f, 1.0f);

    /* Altitude controller */
    setup_pid(&fc->alt_cfg, &fc->alt_state,
              ALT_KP, ALT_KI, ALT_KD,
              -1.0f, 1.0f, 0.5f);

    /* Position controllers */
    setup_pid(&fc->pos_north_cfg, &fc->pos_north_state,
              POS_KP, POS_KI, POS_KD,
              -0.5f, 0.5f, 0.3f);

    setup_pid(&fc->pos_east_cfg, &fc->pos_east_state,
              POS_KP, POS_KI, POS_KD,
              -0.5f, 0.5f, 0.3f);

    /* Zero out motor outputs */
    for (int i = 0; i < NUM_MOTORS; i++) {
        fc->output.motor[i] = 0.0f;
        fc->output.tilt_servo[i] = 0.0f;
    }

    fc->last_rate_update_us = 0;
    fc->last_att_update_us = 0;
    fc->last_alt_update_us = 0;
    fc->last_pos_update_us = 0;
}

void fc_update_rate(struct FlightController *fc,
                    const struct VehicleState *state,
                    const struct ControlCommand *cmd,
                    float dt)
{
    if (!fc->armed || fc->mode == MODE_DISARMED) {
        for (int i = 0; i < NUM_MOTORS; i++) {
            fc->output.motor[i] = 0.0f;
        }
        return;
    }

    /* Compute rate errors and PID outputs */
    float roll_error = cmd->roll_cmd - state->roll_rate;
    float pitch_error = cmd->pitch_cmd - state->pitch_rate;
    float yaw_error = cmd->yaw_cmd - state->yaw_rate;

    float roll_out = pid_update(&fc->rate_roll_cfg,
                                 &fc->rate_roll_state, roll_error, dt);
    float pitch_out = pid_update(&fc->rate_pitch_cfg,
                                  &fc->rate_pitch_state, pitch_error, dt);
    float yaw_out = pid_update(&fc->rate_yaw_cfg,
                                &fc->rate_yaw_state, yaw_error, dt);

    /* Motor mixing for X-frame quad configuration.
     * Motor layout (top view, forward is up):
     *   M1(FL)  M2(FR)
     *     \      /
     *      [CG]
     *     /      \
     *   M3(RL)  M4(RR)
     *
     * Roll: M1,M3 up / M2,M4 down (or vice versa)
     * Pitch: M1,M2 up / M3,M4 down
     * Yaw: M1,M4 CW / M2,M3 CCW (through differential torque) */

    float throttle = cmd->throttle_cmd;
    throttle = constrain_float(throttle, 0.0f, 1.0f);

    fc->output.motor[0] = throttle + roll_out + pitch_out - yaw_out;  /* FL */
    fc->output.motor[1] = throttle - roll_out + pitch_out + yaw_out;  /* FR */
    fc->output.motor[2] = throttle + roll_out - pitch_out + yaw_out;  /* RL */
    fc->output.motor[3] = throttle - roll_out - pitch_out - yaw_out;  /* RR */

    /* Apply limits and idle speed */
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (fc->output.motor[i] < IDLE_THROTTLE) {
            fc->output.motor[i] = IDLE_THROTTLE;
        }
        fc->output.motor[i] = constrain_float(fc->output.motor[i], 0.0f, 1.0f);
    }
}

void fc_update_attitude(struct FlightController *fc,
                        const struct VehicleState *state,
                        const struct ControlCommand *cmd,
                        float dt)
{
    if (fc->mode == MODE_DISARMED) return;

    /* Compute attitude errors */
    float roll_error = cmd->roll_cmd - state->roll;
    float pitch_error = cmd->pitch_cmd - state->pitch;
    float yaw_error = wrap_pi(cmd->yaw_cmd - state->yaw);

    /* The attitude controller outputs desired angular rates,
     * which become the setpoints for the rate controller. */
    pid_update(&fc->att_roll_cfg, &fc->att_roll_state, roll_error, dt);
    pid_update(&fc->att_pitch_cfg, &fc->att_pitch_state, pitch_error, dt);
    pid_update(&fc->att_yaw_cfg, &fc->att_yaw_state, yaw_error, dt);
}

void fc_update_altitude(struct FlightController *fc,
                        const struct VehicleState *state,
                        float dt)
{
    if (fc->mode != MODE_ALT_HOLD && fc->mode != MODE_LOITER &&
        fc->mode != MODE_AUTO && fc->mode != MODE_RTL) {
        return;
    }

    float alt_error = fc->target_altitude - (-state->pos_down);
    pid_update(&fc->alt_cfg, &fc->alt_state, alt_error, dt);
}

void fc_update_position(struct FlightController *fc,
                        const struct VehicleState *state,
                        float dt)
{
    if (fc->mode != MODE_LOITER && fc->mode != MODE_AUTO &&
        fc->mode != MODE_RTL) {
        return;
    }

    float north_error = fc->target_north - state->pos_north;
    float east_error = fc->target_east - state->pos_east;

    pid_update(&fc->pos_north_cfg, &fc->pos_north_state, north_error, dt);
    pid_update(&fc->pos_east_cfg, &fc->pos_east_state, east_error, dt);
}

void fc_set_mode(struct FlightController *fc, enum FlightMode mode)
{
    if (fc->mode == mode) return;

    fc->prev_mode = fc->mode;
    fc->mode = mode;

    /* Reset PID integrators on mode change to prevent transients */
    pid_reset(&fc->rate_roll_state);
    pid_reset(&fc->rate_pitch_state);
    pid_reset(&fc->rate_yaw_state);
    pid_reset(&fc->alt_state);

    if (mode == MODE_TRANSITION) {
        fc->transition_active = 1;
    }
}

void fc_set_armed(struct FlightController *fc, int armed)
{
    if (armed && !fc->armed) {
        /* Arming: reset all controller states */
        fc_init(fc);
        fc->armed = 1;
        fc->mode = MODE_STABILIZE;
    } else if (!armed && fc->armed) {
        /* Disarming: kill motors immediately */
        fc->armed = 0;
        fc->mode = MODE_DISARMED;
        for (int i = 0; i < NUM_MOTORS; i++) {
            fc->output.motor[i] = 0.0f;
        }
    }
}

const struct MotorOutput *fc_get_output(const struct FlightController *fc)
{
    return &fc->output;
}
