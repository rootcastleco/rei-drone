/*
 * REI Drone - PID Controller
 * Rootcastle Engineering & Innovation
 *
 * Generic PID controller implementation with anti-windup, derivative
 * filtering, and output limiting. Used across all control axes in
 * the REI Drone flight controller.
 *
 * Author: Batuhan Ayribas, M.Sc.
 */

#ifndef PID_H
#define PID_H

struct PIDConfig {
    float kp;
    float ki;
    float kd;
    float output_min;
    float output_max;
    float integral_max;
    float d_filter_alpha;   /* Low-pass filter for derivative (0-1, lower = more filtering) */
};

struct PIDState {
    float integral;
    float prev_error;
    float prev_derivative;
    float output;
};

/* Initialize a PID controller state */
void pid_init(struct PIDState *state);

/* Reset the PID controller state (clear integral and derivative history) */
void pid_reset(struct PIDState *state);

/* Compute the PID output for a given error and time step.
 * Returns the control output, clamped to the configured limits. */
float pid_update(const struct PIDConfig *config, struct PIDState *state,
                 float error, float dt);

/* Compute the PID output with a separate measurement for the derivative
 * term. This avoids derivative kick when the setpoint changes. */
float pid_update_with_measurement(const struct PIDConfig *config,
                                   struct PIDState *state,
                                   float error, float measurement, float dt);

#endif /* PID_H */
