/*
 * REI Drone - PID Controller Implementation
 * Rootcastle Engineering & Innovation
 *
 * Author: Batuhan Ayribas, M.Sc.
 */

#include "pid.h"

void pid_init(struct PIDState *state)
{
    state->integral = 0.0f;
    state->prev_error = 0.0f;
    state->prev_derivative = 0.0f;
    state->output = 0.0f;
}

void pid_reset(struct PIDState *state)
{
    state->integral = 0.0f;
    state->prev_error = 0.0f;
    state->prev_derivative = 0.0f;
    state->output = 0.0f;
}

float pid_update(const struct PIDConfig *config, struct PIDState *state,
                 float error, float dt)
{
    if (dt <= 0.0f) {
        return state->output;
    }

    /* Proportional term */
    float p_term = config->kp * error;

    /* Integral term with anti-windup */
    state->integral += error * dt;
    if (state->integral > config->integral_max) {
        state->integral = config->integral_max;
    } else if (state->integral < -config->integral_max) {
        state->integral = -config->integral_max;
    }
    float i_term = config->ki * state->integral;

    /* Derivative term with low-pass filtering */
    float raw_derivative = (error - state->prev_error) / dt;
    float alpha = config->d_filter_alpha;
    float filtered_derivative = alpha * raw_derivative +
                                (1.0f - alpha) * state->prev_derivative;
    float d_term = config->kd * filtered_derivative;

    state->prev_error = error;
    state->prev_derivative = filtered_derivative;

    /* Sum and clamp output */
    float output = p_term + i_term + d_term;
    if (output > config->output_max) {
        output = config->output_max;
        /* Back-calculate integral to prevent windup */
        state->integral -= error * dt;
    } else if (output < config->output_min) {
        output = config->output_min;
        state->integral -= error * dt;
    }

    state->output = output;
    return output;
}

float pid_update_with_measurement(const struct PIDConfig *config,
                                   struct PIDState *state,
                                   float error, float measurement, float dt)
{
    if (dt <= 0.0f) {
        return state->output;
    }

    float p_term = config->kp * error;

    state->integral += error * dt;
    if (state->integral > config->integral_max) {
        state->integral = config->integral_max;
    } else if (state->integral < -config->integral_max) {
        state->integral = -config->integral_max;
    }
    float i_term = config->ki * state->integral;

    /* Use measurement for derivative to avoid setpoint kick */
    float raw_derivative = -(measurement - state->prev_error) / dt;
    float alpha = config->d_filter_alpha;
    float filtered_derivative = alpha * raw_derivative +
                                (1.0f - alpha) * state->prev_derivative;
    float d_term = config->kd * filtered_derivative;

    state->prev_error = measurement;
    state->prev_derivative = filtered_derivative;

    float output = p_term + i_term + d_term;
    if (output > config->output_max) {
        output = config->output_max;
        state->integral -= error * dt;
    } else if (output < config->output_min) {
        output = config->output_min;
        state->integral -= error * dt;
    }

    state->output = output;
    return output;
}
