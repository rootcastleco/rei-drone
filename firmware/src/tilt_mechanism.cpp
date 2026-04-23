/*
 * REI Drone - Tilt Mechanism Control Implementation
 * Rootcastle Engineering & Innovation
 *
 * Author: Batuhan Ayribas, M.Sc.
 */

#include "tilt_mechanism.h"

/* S-curve function for smooth transitions: 3*t^2 - 2*t^3 */
static float s_curve(float t)
{
    if (t <= 0.0f) return 0.0f;
    if (t >= 1.0f) return 1.0f;
    return 3.0f * t * t - 2.0f * t * t * t;
}

static float constrain(float val, float lo, float hi)
{
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

void tilt_init(struct TiltController *tilt)
{
    tilt->state = TILT_HOVER;
    tilt->current_angle = TILT_ANGLE_HOVER_DEG;
    tilt->target_angle = TILT_ANGLE_HOVER_DEG;
    tilt->transition_progress = 0.0f;
    tilt->min_transition_speed = 8.0f;
    tilt->full_transition_speed = TRANSITION_AIRSPEED_MS;
}

void tilt_update(struct TiltController *tilt, float airspeed_ms,
                 float manual_tilt, float dt)
{
    /* If a manual tilt override is provided, use it directly */
    if (manual_tilt >= 0.0f) {
        tilt->target_angle = constrain(manual_tilt,
                                        TILT_ANGLE_HOVER_DEG,
                                        TILT_ANGLE_CRUISE_DEG);
    } else {
        /* Automatic transition based on airspeed */
        switch (tilt->state) {
        case TILT_HOVER:
            tilt->target_angle = TILT_ANGLE_HOVER_DEG;
            if (airspeed_ms > tilt->min_transition_speed) {
                tilt->state = TILT_TRANSITION_FWD;
                tilt->transition_progress = 0.0f;
            }
            break;

        case TILT_TRANSITION_FWD:
            /* Progress based on airspeed buildup */
            if (airspeed_ms >= tilt->full_transition_speed) {
                tilt->transition_progress = 1.0f;
                tilt->state = TILT_CRUISE;
            } else if (airspeed_ms < tilt->min_transition_speed * 0.8f) {
                /* Airspeed dropped too low, revert to hover */
                tilt->state = TILT_TRANSITION_REV;
            } else {
                float speed_range = tilt->full_transition_speed -
                                    tilt->min_transition_speed;
                float speed_frac = (airspeed_ms - tilt->min_transition_speed) /
                                    speed_range;
                speed_frac = constrain(speed_frac, 0.0f, 1.0f);
                tilt->transition_progress = speed_frac;
            }
            tilt->target_angle = TILT_ANGLE_HOVER_DEG +
                s_curve(tilt->transition_progress) *
                (TILT_ANGLE_CRUISE_DEG - TILT_ANGLE_HOVER_DEG);
            break;

        case TILT_CRUISE:
            tilt->target_angle = TILT_ANGLE_CRUISE_DEG;
            if (airspeed_ms < tilt->min_transition_speed) {
                tilt->state = TILT_TRANSITION_REV;
                tilt->transition_progress = 1.0f;
            }
            break;

        case TILT_TRANSITION_REV:
            if (airspeed_ms <= 2.0f) {
                tilt->transition_progress = 0.0f;
                tilt->state = TILT_HOVER;
            } else {
                float speed_range = tilt->full_transition_speed -
                                    tilt->min_transition_speed;
                float speed_frac = (airspeed_ms - 2.0f) /
                                    (tilt->min_transition_speed - 2.0f);
                speed_frac = constrain(speed_frac, 0.0f, 1.0f);
                tilt->transition_progress = speed_frac;
            }
            tilt->target_angle = TILT_ANGLE_HOVER_DEG +
                s_curve(tilt->transition_progress) *
                (TILT_ANGLE_CRUISE_DEG - TILT_ANGLE_HOVER_DEG);
            break;
        }
    }

    /* Slew rate limiting: do not change angle faster than the maximum rate */
    float max_delta = TILT_RATE_DEG_PER_SEC * dt;
    float delta = tilt->target_angle - tilt->current_angle;

    if (delta > max_delta) {
        tilt->current_angle += max_delta;
    } else if (delta < -max_delta) {
        tilt->current_angle -= max_delta;
    } else {
        tilt->current_angle = tilt->target_angle;
    }

    tilt->current_angle = constrain(tilt->current_angle,
                                     TILT_ANGLE_HOVER_DEG,
                                     TILT_ANGLE_CRUISE_DEG);
}

float tilt_get_angle(const struct TiltController *tilt, int motor_index)
{
    /* All motors share the same tilt angle in this configuration.
     * A more advanced design could tilt front and rear motors
     * independently for pitch control during transition. */
    (void)motor_index;
    return tilt->current_angle;
}

int tilt_angle_to_pwm(float angle_deg)
{
    /* Linear mapping from angle to servo PWM.
     * 0 degrees (hover) maps to TILT_SERVO_MIN_US.
     * 90 degrees (cruise) maps to TILT_SERVO_MAX_US. */
    float range_deg = TILT_ANGLE_CRUISE_DEG - TILT_ANGLE_HOVER_DEG;
    float range_us = (float)(TILT_SERVO_MAX_US - TILT_SERVO_MIN_US);
    float normalized = (angle_deg - TILT_ANGLE_HOVER_DEG) / range_deg;

    normalized = constrain(normalized, 0.0f, 1.0f);

    int pwm = TILT_SERVO_MIN_US + (int)(normalized * range_us);
    return pwm;
}

void tilt_emergency_hover(struct TiltController *tilt)
{
    /* In an emergency, command immediate return to hover orientation.
     * The slew rate limiter in tilt_update will still prevent
     * instantaneous changes, but the target is set to hover. */
    tilt->state = TILT_TRANSITION_REV;
    tilt->target_angle = TILT_ANGLE_HOVER_DEG;
    tilt->transition_progress = 0.0f;
}

int tilt_can_transition(const struct TiltController *tilt, float airspeed_ms,
                        float altitude_m)
{
    /* Transition is permitted only when:
     * 1. The vehicle is above a minimum safe altitude (10 m)
     * 2. Airspeed is within the expected range
     * 3. The tilt system is not already in the target state */

    if (altitude_m < 10.0f) return 0;
    if (airspeed_ms < 0.0f || airspeed_ms > 35.0f) return 0;
    if (tilt->state == TILT_CRUISE && airspeed_ms > tilt->min_transition_speed) {
        return 0;  /* Already in cruise, no need to transition forward */
    }
    return 1;
}
