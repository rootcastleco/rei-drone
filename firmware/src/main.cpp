/*
 * REI Drone - Main Application Entry
 * Rootcastle Engineering & Innovation
 *
 * This is the main firmware loop for the REI Drone flight stack.
 * It initializes hardware subsystems (sensors, control surfaces, ESCs),
 * the state estimator, and the flight controller.
 * 
 * In a real RTOS environment (e.g., ChibiOS on ArduPilot/PX4), this
 * would be a high-priority thread running at the defined loop rate.
 *
 * Author: Batuhan Ayribas, M.Sc.
 */

#include <stdio.h>
#include <stdint.h>
#include "config/vehicle_params.h"
#include "sensors.h"
#include "estimator.h"
#include "flight_controller.h"
#include "tilt_mechanism.h"

/* Helper to simulate a delay (in microseconds) */
void delay_us(uint32_t us) {
    /* For native simulation, we just return.
     * On hardware, this would map to a hardware timer delay.
     */
}

int main(void) {
    printf("==========================================\n");
    printf("      REI Drone Flight Controller         \n");
    printf("   Rootcastle Engineering & Innovation    \n");
    printf("==========================================\n\n");

    printf("[INIT] Starting subsystems...\n");

    /* 1. Initialize Sensors */
    if (sensors_init() == 0) {
        printf("[INIT] Sensors initialized successfully.\n");
    } else {
        printf("[ERROR] Sensor initialization failed.\n");
        return -1;
    }

    /* 2. Initialize State Estimator */
    estimator_init();
    printf("[INIT] State estimator initialized.\n");

    /* 3. Initialize Flight Controller */
    fc_init();
    printf("[INIT] Flight controller initialized.\n");

    /* 4. Initialize Tilt Mechanism */
    tilt_init();
    printf("[INIT] Tilt mechanism initialized.\n");

    /* Main Execution Loop */
    printf("\n[SYSTEM] Entering main control loop at %d Hz...\n\n", LOOP_RATE_HZ);

    const float dt = 1.0f / LOOP_RATE_HZ; /* Loop time step (seconds) */
    
    /* Simulated data structures */
    struct IMUData imu;
    struct BaroData baro;
    struct MagData mag;
    struct GPSData gps;
    struct EstimatorState current_state;

    /* Loop variables for the simulation stub */
    uint32_t loop_count = 0;
    const uint32_t MAX_SIM_LOOPS = 1000; /* Run for a finite number of loops in sim */

    while (loop_count < MAX_SIM_LOOPS) {
        
        /* 1. Read Sensors */
        sensors_read_imu(&imu);
        sensors_read_baro(&baro);
        sensors_read_mag(&mag);
        sensors_read_gps(&gps);

        /* 2. Update State Estimator */
        estimator_update(&imu, &baro, &mag, &gps, dt);
        estimator_get_state(&current_state);

        /* 3. Mode Transitions & Mission Logic (Simplified) */
        if (loop_count == 100) {
            printf("[LOG] Arming drone...\n");
            fc_set_armed(true);
        }
        
        if (loop_count == 200) {
            printf("[LOG] Initiating takeoff to 10m...\n");
            fc_set_setpoint(0.0f, 0.0f, 0.0f, 10.0f); 
        }

        if (loop_count == 500) {
            printf("[LOG] Initiating transition to cruise mode...\n");
            tilt_set_state(TILT_STATE_CRUISE);
        }

        /* 4. Update Tilt Mechanism */
        tilt_update(dt);
        
        /* 5. Update Flight Controller (cascaded PID loops) */
        fc_update(current_state.roll, current_state.pitch, current_state.yaw, 
                  current_state.roll_rate, current_state.pitch_rate, current_state.yaw_rate, 
                  current_state.altitude, dt);

        /* Print periodic debug output */
        if (loop_count % 100 == 0) {
            printf("Loop %4u | R: %5.2f P: %5.2f Y: %5.2f | Alt: %5.2f | Tilt: %.2f%%\n",
                   loop_count, 
                   current_state.roll, current_state.pitch, current_state.yaw,
                   current_state.altitude, 
                   tilt_get_progress() * 100.0f);
        }

        /* 6. Wait for next cycle to maintain LOOP_RATE_HZ */
        delay_us((uint32_t)(dt * 1000000.0f));
        loop_count++;
    }

    printf("\n[SYSTEM] Simulation complete. Shutting down.\n");
    return 0;
}
