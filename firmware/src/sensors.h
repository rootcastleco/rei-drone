/*
 * REI Drone - Sensor Interface
 * Rootcastle Engineering & Innovation
 *
 * Provides a unified interface to the REI Drone sensor suite including
 * the inertial measurement unit (IMU), barometric pressure sensor,
 * magnetometer, and GPS receiver. Sensor data is packaged into a
 * common structure for consumption by the state estimator and
 * flight controller.
 *
 * Supported hardware:
 *   - IMU: ICM-42688-P (dual, SPI)
 *   - Barometer: MS5611 (I2C)
 *   - Magnetometer: IST8310 (I2C)
 *   - GPS: u-blox M9N (UART)
 *
 * Author: Batuhan Ayribas, M.Sc.
 */

#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>

struct IMUData {
    float accel_x;      /* Acceleration in body X axis (m/s^2) */
    float accel_y;      /* Acceleration in body Y axis (m/s^2) */
    float accel_z;      /* Acceleration in body Z axis (m/s^2) */
    float gyro_x;       /* Angular rate about body X axis (rad/s) */
    float gyro_y;       /* Angular rate about body Y axis (rad/s) */
    float gyro_z;       /* Angular rate about body Z axis (rad/s) */
    float temperature;  /* IMU die temperature (Celsius) */
    uint32_t timestamp; /* Microsecond timestamp */
};

struct BaroData {
    float pressure;     /* Atmospheric pressure (Pa) */
    float temperature;  /* Temperature (Celsius) */
    float altitude;     /* Pressure altitude (m) */
    uint32_t timestamp;
};

struct MagData {
    float mag_x;        /* Magnetic field in body X axis (Gauss) */
    float mag_y;        /* Magnetic field in body Y axis (Gauss) */
    float mag_z;        /* Magnetic field in body Z axis (Gauss) */
    uint32_t timestamp;
};

struct GPSData {
    double latitude;    /* Latitude (degrees) */
    double longitude;   /* Longitude (degrees) */
    float altitude_msl; /* Altitude above mean sea level (m) */
    float ground_speed; /* Ground speed (m/s) */
    float course;       /* Course over ground (degrees) */
    float hdop;         /* Horizontal dilution of precision */
    float vdop;         /* Vertical dilution of precision */
    int fix_type;       /* 0=no fix, 2=2D, 3=3D */
    int num_satellites; /* Number of visible satellites */
    uint32_t timestamp;
};

/* Initialize all sensor interfaces */
int sensors_init(void);

/* Read the latest IMU data. Returns 0 on success. */
int sensors_read_imu(struct IMUData *data);

/* Read the latest barometer data. Returns 0 on success. */
int sensors_read_baro(struct BaroData *data);

/* Read the latest magnetometer data. Returns 0 on success. */
int sensors_read_mag(struct MagData *data);

/* Read the latest GPS data. Returns 0 on success. */
int sensors_read_gps(struct GPSData *data);

/* Check whether new data is available from each sensor */
int sensors_imu_available(void);
int sensors_baro_available(void);
int sensors_mag_available(void);
int sensors_gps_available(void);

/* Run sensor self-test. Returns a bitmask of failed sensors. */
int sensors_self_test(void);

#endif /* SENSORS_H */
