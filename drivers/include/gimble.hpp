#pragma once

// Contains MPU 6050 (will not use MPU from libhal, does not come with gryoscope reading)
// Contains CAN
// Contains Servo (two axis gimble)

#include <libhal/can.hpp>
#include <libhal/i2c.hpp>
#include <libhal/pwm.hpp>
#include <libhal-actuator/rc_servo.hpp>
#include <libhal/pwm.hpp> 

namespace sjsu::drivers {
    class Gimble {
        public:

            struct MPU6050_data {
                float x;
                float y;
                float z;
            };

            enum mpu_registers {
                accel_xyz = 0x3b, // Starting register to read the HI and LOW byte for the accelerometer x, y, z data
                accel_config = 0x1C, // 0b0 for no self-test and scale to 2g
                gyro_xyz = 0x43, // Starting register to read the HI and LOW byte for the gyroscope x, y, z data
                gyro_config = 0x1B, // 0b0 for no self-test and scale to 250 degrees/s
                enable_reg = 0x6B, // 0b0 starting the mpu
            };

            Gimble(hal::i2c &p_i2c, 
                hal::actuator::rc_servo &servo_pan1, 
                hal::actuator::rc_servo &servo_pan2, 
                hal::actuator::rc_servo &servo_tilt);

            void read_can_input(hal::can &p_can); // Takes CAN input and reads joystick inputs

            void left(hal::degrees init_degree);
            void right(hal::degrees init_degree);
            void up(hal::degrees init_degree);
            void down(hal::degrees init_degree);
            
            void stable(); // Takes current roll/pitch/yaw and use PID Control to continuously set the camera at that setting 

        private:
            // Functions to support public functions 
            MPU6050_data read_accel();
            MPU6050_data read_gyro();
            hal::degrees roll();
            hal::degrees pitch();
            hal::degrees yaw(); 

            hal::i2c& m_i2c; // For MPU

            // For PID Control 
            bool stable_mode = false; 
            hal::degrees current_roll;
            hal::degrees current_pitch;
            hal::degrees current_yaw;

            float Kp_theta; // Proportional Gain
            float Ki_theta; // Integral Gain
            float Kd_theta; // Derivative Gain

            hal::byte const mpu6050_address = 0x68;
            
            //home position
            hal::degrees current_pan = hal::degrees(-90);
            hal::degrees current_tilt = hal::degrees(0);

            hal::actuator::rc_servo &servo_pan1;
            hal::actuator::rc_servo &servo_pan2;
            hal::actuator::rc_servo &servo_tilt;

            void updateHorizontalServos(); 
    };
}