#pragma once

// Contains MPU 6050 (will not use MPU from libhal, does not come with gryoscope
// reading) Contains CAN Contains Servo (two axis gimble)

#include <libhal-actuator/rc_servo.hpp>
#include <libhal-sensor/imu/icm20948.hpp>
#include <libhal/can.hpp>
#include <libhal/i2c.hpp>
#include <libhal/pwm.hpp>

// https://github.com/libhal/libhal-sensor/blob/main/include/libhal-sensor/imu/icm20948.hpp

namespace sjsu::drivers {
class Gimble
{
public:
  Gimble(hal::i2c& p_i2c,
         hal::actuator::rc_servo& servo_pan1,
         hal::actuator::rc_servo& servo_pan2,
         hal::actuator::rc_servo& servo_tilt);

  void read_can_input(
    hal::can& p_can);  // Takes CAN input and reads joystick inputs

  void left(hal::degrees init_degree);
  void right(hal::degrees init_degree);
  void up(hal::degrees init_degree);
  void down(hal::degrees init_degree);

  void stable();  // Takes current roll/pitch/yaw and use PID Control to
                  // continuously set the camera at that setting

private:
  // Will update the current yaw, pitch, roll 
  void update_orientation();

  // Current measurements
  hal::degrees current_roll;
  hal::degrees current_pitch;
  hal::degrees current_yaw;

  // For PID Control
  bool stable_mode = false;
  hal::degrees stable_roll;
  hal::degrees stable_pitch;
  hal::degrees stable_yaw;

  float Kp_theta;  // Proportional Gain
  float Ki_theta;  // Integral Gain
  float Kd_theta;  // Derivative Gain

  // home position
  hal::degrees current_pan = hal::degrees(-90);
  hal::degrees current_tilt = hal::degrees(0);

  // Object declaration 
  hal::actuator::rc_servo& servo_pan1;
  hal::actuator::rc_servo& servo_pan2;
  hal::actuator::rc_servo& servo_tilt;
  hal::i2c& m_i2c;  // For MPU
  hal::sensor::icm20948 icm;

  void updateHorizontalServos();
};
}  // namespace sjsu::drivers