#include "../include/gimble.hpp"
#include <cmath>
#include <cstdint>
#include <libhal-util/i2c.hpp>
#include <libhal/timeout.hpp>
#include <libhal/units.hpp>

using namespace std::chrono_literals;
using namespace hal::literals;

#define PI 3.14159265358979323846

// Complementary Coefficients
float tau = 0.92;  // Tunning var
float dt = 0.1;    // Delay (change in time)
float alpha = tau / (tau + dt);

namespace sjsu::drivers {
/*
Gimble::Gimble(hal::i2c& p_i2c,
             hal::actuator::rc_servo ps1,
             hal::actuator::rc_servo ps2,
             hal::actuator::rc_servo ts)
: m_i2c(p_i2c)
, servo_pan1(ps1)
, servo_pan2(ps2)
, servo_tilt(ts)
{
// Initialize MPU
hal::byte accel_config_value = 0x0;
hal::byte gyro_config_value = 0x0;
hal::byte enable_value = 0x0;

std::array<hal::byte, 2> enable_setting = { mpu_registers::enable_reg,
                                            enable_value };
std::array<hal::byte, 2> accel_config_setting = { mpu_registers::accel_config,
                                                  accel_config_value };
std::array<hal::byte, 2> gyro_config_setting = { mpu_registers::gyro_config,
                                                 gyro_config_value };

hal::write(m_i2c, mpu6050_address, enable_setting, hal::never_timeout());
hal::write(
  m_i2c, mpu6050_address, accel_config_setting, hal::never_timeout());
hal::write(m_i2c, mpu6050_address, gyro_config_setting, hal::never_timeout());

// Initialize SERVO
servo_pan1.position(hal::degrees(-90));
servo_pan2.position(hal::degrees(-90));
servo_tilt.position(hal::degrees(0));
}

Gimble::MPU6050_data Gimble::read_accel()
{
Gimble::MPU6050_data data;

std::array<hal::byte, 1> address_to_write = { mpu_registers::accel_xyz };
std::array<hal::byte, 6> buffer;

hal::write_then_read(m_i2c, mpu6050_address, address_to_write, buffer);

data.x = static_cast<std::int16_t>(buffer[0] << 8 | buffer[1]) / ACCEL_SCALE;
data.y = static_cast<std::int16_t>(buffer[2] << 8 | buffer[3]) / ACCEL_SCALE;
data.z = static_cast<std::int16_t>(buffer[4] << 8 | buffer[5]) / ACCEL_SCALE;
return data;
}

Gimble::MPU6050_data Gimble::read_gyro()
{
Gimble::MPU6050_data data;
std::array<hal::byte, 1> address_to_write = { mpu_registers::gyro_xyz };
std::array<hal::byte, 6> buffer;

hal::write_then_read(m_i2c, mpu6050_address, address_to_write, buffer);

data.x = static_cast<std::int16_t>(buffer[0] << 8 | buffer[1]) / GYRO_SCALE;
data.y = static_cast<std::int16_t>(buffer[2] << 8 | buffer[3]) / GYRO_SCALE;
data.z = static_cast<std::int16_t>(buffer[4] << 8 | buffer[5]) / GYRO_SCALE;
return data;
}
*/

Gimble::Gimble(hal::i2c& p_i2c,
               hal::actuator::rc_servo& servo_pan1,
               hal::actuator::rc_servo& servo_pan2,
               hal::actuator::rc_servo& servo_tilt)
  : m_i2c(p_i2c)
{
  // Initialize ICM
  icm = hal::sensor::icm20948(m_i2c);
  current_pitch = 0;
  current_roll = 0;
  current_yaw = 0;

  // Initialize SERVO
  servo_pan1.position(hal::degrees(-90));
  servo_pan2.position(hal::degrees(-90));
  servo_tilt.position(hal::degrees(0));
}

void Gimble::update_orientation()
{
  auto accel_data = icm.read_acceleration();
  auto gyro_data = icm.read_gyroscope();
  auto mag_data = icm.read_magnetometer();

  hal::degrees calculated_roll =
    std::atan((accel_data.y) / std::sqrt(accel_data.x * accel_data.x +
                                         accel_data.z * accel_data.z)) *
    180 / PI;

  hal::degrees calculated_pitch =
    (-1) *
    std::atan(accel_data.x / std::sqrt(accel_data.y * accel_data.y +
                                       accel_data.z * accel_data.z)) *
    180 / PI;

  hal::degrees calculated_yaw = atan2(mag_data.x, mag_data.y);

  // Complementary Filter for pitch and roll
  current_roll =
    alpha * (current_roll + gyro_data.x * dt) + (1 - alpha) * calculated_roll;
  current_pitch =
    alpha * (current_pitch + gyro_data.y * dt) + (1 - alpha) * calculated_pitch;
  current_yaw =
    alpha * (current_yaw + gyro_data.z * dt) + (1 - alpha) * calculated_yaw;
}

void Gimble::read_can_input(hal::can& p_can)
{
}

void Gimble::updateHorizontalServos()
{
  hal::degrees servo1_position;
  hal::degrees servo2_position;

  if (current_pan > -90 && current_pan < 90) {
    servo_pan1.position(hal::degrees(current_pan));
    servo_pan2.position(hal::degrees(-90));
  } else if (current_pan > 90) {
    servo_pan1.position(hal::degrees(90));
    servo_pan2.position(hal::degrees(-90) +
                        hal::degrees(current_pan - hal::degrees(90)));
  } else {  // current pan < -90
    servo_pan1.position(hal::degrees(-90));
    servo_pan2.position(hal::degrees(90) +
                        hal::degrees(current_pan + hal::degree(90)));
  }
}
void Gimble::right(hal::degrees init_degree)
{
  current_pan = current_pan + init_degree;
  updateHorizontalServos();
}

void Gimble::left(hal::degrees init_degree)
{
  current_pan = current_pan - init_degree;
  updateHorizontalServos();
}

void Gimble::up(hal::degrees init_degree)
{
  current_tilt = current_tilt + init_degree;
  if (current_tilt > 90.0)
    current_tilt = 90.0;
  servo_tilt.position(current_tilt);
}

void Gimble::down(hal::degrees init_degree)
{
  current_tilt = current_tilt - init_degree;
  if (current_tilt < -90.0)
    current_tilt = -90.0;
  servo_tilt.position(current_tilt);
}

void Gimble::stable()
{
  // set the target, maybe default is when pitch and yaw is 0
  // measure current angle, where current pitch and yaw is
  // calculate error
  // proportional: current error
  // integral: past errors
  // deriative: future errors
  // compute the correction: calculates correct output using the three parts P,
  // I, and D add the corrections to our current servo positions move the servo
}

}  // namespace sjsu::drivers