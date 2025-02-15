#include <cmath>
#include <cstdint>
#include <gimble.hpp>
#include <libhal-util/i2c.hpp>
#include <libhal/timeout.hpp>
#include <libhal/units.hpp>

using namespace std::chrono_literals;
using namespace hal::literals;

// The scales for both measurements are default
#define ACCEL_SCALE = 16384.0
#define GYRO_SCALE = 131.0

#define PI 3.14159265358979323846

namespace sjsu::drivers {
Gimble::Gimble(hal::i2c& p_i2c)
  : m_i2c(p_i2c)
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

// http://www.starlino.com/dcm_tutorial.html <-- resource
// https://www.youtube.com/watch?v=7VW_XVbtu9k
hal::degrees Gimble::roll()
{
  hal::degrees calculated_roll = 0.0;
  auto accel_data = read_accel();
  calculated_roll = std::atan();

  return calculated_roll;
}

hal::degrees Gimble::pitch()
{
  hal::degrees calculated_pitch = 0.0;

  // TODO

  return calculated_pitch;
}

hal::degrees Gimble::yaw()
{
  hal::degrees calculated_yaw = 0.0;

  // TODO

  return calculated_yaw;
}

void Gimble::read_can_input(hal::can& p_can)
{
}

void Gimble::left(hal::degrees init_degree)
{
}

void Gimble::right(hal::degrees init_degree)
{
}

void Gimble::up(hal::degrees init_degree)
{
}

void Gimble::down(hal::degrees init_degree)
{
}

}  // namespace sjsu::drivers