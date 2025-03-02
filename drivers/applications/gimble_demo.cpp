#include <libhal-micromod/micromod.hpp>
#include <libhal-util/bit_bang_i2c.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/output_pin.hpp>
#include <libhal/units.hpp>

#include "../hardware_map.hpp"
#include "../include/gimble.hpp"

using namespace hal::literals;
using namespace std::chrono_literals;

namespace sjsu::drivers {
void application(application_framework& p_framework)
{
  auto& clock = *p_framework.steady_clock;
  auto& console = *p_framework.terminal;
  // The STM doesn't have i2c :(
  //   auto& i2c = *p_framework.i2c;
  hal::print(console, "Terminal Initialized");
  //   hal::print<40>(console, "Terminal Initialized");

  // Change it later, using bit bang temporary
  auto& sda = *p_framework.out_pin3;
  auto& scl = *p_framework.out_pin4;

  hal::bit_bang_i2c i2c({ .sda = &sda, .scl = &scl }, clock);

  Gimble g1(i2c);
  hal::print<40>(console, "Gimble Initialized\n");

  //   while (true) {
  //     auto accel_data = read_accel();
  //     auto gyro_data = read_gyro();

  //     hal::print<40>(console,
  //                    "ax = %f, ay = %f, az = %f\t",
  //                    accel_data.x,
  //                    accel_data.y,
  //                    accel_data.z);
  //     hal::print<40>(console,
  //                    "gx = %f, gy = %f, gz = %f\n",
  //                    gyro_data.x,
  //                    gyro_data.y,
  //                    gyro_data.z);
  //     hal::delay(clock, 100ms);
  //   }
}
}  // namespace sjsu::drivers
