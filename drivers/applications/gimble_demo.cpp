#include <libhal-micromod/micromod.hpp>
#include <libhal-util/bit_bang_i2c.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/output_pin.hpp>
#include <libhal/units.hpp>
#include <libhal-util/can.hpp>

#include "../hardware_map.hpp"
#include "../include/gimble.hpp"

using namespace hal::literals;
using namespace std::chrono_literals;

namespace sjsu::drivers {
void application(application_framework& p_framework)
{
  auto& clock = *p_framework.steady_clock;
  auto& console = *p_framework.terminal;
  auto& can_transceiver = *p_framework.can_transceiver.value();
  // auto& can_bus_manager = *hardware_map.can_bus_manager.value();
  // auto& can_identifier_filter = *hardware_map.can_identifier_filter.value();

  //will find messages of 0x105 
  hal::can_message_finder homing_reader(can_transceiver, 0x105);

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

  while (true) {
    try {
      std::optional<hal::can_message> msg = homing_reader.find();
  
      if (msg) {
        hal::print(console, "found message\n");
        hal::print(console, "Done homing\n");
      }
      hal::print<128>(console,
                        "Circular Buffer Size: %d\n",
                        can_transceiver.receive_cursor());
    } catch (hal::timed_out const&) {
      hal::print(console,
        "hal::timed_out exception! which means that the device did not respond. "
        "Moving to the next device address in the list.\n");
    } catch (hal::resource_unavailable_try_again const& p_error) {
      hal::print(console, "hal::resource_unavailable_try_again\n");
      if (p_error.instance() == &can_transceiver) {
        hal::print(console,
          "\n"
          "device on the bus. It appears as if the peripheral is not connected "
          "to a can network. This can happen if the baud rate is incorrect, "
          "the CAN transceiver is not functioning, or the devices on the bus "
          "are not responding.\n"
          "Calling terminate!\n"
          "Consider powering down the system and checking all of your connections before "
          "restarting the application.");
        std::terminate();
      }
    } catch (...) {
      hal::print(console, "Unknown exception caught in (...) block\n");
      throw;
    }
  

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
}