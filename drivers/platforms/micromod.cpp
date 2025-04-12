  #include <libhal-micromod/micromod.hpp>
  #include <libhal/can.hpp>

  #include "../hardware_map.hpp"

  namespace sjsu::drivers {

  application_framework initialize_platform()
  {
    using namespace hal::literals;
    static auto& terminal = hal::micromod::v1::console(hal::buffer<1024>);
    static auto& counter = hal::micromod::v1::uptime_clock();


    hal::micromod::v1::initialize_platform();
    
    //interface to send a receive CAN messags 
    static hal::can_transceiver* local_can_transceiver;
    //manages settings 
    static hal::can_bus_manager* bus_man;
    //only messages with certain ids can go through 
    static hal::can_identifier_filter* idf0;
    static hal::can_identifier_filter* idf1;

    //array for storing CAN messages that are coming through 
    static std::array<hal::can_message, 8> receive_buffer{};

    //getting the stuff from micromod 
    local_can_transceiver = &hal::micromod::v1::can_transceiver(receive_buffer);
    bus_man = &hal::micromod::v1::can_bus_manager();
    idf0 = &hal::micromod::v1::can_identifier_filter0();
    idf1 = &hal::micromod::v1::can_identifier_filter1();

    //make a filter to only allow messages with 0x105 
    idf0->allow(0x105);
    idf1 -> allow(0x106);
    //set baud rate of CAN bus 
    bus_man->baud_rate(1.0_MHz);

    return {
      .clock = &counter,
      .terminal = &terminal,
      //.can = &hal::micromod::v1::can(),
      .in_pin0 = &hal::micromod::v1::input_g0(),
      .in_pin1 = &hal::micromod::v1::input_g1(),
      .in_pin2 = &hal::micromod::v1::input_g2(),
      .led = &hal::micromod::v1::led(), 
      .out_pin0 = &hal::micromod::v1::output_g0(),
      .out_pin1 = &hal::micromod::v1::output_g1(),
      .out_pin2 = &hal::micromod::v1::output_g2(),
      .out_pin3 = &hal::micromod::v1::output_g3(),
      .out_pin4 = &hal::micromod::v1::output_g4(),
      //not defined or something 
      // .pwm0 = &hal::micromod::v1::pwm0(),
      // .pwm1 = &hal::micromod::v1::pwm1(),
      // //.pwm2 =  &hal::micromod::v1::pwm2(),
      // .adc0 = &hal::micromod::v1::a0(),
      // .adc1 = &hal::micromod::v1::a1(),
      // .esp,
      .can_transceiver = local_can_transceiver,
      .can_bus_manager = bus_man,
      .i2c = &hal::micromod::v1::i2c(),
      .steady_clock = &hal::micromod::v1::uptime_clock(),
      .reset = +[]() { hal::micromod::v1::reset(); },
    };
  }
  }