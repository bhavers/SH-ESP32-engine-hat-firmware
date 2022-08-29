/*
Moving average example from @butch. He sent me this on the 6th of June 2022.

*/

/*

#include <Arduino.h>
#define USE_LIB_WEBSOCKET true
#include "sensesp_app.h"
#include "sensesp_app_builder.h"
#include "sensors/digital_input.h"
#include "transforms/linear.h
#include "transforms/frequency.h"
#include "transforms/moving_average.h"
#include "signalk/signalk_output.h"

#define TACH_PIN 14

ReactESP app([] () {
  SetupSerialDebug(115200);
  SensESPAppBuilder builder;
  sensesp_app = builder.set_standard_sensors(UPTIME)
              ->set_hostname("ME_Tach")
              ->set_wifi("turtlecube", "******")
              ->set_sk_server("192.168.1.170", 3000)
              ->get_app();

  const char* config_path_multiplier = "/Tach/frequency";
  const char* config_path_moving_avg = "/Tach/moving avg samples";

// A DigitalInputCounter implements an interrupt to count pulses from a Hall Effect Sensor and reports the readings every read_delay ms
// (1000 in the example). A Frequency transform converts the number of pulses into a frequency. The multiplier is 0.25 because I'm using 
// four magnets on the perimeter of the main engine's harmonic balancer, rather than just one.
  const float freq_multiplier = 0.25;
  const uint read_delay = 1000; // used for all sensors in this program

  auto* pDigitalIn = new DigitalInputCounter(TACH_PIN, INPUT_PULLUP, RISING, read_delay, "/Tach/read_delay");

  pDigitalIn->connect_to(new Frequency(freq_multiplier, config_path_multiplier))
        ->connect_to(new MovingAverage(15, 1.0, config_path_moving_avg))  // Averaging the last 15 samples gives a smoother output
        ->connect_to(new SKOutputNumber("propulsion.main.revolutions"));

  // Create a second instance (of a consumer) to display the immediate output of the tachometer
  pDigitalIn->connect_to(new Frequency(freq_multiplier)) // connect the output of pSensor to the input of Frequency()
        ->connect_to(new MovingAverage(2, 1.0)) // Averaging over two samples gives an almost-immediate, although less-smooth, output
        ->connect_to(new SKOutputNumber("propulsion.immed.revolutions"));
 
  sensesp_app->enable();
});

*/