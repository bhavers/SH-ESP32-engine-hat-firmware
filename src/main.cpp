// Signal K application template file.
//
// This application demonstrates core SensESP concepts in a very
// concise manner. You can build and upload the application as is
// and observe the value changes on the serial port monitor.
//
// You can use this source file as a basis for your own projects.
// Remove the parts that are not relevant to you, and add your own code
// for external hardware libraries.

#include <Adafruit_ADS1X15.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//#include <iostream>
//#include <cstring>

#include "eh_analog.h"
#include "eh_digital.h"
#include "eh_display.h"
#include "sensesp/sensors/analog_input.h"
#include "sensesp/sensors/digital_input.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp_app_builder.h"
#include "sensesp_onewire/onewire_temperature.h"
#include <Adafruit_BME280.h>
#include <Wire.h>

using namespace sensesp;
using namespace std;

// define temperature display units
#define TEMP_DISPLAY_FUNC KelvinToCelsius

// 1-Wire data pin on SH-ESP32
#define ONEWIRE_PIN 4

// I2C pins on SH-ESP32
const int kSDAPin = 16;
const int kSCLPin = 17;

// ADS1115 I2C address
const int kADS1115Address = 0x4b;

// CAN bus (NMEA 2000) pins on SH-ESP32
const int kCANRxPin = GPIO_NUM_34;
const int kCANTxPin = GPIO_NUM_32;

// Engine hat digital input pins
const int kDigitalInputPin1 = GPIO_NUM_15;
const int kDigitalInputPin2 = GPIO_NUM_13;
const int kDigitalInputPin3 = GPIO_NUM_14;
const int kDigitalInputPin4 = GPIO_NUM_12;

// Test output pin configuration
#define ENABLE_TEST_OUTPUT_PIN
#ifdef ENABLE_TEST_OUTPUT_PIN
const int kTestOutputPin = GPIO_NUM_18;
// repetition interval in ms; corresponds to 1000/(2*5)=100 Hz
const int kTestOutputInterval = 5;
#endif

Adafruit_BME280 bme280;

TwoWire* i2c;
Adafruit_SSD1306* display;


reactesp::ReactESP app;

float KelvinToCelsius(float temp) { return temp - 273.15; }
float KelvinToFahrenheit(float temp) { return (temp - 273.15) * 9. / 5. + 32.; }

// Store alarm states in an array for local display output
bool alarm_states[4] = {false, false, false, false};

// Convenience function to print the addresses found on the I2C bus
void ScanI2C(TwoWire* i2c) {
  uint8_t error, address;

  Serial.println("Scanning...");

  for (address = 1; address < 127; address++) {
    i2c->beginTransmission(address);
    error = i2c->endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("");
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  Serial.println("done");
}

  
#ifdef ENABLE_TEST_OUTPUT_PIN
void ToggleTestOutputPin(void * parameter) {
  while (true) {
    digitalWrite(kTestOutputPin, !digitalRead(kTestOutputPin));
    delay(kTestOutputInterval);
  }
}
#endif

float read_temp_callback() { return (bme280.readTemperature() + 273.15);}
float read_pressure_callback() { return (bme280.readPressure());}
float read_humidity_callback() { return (bme280.readHumidity());}


// The setup function performs one-time application initialization.
void setup() {
  #ifndef SERIAL_DEBUG_DISABLED
    SetupSerialDebug(115200);
  #endif

  // initialize the I2C bus
  i2c = new TwoWire(0);
  i2c->begin(kSDAPin, kSCLPin);

  ScanI2C(i2c);

  // Initialize ADS1115
  auto ads1115 = new Adafruit_ADS1115();
  ads1115->setGain(GAIN_ONE);
  bool ads_initialized = ads1115->begin(kADS1115Address, i2c);
  debugD("ADS1115 initialized: %d", ads_initialized);

  #ifdef ENABLE_TEST_OUTPUT_PIN
    pinMode(kTestOutputPin, OUTPUT);
    xTaskCreate(ToggleTestOutputPin, "toggler", 2048, NULL, 1, NULL);
  #endif

  // Construct the global SensESPApp() object
  SensESPAppBuilder builder;
  sensesp_app = (&builder)
                    // Set a custom hostname for the app.
                    ->set_hostname("engine-hat")
                    // Optionally, hard-code the WiFi and Signal K server
                    // settings. This is normally not needed.
                    //->set_wifi("My WiFi SSID", "my_wifi_password")
                    //->set_sk_server("192.168.10.3", 80)
                    ->get_app();


  // Initialize the OLED display
  bool display_present = InitializeSSD1306(&app, sensesp_app, &display, i2c);

  // Define the 1-wires sensors
  DallasTemperatureSensors* dts = new DallasTemperatureSensors(ONEWIRE_PIN);

  // define three 1-Wire temperature sensors that update every 1000 ms
  // and have specific web UI configuration paths

  auto onewire_1_temp = new OneWireTemperature(dts, 1000, "/engineOilTemp/oneWire");
  auto onewire_2_temp = new OneWireTemperature(dts, 1000, "/engineRoomTemp/oneWire");
  auto onewire_3_temp = new OneWireTemperature(dts, 1000, "/cabinTemp/oneWire");

  // define metadata for sensors
  auto onewire_1_temp_metadata =
      new SKMetadata("K",                       // units
                     "Engine Oil Temperature",  // display name
                     "Engine Oil Temperature",  // description
                     "Oil Temperature",         // short name
                     10.                        // timeout, in seconds
      );
  auto onewire_2_temp_metadata =
      new SKMetadata("K",                           // units
                     "Engine Room Temperature",  // display name
                     "Engine Room Temperature",  // description
                     "Engine Room Temperature",     // short name
                     10.                            // timeout, in seconds
      );
  auto onewire_3_temp_metadata =
      new SKMetadata("K",                        // units
                     "Cabin Room Temperature",  // display name
                     "Cabin Room Temperature; also used for outside temperature",  // description
                     "Cabin Room Temperature",      // short name
                     10.                         // timeout, in seconds
      );
  auto main_engine_temperature_metadata =
      new SKMetadata("K",                   // units
                     "Engine Temperature",  // display name
                     "Engine Temperature",  // description
                     "Temperature",         // short name
                     10.                    // timeout, in seconds
      );


  // N2K (taken from plugin) - SK mapping
  // Inside Temperature (130312) = environment.inside.temperature
  // Engine Room Temperature (130312) = environment.inside.engineRoom.temperature
  // Refridgerator Temperature (130312) = environment.inside.refrigerator.temperature
  // Main Cabin Temperature (130312)


  // connect the 1-wire sensors to Signal K output paths
  onewire_1_temp->connect_to(new SKOutput<float>("propulsion.1.oilTemperature", "/mainEngineOilTemp/skPath", onewire_1_temp_metadata));
  onewire_2_temp->connect_to(new SKOutput<float>("environment.inside.engineRoom.temperature	", "/mainEngineCoolantTemp/skPath", onewire_2_temp_metadata));
  onewire_3_temp->connect_to(new SKOutput<float>("environment.outside.temperature", "/mainEngineTemp/skPath", onewire_3_temp_metadata));
  // propulsion.*.wetExhaustTemperature is a non-standard path
  //onewire_3_temp->connect_to(
  //    new SKOutput<float>("propulsion.1.wetExhaustTemperature",
  //                        "/mainEngineWetExhaustTemp/skPath",
  //                        onewire_3_temp_metadata));

  // Connect the tank senders
  auto analog_1_input = ConnectTempSender(ads1115, 0, "1");  // de "1" is de eerste motor (wordt in sk-path ingevoegd, proposulsion.1.etc)
  // auto oilpressure_b_volume = ConnectTankSender(ads1115, 1, "B"); //20220827 placeholder toegevoegd i
  auto analog_3_input = ConnectTankSender(ads1115, 2, "1"); // Dieseltankvlotter. "1" is de eerste tank
  // auto tank_d_volume = ConnectTankSender(ads1115, 3, "D");

  

  // Connect the digital inputs

  // auto alarm_1_input = ConnectAlarmSender(kDigitalInputPin1, "1");
  auto digital_2_input = ConnectTachoSender(kDigitalInputPin2, "1"); // Connect the tacho senders (frequency), W-terminal on alternator
// auto alarm_3_input = ConnectAlarmSender(kDigitalInputPin3, "1"); // Oil pressure relay / alarm
  auto digital_4_input = ConnectAlarmSender(kDigitalInputPin4, "1"); // Temperature relay / alarm
  
  
  
  

  // Update the alarm states based on the input value changes
  digital_4_input->connect_to(new LambdaConsumer<bool>([](bool value) { alarm_states[1] = value; }));
  // alarm_3_input->connect_to(
  //     new LambdaConsumer<bool>([](bool value) { alarm_states[2] = value; }));
  // alarm_4_input->connect_to(
  //     new LambdaConsumer<bool>([](bool value) { alarm_states[3] = value; }));



  // Connect the outputs to the display, 1000 msec = 1 sec
  if (display_present) {
    app.onRepeat(1000, []() {
      PrintValue(display, 1, "IP:", WiFi.localIP().toString());
    });

    // Add display updaters for temperature values
    analog_3_input->connect_to(new LambdaConsumer<float>([](float value) { PrintValue(display, 2, "Tank A", 100 * value); }));

    digital_2_input->connect_to(new LambdaConsumer<float>([](float value) { PrintValue(display, 3, "RPM 1", 60 * value); }));

    // Create a "christmas tree" display for the alarms
    app.onRepeat(1000, []() {
      char state_string[5] = {};
      for (int i = 0; i < 4; i++) {
        state_string[i] = alarm_states[i] ? '*' : '_';
      }
      PrintValue(display, 4, "Alarm", state_string);
    });

     // Show 1-wire values
    // Add display updaters for temperature values
   //std::string strTemp = "";
   //String strTemp = "";
   
      onewire_1_temp->connect_to(new LambdaConsumer<float>(
          [](float temperature) {PrintValue(display, 5, "Temp 1:", TEMP_DISPLAY_FUNC(temperature));}));
          //[](float temperature) {PrintValue(display, 5, 0, "-", TEMP_DISPLAY_FUNC(temperature), true);}));
          //[strTemp](float temperature) mutable {return strTemp + std::to_string(TEMP_DISPLAY_FUNC(temperature));}));
      onewire_2_temp->connect_to(new LambdaConsumer<float>(
          [](float temperature) {PrintValue(display, 6, "Temp 2:", TEMP_DISPLAY_FUNC(temperature));}));
          //[](float temperature) {PrintValue(display, 5, 25, " ", TEMP_DISPLAY_FUNC(temperature), false);}));
          //[strTemp](float temperature) mutable {return strTemp + " / " + std::to_string(TEMP_DISPLAY_FUNC(temperature));}));
      onewire_3_temp->connect_to(new LambdaConsumer<float>(
          [](float temperature) {PrintValue(display, 7, "Temp 3:", TEMP_DISPLAY_FUNC(temperature));}));
          //[](float temperature) {PrintValue(display, 5, 50, " ", TEMP_DISPLAY_FUNC(temperature), false);}));
          //[strTemp](float temperature) mutable {return strTemp + " / " + std::to_string(TEMP_DISPLAY_FUNC(temperature));}));
      //PrintValue(display, 5, "1wire: ", strTemp); 
      
  }


  Wire.begin();
  //bme280.begin();
  unsigned status;
  status = bme280.begin();  
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
        Serial.print("SensorID was: 0x"); Serial.println(bme280.sensorID(),16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("        ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
        //while (1) delay(10);
    }
  
  // Create a RepeatSensor with float output that reads the temperature
  // using the function defined above.
  auto* bme280_temp = new RepeatSensor<float>(5000, read_temp_callback);
  auto* bme280_pressure = new RepeatSensor<float>(60000, read_pressure_callback);
  auto* bme280_humidity =  new RepeatSensor<float>(60000, read_humidity_callback);     
 
  // Send the temperature to the Signal K server as a Float
  bme280_temp->connect_to(new SKOutputFloat("environment.inside.engineBay.temperature"));
  bme280_pressure->connect_to(new SKOutputFloat("environment.inside.engineBay.pressure"));
  bme280_humidity->connect_to(new SKOutputFloat("environment.inside.engineBay.relativeHumidity"));

  // Start networking, SK server connections and other SensESP internals
  sensesp_app->start();
}

void loop() { app.tick(); }
