// Firmware for the SH-ESP32 with Engine Hat.
// This code is based on the example template code.

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
#include "sensesp/transforms/curveinterpolator.h"
#include "sensesp/transforms/linear.h"
#include "sensesp/transforms/frequency.h"
#include "sensesp/transforms/moving_average.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp_app_builder.h"
#include "sensesp_onewire/onewire_temperature.h"
#include <Adafruit_BME280.h>
#include <Wire.h>

using namespace sensesp;
using namespace std;

// define temperature display units
#define TEMP_DISPLAY_FUNC KelvinToCelsius

// Pin definitions
#define ONEWIRE_PIN 4 // 1-Wire data pin on SH-ESP32
const int kSDAPin = 16; // I2C pin on SH-ESP32
const int kSCLPin = 17; // I2C pin on SH-ESP32
const int kADS1115Address = 0x4b; // ADS1115 I2C address
const int kCANRxPin = GPIO_NUM_34; // CAN bus (NMEA 2000) pin on SH-ESP32
const int kCANTxPin = GPIO_NUM_32; // CAN bus (NMEA 2000) pin on SH-ESP32
const int kDigitalInputPin1 = GPIO_NUM_15; // Engine hat digital input pin
const int kDigitalInputPin2 = GPIO_NUM_13; // Engine hat digital input pin
const int kDigitalInputPin3 = GPIO_NUM_14; // Engine hat digital input pin
const int kDigitalInputPin4 = GPIO_NUM_12; // Engine hat digital input pin

const float kAnalogInputScale = 29. / 2.048; // ADS1115 input hardware scale factor (input voltage vs voltage at ADS1115)
const float kMeasurementCurrent = 0.01; // Engine Hat constant measurement current (A)
const uint ads_read_delay = 500;  // ms

// Test output pin configuration
#define ENABLE_TEST_OUTPUT_PIN
#ifdef ENABLE_TEST_OUTPUT_PIN
const int kTestOutputPin = GPIO_NUM_18;
// repetition interval in ms; corresponds to 1000/(2*5)=100 Hz
const int kTestOutputInterval = 5;
#endif

// Global definitions
const uint sensor_read_interval = 1000; // the delay / interval to read the sensors, used for all sensors in this program


// Specific defintions per sensor
// - config_* is the configuration path (see SensESP concepts documentations) that is shown (and editable at runtime) in the UI of the Engine Hat at http://enginehat.local
// - value_* is the default setting for a sensor configuration (also available through engine hat UI)
// - sk_path_* is the signalk path that the sensor reading is assigned to; this will show up in the signalk-server Data Browser.
// - SKMetadata is SignalK metadata parameters: unit of measure, display name, description, short name, timeout in secs

// Definitions: Engine RPM (W-terminal on alternator)
const char* config_engine1_freq_multiplier = "/engine1/tach/frequency multiplier"; // the alternator report a frequency that needs to be multiplied/divided to represent the actual RPM.
const char* config_engine2_freq_multiplier = "/engine2/tach/frequency multiplier"; // define a virtual second engine to define alternative parameters, see moving avg 2.
const char* config_engine1_moving_avg = "/engine1/tach/moving avg samples"; // the rpm might 'jump' around, use this number of samples to calculate a smooth rpm.
const char* config_engine2_moving_avg = "/engine2/tach/moving avg samples"; // the first moving_avg might be slow, configure a faster (but more jumpy moving avg on a virtual second engine).
const char* config_engine1_revolutions_sk_path = "/engine1/tach/revolutions/sk_path";
const char* config_engine2_revolutions_sk_path = "/engine2/tach/revolutions/sk_path";
auto metadata_engine_tach_revolutions = new SKMetadata("Hz", "Revolutions", "Revolutions per second", "Revolutions", 3);
const char* value_engine1_tach_revolutions_sk_path = "propulsion.1.revolutions";
const char* value_engine2_tach_revolutions_sk_path = "propulsion.2.revolutions";
const float value_engine_freq_multiplier = 1. / 9.5238; // 2022-08-26: Multiplier for Volvo Penta MD2020 is 0.105, so 1/ 9.5238
const float value_engine1_moving_avg = 15;
const float value_engine2_moving_avg = 2;


//auto engine1_tacho_frequency = new Frequency(value_engine1_freq_multiplier, config_engine1_freq_multiplier);
//auto engine1_tacho_mov_avg = new MovingAverage(value_engine1_moving_avg, 1.0, config_engine1_moving_avg);
// TODO: check if SKMetadata might interfere with stuff. Otherwise remove this line.
//auto engine1_tacho_frequency_sk_output = new SKOutputFloat(value_engine1_tach_revolutions_sk_path, config_engine1_revolutions_sk_path,metadata_engine1_tach_revolutions);

// Definitions: 1-wire sensors
const char* config_onewire_1_address = "/onewire/1/address"; 
const char* config_onewire_2_address = "/oneWire/2/address"; 
const char* config_onewire_3_address = "/oneWire/3/address";
const char* config_onewire_1_sk_path = "/onewire/1/signalk path";
const char* config_onewire_2_sk_path = "/oneWire/2/signalk path";
const char* config_onewire_3_sk_path = "/oneWire/3/signalk path";
auto onewire_1_metadata = new SKMetadata("K", "Engine Oil Temperature", "Engine Oil Temperature", "Oil Temperature", 10);
auto onewire_2_metadata = new SKMetadata("K", "Engine Room Temperature", "Engine Room Temperature", "Engine Room Temperature", 10);
auto onewire_3_metadata = new SKMetadata("K", "Cabin Room Temperature", "Cabin Room Temperature; also used for outside temperature", "Cabin Room Temperature", 10);
const char* value_onewire_1_sk_path = "propulsion.1.oilTemperature";
const char* value_onewire_2_sk_path = "environment.inside.engineRoom.temperature";
const char* value_onewire_3_sk_path = "environment.outside.temperature";

// Definition: Engine temperature sender
uint8_t enginehat_pin_temp = 0; // attached to pin A on the Engine Hat.
const char* config_engine1_temp_resistance_sk_path = "/engine1/temp/resistance/sk_path"; // the sk_path of the resistance of the temp sender in the engine.
const char* config_engine1_temp_temperature_sk_path = "/engine1/temp/temperature/sk_path"; // the sk_path of the temperature of the temp sender in the engine.
const char* config_engine1_temp_level_curve = "/engine1/temp/Level Curve";
const char* value_engine1_temp_resistance_sk_path = "propulsion.1.temperature.senderResistance";
const char* value_engine1_temp_temperature_sk_path = "propulsion.1.temperature";
auto metadata_engine1_temp_resistance = new SKMetadata("ohm", "Resistance", "Measured resistance of temperature sender", "Resistance", 10);
auto metadata_engine1_temp_temperature = new SKMetadata("K", "Engine temperature", "Temperature of cooling water sender in engine", "Temperature", 10);


// Definition: Engine temperature alarm (relay)


// Definition: Engine oil pressure sender


// Definition: Engine oil pressure alarm (relay)


// Definition: BME280 sensors (temperature, barometric pressure, relative humidy)


Adafruit_BME280 bme280;
TwoWire* i2c;
Adafruit_SSD1306* display;
reactesp::ReactESP app;

bool alarm_states[4] = {false, false, false, false}; // Store alarm states in an array for local display output

  
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


// Convenience functions

// Convenience: convert temperatures between Celsius, Kelvin and Fahrenheit 
float KelvinToCelsius(float temp) { return temp - 273.15; }
float KelvinToFahrenheit(float temp) { return (temp - 273.15) * 9. / 5. + 32.; }

// Convenience: print the addresses found on the I2C bus
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
                    ->set_hostname("engine-hat")
                    ->get_app();

  // Initialize the OLED display
  bool display_present = InitializeSSD1306(&app, sensesp_app, &display, i2c);

  // Activate the 1-wires senders
  DallasTemperatureSensors* dts = new DallasTemperatureSensors(ONEWIRE_PIN);
  auto onewire_1_temp = new OneWireTemperature(dts, sensor_read_interval, config_onewire_1_address);
  auto onewire_2_temp = new OneWireTemperature(dts, sensor_read_interval, config_onewire_2_address);
  auto onewire_3_temp = new OneWireTemperature(dts, sensor_read_interval, config_onewire_3_address);
  onewire_1_temp->connect_to(new SKOutput<float>(value_onewire_1_sk_path, config_onewire_1_sk_path, onewire_1_metadata)); 
  onewire_2_temp->connect_to(new SKOutput<float>(value_onewire_2_sk_path, config_onewire_2_sk_path, onewire_2_metadata));
  onewire_3_temp->connect_to(new SKOutput<float>(value_onewire_3_sk_path, config_onewire_3_sk_path, onewire_3_metadata));

  // Analog senders
  
  // Engine Temperature Sender 
  //auto analog_1_input = ConnectTempSender(ads1115, 0, "1");  // de "1" is de eerste motor (wordt in sk-path ingevoegd, proposulsion.1.etc)
  // TODO: Simply this code, removing variables and using connect_to()
  // TODO: in het orgineel wordt enginehat_pin_temp meegegeven: auto engine1_temp_sender_resistance = new RepeatSensor<float>(ads_read_delay, [ads1115, enginehat_pin_temp]()
  // Maar dat werkt niet. Volgens de doc hoef je helemaal niks mee te geven. Het is een callback function, maar tussen {} staat al de callback.
  auto engine1_temp_sender_resistance = new RepeatSensor<float>(ads_read_delay, [ads1115]() {
        int16_t adc_output = ads1115->readADC_SingleEnded(enginehat_pin_temp);
        float adc_output_volts = ads1115->computeVolts(adc_output);
        return kAnalogInputScale * adc_output_volts / kMeasurementCurrent;
      });
  auto engine1_temp_sender_resistance_sk_output = new SKOutputFloat(value_engine1_temp_resistance_sk_path, config_engine1_temp_temperature_sk_path, metadata_engine1_temp_resistance);
  auto engine1_temperature = (new CurveInterpolator(nullptr, config_engine1_temp_level_curve))
                        ->set_input_title("Sender Resistance (ohms)")
                        ->set_output_title("Temp Level (ratio)");
  if (engine1_temperature->get_samples().empty()) {
    // If there's no prior configuration, provide a default curve
    engine1_temperature->clear_samples();
    engine1_temperature->add_sample(CurveInterpolator::Sample(800, 0)); 
    engine1_temperature->add_sample(CurveInterpolator::Sample(760, 15));
    engine1_temperature->add_sample(CurveInterpolator::Sample(484, 20));
    engine1_temperature->add_sample(CurveInterpolator::Sample(304, 30));
    engine1_temperature->add_sample(CurveInterpolator::Sample(209, 40));
    engine1_temperature->add_sample(CurveInterpolator::Sample(149, 50));
    engine1_temperature->add_sample(CurveInterpolator::Sample(104, 60));
    engine1_temperature->add_sample(CurveInterpolator::Sample(79, 70));
    engine1_temperature->add_sample(CurveInterpolator::Sample(72, 80));
    engine1_temperature->add_sample(CurveInterpolator::Sample(40, 90));
    engine1_temperature->add_sample(CurveInterpolator::Sample(29, 95)); 
  }
  auto engine1_temperature_sk_output = new SKOutputFloat(value_engine1_temp_temperature_sk_path, config_engine1_temp_temperature_sk_path, metadata_engine1_temp_temperature); 
  engine1_temp_sender_resistance->connect_to(engine1_temp_sender_resistance_sk_output);
  engine1_temp_sender_resistance->connect_to(engine1_temperature)->connect_to(engine1_temperature_sk_output);

  // Engine Oil Pressure Sender
   //auto oilpressure_b_volume = ConnectTankSender(ads1115, 1, "B"); //20220827 placeholder toegevoegd i


  // Fuel Tank Sender
  auto analog_3_input = ConnectTankSender(ads1115, 2, "1"); // Dieseltankvlotter. "1" is de eerste tank
  

  // Digital Senders
  // Engine Temperature Alarm/Relay
  // auto alarm_1_input = ConnectAlarmSender(kDigitalInputPin1, "1");

  // Engine Tacho (RPM) Sender (W-terminal on alternator)
  auto engine1_tacho_sender = new DigitalInputCounter(kDigitalInputPin2, INPUT, RISING, 500, "");
  engine1_tacho_sender
    ->connect_to(new Frequency(value_engine_freq_multiplier, config_engine1_freq_multiplier))
    ->connect_to(new MovingAverage(value_engine1_moving_avg, 1.0, config_engine1_moving_avg))
    ->connect_to(new SKOutputFloat(value_engine1_tach_revolutions_sk_path, config_engine1_revolutions_sk_path,metadata_engine_tach_revolutions));

   engine1_tacho_sender
    ->connect_to(new Frequency(value_engine_freq_multiplier, config_engine2_freq_multiplier))
    ->connect_to(new MovingAverage(value_engine2_moving_avg, 1.0, config_engine2_moving_avg))
    ->connect_to(new SKOutputFloat(value_engine2_tach_revolutions_sk_path, config_engine2_revolutions_sk_path,metadata_engine_tach_revolutions));

  
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

    engine1_tacho_sender->connect_to(new LambdaConsumer<float>([](float value) { PrintValue(display, 3, "RPM 1", 60 * value); }));

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




