// See Engine Temperature Alarm code (somewhere below line 300)
// Tried to make sending a JsonObject with a sk notification work.
// That code didn't work

/*

  // ===== Engine Temperature Alarm/Relay =====
  auto* alarm_input = new DigitalInputChange(kDigitalInputPin1, INPUT, CHANGE);
  // TODO: set output to "normal" or "alarm", instead of 1 or 0.
  // this now sets the value to 0 or 1 but singalk spec (actually sbender on slack) says it is an Enum which can be normal, alarm, alert, emergency, etc.
  // an alarm is triggered by logic that would set the bit to 1 (in the N2K PGN) if state !== “normal”
  // so with this relay set it to "normal" and "alarm". 
  // explanation lambda functions in c++: https://docs.microsoft.com/en-us/cpp/cpp/lambda-expressions-in-cpp?view=msvc-170
  // sensesp docs LambdaConsumer: https://signalk.org/SensESP/pages/internals/
  // signalk notification specs: https://signalk.org/specification/1.7.0/doc/notifications.html
  // example spec: "value": {
  //          "message": "MOB",
  //          "state": "emergency",
  //          "method": ["visual", "sound"]
  
  // Below the test that i stopped working on, as i couldn't get it to work.

  // 1. Create a test JSON object
    auto json_function_test = [](int input) -> JsonObject {  
    DynamicJsonDocument doc(256);
    JsonObject obj;
    char msg[] = "{\"method\":[\"visual\"],\"state\":\"alarm\",\"message\":\"Engine overheating!\"}";
    //char msg[] = "{\"hello\":\"world\"}";
    DeserializationError err = deserializeJson(doc, msg); 
    if (err) {
      //debugD("Error: %s", err.f_str());
      PrintValue(display, 1, "Err: ","");
      return obj;
    }
    obj = doc.as<JsonObject>();
    const char* tmp = obj["message"];
    PrintValue(display, 1, "Result: ", tmp); // this prints sort of the right text (although with some strange starting characters)
    //serializeJson(doc, Serial); // shows the whole json string correctly
    return obj;
  };
  
  // 2. Extract the json object and print what is in it as a test.
  JsonObject json = json_function_test(0);
  const char* tmp22 = json["message"];
  PrintValue(display, 1, "Res:", tmp22);
  Serial.print("ALARM: ");
  Serial.println(tmp22); // prints strange codes
  debugD("ALARM - Sending to sk: %s", tmp22); // prints nothing

  // 3. connect the alarm to the modified LambdaTransform. This causes a crash of the esp32.
  // I made a small change the the SensESP library. I moved the generic (template) get_schema_type_string from lambda_transform.cpp to lambda_transform.h (in my local clone of the SensESP repo)
  // I included the local SensESP repo in platformio.ini by replacing the libs_dep entry for SensESP with /Users/bramhavers/Documents/code/SensESP/
  // If you make changes to SensESP than you have to remove the SensESPs in ./pio/libdeps (can also remove all dirs under libdeps).
  // For some strange reason the new one is build but the older is included as well. The right version is used.
  alarm_input
    ->connect_to(new LambdaTransform<int, JsonObject>(json_function_test))
    ->connect_to(new SKOutput<JsonObject>("notifications.propulsion.1.overTemperature"));  

  // 4. Lastly, i found debugging very hard. Writing things to Serial provides different results than using debugD and different than writing to the display.
  // I couldn't figure out how to debug on the device. 
  // Also some test code below DISPLAY. Commented out.
*/