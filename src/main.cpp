/*
 * IRremoteESP8266: IRServer - MQTT IR transceiver
 * used library:
 * https://github.com/markszabo/IRremoteESP8266
 * based on:
 * https://github.com/z3t0/Arduino-IRremote/
 * Version 0.2 May, 2016
 */

/*
 * broker -> module:
 *    "_mqtt_prefix_/sender/storeRaw/_store_id_"    msg: "\d+(,\d+)*"
 *                - store raw code in given slot _store_id_
 *    "_mqtt_prefix_/sender/sendStoredRaw"   msg: "\d+"
 *                - transmit raw code from given slot
 *    "_mqtt_prefix_/sender/sendStoredRawSequence"   msg: "\d(,\d+)*"
 *                - transmit raw codes seqnece of given slots
 *    "_mqtt_prefix_/sender/cmd"             msg: "(ls|sysinfo)"
 *                - send system info query response in "esp8266/02/sender/cmd/result"
 *    "_mqtt_prefix_/sender/NC/HDMI"         msg: ".+"
 *                - send NC+ HDMI sequence
 *    "_mqtt_prefix_/sender/NC/EURO"         msg: ".+"
 *                - send NC+ EURO sequence
 *    "_mqtt_prefix_/sender/rawMode"         msg: "(1|ON|true|.*)"
 *                - enable/disable raw mode
 *    "_mqtt_prefix_/sender/type(/\d+(/\d+)) msg: "\d+"
 *                - esp8266/02/sender/type[/bits[/panasonic_address]] - type: NEC, RC_5, RC_6, SAMSUNG, SONY
 *    "_mqtt_prefix_/wipe" msg: ".*"
 *                - wipe config file
 *
 * module -> broker
 *    "_mqtt_prefix_/receiver/_type_/_bits_"               msg: "\d+"
 *                - recived message with given type and n-bits
 *    "_mqtt_prefix_/receiver/_type_/_bits_/_panas_addr_"  msg: "\d+"
 *                - recived message with given type and n-bits  and panasonic addres
 *
 */

//#define MQTT_MAX_PACKET_SIZE 800
#include "globals.h"

/***************************************************
 * Setup
 */
void setup(void)
{

  // delay for reset button
  delay(5000);

  #ifdef DEBUG
    Serial.begin(CUST_SERIAL_SPEED);

    struct rst_info *rtc_info = system_get_rst_info();
    Serial.printf("reset reason: %x\n", rtc_info->reason);
    if (rtc_info->reason == REASON_WDT_RST || rtc_info->reason == REASON_EXCEPTION_RST || rtc_info->reason == REASON_SOFT_WDT_RST) {
      if (rtc_info->reason == REASON_EXCEPTION_RST) {
        Serial.printf("Fatal exception (%d):\n", rtc_info->exccause);
      }
      Serial.printf("epc1=0x%08x, epc2=0x%08x, epc3=0x%08x, excvaddr=0x%08x, depc=0x%08x\n", rtc_info->epc1, rtc_info->epc2, rtc_info->epc3, rtc_info->excvaddr, rtc_info->depc); //The address of the last crash is printed, which is used to debug garbled output.
    }

    uint32_t realSize = ESP.getFlashChipRealSize();
    uint32_t ideSize = ESP.getFlashChipSize();
    FlashMode_t ideMode = ESP.getFlashChipMode();
    Serial.printf("Flash real id:   %08X\n", ESP.getFlashChipId());
    Serial.printf("Flash real size: %u bytes\n\n", realSize);
    Serial.printf("Flash ide  size: %u bytes\n", ideSize);
    Serial.printf("Flash ide speed: %u Hz\n", ESP.getFlashChipSpeed());
    Serial.printf("Flash ide mode:  %s\n", (ideMode == FM_QIO ? "QIO" : ideMode == FM_QOUT ? "QOUT" : ideMode == FM_DIO ? "DIO" : ideMode == FM_DOUT ? "DOUT" : "UNKNOWN"));
  #else
    Serial.begin(CUST_SERIAL_SPEED,SERIAL_8N1,SERIAL_TX_ONLY);
    sendToDebug("*IR: non debug init\n");
  #endif

  // Init EEPROM
  EEPROM.begin(sizeof(EEpromData));
  EEPROM.get(0,EEpromData);
  if (EEpromData.autoSendMode!=true && EEpromData.autoSendMode!=false)
  {
    EEpromData.autoSendMode=false;
    EEPROM.put(0, EEpromData);
    EEPROM.commit();
  }

  pinMode(TRIGGER_PIN, INPUT);

  #ifdef LED_PIN
  pinMode(LED_PIN,OUTPUT);
  #endif

  #ifdef DISPLAY_SIZE
  // Initialising the UI will init the display too.
  display.init();

  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_24);
  printOled("MQTT-IR");
  display.setFont(ArialMT_Plain_10);
  #endif

  if (LittleFS.begin())
  {
    // LittleFS.format();
    // sendToDebug("*IR: format file system\n");
    
    sendToDebug("*IR: mounted file system\n");
    if (LittleFS.exists("/config.json"))
    {
      //file exists, reading and loading
      sendToDebug("*IR: reading config file\n");
      File configFile = LittleFS.open("/config.json", "r");
      if (configFile)
      {
        sendToDebug("*IR: opened config file\n");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        char tmpBuff[400];
        json.printTo(tmpBuff, sizeof(tmpBuff));
        sendToDebug(String("*IR: config: "+String(tmpBuff) + "\n"));
        if (json.success())
        {
          sendToDebug("*IR: parsed json\n");
          if (json.containsKey("mqtt_server"))
            strcpy(mqtt_server, json["mqtt_server"]);
          if (json.containsKey("mqtt_port"))
            strcpy(mqtt_port, json["mqtt_port"]);
          if (json.containsKey("mqtt_secure"))
            strcpy(mqtt_secure, json["mqtt_secure"]);
          if (json.containsKey("mqtt_user"))
            strcpy(mqtt_user, json["mqtt_user"]);
          if (json.containsKey("mqtt_pass"))
            strcpy(mqtt_pass, json["mqtt_pass"]);
          if (json.containsKey("mqtt_prefix"))
            strcpy(mqtt_prefix, json["mqtt_prefix"]);
        }
        else
        {
          sendToDebug("*IR: failed to load json config\n");
        }
      }
    }
  }
  else
  {
    sendToDebug("*IR: failed to mount FS\n");
  }
  sendToDebug("*IR: Start setup\n");

  WiFiManagerParameter custom_mqtt_secure("secure", "is secure server 0-no / 1-yes", mqtt_secure, 2);
  WiFiManagerParameter custom_mqtt_server("server", "MQTT server address", mqtt_server, 40);
  WiFiManagerParameter custom_mqtt_port("port", "MQTT server port", mqtt_port, 5);
  WiFiManagerParameter custom_mqtt_user("user", "MQTT user", mqtt_user, 32);
  WiFiManagerParameter custom_mqtt_pass("pass", "MQTT password", mqtt_pass, 32);
  WiFiManagerParameter custom_mqtt_prefix("prefix", "MQTT prefix", mqtt_prefix, 80);
  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  #ifdef LED_PIN
  digitalWrite(LED_PIN, LOW);
  #endif
  WiFiManager wifiManager;
  wifiManager.setTimeout(180);

  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  wifiManager.addParameter(&custom_mqtt_secure);
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_mqtt_user);
  wifiManager.addParameter(&custom_mqtt_pass);
  wifiManager.addParameter(&custom_mqtt_prefix);

  if ( digitalRead(TRIGGER_PIN) == BUTTON_ACTIVE_LEVEL || (!LittleFS.exists("/config.json")) )
  {
    // Force enter configuration
    wifiManager.resetSettings();

    // Set EEprom defaults
    EEpromData.autoSendMode=false;
    EEPROM.put(0, EEpromData);
    EEPROM.commit();
  }
  #ifndef DEBUG
    wifiManager.setDebugOutput(false);
  #endif
  char mySSID[17];
  char myPASS[9];
  sprintf(mySSID,"IRTRANS-00%06X", ESP.getChipId());
  sprintf(myPASS,"00%06X", ESP.getChipId());
  if (!wifiManager.autoConnect(mySSID, myPASS) )
  {
    sendToDebug("*IR: failed to connect and hit timeout\n");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(5000);
  }
  #ifdef LED_PIN
  digitalWrite(LED_PIN, HIGH);
  #endif

  //read updated parameters
  strcpy(mqtt_secure, custom_mqtt_secure.getValue());
  strcpy(mqtt_server, custom_mqtt_server.getValue());
  strcpy(mqtt_port, custom_mqtt_port.getValue());
  strcpy(mqtt_user, custom_mqtt_user.getValue());
  strcpy(mqtt_pass, custom_mqtt_pass.getValue());
  strcpy(mqtt_prefix, custom_mqtt_prefix.getValue());

  String tmp = mqtt_port;
  if (tmp.toInt())
  {
    mqtt_port_i = tmp.toInt();
  } else {
    mqtt_port_i = DEFAULT_MQTT_PORT;
  }
  if (mqtt_secure[0]=='1')
  {
    mqtt_secure_b = true;
  } else {
    mqtt_secure_b = false;
  }

  irsend.begin();
  irrecv.enableIRIn();  // Start the receiver

  //save the custom parameters to FS
  if (shouldSaveConfig)
  {
    sendToDebug("*IR: saving config\n");
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["mqtt_server"] = mqtt_server;
    json["mqtt_user"] = mqtt_user;
    json["mqtt_pass"] = mqtt_pass;
    json["mqtt_prefix"] = mqtt_prefix;
    json["mqtt_port"] = mqtt_port;
    json["mqtt_secure"] = mqtt_secure;

    File configFile = LittleFS.open("/config.json", "w");
    if (configFile)
    {
      char tmpBuff[400];
      json.printTo(tmpBuff, sizeof(tmpBuff));
      sendToDebug(String("*IR: writing config: "+String(tmpBuff) + "\n"));

      json.printTo(configFile);
      configFile.close();
    }
    else
    {
      sendToDebug("*IR: failed to open config file for writing\n");
    }
  }

  sendToDebug(String("*IR: Connected to ")+String (WiFi.SSID())+"\n");
  sendToDebug(String("*IR: IP address: ")+WiFi.localIP().toString()+"\n");

  clientName += "IRGW-";
  uint8_t mac[6];
  WiFi.macAddress(mac);
  clientName += macToStr(mac);
  clientName += "-";
  clientName += String(micros() & 0xff, 16);

  connect_to_MQTT();

  loadDefaultIR();
}



/****************************************************************
 * Main loop
 */
void loop(void)
{
  #ifdef DISPLAY_SIZE
  if ((millis() - lastDisplay) > 20000UL) printOled(String(""));
  #endif

  if (MQTTMode)
  {
    mqttClient.loop();

    if (! mqttClient.connected())
    {

      sendToDebug("*IR: Not connected to MQTT....\n");
      delay(2000);
      connect_to_MQTT();
    }
    decode_results  results;        // Somewhere to store the results
    if (irrecv.decode(&results))
    {  // Grab an IR code
      char myTopic[100];
      char myTmp[50];
      char myValue[500];
      String myMessage = "";
      getIrEncoding (&results, myTmp);
      if (results.decode_type == PANASONIC)
      { //Panasonic has address
        // structure "prefix/typ/bits[/panasonic_address]"
        sprintf(myTopic, "%s/receiver/%s/%d/%d", mqtt_prefix, myTmp, results.bits, results.address );
        myMessage = String("Received ") + myTmp + "/" + results.bits + "/" + results.address + "\n";
      }
      else
      {
        sprintf(myTopic, "%s/receiver/%s/%d", mqtt_prefix, myTmp, results.bits );
        if ((strncmp(myTmp,"NEC",3) != 0) || ((int)results.bits != 0))  myMessage = String("Received ") + myTmp + "/" + results.bits + "\n";
      }
      if (results.decode_type != UNKNOWN)
      {
        // any other has code and bits
        sprintf(myValue, "%d", (int)results.value);
        if (myMessage != "") myMessage += myValue;
        mqttClient.publish((char*) myTopic, (char*) myValue );
      }
      else if (rawMode==true)
      {
        // RAW MODE
        String myString;
        for (int i = 1;  i < results.rawlen;  i++)
        {
          myString+= (results.rawbuf[i] * RAWTICK);
          if ( i < results.rawlen-1 )
            myString+=","; // ',' not needed on last one
        }
        myString.toCharArray(myValue,500);
        sprintf(myTopic, "%s/receiver/raw", mqtt_prefix );
        myMessage = String("Received Raw\n") + myValue;
        mqttClient.publish( (char*) myTopic, (char*) myValue );
      }
      else myMessage = "Received unknown";
      #ifdef DISPLAY_SIZE
      if (myMessage != "") printOled(myMessage);
      #endif
      irrecv.resume();              // Prepare for the next value
    }
  }
  else if (millis() - lastTSMQTTReconect > 60000)
  {
    // Try to reconnect MQTT every 60 seconds if dev is in nonmqtt mode
    mqttClient.loop();

    if (! mqttClient.connected())
    {
      sendToDebug("*IR: Not connected to MQTT....\n");
      delay(2000);
      connect_to_MQTT();
    }
    lastTSMQTTReconect = millis();
  }

  bool newButtonState = digitalRead(TRIGGER_PIN);

  // Manual force of sequence
  if (buttonState != newButtonState && newButtonState == BUTTON_ACTIVE_LEVEL)
  {
    // Button pressed - send 1st code
    lastTSAutoStart=millis(); // delay auto transmission
    #ifdef LED_PIN
    digitalWrite(LED_PIN, LOW);
    #endif
    if (rawIR1size>0)
    {
      sendToDebug("*IR: Button pressed - transmitting 1\n");
      #ifdef DISPLAY_SIZE
      printOled("Send Button 1 raw");
      #endif
      irsend.sendRaw(rawIR1, rawIR1size, TRANSMITTER_FREQ);
    }
    #ifdef LED_PIN
    digitalWrite(LED_PIN, HIGH);
    #endif

  }
  else if (buttonState != newButtonState && newButtonState != BUTTON_ACTIVE_LEVEL)
  {
    // Button released - send 2nd code
    #ifdef LED_PIN
    digitalWrite(LED_PIN, LOW);
    #endif
    if (rawIR2size>0)
    {
      sendToDebug("*IR: Button released - transmitting 2\n");
      #ifdef DISPLAY_SIZE
      printOled("Send Button 2 raw");
      #endif
      irsend.sendRaw(rawIR2, rawIR2size, TRANSMITTER_FREQ);
    }
    #ifdef LED_PIN
    digitalWrite(LED_PIN, HIGH);
    #endif
  }
  buttonState = newButtonState;

  if (EEpromData.autoSendMode)
  {
    if (autoStartSecond && (millis() - lastTSAutoStart > 3000))
    {
      if (rawIR2size>0)
      {
        sendToDebug("*IR: Auto sender - transmitting 2\n");
        #ifdef DISPLAY_SIZE
        printOled("Send Auto 2 raw");
        #endif
        irsend.sendRaw(rawIR2, rawIR2size, TRANSMITTER_FREQ);
      }
      #ifdef LED_PIN
      digitalWrite(LED_PIN, HIGH);
      #endif
      autoStartSecond = false;
    }
    if (millis() - lastTSAutoStart > autoStartFreq)
    {
      // Autostart
      #ifdef LED_PIN
      digitalWrite(LED_PIN, LOW);
      #endif
      if (rawIR1size>0)
      {
        sendToDebug("*IR: Auto sender - transmitting 1\n");
        #ifdef DISPLAY_SIZE
        printOled("Send Auto 1 raw");
        #endif
        irsend.sendRaw(rawIR1, rawIR1size, TRANSMITTER_FREQ);
      }
      autoStartSecond = true;
      lastTSAutoStart=millis();
    }
  }
}
