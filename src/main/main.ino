#include <FS.h>                   //this needs to be first, or it all crashes and burns...

#include <Wire.h>
#include <SPI.h>

#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino

//needed for library
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager

#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson
#include <Adafruit_CAP1188.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>

Adafruit_CAP1188 cap = Adafruit_CAP1188();
WiFiManager wifiManager;
WiFiClient espClient;
PubSubClient mqttClient(espClient);
std::unique_ptr<ESP8266WebServer> webServer;

WiFiManagerParameter parameter_mqtt_server("server", "mqtt server", "192.168.1.8", 40);
WiFiManagerParameter parameter_mqtt_topic("topic", "mqtt topic", "/touchlogo", 40);
WiFiManagerParameter parameter_mqtt_port("port", "mqtt port", "1883", 6);
WiFiManagerParameter parameter_mqtt_user("user", "mqtt user", "", 20);
WiFiManagerParameter parameter_mqtt_pass("pass", "mqtt pass", "", 20);
WiFiManagerParameter parameter_hostname("hostname", "hostname", "touchlogo", 40);

char mqtt_server[40] = "192.168.1.8";
char mqtt_topic[40] = "/touchlogo";
uint16_t mqtt_port = 1883;
char mqtt_user[20] = "";
char mqtt_pass[20] = "";
char hostname[40] = "touchlogo";

const int SENSOR_COUNT = 5;

typedef struct {
  bool touched;
  unsigned long touchTime;
} TouchState;

typedef struct {
  int movement;
  unsigned long minTouchTime;
  unsigned long maxTouchTime;
  unsigned long diffTouchTime;
  uint8_t numberTouched;
}totalTouchStateType;


unsigned long lastSampleTime = 0;
unsigned long lastLedTime = 0;
unsigned long lastMqttConnectTime = (unsigned long) -10000;

TouchState touchStates[SENSOR_COUNT];
totalTouchStateType totalTouchState;

typedef enum buttonstates {
  BUTTON_RELEASED,
  BUTTON_PRESSED,
  BUTTON_LONGPRESSED,
  BUTTON_SWIPED_LEFT,
  BUTTON_SWIPED_RIGHT,
}ButtonStates;


ButtonStates buttonState = BUTTON_RELEASED;
bool shouldSaveConfig = false;

void configModeCallback (WiFiManager *myWiFiManager) {
  cap.writeRegister(0x72, 0x1B);
  cap.writeRegister(0x81, 0x30);
  cap.writeRegister(0x74, 0x04);
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  //if you used auto generated SSID, print it
  Serial.println(myWiFiManager->getConfigPortalSSID());
}

void saveConfigCallback () {
  cap.writeRegister(0x72, 0x1f);
  cap.writeRegister(0x81,0x00);
//  cap.writeRegister(0x81, 0x30);
  shouldSaveConfig = true;
}

void publishEvent(const char *eventName) {
  mqttClient.publish(mqtt_topic, eventName, true);
}

void setup() {
//  resetTouchStates();
  Serial.begin(115200);
  Serial.println("CAP1188 test!");

  if (!cap.begin(43)) {
    Serial.println("CAP1188 not found");
    while (1);
  }
  Serial.println("CAP1188 found!");

  cap.writeRegister(0x00,0b10000000); //sensitivity control
  cap.writeRegister(0x26,0x1f);
  cap.writeRegister(0x71,0xff);       //Led output register
  cap.writeRegister(0x72,0x1f);       //Led linking register
  cap.writeRegister(0x73,0xff);       //Led polarity register
  cap.writeRegister(0x81,0b00000000); //Set led breath or pulse.
  cap.writeRegister(0x82,0b00000000);
  cap.writeRegister(0x86,0x40);       // Breathing speed
  cap.writeRegister(0x93,0xf0);       // Max-min duty cycle set
  cap.writeRegister(0x92,0xf0);       // Max-min breathing duty cycle.
  cap.writeRegister(0x94,0b00001001); //Ramp rise and fall rate at touch
  cap.writeRegister(0x95,0b01100000); //Led off time between breathing
  cap.writeRegister(0x81, 0x00);    //Disable all breathing leds.


  if (SPIFFS.begin()) {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success()) {
          Serial.println("\nparsed json");

          strcpy(mqtt_server, json["mqtt_server"]);
          strcpy(mqtt_topic, json["mqtt_topic"]);
          mqtt_port = json.get<uint16_t>("mqtt_port");
          strcpy(mqtt_user, json["mqtt_user"]);
          strcpy(mqtt_pass, json["mqtt_pass"]);
          if (json.containsKey("hostname")) {
            strcpy(hostname, json["hostname"]);
          }
          Serial.print(mqtt_server);
          Serial.print(mqtt_topic);
          Serial.print(mqtt_port);
          Serial.print(mqtt_user);
          Serial.print(mqtt_pass);
          Serial.print(hostname);
          Serial.println("");
        } else {
          Serial.println("failed to load json config");
        }
      }
    }
  } else {
    Serial.println("failed to mount FS");
  }
  //end read

  ArduinoOTA.setHostname(hostname);
  ArduinoOTA.begin();

  wifiManager.addParameter(&parameter_mqtt_topic);
  wifiManager.addParameter(&parameter_mqtt_server);
  wifiManager.addParameter(&parameter_mqtt_port);
  wifiManager.addParameter(&parameter_mqtt_user);
  wifiManager.addParameter(&parameter_mqtt_pass);
  wifiManager.addParameter(&parameter_hostname);

  wifiManager.setSaveConfigCallback(saveConfigCallback);
  wifiManager.setAPCallback(configModeCallback);

  if (!wifiManager.autoConnect("AutoConnectAP")) {
    delay(3000);
    ESP.reset();
    delay(5000);
  }

  if (shouldSaveConfig) {
    strcpy(mqtt_server, parameter_mqtt_server.getValue());
    mqtt_port = atoi(parameter_mqtt_port.getValue());
    strcpy(mqtt_user, parameter_mqtt_user.getValue());
    strcpy(mqtt_pass, parameter_mqtt_pass.getValue());
    strcpy(mqtt_topic, parameter_mqtt_topic.getValue());
    strcpy(hostname, parameter_hostname.getValue());

    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["mqtt_server"] = mqtt_server;
    json["mqtt_port"] = mqtt_port;
    json["mqtt_user"] = mqtt_user;
    json["mqtt_pass"] = mqtt_pass;
    json["mqtt_topic"] = mqtt_topic;
    json["hostname"] = hostname;

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      Serial.println("failed to open config file for writing");
    }

    json.printTo(Serial);
    json.printTo(configFile);
    configFile.close();
    //end save
  }

  mqttClient.setServer(mqtt_server, mqtt_port);

  webServer.reset(new ESP8266WebServer(WiFi.localIP(), 80));
  webServer->on("/", []() {
    webServer->send(200, "text/plain",
    "/settings/reset   Reset all settings\n"
    "/calibrate        Calibrate CAP\n"
    "/reboot           Reboot Module");
  });
  webServer->on("/settings/reset", []() {
    wifiManager.resetSettings();
    SPIFFS.remove("/config.json");
    webServer->send(200, "text/plain", "ok");
    delay(1000);
    ESP.reset();
  });
  webServer->on("/calibrate", []() {
    cap.writeRegister(0x26, 0x1f);
    webServer->send(200, "text/plain", "ok");
  });
  webServer->on("/reboot",[](){
    webServer->send(200, "text/plain", "reboot");
    delay(1000);
    ESP.reset();
  });

  webServer->begin();
}

inline bool isSensorTouching(uint8_t capState, int sensorIndex) {
  return capState & (1 << sensorIndex);
}

void reconnectMqtt() {
  // Loop until we're reconnected
  if(!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    // If you do not want to use a username and password, change next line to
  // if (mqttClient.connect(hostname)) {
  if (mqttClient.connect("ESP8266Client", mqtt_user, mqtt_pass)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 10 seconds");
    }
  }
}

int sgn(signed long val) {
    return
      val < 0
        ? -1
        : val > 0
          ? 1
          : 0;
}

void loop() {
  ArduinoOTA.handle();

  webServer->handleClient();

  if (!mqttClient.connected()) {
    if(millis()-lastMqttConnectTime>10000){
      lastMqttConnectTime=millis();
      cap.writeRegister(0x82,0x3C); //Breath side leds;
      cap.writeRegister(0x74,0b01100000); //Switch side leds on.
      reconnectMqtt();
      if(mqttClient.connected()){
        cap.writeRegister(0x74,0b00000000);
        cap.writeRegister(0x82,0x00);
      }
    }
  }
  mqttClient.loop();

  unsigned int now = millis();
  if (now - lastSampleTime < 50) {
    return;
  }

  if(millis()-lastLedTime>1000){
    lastLedTime = 2147483647;
    cap.writeRegister(0x74,0b00000000);
  }

  lastSampleTime = now;

  analysePress();
  
  switch(buttonState){
    case BUTTON_RELEASED:
      if(totalTouchState.numberTouched){
        buttonState = BUTTON_PRESSED;
        Serial.println("Button pressed");
        totalTouchState.movement = 0;  
      }
      break;
    case BUTTON_LONGPRESSED:
      if(totalTouchState.numberTouched==0){
        buttonState = BUTTON_RELEASED;
        Serial.println("Longpress released");
      }
     break;
    case BUTTON_PRESSED:
      if (now-totalTouchState.minTouchTime > 2000 && totalTouchState.diffTouchTime < 200){
        buttonState = BUTTON_LONGPRESSED;
          // Long Touch
          publishEvent("longtouch");
          cap.writeRegister(0x74,0b01100000);
          lastLedTime = millis();     
          Serial.println("Longpress");

      } 
      if (totalTouchState.numberTouched==0){
         printTouchState();
         Serial.print("Button released: ");   
         if (totalTouchState.diffTouchTime < 200) {
          // Touch
          Serial.println("touched");
          publishEvent("touch");
          cap.writeRegister(0x74,0b01100000);
          lastLedTime = millis();
        } else if (totalTouchState.diffTouchTime < 2000 && totalTouchState.movement <= -4) {
          // Swipe left
          publishEvent("swipe_left");
          Serial.println("swipe left");
          cap.writeRegister(0x74,0b00100000);
          lastLedTime = millis();
        } else if (totalTouchState.diffTouchTime < 2000 && totalTouchState.movement >= 4) {
          // Swipe right
          publishEvent("swipe_right");
          Serial.println("swipe right");          
          cap.writeRegister(0x74,0b01000000);
          lastLedTime = millis();
        } else {
          Serial.println("no touch");
        }
        buttonState = BUTTON_RELEASED;
      }
      break;
    default:
      buttonState = BUTTON_RELEASED;
      break;
  }  
}


StaticJsonBuffer<200> jsonBuffer;
  
void printTouchState(void){
    jsonBuffer.clear();
    JsonObject& json = jsonBuffer.createObject();
    json["mov"] = totalTouchState.movement;
    json["minT"] = totalTouchState.minTouchTime;
    json["maxT"] = totalTouchState.maxTouchTime;
    json["diffT"] = totalTouchState.diffTouchTime;
    json["numberTouched"] = totalTouchState.numberTouched;
    json.printTo(Serial);
}

void analysePress(void){
  uint8_t newTouchState = cap.touched();
  totalTouchState.numberTouched = 0;
  for (uint8_t i=0; i<SENSOR_COUNT; i++) {
    TouchState &state = touchStates[i];
    bool isTouching = isSensorTouching(newTouchState, i);
    // Has sensor changed to touched?
    if (state.touched != isTouching && isTouching) {
      state.touchTime = millis();
    }
    if(isTouching){
       totalTouchState.numberTouched++;
    }
    state.touched = isTouching;
  }
  
  
  totalTouchState.minTouchTime = 0xffffffff;
  totalTouchState.maxTouchTime = 0;
  for (uint8_t i=0;i<SENSOR_COUNT;i++) {
    TouchState &state = touchStates[i];
    totalTouchState.minTouchTime = min(totalTouchState.minTouchTime, state.touchTime);
    totalTouchState.maxTouchTime = max(totalTouchState.maxTouchTime, state.touchTime);
  }
  totalTouchState.diffTouchTime = totalTouchState.maxTouchTime - totalTouchState.minTouchTime;

    for(uint8_t i=1;i<SENSOR_COUNT;i++) {
    TouchState &stateLeft = touchStates[i-1];
    TouchState &stateRight = touchStates[i];

    totalTouchState.movement += sgn((signed long) (stateRight.touchTime - stateLeft.touchTime));
  }
}

