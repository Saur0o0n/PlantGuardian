#include "ESP8266WiFi.h" // Enables the ESP8266 to connect to the local network (via WiFi)
#include "PubSubClient.h" // Allows us to connect to, and publish to the MQTT broker
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <BH1750.h>

#define PLANTGUARDIAN "plantguardian"

// Moisture stuff
const int MoistPin01 = A0;  // ESP8266 Analog Pin ADC0 = A0
const int Plant01Pin = D8;
const int Plant02Pin = D7;

const int MoistMax = 950;
const int MoistMin = 1;

// Other sensors
const int SensorEnaPin = D1; // This ping enables transistor to all other sensors (water level, temperature, humidity, pressure etc)
const int WaterPin = D2;     // Water level pin
const int I2C_SCL = D6;
const int I2C_SCA = D5;

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C
BH1750 lightMeter(0x23);


// WiFi
const char* ssid = "wifi";
const char* wifi_password = "pass";

int sensorValue01 = 0;  // value read from the pot
int outputValue01 = 0;  // value to output to a PWM pin
int sensorValue02 = 0;  // value read from the pot
int outputValue02 = 0;  // value to output to a PWM pin
float temperatureValue = 0;
float humidityValue = 0;
float pressureValue = 0;
float illuValue = 0;
float altitudeValue = 0;
bool waterValue = 1;    // 1 = full, 0 = empty
byte error = 0;


// MQTT
// Make sure to update this for your own MQTT Broker!
const char* mqtt_server = "192.168.70.100";
const char* plant_topic = "homeassistant/sensor/"PLANTGUARDIAN"/state";
const char* mqtt_username = "username";
const char* mqtt_password = "pass";

const char* plant1_config_topic = "homeassistant/sensor/"PLANTGUARDIAN"/plant1/config";
const char* plant2_config_topic = "homeassistant/sensor/"PLANTGUARDIAN"/plant2/config";
const char* plant1_config = "{\"device_class\": \"humidity\", \"name\": \"Plant 1 moisture\", \"state_topic\": \"homeassistant/sensor/"PLANTGUARDIAN"/state\", \"unit_of_measurement\": \"%\", \"unique_id\": \"plantguard_sensor_01\", \"value_template\": \"{{ value_json.moisture1}}\" }";
const char* plant2_config = "{\"device_class\": \"humidity\", \"name\": \"Plant 2 moisture\", \"state_topic\": \"homeassistant/sensor/"PLANTGUARDIAN"/state\", \"unit_of_measurement\": \"%\", \"unique_id\": \"plantguard_sensor_02\", \"value_template\": \"{{ value_json.moisture2}}\" }";

const char* temp_config_topic = "homeassistant/sensor/"PLANTGUARDIAN"/temperature/config";
const char* temp_config = "{\"device_class\": \"temperature\", \"name\": \"Plant Temperature\", \"state_topic\": \"homeassistant/sensor/"PLANTGUARDIAN"/state\", \"unit_of_measurement\": \"°C\", \"unique_id\": \"plantguard_temperature\", \"value_template\": \"{{ value_json.temperature}}\" }";

const char* hum_config_topic = "homeassistant/sensor/"PLANTGUARDIAN"/humidity/config";
const char* hum_config = "{\"device_class\": \"humidity\", \"name\": \"Plant Humidity\", \"state_topic\": \"homeassistant/sensor/"PLANTGUARDIAN"/state\", \"unit_of_measurement\": \"%\", \"unique_id\": \"plantguard_humidity\", \"value_template\": \"{{ value_json.humidity}}\" }";

const char* press_config_topic = "homeassistant/sensor/"PLANTGUARDIAN"/pressure/config";
const char* press_config = "{\"device_class\": \"pressure\", \"name\": \"Plant Pressure\", \"state_topic\": \"homeassistant/sensor/"PLANTGUARDIAN"/state\", \"unit_of_measurement\": \"hPa\", \"unique_id\": \"plantguard_pressure\", \"value_template\": \"{{ value_json.pressure}}\" }";

const char* alti_config_topic = "homeassistant/sensor/"PLANTGUARDIAN"/altitude/config";
const char* alti_config = "{ \"name\": \"Plant Altitude\", \"state_topic\": \"homeassistant/sensor/"PLANTGUARDIAN"/state\", \"unit_of_measurement\": \"m\", \"unique_id\": \"plantguard_altitude\", \"value_template\": \"{{ value_json.altitude}}\" }";

const char* illu_config_topic = "homeassistant/sensor/"PLANTGUARDIAN"/illuminance/config";
const char* illu_config = "{\"device_class\": \"illuminance\", \"name\": \"Plant Illuminance\", \"state_topic\": \"homeassistant/sensor/"PLANTGUARDIAN"/state\", \"unit_of_measurement\": \"lx\", \"unique_id\": \"plantguard_illuminance\", \"value_template\": \"{{ value_json.illuminance}}\" }";

const char* water_config_topic = "homeassistant/sensor/"PLANTGUARDIAN"/waterlevel/config";
const char* water_config = "{ \"name\": \"Water level\", \"state_topic\": \"homeassistant/sensor/"PLANTGUARDIAN"/state\", \"unique_id\": \"water_level\", \"value_template\": \"{{ value_json.water_level}}\" }";

// The client id identifies the ESP8266 device. Think of it a bit like a hostname (Or just a name, like Greg).
const char* clientID = "PlantSensors";

// Initialise the WiFi and MQTT Client objects
WiFiClient wifiClient;
PubSubClient client(mqtt_server, 1883, wifiClient); // 1883 is the listener port for the Broker



// Write 16 bit int to RTC memory with checksum, return true if verified OK
// Slot 0-127
// (C) Turo Heikkinen 2019
bool writeRtcMem(uint16_t *inVal, uint8_t slot = 0) {
  uint32_t valToStore = *inVal | ((*inVal ^ 0xffff) << 16); //Add checksum
  uint32_t valFromMemory;
  if (ESP.rtcUserMemoryWrite(slot, &valToStore, sizeof(valToStore)) &&
      ESP.rtcUserMemoryRead(slot, &valFromMemory, sizeof(valFromMemory)) &&
      valToStore == valFromMemory) {
        return true;
  }
  return false;
}

// Read 16 bit int from RTC memory, return true if checksum OK
// Only overwrite variable, if checksum OK
// Slot 0-127
// (C) Turo Heikkinen 2019
bool readRtcMem(uint16_t *inVal, uint8_t slot = 0) {
  uint32_t valFromMemory;
  if (ESP.rtcUserMemoryRead(slot, &valFromMemory, sizeof(valFromMemory)) &&
      ((valFromMemory >> 16) ^ (valFromMemory & 0xffff)) == 0xffff) {
        *inVal = valFromMemory;
        return true;
  }
  return false;
}


/*
** Try to connect to MQTT server
*/
void connect_MQTT(){
  Serial.print("Connecting to ");
  Serial.println(ssid);

  // Connect to the WiFi
  WiFi.begin(ssid, wifi_password);

  // Wait until the connection has been confirmed before continuing
  byte cnt=0;
  while (WiFi.status() != WL_CONNECTED && cnt++<15) {
    delay(500);
    Serial.print(".");
  }

  if(WiFi.status() != WL_CONNECTED){
    error=1;
    exit;
  }
  
  // Debugging - Output the IP Address of the ESP8266
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Connect to MQTT Broker
  // client.connect returns a boolean value to let us know if the connection was successful.
  // If the connection is failing, make sure you are using the correct MQTT Username and Password (Setup Earlier in the Instructable)
  if (client.connect(clientID, mqtt_username, mqtt_password)) {
    Serial.println("Connected to MQTT Broker!");
    client.setBufferSize(1024);
  }
  else {
    Serial.println("Connection to MQTT Broker failed...");
    error=2;
  }
}


/*
** Advertise device to MQTT server for autodiscovery by Home Assistant
*/
void advertise_MQTT(){

  Serial.println("Adv to mqtt");
  Serial.print(plant1_config_topic);
  Serial.print(" ");
  Serial.println(plant1_config);
  if (client.publish(plant1_config_topic, plant1_config)) {
    Serial.println("Adv first moisture sensor");
  }
  if (client.publish(plant2_config_topic, plant2_config)) {
    Serial.println("Adv second moisture sensor");
  }
  client.publish(hum_config_topic,hum_config);
  client.publish(temp_config_topic,temp_config);
  client.publish(press_config_topic,press_config);
  client.publish(illu_config_topic,illu_config);
  client.publish(alti_config_topic,alti_config);
  client.publish(water_config_topic,water_config);
}




void readMoisture() {

  digitalWrite(LED_BUILTIN, LOW);  // Turn the LED on by making the voltage LOW - just to see the sensor is operating

//
// Plant 01
  digitalWrite(Plant01Pin, HIGH);  // Turn the relay ON
  delay(200);

  sensorValue01=0;
  for(int a=0; a<10; a++ ) sensorValue01 += analogRead(MoistPin01);
  sensorValue01=sensorValue01/10;
  
  // map it to the range of the PWM out
  outputValue01 = map(sensorValue01, MoistMin, MoistMax, 0, 100);
  
  // print the readings in the Serial Monitor
  Serial.printf("Plant1 sensor = %d \t output = %d\n", sensorValue01, outputValue01);
  digitalWrite(Plant01Pin, LOW);   // Turn off relay

  delay(200);

//
// Plant 02
  digitalWrite(Plant02Pin, HIGH);  // Turn the relay ON
  delay(200);

  sensorValue02=0;
  for(int a=0; a<10; a++ ) sensorValue02 += analogRead(MoistPin01);
  sensorValue02=sensorValue02/10;
  
  // map it to the range of the PWM out
  outputValue02 = map(sensorValue02, MoistMin, MoistMax, 0, 100);
  
  // print the readings in the Serial Monitor
  Serial.printf("Plant2 sensor = %d \t output = %d\n", sensorValue02, outputValue02);
  digitalWrite(Plant02Pin, LOW);   // Turn off relay

  digitalWrite(LED_BUILTIN, HIGH);
}


void read_sensors(){
  digitalWrite(LED_BUILTIN, LOW);  // Turn the LED on by making the voltage LOW - just to see the sensor is operating

//
// Enable sensors
  digitalWrite(SensorEnaPin, HIGH);  // Turn the relay ON
  delay(200);

// Read water level

// Read temperature, pressure, humidity and light lux
  Wire.begin(I2C_SCL,I2C_SCA);
// Start BME280
  if (bme.begin(0x76, &Wire)) {
       bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X16,  // temperature
                    Adafruit_BME280::SAMPLING_X16,  // pressure
                    Adafruit_BME280::SAMPLING_X16,  // humidity
                    Adafruit_BME280::FILTER_OFF);

    Serial.print("Temperature = ");
    Serial.print(temperatureValue=bme.readTemperature());
    Serial.println(" *C");

    Serial.print("Pressure = ");

    Serial.print(pressureValue=bme.readPressure() / 100.0F);
    Serial.println(" hPa");

    Serial.print("Approx. Altitude = ");
    Serial.print(altitudeValue=bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");

    Serial.print("Humidity = ");
    Serial.print(humidityValue=bme.readHumidity());
    Serial.println(" %");
  }


// Start light sensor
  if (lightMeter.begin(BH1750::ONE_TIME_HIGH_RES_MODE)) {
    delay(200);
    illuValue = lightMeter.readLightLevel();
    Serial.print("Light: ");
    Serial.print(illuValue);
    Serial.println(" lx");
  }

// Start capacitance water sensor (for water tank)
  byte waterTmp=0;
  for(byte a=0; a<5; a++){
    waterTmp+=digitalRead(WaterPin);
    delay(10);
  }
  if(waterTmp>3) waterValue=1;  // If more then 3 check where high = report 1 (there is water)
  else waterValue=0;
  Serial.printf("Water sensor report %d of 5 checks, reporting %d\n",waterTmp,waterValue);

  Serial.println();
  delay(3000);
  digitalWrite(SensorEnaPin, LOW);  // Turn the relay OFF
  digitalWrite(LED_BUILTIN, HIGH);
}


void setup() {
  uint16_t next_boot;

  delay(100);
  Serial.begin(115200);

  pinMode(Plant01Pin, OUTPUT);    // Initialize the transistor pin as an output
  pinMode(Plant02Pin, OUTPUT);
  pinMode(SensorEnaPin, OUTPUT);
  pinMode(WaterPin, INPUT);
  digitalWrite(Plant01Pin, LOW);  // Just in case...
  digitalWrite(Plant02Pin, LOW);
  digitalWrite(SensorEnaPin, LOW);
//  Serial.setTimeout(2000);

  connect_MQTT();

  if(!error){

    // Boot counter and RTC stuff
    //
    if(!readRtcMem(&next_boot)){
      next_boot=1;
      Serial.printf("First boot - no RTC data\n");
    }else Serial.printf("Next boot - read RTC data\n");

    Serial.printf("RTC boot value:%d\n",next_boot);
    if(next_boot==1) advertise_MQTT();
    
    if(next_boot++>20) next_boot=1;
        
    if(!writeRtcMem(&next_boot)) {
         Serial.printf("RTC write failed\n");
    }
    // End of RTC

    
    readMoisture();

    read_sensors();
  
    char msg[140];
    sprintf(msg,"{\"moisture1\":%d,\"moisture2\":%d,\"temperature\":%.2f,\"humidity\":%.1f,\"pressure\":%.1f,\"altitude\":%.1f,\"illuminance\":%.1f,\"water_level\":%d}",outputValue01,outputValue02,temperatureValue,humidityValue,pressureValue,altitudeValue,illuValue,waterValue);
  // PUBLISH to the MQTT Broker
    if (client.publish(plant_topic, msg)) {
      Serial.printf("Sensors data sent as: %s\n",plant_topic);
    }
  
    delay(500);
  }
  Serial.println("Sleeping...");

  digitalWrite(Plant01Pin, LOW);  // Just in case...
  digitalWrite(Plant02Pin, LOW);
  digitalWrite(SensorEnaPin, LOW);
  pinMode(I2C_SCL, OUTPUT);       // without that, those pins are randomly getting high (2,isch volts) during deep sleep :(
  pinMode(I2C_SCA, OUTPUT);
  digitalWrite(I2C_SCL, LOW);
  digitalWrite(I2C_SCA, LOW);

  //ESP.deepSleep(30e6);
  //ESP.deepSleep(60e6);
  ESP.deepSleep(120e6);
  //ESP.deepSleep(360e6);
}

void loop() {
} 
