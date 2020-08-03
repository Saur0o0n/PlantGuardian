#include "ESP8266WiFi.h" // Enables the ESP8266 to connect to the local network (via WiFi)
#include "PubSubClient.h" // Allows us to connect to, and publish to the MQTT broker
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <BH1750.h>

#define PLANTGUARDIAN "plantguardian01"

// Moisture stuff
const int MoistPin01 = A0;  // ESP8266 Analog Pin ADC0 = A0
const int Plant01Pin = D8;
const int Plant02Pin = D7;

const int MoistMax = 700;
const int MoistMin = 1;

// Other sensors
const int SensorEnaPin = D3; // This ping enables transistor to all other sensors (water level, temperature, humidity, pressure etc)
const int WaterPin = D1;     // Water level pin
const int I2C_SCL = D6;
const int I2C_SCA = D5;

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C
BH1750 lightMeter(0x23);


// WiFi
const char* ssid = "WiFi";
const char* wifi_password = "Pass";

int sensorValue01 = 0;  // value read from the pot
int outputValue01 = 0;  // value to output to a PWM pin
int sensorValue02 = 0;  // value read from the pot
int outputValue02 = 0;  // value to output to a PWM pin
byte error = 0;


// MQTT
// Make sure to update this for your own MQTT Broker!
const char* mqtt_server = "192.168.70.100";
const char* plant_topic = "homeassistant/sensor/"PLANTGUARDIAN"/state";
const char* plant1_config_topic = "homeassistant/sensor/"PLANTGUARDIAN"/plant1/config";
const char* plant2_config_topic = "homeassistant/sensor/"PLANTGUARDIAN"/plant2/config";
const char* plant1_config = "{\"device_class\": \"humidity\", \"name\": \"Plant 1 moisture\", \"state_topic\": \"homeassistant/sensor/"PLANTGUARDIAN"/state\", \"unit_of_measurement\": \"%\", \"unique_id\": \"plantguard_sensor_01\", \"value_template\": \"{{ value_json.moisture1}}\" }";
const char* plant2_config = "{\"device_class\": \"humidity\", \"name\": \"Plant 2 moisture\", \"state_topic\": \"homeassistant/sensor/"PLANTGUARDIAN"/state\", \"unit_of_measurement\": \"%\", \"unique_id\": \"plantguard_sensor_02\", \"value_template\": \"{{ value_json.moisture2}}\" }";

const char* mqtt_username = "mqttname";
const char* mqtt_password = "mqttpass";
// The client id identifies the ESP8266 device. Think of it a bit like a hostname (Or just a name, like Greg).
const char* clientID = "PlantSensors";

// Initialise the WiFi and MQTT Client objects
WiFiClient wifiClient;
PubSubClient client(mqtt_server, 1883, wifiClient); // 1883 is the listener port for the Broker


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
  while (WiFi.status() != WL_CONNECTED && cnt++<10) {
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
    Serial.println("Adv first sensor");
  }
  if (client.publish(plant2_config_topic, plant2_config)) {
    Serial.println("Adv second sensor");
  }
}


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


void readMoisture() {

  digitalWrite(LED_BUILTIN, LOW);  // Turn the LED off by making the voltage LOW - just to see the sensor is operating

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
  digitalWrite(LED_BUILTIN, LOW);  // Turn the LED off by making the voltage LOW - just to see the sensor is operating

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
    Serial.print(bme.readTemperature());
    Serial.println(" *C");

    Serial.print("Pressure = ");

    Serial.print(bme.readPressure() / 100.0F);
    Serial.println(" hPa");

    Serial.print("Approx. Altitude = ");
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");

    Serial.print("Humidity = ");
    Serial.print(bme.readHumidity());
    Serial.println(" %");

    Serial.println();
  }
// Start light sensor
  if (lightMeter.begin(BH1750::ONE_TIME_HIGH_RES_MODE)) {
    Serial.println(F("BH1750 Advanced begin"));
    float lux = lightMeter.readLightLevel();
    Serial.print("Light: ");
    Serial.print(lux);
    Serial.println(" lx");
  }

  digitalWrite(SensorEnaPin, LOW);  // Turn the relay OFF
  digitalWrite(LED_BUILTIN, HIGH);
}


void setup() {
  uint16_t next_boot;

  delay(1000);
  Serial.begin(115200);

  pinMode(Plant01Pin, OUTPUT);    // Initialize the transistor pin as an output
  digitalWrite(Plant01Pin, LOW);  // Just in case...
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
  
    char msg[20];
    sprintf(msg,"{\"moisture1\":%d,\"moisture2\":%d}",outputValue01,outputValue02);
  // PUBLISH to the MQTT Broker
    if (client.publish(plant_topic, msg)) {
      Serial.printf("Moisture sent as: %s\n",plant_topic);
    }
  
    delay(500);
  }
  //ESP.deepSleep(60e6);
  Serial.println("Sleeping...");
  ESP.deepSleep(30e6);
}

void loop() {
} 
