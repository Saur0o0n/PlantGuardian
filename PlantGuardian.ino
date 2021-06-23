/*
** PlantGuardian by Sauron 2020-2021
** Last update: 23.06.2021
*/
#include "ESP8266WiFi.h" // Enables the ESP8266 to connect to the local network (via WiFi)
#include "PubSubClient.h" // Allows us to connect to, and publish to the MQTT broker
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_BMP280.h>
#include <BH1750.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>
#include <ADS1115_WE.h>

#define PLANTGUARDIAN "plantguardian"
#define PGVer "v1.1"
#define I2C_ADDRESS 0x48

// Moisture stuff
const int MoistPin01 = A0;  // ESP8266 Analog Pin ADC0 = A0
const int Plant01Pin = D8;
const int Plant02Pin = D7;

const int MoistMax = 1024;
const int MoistMin = 1;

// Other sensors
const int SensorEnaPin = D1; // This ping enables transistor to all other sensors (water level, temperature, humidity, pressure etc)
const int WaterEnaPin = D2;  // Pin that enables water level transistor and step up board
const int WaterPin = D3;     // Water level pin
const int I2C_SCL = D5;
const int I2C_SCA = D6;

#define SEALEVELPRESSURE_HPA (1013.25)

BH1750 lightMeter(0x23);
ADS1115_WE adc(I2C_ADDRESS);

// WiFi
const char* ssid = "SauronIoT";
const char* wifi_password = "Dupa123123";

const char* PG_UpdateFile = "http://adrian.siemieniak.net/Rozne/PlantGuardian"PGVer".ino.bin";

int sensorValue01 = 0;  // value read from the pot
int outputValue01 = 0;  // value to output to a PWM pin
int sensorValue02 = 0;  // value read from the pot
int outputValue02 = 0;  // value to output to a PWM pin
float temperatureValue = 0;
float humidityValue = 0;
float pressureValue = 0;
float illuValue = 0;
float altitudeValue = 0;
float bat_volt = 0, sol_volt = 0;

bool waterValue = 1;    // 1 = full, 0 = empty
byte error = 0;


// MQTT
// Make sure to update this for your own MQTT Broker!
const char* mqtt_server = "192.168.70.100";
const char* plant_topic = "homeassistant/sensor/"PLANTGUARDIAN"/state";
const char* mqtt_username = "saurons_iot";
const char* mqtt_password = "heeZeel8ohkio9eidaiRoo";

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

const char* voltage_bat_config_topic = "homeassistant/sensor/"PLANTGUARDIAN"/voltage_battery/config";
const char* voltage_bat_config = "{\"device_class\": \"voltage\", \"name\": \"Battery Voltage\", \"state_topic\": \"homeassistant/sensor/"PLANTGUARDIAN"/state\", \"unit_of_measurement\": \"V\", \"unique_id\": \"voltage_battery\", \"value_template\": \"{{ value_json.voltage_battery}}\" }";

const char* voltage_sol_config_topic = "homeassistant/sensor/"PLANTGUARDIAN"/voltage_solar/config";
const char* voltage_sol_config = "{\"device_class\": \"voltage\", \"name\": \"Solar Voltage\", \"state_topic\": \"homeassistant/sensor/"PLANTGUARDIAN"/state\", \"unit_of_measurement\": \"V\", \"unique_id\": \"voltage_solar\", \"value_template\": \"{{ value_json.voltage_solar}}\" }";

// The client id identifies the ESP8266 device. Think of it a bit like a hostname (Or just a name, like Greg).
const char* clientID = "PlantSensors";

// Initialise the WiFi and MQTT Client objects
WiFiClient wifiClient;
PubSubClient client(mqtt_server, 1883, wifiClient); // 1883 is the listener port for the Broker

//Static IP address configuration
IPAddress staticIP(192, 168, 70, 34); //ESP static ip
IPAddress gateway(192, 168, 70, 100); //IP Address of your WiFi Router (Gateway)
IPAddress subnet(255, 255, 255, 0);   //Subnet mask
IPAddress dns(192, 168, 70, 100);     //DNS
 
 
const char* deviceName = "PlantGuardian";



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

  WiFi.hostname(deviceName);      // DHCP Hostname (useful for finding device for static lease)
  WiFi.config(staticIP, subnet, gateway, dns);

  // Connect to the WiFi
  WiFi.begin(ssid, wifi_password);

  // Wait until the connection has been confirmed before continuing
  byte cnt=0;
  while (WiFi.status() != WL_CONNECTED && cnt++<15) {
    delay(1000);
    Serial.print(".");
  }

  if(WiFi.status() != WL_CONNECTED){
    error=1;
    exit;
  }
  
  // Debugging - Output the IP Address of the ESP8266
  Serial.println("WiFi connected");
  Serial.print("MAC: ");
  Serial.println(WiFi.macAddress());
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
  client.publish(voltage_bat_config_topic,voltage_bat_config);
  client.publish(voltage_sol_config_topic,voltage_sol_config);
    
}


/*
** 
*/
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


/*
** 
*/
float readChannel(ADS1115_MUX channel) {
  float voltage = 0.0;
  adc.setCompareChannels(channel);
  adc.startSingleMeasurement();
  while(adc.isBusy()){}
  voltage = adc.getResult_V(); // alternative: getResult_mV for Millivolt
  return voltage;
}


/*
** 
*/
void i2c_scanner(){
  byte error, address;
  int nDevices;

  Serial.println("Scanning for i2c devices...");
  
    nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknown error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}


/*
** Read poor version of sensor (without humidity)
*/
void read_bmp(){
  // Start BMP280
  Adafruit_BMP280 bmp; // I2C
  
  if (bmp.begin(0x76)) {
    /* Default settings from datasheet. */
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

    Serial.print("Temperature = ");
    Serial.print(temperatureValue=bmp.readTemperature());
    Serial.println(" *C");

    Serial.print("Pressure = ");

    Serial.print(pressureValue=bmp.readPressure() / 100.0F);
    Serial.println(" hPa");

    Serial.print("Approx. Altitude = ");
    Serial.print(altitudeValue=bmp.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");

    Serial.print("Humidity = ");
    Serial.print(humidityValue=0);
    Serial.println(" %");
  }
}

/*
** Read reacher version of sensor (with humidity)
*/
void read_bme(){
  // Start BME280
  Adafruit_BME280 bme; // I2C

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

}

/*
** 
*/
void read_sensors(){
  digitalWrite(LED_BUILTIN, LOW);  // Turn the LED on by making the voltage LOW - just to see the sensor is operating

//
// Enable sensors
  digitalWrite(SensorEnaPin, HIGH);  // Turn the relay ON
  delay(200);

  Serial.println("Trying to read outside sensors...");
// Read temperature, pressure, humidity and light lux
  Wire.begin(I2C_SCL,I2C_SCA);

  read_bmp();
// Start light sensor
  if (lightMeter.begin(BH1750::ONE_TIME_HIGH_RES_MODE)) {
    delay(200);
    illuValue = lightMeter.readLightLevel();
    Serial.print("Light: ");
    Serial.print(illuValue);
    Serial.println(" lx");
  }

// Detect voltage
  if(!adc.init()){
    Serial.println("ADS1115 not connected!");
  }else{
    adc.setVoltageRange_mV(ADS1115_RANGE_6144);
    bat_volt = readChannel(ADS1115_COMP_0_GND);
    sol_volt = readChannel(ADS1115_COMP_1_GND);
    Serial.printf(" Battery voltage: %f,  solar voltage: %f\n",bat_volt, sol_volt);
  }

  delay(3000);
  digitalWrite(SensorEnaPin, LOW);  // Turn the sensor relay OFF
  
  Serial.println("Read outside sensors ends now.");



// Read water level
// Enable water sensor
  digitalWrite(WaterEnaPin, HIGH);  // Turn the relay ON
  delay(500);
// Start capacitance water sensor (for water tank)
  byte waterTmp=0;
  for(byte a=0; a<5; a++){
    waterTmp+=digitalRead(WaterPin);
    delay(20);
  }
  if(waterTmp>3) waterValue=1;  // If more then 3 check where high = report 1 (there is water)
  else waterValue=0;
  Serial.printf("Water sensor report %d of 5 checks, reporting %d\n",waterTmp,waterValue);

  Serial.println();
  delay(200);
  digitalWrite(WaterEnaPin, LOW);  // Turn the Water relay OFF
  delay(1000);  // just to see the led light on
  digitalWrite(LED_BUILTIN, HIGH);
}

void WiFi_update(){
  WiFiClient client;

  Serial.printf("Trying to download update file: %s\n",PG_UpdateFile);
  t_httpUpdate_return ret = ESPhttpUpdate.update(client, PG_UpdateFile);
  //t_httpUpdate_return ret = ESPhttpUpdate.update(client, "server", 80, "file.bin");
  
  switch (ret) {
      case HTTP_UPDATE_FAILED:
        Serial.printf("HTTP_UPDATE_FAILD Error (%d): %s\n", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
        break;

      case HTTP_UPDATE_NO_UPDATES:
        Serial.println("HTTP_UPDATE_NO_UPDATES");
        break;

      case HTTP_UPDATE_OK:
        Serial.println("HTTP_UPDATE_OK");
        break;
    }
}


void setup() {
  uint16_t next_boot;

  delay(100);
  Serial.begin(115200);

  pinMode(Plant01Pin, OUTPUT);    // Initialize the transistor pin as an output
  pinMode(Plant02Pin, OUTPUT);
  pinMode(SensorEnaPin, OUTPUT);
  pinMode(WaterEnaPin, OUTPUT);
  pinMode(WaterPin, INPUT);
  digitalWrite(Plant01Pin, LOW);  // Just in case...
  digitalWrite(Plant02Pin, LOW);
  digitalWrite(SensorEnaPin, LOW);
  digitalWrite(WaterEnaPin, LOW);
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
    if(next_boot==1){
      advertise_MQTT();
      WiFi_update();  // check if there is an update
    }
    
    if(next_boot++>20) next_boot=1;
        
    if(!writeRtcMem(&next_boot)) {
         Serial.printf("RTC write failed\n");
    }
    // End of RTC

    readMoisture();

    read_sensors();

//    i2c_scanner();  // for debuging
  
    char msg[220];
    sprintf(msg,"{\"moisture1\":%d,\"moisture2\":%d,\"temperature\":%.2f,\"humidity\":%.1f,\"pressure\":%.1f,\"altitude\":%.1f,\"illuminance\":%.1f,\"water_level\":%d,\"voltage_battery\":%0.2f,\"voltage_solar\":%0.2f}",outputValue01,outputValue02,temperatureValue,humidityValue,pressureValue,altitudeValue,illuValue,waterValue,bat_volt,sol_volt);
  // PUBLISH to the MQTT Broker
    if (client.publish(plant_topic, msg)) {
      Serial.printf("Sensors data sent as: %s\n",plant_topic);
    }
  
    delay(500);
  }
  Serial.println("Sleeping...");
  Serial.end();

  digitalWrite(Plant01Pin, LOW);  // Just in case...
  digitalWrite(Plant02Pin, LOW);
  digitalWrite(SensorEnaPin, LOW);
  pinMode(I2C_SCL, OUTPUT);       // without that, those pins are randomly getting high (2,isch volts) during deep sleep :(
  pinMode(I2C_SCA, OUTPUT);
  digitalWrite(I2C_SCL, LOW);
  digitalWrite(I2C_SCA, LOW);

  Serial.flush();   // no deep sleep without this on some boards
  
  //ESP.deepSleep(30e6);
  //ESP.deepSleep(60e6);
  //ESP.deepSleep(120e6);
  ESP.deepSleep(360e6);
}

void loop() {
} 
