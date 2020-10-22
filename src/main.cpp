#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
//#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include "esp32-hal-ledc.h"
#include <driver/adc.h>
#include "esp_adc_cal.h"
#include <list>
#include <iterator>

const int ADXL_DEVICE = 0x53;
const int DATA_REG = 0x32;
const unsigned int TO_READ = 6;

const float g = 9.8;
const float ACCELERATION_THRESHOLD_IN_G = 2.97;
// const int SQUARE_THRESHOLD = 400;
const int SERIES_INTERVAL = 700;

const int m = 2;

const unsigned int BLUE_LED = 13; // 13
const unsigned int RED_LED = 15; // 15

const unsigned int SLEEP_MODE_PIN = 2; // 2

const unsigned int SDA_PIN = 5; // 5
const unsigned int SCL_PIN = 4; // 4

const unsigned int BATTERY_LEVEL_PIN = 32; //32

const float BATTERY_MAX_VOLTAGE = 4.08f;
const float BATTERY_MIN_VOLTAGE = 3.5f;
const float SHUTDOWN_VOLTAGE = 3.45f;

float vRef = 1.9f;

const unsigned int sleepDelay = 3000;

int bagWeight = 50;
int threshold  = 160;

//const char *BATTERY_SERVICE_UUID = "1c3db28e-cb5d-11ea-87d0-0242ac130003";


const char *BOXPROJECT_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";

const char *BATTERY_LEVEL_CHAR_UUID = "C821907A-CB5D-11EA-87D0-0242AC130003";
const char *TRAINING_DATA_CHAR_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"; 
const char *TRAINING_SETTING_CHAR_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";

const std::__cxx11::string DEVICE_NAME = "UART Service";

BLEServer *pServer;
BLECharacteristic batteryChar(BATTERY_LEVEL_CHAR_UUID, BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic dataChar(TRAINING_DATA_CHAR_UUID, BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic settingChar(TRAINING_SETTING_CHAR_UUID, BLECharacteristic::PROPERTY_WRITE);

int batteryLevel = 100;
float batteryVoltage = BATTERY_MAX_VOLTAGE;

bool deviceConnected = false;
bool trainingState = false;

const unsigned int TRAINING_BUFF_SIZE = 500;

TaskHandle_t sleepMode;
TaskHandle_t ledI;
TaskHandle_t battery;
TaskHandle_t ble_Serial;

int indicationMode = 0;
bool iModeChanged = false;
bool batteryState = true;

void writeTo(int device, byte address, byte val) {
    Wire.beginTransmission(device);
    Wire.write(address);
    Wire.write(val);
    Wire.endTransmission();
}
void readFrom(int device, byte address, int num, byte buff[]) {
    Wire.beginTransmission(device);
    Wire.write(address);
    Wire.endTransmission();

    Wire.beginTransmission(device);
    Wire.requestFrom(device, num);
    int i = 0;
    while (Wire.available())
    {
        buff[i] = Wire.read();
        i++;
    }
}

float coeffX = 39.0f;
float coeffY = 38.0f;
float coeffZ = 37.0f;
int offsetX = -2;
int offsetY = -20;
int offsetZ = 136;

float readAcceleration(int samplesNumber) {
    float mod=0;
    for(int i = 0; i < samplesNumber; i++)
    {
        byte buff[TO_READ];
        readFrom(ADXL_DEVICE, DATA_REG, TO_READ, buff);
        short x = (((((int)buff[1]) << 8) | buff[0]) - offsetX);
        short y = (((((int)buff[3]) << 8) | buff[2]) - offsetY);
        short z = (((((int)buff[5]) << 8) | buff[4]) - offsetZ);
        mod += sqrt((float)((float)(x*x)/coeffX/coeffX + (float)(y*y)/coeffY/coeffY + (float)(z*z)/coeffZ/coeffZ));
    }
    return mod / (float)samplesNumber;
}

void adxlMeasure()
{
    writeTo(ADXL_DEVICE, 0x2D, 0);
    writeTo(ADXL_DEVICE, 0x2D, 16);
    writeTo(ADXL_DEVICE, 0x2D, 8);
    writeTo(ADXL_DEVICE, 0x2C, 14); // Normal mode, 1600 Hz, 90 mA
    writeTo(ADXL_DEVICE, 0x31, 3); // 16g
}
void adxlSleep()
{
    writeTo(ADXL_DEVICE, 0x2D, 4);
}
void adxlCalibrate(){

}

void setIndicationMode(int mode, bool batSt){
  if(mode != indicationMode)
  {
    iModeChanged = true;
    indicationMode = mode;
  }
  if(batSt){
    ledcDetachPin(RED_LED);
    ledcAttachPin(BLUE_LED, 0);
    batteryState = true;
  }
  else{
    ledcAttachPin(RED_LED, 0);
    ledcDetachPin(BLUE_LED);
    batteryState = false;
  }
}
void ledIndicate(void * parameter){

  while(true){
    if(iModeChanged){
      ledcWrite(0,0);
      delay(300);
      for(int i=0; i<2; i++){
        ledcWrite(0,255);
        delay(50);
        ledcWrite(0,0);
        delay(180);
      }
      delay(300);
      iModeChanged = false;
    }
    switch (indicationMode)
    {
      case 0: //Device not connected
      {
        for(float c = 0; c <= 30; c++)
        {
          ledcWrite(0,fabs(sin(c*3.14/30))*255);
          if(iModeChanged)break;
          delay(20);
        }
        unsigned long seriesTimer = millis();
        while((millis() - seriesTimer) < 3000){
          if(iModeChanged)break;
          delay(50);
        }
        break;
      }
      case 1: // Device connected
      {
        for(float c = 0; c <= 230; c++)
        {
          ledcWrite(0,fabs(sin(c*3.14/230))*255);
          if(iModeChanged)break;
          delay(20);
        }
        unsigned long t2 = millis();
        while((millis() - t2) < 8000){
          if(iModeChanged)break;
          delay(50);
        }
        break;
      }
    }
  }
}

float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
  if(x < in_min) return out_min;
  else if(x > in_max) return out_max;
  else return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float readVoltage(int pin){
  int c = 0;
  float average = 0;
  for(int t = millis(); millis() - t < 50; c++, average += analogRead(pin));
  // Serial.println(average / (float)c);
  // Serial.println(vRef);
  // Serial.println((average / (float)c / 2047.0f * (vRef - 0.1f)) + 0.1f);
  return (average / (float)c / 2047.0f * (vRef - 0.1f)) + 0.1f;
}

const float R1 = 47.0f; //47.0f
const float R2 = 33.0f; //33.0f
const unsigned int BATTERY_DELAY = 5000;
void batteryMeasurement( void * parameter) {
  while(true)
  {
    delay(100);
    batteryVoltage = readVoltage(BATTERY_LEVEL_PIN) / R2 * (R1 + R2);
    batteryLevel = (int)fmap(batteryVoltage, BATTERY_MIN_VOLTAGE, BATTERY_MAX_VOLTAGE, 0, 100.0f);
    if(batteryLevel <= 20)
      setIndicationMode(indicationMode, false);
    else setIndicationMode(indicationMode, true);
    delay(BATTERY_DELAY);
    if(deviceConnected){
      uint8_t batteryValue[2] = {0};
      batteryValue[0] = lowByte(batteryLevel);
      batteryValue[1] = highByte(batteryLevel);
      batteryChar.setValue(batteryValue, 2);
      batteryChar.notify();
      delay(5000);
    }   
  }
}

void goToSleep(){
  vTaskDelete(ledI);
  ledcAttachPin(BLUE_LED, 0);
  ledcAttachPin(RED_LED, 0);
  ledcWrite(0,0);
  delay(200);
  for(int i=0; i<2; i++){
    ledcWrite(0,255);
    delay(50);
    ledcWrite(0,0);
    delay(180);
  }
  adxlSleep();
  esp_deep_sleep_start();
}
void sleep( void * parameter) {
  while(true)
  {
    if(batteryVoltage <= SHUTDOWN_VOLTAGE){
      goToSleep();
      delay(100);
    }
    else{
      int t1 = millis();
      while(digitalRead(SLEEP_MODE_PIN))
      {
        if(millis()-t1 > sleepDelay){
          goToSleep();
        }
        delay(50);
      }
      delay(100);
    }
  }
}
void wakeUp(int wakeUpReason) {
  if(wakeUpReason == 0)
    return;
  else
  {
    long t1 = millis();
    while(digitalRead(SLEEP_MODE_PIN))
    {
      if(millis()-t1 > sleepDelay){
        ledcAttachPin(BLUE_LED, 0);
        ledcAttachPin(RED_LED, 0);
        for(int i=0; i<2; i++){
          ledcWrite(0,255);
          delay(50);
          ledcWrite(0,0);
          delay(180);
        }
        // Serial.println(F("Yeah boy, ama wakeup"));
        return;
      }
    }
    esp_deep_sleep_start();
  }
}

std::list<uint8_t *> dataList;
std::list<uint8_t *> notifyList;

void startTraining(uint8_t *val);
void stopTraining();

void bleSerial( void * parameter){
  while(true){
    int dataListSize = dataList.size();
    int notifyListSize = notifyList.size();
    if(dataListSize > TRAINING_BUFF_SIZE) stopTraining();
    for(int i = 0; i < dataListSize && trainingState && deviceConnected; i++){
      std::list<uint8_t *>::iterator it = dataList.begin();
      std::advance(it, 0);

      dataChar.setValue(*it, 5);
      dataChar.notify();
      delete[] *it;
      dataList.pop_front();

      // Serial.println(F("Data send"));
      delay(20);
    }
    for(int i = 0; i < notifyListSize && deviceConnected; i++){
      std::list<uint8_t *>::iterator it = notifyList.begin();
      std::advance(it, 0);

      dataChar.setValue(*it, 1);
      dataChar.notify();
      delete[] *it;
      notifyList.pop_front();

      // Serial.println(F("Notification send"));
      delay(20);
    }
    delay(30);
  }
}
void sendData(uint8_t extra, int first, int second){
  uint8_t *txValue = new uint8_t[5];
  txValue[0] = extra;
  txValue[1] = lowByte(first);
  txValue[2] = highByte(first);
  txValue[3] = lowByte(second);
  txValue[4] = highByte(second);
  dataList.push_back(txValue);
  // Serial.println(F("Data added"));
}
void sendNotification(uint16_t extra){
  uint8_t *txValue = new uint8_t[1];
  txValue[0] = extra;
  notifyList.push_back(txValue);
  // Serial.println(F("Notification added"));
}
// bool tr = false;
class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    delay(2000);
    sendNotification(1);
    deviceConnected = true;
    setIndicationMode(1, batteryState);
    // Serial.println(F("Device connection: true"));
  };
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    setIndicationMode(0, batteryState);
    // Serial.println(F("Device connection: false"));
  }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pChar) {
      // std::string rxValue = pChar->getValue();
      uint8_t* rxValue = pChar->getData();
      switch (rxValue[0]) {
        case 0: //stop
          stopTraining();
        break;
        case 1: //start
          startTraining(rxValue);
        break;
      }
    }
};

static esp_adc_cal_characteristics_t *adc_chars;
void adcSetup(){
  pinMode(BATTERY_LEVEL_PIN, INPUT);

  analogReadResolution(11);
  analogSetAttenuation(ADC_6db);


  adc_chars = (esp_adc_cal_characteristics_t *)calloc(1, sizeof(esp_adc_cal_characteristics_t));
  /*esp_adc_cal_value_t val_type = */esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_6, ADC_WIDTH_BIT_11, ESP_ADC_CAL_VAL_DEFAULT_VREF, adc_chars);
  vRef = vRef *(0.5f + (float)adc_chars->vref / 2200.0f);
  
  delay(200);

  // int min = 2047;
  // int max = 0;
  // for(int i = 0; i < 10000; i++){
  //   int adc = analogRead(BATTERY_LEVEL_PIN);
  //   if(adc > max) max = adc;
  //   if(adc < min) min = adc;
  // }
  // // Serial.println(analogRead(BATTERY_LEVEL_PIN));
  // Serial.println(max);
  // Serial.println(min);
  // Serial.println(max - min);

  // Serial.println("ADC number:\t" + String(adc_chars->adc_num));
  // Serial.println("ADC coeff_a:\t" + String(adc_chars->coeff_a));
  // Serial.println("ADC coeff_b:\t" + String(adc_chars->coeff_b));
  // Serial.println("ADC VRef:\t" + String(adc_chars->vref));
}

void setup() {
  pinMode(BLUE_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(SLEEP_MODE_PIN, INPUT);
  ledcSetup(0,500,8);
  //**************** Sleep mode part
  Serial.begin(115200);
  esp_sleep_enable_ext0_wakeup(gpio_num_t(SLEEP_MODE_PIN), 1);
  wakeUp(esp_sleep_get_wakeup_cause());
  //**************** Sleep mode part
  Wire.begin(SDA_PIN, SCL_PIN);

  ledcAttachPin(BLUE_LED, 0);
  ledcDetachPin(RED_LED);

  BLEDevice::init(DEVICE_NAME);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pBoxProjectService = pServer->createService(BOXPROJECT_SERVICE_UUID);
  pBoxProjectService->addCharacteristic(&batteryChar);
  pBoxProjectService->addCharacteristic(&dataChar);
  pBoxProjectService->addCharacteristic(&settingChar);

  batteryChar.addDescriptor(new BLE2902());
  dataChar.addDescriptor(new BLE2902());
  settingChar.setCallbacks(new MyCallbacks());

  pBoxProjectService->start();
  pServer->getAdvertising()->start();

  adcSetup();
  adxlMeasure();
  adxlSleep();

  xTaskCreatePinnedToCore(batteryMeasurement, "BatteryLevel", 10000, NULL, 1, &battery, 1);
  xTaskCreatePinnedToCore(sleep, "SleepMode", 10000, NULL, 1, &sleepMode, 1);
  xTaskCreatePinnedToCore(ledIndicate, "LedIndicate", 10000, NULL, 1, &ledI, 1);
  xTaskCreatePinnedToCore(bleSerial, "BleSerial", 10000, NULL, 1, &ble_Serial, 0);
  delay(500);
}

float MaxFloat(float array[], int length){
  float max = 0;
  for(int i = 0; i < length; i++)
    if(max < array[i]) max = array[i];
    return max;
}

int hitForce(float accInG) {
    return accInG * (float)bagWeight;
}

unsigned long seriesTimer = 0;
unsigned int seriesHits = 0;

int hitTimer = 0;

unsigned int seriesNumber = 0;
unsigned int hits = 0;

void loop() {
  if(trainingState){
    float acceleration = readAcceleration(1);
    if(acceleration >= ACCELERATION_THRESHOLD_IN_G){
      float array[100] = {0};
      seriesTimer = millis();
      array[0] = acceleration;
      for(int i = 1; i < 100; i++){
        array[i] = readAcceleration(1);
      }
      float max = MaxFloat(array, 100);
      int force = hitForce(max);
      if(force > threshold){
        hits ++;
        sendData(1, force, hits);
        seriesHits ++;
      }
    }
    if(seriesHits > 1){
      if(millis() - seriesTimer > SERIES_INTERVAL){
        seriesNumber++;
        sendData(2, seriesHits, seriesNumber);
        seriesHits = 0;
       }
    }
    else if(seriesHits == 1){
        if(millis() - seriesTimer > SERIES_INTERVAL) seriesHits = 0;
    }
  }
  // // Serial.println(ESP.getFreeHeap());
  // // delay(1000);
}

void startTraining(uint8_t *val){
  adxlMeasure();
  bagWeight = (val[1] | val[2]<<8);
  threshold = (val[3] | val[4]<<8);
  // Serial.print(F("Bag weight: "));
  // Serial.print(bagWeight);
  // Serial.print("\t");
  // Serial.print(F("Threshold: "));
  // Serial.println(threshold);
  seriesHits = 0;
  seriesNumber = 0;
  hits = 0;
  seriesTimer = 0;
  trainingState = true;
}
void stopTraining(){
  trainingState = false;
  seriesHits = 0;
  seriesNumber = 0;
  hits = 0;
  seriesTimer = 0;
  adxlSleep();
  int dataListSize = dataList.size();
  int notifyListSize = notifyList.size();
  for(int i = 0; i < dataListSize; i++){
    std::list<uint8_t *>::iterator it = dataList.begin();
    std::advance(it, 0);
    delete[] *it;
    dataList.pop_front();
  }
  for(int i = 0; i < notifyListSize; i++){
    std::list<uint8_t *>::iterator it = notifyList.begin();
    std::advance(it, 0);
    delete[] *it;
    notifyList.pop_front();
  }
  // Serial.println(F("Training stopped"));
}

