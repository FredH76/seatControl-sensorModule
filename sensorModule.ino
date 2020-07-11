////////////////////////////////////////////////////////////////////////////////////////////////
//    This code is for ESP 32 card ( <=> #if defined(ESP_H))
//    and make use of BluetoothSerial.h
//    otherwise you shoudh use SoftwareSerial.h + update begin() method accrodingly
////////////////////////////////////////////////////////////////////////////////////////////////

#include <SPI.h>
#include <Wire.h>
//#include "logo.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "esp_system.h"
//#include "BluetoothSerial.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <EEPROM.h>

// BLE configuration
static BLEUUID SERVICE_UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E"); // UART service UUID
static BLEUUID CHARACTERISTIC_UUID_RX("6E400002-B5A3-F393-E0A9-E50E24DCCA9E");
static BLEUUID CHARACTERISTIC_UUID_TX("6E400003-B5A3-F393-E0A9-E50E24DCCA9E");
BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic;
BLECharacteristic *pRxCharacteristic;
uint8_t txValue = 0;
boolean joystickConnected = false;

// OLED configuration
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// define the number of bytes to use
#define EEPROM_SIZE 32

////////////////////////////////////   APP CONSTANT and ENUM   /////////////////////////////////
const int SR04T_DISCONNECTED = -1; // UltraSonar Sensor disconnected
enum SIDE
{ // use bit identification (up to 128)
  LEFT = 1,
  FRONT_L = 2,
  FRONT_R = 4,
  RIGHT = 8
};

////////////////////////////////////      PIN CONFIGURATION    /////////////////////////////////
// DIGITAL
const int trigLEFT = 12;
const int echoLEFT = 14;
const int trigFRONT_L = 27;
const int echoFRONT_L = 26;
const int trigFRONT_R = 25;
const int echoFRONT_R = 33;
const int trigRIGHT = 32;
const int echoRIGHT = 35;

const int buzzerPlus = 18;
const int buzzerGround = 19;
const int led = 16;

// ANALOG
// none for now ...

////////////////////////////////////      COMMAND HEX CODE      /////////////////////////////////
const int CONFIG_STOP_DURATION = 0xC1;
const int EMERGENCY_STOP = 0xE0;

////////////////////////////////////      VAR DECLARATION      /////////////////////////////////
//  EEPROM adress to save configuration param
int eeAdr_stopDuration = 0;
int eeAdr_sensorFlags = 2;
int eeAdr_left_Threshold = 10;
int eeAdr_frontL_Threshold = 11;
int eeAdr_frontR_Threshold = 12;
int eeAdr_right_Threshold = 13;

//  GENERAL
int eeVal;             // value read in EEPROM
int stopDuration;      // time before releasing control to joystick (in ms)
int sensorFlags = 255; // LEFT < FRONT_L < FRONT_R < RIGHT < ...
int left_Threshold;
int frontL_Threshold;
int frontR_Threshold;
int right_Threshold;
int buzzFreq; // frequency (in Hz)
int btByte;   // store byte to be read from BLE serial

/***********************************************************************************************
 * setUp()
***********************************************************************************************/
void setup()
{
  // start Serial for debbuging
  Serial.begin(9600);

  // configure pin INPUT/OUTPUT
  pinMode(trigLEFT, OUTPUT);
  pinMode(echoLEFT, INPUT);
  pinMode(trigFRONT_L, OUTPUT);
  pinMode(echoFRONT_L, INPUT);
  pinMode(trigFRONT_R, OUTPUT);
  pinMode(echoFRONT_R, INPUT);
  pinMode(trigRIGHT, OUTPUT);
  pinMode(echoRIGHT, INPUT);

  pinMode(buzzerPlus, OUTPUT);
  pinMode(buzzerGround, OUTPUT);
  pinMode(led, OUTPUT);
 
  // LED OFF
  digitalWrite(led, HIGH);

  // init OLED display
  Wire.begin(5, 4); // overwrite default SDA, SLC configuration to fit ESP 32
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  splashScreen(); // display welcoming animation

  // start Scan for joystick BLE connection
  Serial.println(F("initialize BLE connection ..."));
  initBLE();

  // set BUZZER Frequence
  buzzFreq = 6000; // frequency (in Hz)

  // initialize EEPROM with predefined size
  EEPROM.begin(EEPROM_SIZE);

  // LOAD TEST DATA into EEPROM //
  EEPROM.write(eeAdr_sensorFlags, LEFT | FRONT_L | FRONT_R | RIGHT); // LEFT | FRONT_L | FRONT_R | RIGHT
  EEPROM.write(eeAdr_left_Threshold, 30);
  EEPROM.write(eeAdr_frontL_Threshold, 30);
  EEPROM.write(eeAdr_frontR_Threshold, 30);
  EEPROM.write(eeAdr_right_Threshold, 30);
  EEPROM.commit();

  // load CONFIGURATION from EEPROM storage :
  loadEEPROMconf();

  // initBarGraphMode
  initBarGraphMode();
}

/***********************************************************************************************
 * loop()
***********************************************************************************************/
void loop()
{
  int res;

  // read LEFT SENSOR
  if (sensorFlags & LEFT)
  {
    res = getDistByTrig(trigLEFT, echoLEFT);

    drawColumnBar(0, res, frontR_Threshold);
    if (res > 0 && res < left_Threshold)
    {
      Serial.println("obstacle LEFT");
      emergencyProcedure();
    }
  }

  // read FRONT LEFT SENSOR
  if (sensorFlags & FRONT_L)
  {
    res = getDistByTrig(trigFRONT_L, echoFRONT_L);
    drawColumnBar(1, res, frontR_Threshold);
    if (res > 0 && res < frontL_Threshold)
    {
      Serial.println("obstacle FRONT LEFT");
      emergencyProcedure();
    }
  }

  // read FRONT RIGHT SENSOR
  if (sensorFlags & FRONT_R)
  {
    res = getDistByTrig(trigFRONT_R, echoFRONT_R);
    /*Serial.print("distance = ");
    Serial.print(res / 10);
    Serial.println(" cm");*/
    drawColumnBar(2, res, frontR_Threshold);
    if (res > 0 && res < frontR_Threshold)
    {
      Serial.println("obstacle FRONT RIGHT");
      emergencyProcedure();
    }
  }

  // read RIGHT SENSOR
  if (sensorFlags & RIGHT)
  {
    res = getDistByTrig(trigRIGHT, echoRIGHT);
    drawColumnBar(3, res, frontR_Threshold);
    if (res > 0 && res < right_Threshold)
    {
      Serial.println("obstacle RIGHT");
      emergencyProcedure();
    }
  }
}

/***********************************************************************************************
 * Emergency Procedure to Stop Elec wheelchair
***********************************************************************************************/
void emergencyProcedure()
{
  // for TEST : buzz x times
  for (int i = 0; i < 1; i++)
  {
    digitalWrite(led, LOW);
    tone(buzzerPlus, buzzFreq, 100);
    digitalWrite(led, HIGH);
  }

  if (joystickConnected)
  {
    // Notify emergnecy to joystick.
    txValue = 0xE0;
    pTxCharacteristic->setValue(&txValue,1);
    pTxCharacteristic->notify();
  }
};

/***********************************************************************************************
 *  BLE callbacks
***********************************************************************************************/
class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    joystickConnected = true;
  };

  void onDisconnect(BLEServer *pServer)
  {
    joystickConnected = false;
  }
};

class MyRxCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic)
  {
    std::string rxValue = pCharacteristic->getValue();

    if (rxValue.length() > 0)
    {
      Serial.println("*********");
      Serial.print("Received Value: ");
      for (int i = 0; i < rxValue.length(); i++)
        Serial.print(rxValue[i]);

      Serial.println();
      Serial.println("*********");
    }

    /* read from BLE
    while (btSerial.available())
    {
      btByte = btSerial.read();
      delay(3);
      Serial.print(F("received : Ox"));
      Serial.println(btByte);

      switch (btByte)
      {
      case CONFIG_STOP_DURATION:
        // get stop duration from BLE
        btByte = btSerial.read();
        Serial.print(F("received : Ox"));
        Serial.println(btByte); //check if value is in range 1 to 10
        if (btByte < 0 || btByte > 10)
          btByte = 3;
        //save new value in EEPROM
        EEPROM.write(eeAdr_stopDuration, btByte);
        EEPROM.commit();
        // update local stopDuration (and convert into millis)
        stopDuration = btByte * 1000;
        break;

      case EMERGENCY_STOP:
        // launch emergency procedure
        emergencyProcedure();
        // clear input buffer
        while (btSerial.available())
          btSerial.read();
        break;
      }
    }*/
  }
};

/***********************************************************************************************
 * initialize BLE
***********************************************************************************************/
void initBLE()
{
  // Create the BLE Device
  BLEDevice::init("Sensor Module BLE");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE TX Characteristic
  pTxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
  pTxCharacteristic->addDescriptor(new BLE2902());

  // Create a BLE RX Characteristic
  pRxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE);
  pRxCharacteristic->setCallbacks(new MyRxCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->addServiceUUID(SERVICE_UUID);
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
}

///////////////////////////////////    ULTRA SONIC FUNCTIONS   /////////////////////////////////

/***********************************************************************************************
 * READ DISTANCE BY TRIG
***********************************************************************************************/
int getDistByTrig(int trigPin, int echoPin)
{
  // Create pulse
  digitalWrite(trigPin, LOW); // 0- clear trigPin
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH); // 1- rise UP
  delayMicroseconds(10);       // 2- pulse width
  digitalWrite(trigPin, LOW);  // 3- rise down

  // Get echo
  unsigned long period;
  period = pulseIn(echoPin, HIGH, 6000); // max distance = 1M

  return (period * 343 / 2000); // return distance in mm
}

////////////////////////////////        OLED FUNCTIONS          ////////////////////////////////
/***********************************************************************************************
 * DISPLAY WHEELCHAIR ANIMATION
***********************************************************************************************/
void splashScreen()
{
  for (int i = 0; i < 96; i = i + 2)
  {
    display.clearDisplay(); // Clear the display buffer
    display.drawBitmap(-64 + i, 3, image_data_fauteuil_down, 58, 58, WHITE);
    display.drawBitmap(124 - i, 3, image_data_cadre, 58, 58, WHITE);
    display.display(); // Update screen with each newly-drawn line
    delay(10);
  }
}

/***********************************************************************************************
 * INIT BAR GRAPH DISPLAY
***********************************************************************************************/
#define X_BAR_BASE 27
void initBarGraphMode()
{
  display.clearDisplay(); // Clear the display buffer

  // insert title text "sonar mode"
  display.setRotation(3);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("detection obstacles en cours...");
  display.setRotation(0);

  // draw baseline (Zero Ref for X Distance)
  // display.drawLine(X_BAR_BASE, 0, X_BAR_BASE, 63, WHITE);
  display.display();
}

/***********************************************************************************************
 * DISPLAY COLUMN BAR
***********************************************************************************************/
void drawColumnBar(int col, int dist, int threshold)
{
  //clear previous data
  display.fillRect(X_BAR_BASE + 1, col * 16, 127, 16, BLACK);
  //draw bar
  if (dist > 0)
    display.fillRect(dist / 10 + X_BAR_BASE, col * 16, 127, 16, WHITE);
  //draw threshold
  int thresholdColor = WHITE;
  (dist > 0 && dist < threshold) ? thresholdColor = BLACK : thresholdColor = WHITE;
  display.drawLine(X_BAR_BASE + threshold / 10, col * 16, X_BAR_BASE + threshold / 10, (col + 1) * 16, thresholdColor);
  display.display();
}

////////////////////////////////       TOOL BOX FUNCTIONS        ///////////////////////////////

void loadEEPROMconf()
{
  // load Stop duration from EEPROM (default 3sec)
  eeVal = EEPROM.read(eeAdr_stopDuration);
  (eeVal < 0 || eeVal > 10) ? stopDuration = 3000 : stopDuration = eeVal * 1000;
  // load Sensor Flag Configuration (default 8 sensors activated <=> 255)
  eeVal = EEPROM.read(eeAdr_sensorFlags);
  (eeVal < 0 || eeVal > 255) ? sensorFlags = 255 : sensorFlags = eeVal;
  // load LEFT threshold from EEPROM (default 50cmm)
  eeVal = EEPROM.read(eeAdr_left_Threshold);
  (eeVal < 15 || eeVal > 200) ? left_Threshold = 500 : left_Threshold = eeVal * 10;
  // load FRONT_L threshold from EEPROM (default 50cmm)
  eeVal = EEPROM.read(eeAdr_frontL_Threshold);
  (eeVal < 15 || eeVal > 200) ? frontL_Threshold = 500 : frontL_Threshold = eeVal * 10;
  // load FRONT_R threshold from EEPROM (default 50cmm)
  eeVal = EEPROM.read(eeAdr_frontR_Threshold);
  (eeVal < 15 || eeVal > 200) ? frontR_Threshold = 500 : frontR_Threshold = eeVal * 10;
  // load RIGHT threshold from EEPROM (default 50cmm)
  eeVal = EEPROM.read(eeAdr_right_Threshold);
  (eeVal < 15 || eeVal > 200) ? right_Threshold = 500 : right_Threshold = eeVal * 10;
}

void tone(byte pin, int freq, int duration)
{
  digitalWrite(led, LOW);

  ledcSetup(0, 2000, 8);  // setup beeper
  ledcAttachPin(pin, 0);  // attach beeper
  ledcWriteTone(0, freq); // play tone
  delay(duration);
  ledcWriteTone(0, 0); // stop tone

  digitalWrite(led, HIGH);
}
