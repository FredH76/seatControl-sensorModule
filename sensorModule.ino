#include <EEPROM.h>
#include <SoftwareSerial.h>

const int SR04T_DISCONNECTED = -1;
enum SIDE {
  LEFT = 1,
  FRONT= 2,
  RIGHT= 4
};

////////////////////////////////////      PIN CONFIGURATION    /////////////////////////////////
// BLE HC-05 pin
const int txBT = 14; // TX3
const int rxBT = 15; // RX3

// DIGITAL 
const int trigRIGHT = 2;
const int echoRIGHT = 3;
const int trigFRONT = 4;
const int echoFRONT = 5;
const int trigLEFT = 6;
const int echoLEFT = 7;

const int buzzerPlus = 10;
const int buzzerGround = 9;
const int led = 13;

// ANALOG

////////////////////////////////////      COMMAND HEX CODE      /////////////////////////////////
const int CONFIG_STOP_DURATION = 0xC1;
const int EMERGENCY_STOP = 0xE0;

////////////////////////////////////      VAR DECLARATION      /////////////////////////////////

//   EEPROM adress       
int eeAdr_stopDuration = 0;
int eeAdr_sensorFlags = 2;
int eeAdr_leftThreshold  = 10;
int eeAdr_frontThreshold = 11;
int eeAdr_rightThreshold = 12;

SoftwareSerial btSerial = SoftwareSerial(rxBT, txBT);
int eeVal;        // value read in EEPROM
int stopDuration; // time before releasing control to joystick (in ms)
int sensorFlags = 0; // LEFT < FRONT < RIGHT < ...
int leftThreshold;
int frontThreshold;
int rightThreshold;
int buzzFreq;     // frequency (in Hz)
int btByte;       // store byte to be read from BLE serial

/***********************************************************************************************
 * setUp()
***********************************************************************************************/
void setup()
{
  Serial.begin(9600);

  // configure pin INPUT/OUTPUT
  pinMode(txBT, OUTPUT);
  pinMode(rxBT, INPUT);
  pinMode(trigRIGHT, OUTPUT);
  pinMode(echoRIGHT, INPUT);
  pinMode(trigFRONT, OUTPUT);
  pinMode(echoFRONT, INPUT);
  pinMode(trigLEFT, OUTPUT);
  pinMode(echoLEFT, INPUT);
  pinMode(buzzerPlus, OUTPUT);
  pinMode(buzzerGround, OUTPUT);
  pinMode(led, OUTPUT);

  // LOAD TEST DATA into EEPROM //
  EEPROM.write(eeAdr_sensorFlags,LEFT | FRONT | RIGHT );
  EEPROM.write(eeAdr_leftThreshold,22);
  EEPROM.write(eeAdr_frontThreshold,30);
  EEPROM.write(eeAdr_rightThreshold,40);


  // load Stop duration from EEPROM (default 3sec)
  eeVal = EEPROM.read(eeAdr_stopDuration);
  (eeVal<0 || eeVal > 10)  ? stopDuration = 3000 : stopDuration = eeVal * 1000;

  // load Sensor Flag Configuration (default 8 sensors activated <=> 256)
  eeVal = EEPROM.read(eeAdr_sensorFlags);
  (eeVal<0 || eeVal > 256) ? sensorFlags = 256 :  sensorFlags = eeVal;

  // load LEFT threshold from EEPROM (default 2m)
  eeVal = EEPROM.read(eeAdr_leftThreshold);
  (eeVal<15 || eeVal > 200) ? leftThreshold = 2000 :  leftThreshold = eeVal*10;

  // load FRONT threshold from EEPROM (default 2m)
  eeVal = EEPROM.read(eeAdr_frontThreshold);
  (eeVal<15 || eeVal > 200) ? frontThreshold = 2000 :  frontThreshold = eeVal*10;

  // load RIGHT threshold from EEPROM (default 2m)
  eeVal = EEPROM.read(eeAdr_rightThreshold);
  (eeVal<15 || eeVal > 200) ? rightThreshold = 2000 :  rightThreshold = eeVal*10;

  // init BLE
  Serial.println(F("Initializing BLE ..."));
  btSerial.begin(9600);  
  
  // set Buzzer Frequence
  buzzFreq = 6000;     // frequency (in Hz)

  // test buzzer and relay
  testUSsensor();
}

/***********************************************************************************************
 * loop()
***********************************************************************************************/
void loop()
{
  int res;

  // read from BLE
  while (btSerial.available()){
    btByte = btSerial.read();
    delay(3);
    // TODO : 
  }

  // read LEFT SENSOR
  if(sensorFlags & LEFT){
    res = getDistByTrig(trigLEFT,echoLEFT);
    if(res > 0 && res < leftThreshold)
      emergencyProcedure();
  }  

  // read FRONT SENSOR
  if(sensorFlags & FRONT){
    res = getDistByTrig(trigFRONT,echoFRONT);
    if(res > 0 && res < frontThreshold)
      emergencyProcedure();
  }

  // read RIGHT SENSOR
  if(sensorFlags & RIGHT){
    res = getDistByTrig(trigRIGHT,echoRIGHT);
    if(res > 0 && res < rightThreshold)
      emergencyProcedure();
  }
}

/***********************************************************************************************
 * Emergency Procedure to Stop Elec wheelchair
***********************************************************************************************/
void emergencyProcedure(){

    /* for TEST : buzz 3 times
    for(int i = 0 ; i < 3; i++){
        tone(buzzerPlus, buzzFreq, 100);
        digitalWrite(led,HIGH);
        delay(200);        
        digitalWrite(led,LOW);
        delay(200);        
    }*/ 
    tone(buzzerPlus, buzzFreq, 100);


};


///////////////////////////////////    ULTRA SONIC FUNCTIONS   /////////////////////////////////
/***********************************************************************************************
 * TEST US SENSOR
***********************************************************************************************/
int testUSsensor(){
  int res;

  Serial.println("_________________ test SENSOR ____________________________");
  
  res= getDistByTrig(trigLEFT,echoLEFT);
  if(res>0){
    Serial.print("SUCCESS: LEFT  ultra sonar measure = ");
    Serial.print(((float)res) /10); Serial.println(" cm");
  } else if (res == SR04T_DISCONNECTED) 
        Serial.println("ERROR: LEFT Ultra sonar SENSOR is not connected");
      else
        Serial.println("ERROR: LEFT Ultra sonar MODULE is not connected");

  res= getDistByTrig(trigFRONT,echoFRONT);
  if(res>0){
    Serial.print("SUCCESS: FRONT ultra sonar measure = ");
    Serial.print(((float)res) /10); Serial.println(" cm");
  } else if (res == SR04T_DISCONNECTED) 
        Serial.println("ERROR: FRONT Ultra sonar SENSOR is not connected");
      else
        Serial.println("ERROR: FRONT Ultra sonar MODULE is not connected");

  res= getDistByTrig(trigRIGHT,echoRIGHT);
  if(res>0){
    Serial.print("SUCCESS: RIGHT ultra sonar measure = ");
    Serial.print(((float)res) /10); Serial.println(" cm");
  } else if (res == SR04T_DISCONNECTED) 
        Serial.println("ERROR: RIGHT Ultra sonar SENSOR is not connected");
      else
        Serial.println("ERROR: RIGHT Ultra sonar MODULE is not connected");

}

/***********************************************************************************************
 * READ DISTANCE BY TRIG
***********************************************************************************************/
int getDistByTrig(int trigPin, int echoPin)
{
  // Create pulse
  digitalWrite(trigPin, LOW); // 0- clear trigPin
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH); // 1- rise UP
  delayMicroseconds(50);       // 2- pulse width
  digitalWrite(trigPin, LOW);  // 3- rise down

  // Get echo
  unsigned long period;
  period = pulseIn(echoPin, HIGH);

  // patch for SR04T
  if (period >20000) return SR04T_DISCONNECTED;
  
  //delay(30); // wait for signal to come back
  return (period * 343 / 2000);
}

/***********************************************************************************************
 * READ DISTANCE BY SERIAL_X
***********************************************************************************************
unsigned int getDistBySerial_1(){
  unsigned long distanceMM;
  unsigned long startingTime;
  unsigned long timeout = 1000;
  bool runLoop = true;

  // send command to start measurement
  Serial1.write(0x55);
  
  // wait for measure
  startingTime = millis(); // initialize timeout 
  while (Serial1.available()<=0 && runLoop){
    runLoop = ((millis() - startingTime) < timeout); 
  };
  if(runLoop == false){
    Serial.println("ERROR: no answer from ultrasonar module");
    return 0;
  }
  
  // check if frame start with proper code (0xFF)
  btByte = Serial1.read();
  if(btByte != 0xFF){
    Serial.print("ERROR: frame do not start with proper code. received : 0x");
    Serial.println(btByte,HEX);
    return 0;   
  }

  // read distance
  int H_DATA = Serial1.read(); // H_DATA
  int L_DATA = Serial1.read(); // L_DATA
  distanceMM = (H_DATA << 8) + L_DATA;

  // check validity
  if (distanceMM <=0 || distanceMM > 6000){
     Serial.print("ERROR: out of range value : ");
     Serial.print(distanceMM);
     Serial.print(" H_DATA=Ox"); Serial.print(H_DATA,HEX);
     Serial.print(" L_DATA=Ox"); Serial.print(L_DATA,HEX);
     return 0;
  }

  // clear serial
  while (Serial1.available()>0) Serial1.read();
  
  return distanceMM;  
  }
  */
