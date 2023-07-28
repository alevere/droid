/* Edge Impulse ingestion SDK
 * Copyright (c) 2022 EdgeImpulse Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

// If your target is limited in memory remove this macro to save 10K RAM
#define EIDSP_QUANTIZE_FILTERBANK   0

/**
 * Define the number of slices per model window. E.g. a model window of 1000 ms
 * with slices per model window set to 4. Results in a slice size of 250 ms.
 * For more info: https://docs.edgeimpulse.com/docs/continuous-audio-sampling
 */
#define EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW 2

/*
 ** NOTE: If you run into TFLite arena allocation issue.
 **
 ** This may be due to may dynamic memory fragmentation.
 ** Try defining "-DEI_CLASSIFIER_ALLOCATION_STATIC" in boards.local.txt (create
 ** if it doesn't exist) and copy this file to
 ** `<ARDUINO_CORE_INSTALL_PATH>/arduino/hardware/<mbed_core>/<core_version>/`.
 **
 ** See
 ** (https://support.arduino.cc/hc/en-us/articles/360012076960-Where-are-the-installed-cores-located-)
 ** to find where Arduino installs cores on your machine.
 **
 ** If the problem persists then there's not enough memory for this model and application.
 */

/* Includes ---------------------------------------------------------------- */
//#include <PDM.h>
//#include <DroiDepotSpeech_inferencing.h>
//#include <Arduino_APDS9960.h>
//dont use with impulse edge
//#include <arm_math.h>
//#include <Arduino_LSM9DS1.h>
//install arduino ble 1.2.2 only
#include <ArduinoBLE.h> //see note above!
//#define ARM_MATH_CM4
#include <VL53L1X.h>
#include <Arduino_LSM6DSOX.h>

// Jul 2023
// Code to make a smarter and somewhat autonomous Droid from Galaxy's Edge brought from the Droid Depot
// This code is for Nicla Vision with lipoly pack in head
// BLE code is not great and dependent upon older arduino ble versions
// This is for a BB-8 style droid
// Some code is from https://github.com/arduino/ArduinoAI
// Some code is from https://forum.arduino.cc/u/gssd/summary
//Excellent write up at https://medium.com/@baptistelaget/controlling-disneys-droids-from-droid-depots-with-webbluetooth-febbabe50587
// Using Arduino IDE 2.1.1
// Imported zip library from Impulse Edge into IDE

 #define RED 22     
 #define BLUE 24     
 #define GREEN 23
 #define LED_PWR 25


static bool record_ready = false;
static signed short *sampleBuffer;
static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal
static int print_results = -(EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW);
volatile bool paired;
volatile bool droidFlag;
volatile int loopCount;
static char firstCommand[] = {0x22,0x20,0x01};  
static char secondCommand[] = {0x27,0x42,0x0f,0x44,0x44,0x00,0x1f,0x00};
static char thirdCommand[] = {0x27,0x42,0x0f,0x44,0x44,0x00,0x18,0x02}; 
static char fourthCommand[] = {0x29,0x42,0x05,0x46,0x00,0x60,0x01,0x2c,0x00,0x00}; //first motor forwards half (0x60) power
static char fifthCommand[] = {0x29,0x42,0x05,0x46,0x00,0x00,0x01,0x2c,0x00,0x00};  //stop motor  
static char sixthCommand[] = {0x25,0x00,0x0c,0x42,0x08,0x02}; //rotate head
static char seventhCommand[] = {0x29,0x42,0x05,0x46,0x01,0x60,0x01,0x2c,0x00,0x00}; //second motor 0x46,0x01 is passenger leg
static char eighthCommand[] = {0x29,0x42,0x05,0x46,0x01,0x00,0x01,0x2c,0x00,0x00}; //stop motor
static char ninthCommand[] = {0x29,0x42,0x05,0x46,0x80,0x90,0x01,0x2c,0x00,0x00}; 
static char tenthCommand[] = {0x29,0x42,0x05,0x46,0x81,0x90,0x01,0x2c,0x00,0x00}; 
static char eleventhCommand[] = {0x29,0x42,0x05,0x46,0x80,0x70,0x01,0x2c,0x00,0x00}; //first motor backwards half power
static char twelthCommand[] = {0x29,0x42,0x05,0x46,0x81,0x70,0x01,0x2c,0x00,0x00}; //second motor backwards half power

static char firstBank[] = {0x27,0x42,0x0f,0x44,0x44,0x00,0x1f,0x00};  
static char secondBank[] = {0x27,0x42,0x0f,0x44,0x44,0x00,0x1f,0x01}; 
static char thirdBank[] =  {0x27,0x42,0x0f,0x44,0x44,0x00,0x1f,0x02}; 
static char fourthBank[] =  {0x27,0x42,0x0f,0x44,0x44,0x00,0x1f,0x03}; 
static char fifthBank[] =  {0x27,0x42,0x0f,0x44,0x44,0x00,0x1f,0x04};
static char firstSound[] = {0x27,0x42,0x0f,0x44,0x44,0x00,0x18,0x00};  
static char secondSound[] = {0x27,0x42,0x0f,0x44,0x44,0x00,0x18,0x01};     
static char thirdSound[] = {0x27,0x42,0x0f,0x44,0x44,0x00,0x18,0x02};   
static char fourthSound[] = {0x27,0x42,0x0f,0x44,0x44,0x00,0x18,0x03};     
static char fifthSound[] = {0x27,0x42,0x0f,0x44,0x44,0x00,0x18,0x04};  
static char sixthSound[] = {0x27,0x42,0x0f,0x44,0x44,0x00,0x18,0x05}; 
float Ax, Ay, Az;
float Gx, Gy, Gz;

//initialize variables
BLECharacteristic droidCharacteristic;
//BLECharacteristic notifyCharacteristic;
VL53L1X proximity;
/**
 * @brief      Arduino setup function
 */
void setup()
{
    Serial.begin(115200);
    // put your setup code here, to run once:
    droidFlag = false;
    loopCount=0;
    // intitialize the digital Pin as an output
    paired = false; 
    pinMode(RED, OUTPUT);
    pinMode(BLUE, OUTPUT);
    pinMode(GREEN, OUTPUT);
    pinMode(LED_PWR, OUTPUT);
    lightsOn(0); //blue
    delay(2000);
    lightsOn(1);  //red 
    delay(2000);
    lightsOn(2);   //green
    delay(2000);
    lightsOn(3);  //blue
    delay(1000);
    //initialize BLE  
    if (!BLE.begin()) {
       Serial.println("starting Bluetooth® Low Energy module failed!");
    }
    Wire1.begin();
    Wire1.setClock(400000); // use 400 kHz I2C
    proximity.setBus(&Wire1);
    if (!proximity.init()) {
       Serial.println("Failed to detect and initialize sensor!");
    }
    proximity.setDistanceMode(VL53L1X::Long);
    proximity.setTimeout(500);
    proximity.setMeasurementTimingBudget(50000);
    proximity.startContinuous(75);
    if (!IMU.begin()){
      Serial.println("failed to initialize IMU");
    }
    
}

/**
 * @brief      Arduino main function. Runs the inferencing loop over and over and over.
 */
void loop()
{
    BLEDevice peripheral;
    loopCount++;
    String droidString = "DROID";
    unsigned long currentMillis = 0; 
    unsigned long previousMillis = 0;  
    String myName;
    lightsOn(0);
    //digitalWrite(LED_PWR, HIGH); //turn LED off 

    int i = 0;
    int rand = 0;
    previousMillis = millis(); 
    currentMillis = millis();     
    while((currentMillis-30)<previousMillis){currentMillis = millis(); } //loop for 30ms
    if (loopCount%100==0) {
      //Serial.print(proximity.read()); //often less than 100 unless near wall, will be 100-200
      //if (proximity.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
     // Serial.println();
      if (IMU.accelerationAvailable()){
        IMU.readAcceleration(Ax, Ay, Az);
      //  Serial.print(Ax); //usually 1.0
      //  Serial.print('\t');
      //  Serial.print(Ay); //usually -0.01
      //  Serial.print('\t'); 
      //  Serial.print(Az); //usually -0.08
      //  Serial.println('\t'); 
      }
      if (IMU.gyroscopeAvailable()){
        IMU.readGyroscope(Gx, Gy, Gz);
     //   Serial.print(Gx); //usually 0.92
     //   Serial.print('\t');
     //   Serial.print(Gy); //usually 0.06
     //   Serial.print('\t'); 
     //   Serial.print(Gz); //usually -0.31
     //   Serial.print('\t'); 
      }
    }

  if(!paired){
    if (loopCount%200==0){
    Serial.print("+");  
    lightsOn(1);    
    //peripheral = BLE.available();
    BLE.scanForName(droidString);
    peripheral = BLE.available();
    if(peripheral) {
       BLE.stopScan();
       explorePeripheral(peripheral);
    }
  } 
 } //end if paired

  if(paired){
    //droid is connected and communicating
    //Serial.print(1);
    //BLE.poll();
    lightsOn(2);
    moveForward(600);
    previousMillis = millis(); 
    currentMillis = millis();     
    while((currentMillis-30)<previousMillis){currentMillis = millis(); } //loop for 30ms
    /*
    rand = random(6);
    if (IMU.accelerationAvailable()){
        IMU.readAcceleration(Ax, Ay, Az);
        if (Ax>3.0 || Ax < -1.0) {rand=3;lightsOn(2);}
        IMU.readGyroscope(Gx, Gy, Gz);
        if (Gz>4.0 || Gz < -2.0) {rand=3;lightsOn(2);}
        if (Ay>0.6 || Ay < -0.3) {rand=3;lightsOn(2);}
      }
    if(proximity.read()>115){rand=0;} 
    if(rand==0){moveForward(600);} 
    if(rand==1){moveHead();}    
    if(rand==3){moveBackward(200);}  
    */    
  } //end paired loop
} //end loop


void lightsOn(int color) {
  if(color==0){
    //off
    digitalWrite(RED, HIGH); // turn the LED off by making the voltage HIGH
    digitalWrite(GREEN, HIGH);
    digitalWrite(BLUE, HIGH);
  }  
  if(color==1){
    //red
    digitalWrite(RED, HIGH); // turn the LED off by making the voltage HIGH
    digitalWrite(GREEN, LOW);
    digitalWrite(BLUE, HIGH);
  }  
if(color==2){
    //green
    digitalWrite(RED, LOW); // turn the LED off by making the voltage LOW
    digitalWrite(GREEN, HIGH);
    digitalWrite(BLUE, LOW);
  }    
  if(color==3){
    //blue
    digitalWrite(RED, HIGH); // turn the LED off by making the voltage LOW
    digitalWrite(GREEN, LOW);
    digitalWrite(BLUE, LOW);
  }  
    if(color==4){
    //red
    digitalWrite(RED, LOW); // turn the LED off by making the voltage LOW
    digitalWrite(GREEN, HIGH);
    digitalWrite(BLUE, HIGH);
  }  
}

void startBLE() {
//initialize BLE  
  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy module failed!");
    //while (1);
  }
    
Serial.println("BLE started...");
  //BLE central scan
   BLE.scan(); //scan run continuously until stopped
  //BLE.stopScan();
}

void blePeripheralConnectHandler(BLEDevice central) {
  // central connected event handler
  Serial.print("Connected event, central: ");
  Serial.println(central.address());
  paired = true;
}

void blePeripheralDisconnectHandler(BLEDevice central) {
  // central disconnected event handler
  Serial.print("Disconnected event, central: ");
  Serial.println(central.address());
  paired = false;
  unsigned long currentMillis = 0; 
  unsigned long previousMillis = 0; 
  previousMillis = millis(); 
  currentMillis = millis();     
  while((currentMillis-40)<previousMillis){currentMillis = millis(); } //loop for 30ms
}
/*
// function to play a sound from the sound bank based on two parameters passed in
void makeNoise(int bank, int slot) {
    unsigned long currentMillis = 0; 
    unsigned long previousMillis = 0;  
    currentMillis=millis();
    previousMillis=millis();    
    if(bank==0){droidCharacteristic.writeValue(firstBank,8,true);}
    if(bank==1){droidCharacteristic.writeValue(secondBank,8,true);}
    else{droidCharacteristic.writeValue(thirdBank,8,true);}
    while((currentMillis-30)<previousMillis){currentMillis = millis(); } //loop for 30ms
    if(slot==0){droidCharacteristic.writeValue(firstSound,8,true);}  
    if(slot==1){droidCharacteristic.writeValue(secondSound,8,true);}  
    if(slot==2){droidCharacteristic.writeValue(thirdSound,8,true);}  
    if(slot==3){droidCharacteristic.writeValue(fourthSound,8,true);}  
    if(slot==4){droidCharacteristic.writeValue(fifthSound,8,true);}     
    else {droidCharacteristic.writeValue(sixthSound,8,true);} 
    while((currentMillis-100)<previousMillis){currentMillis = millis(); } //loop for 500ms       
}
*/
// function to play a sound from the sound bank based on two parameters passed in
void moveForward(int distance) {
    unsigned long currentMillis = 0; 
    unsigned long previousMillis = 0;  
    Serial.println("forward");
    currentMillis=millis();
    previousMillis=millis();    
    droidCharacteristic.writeValue(fourthCommand,10,true);
    droidCharacteristic.writeValue(seventhCommand,10,true);
    while((currentMillis-distance)<previousMillis){currentMillis = millis(); } //loop for 30ms
    droidCharacteristic.writeValue(fifthCommand,10,true);  
    droidCharacteristic.writeValue(eighthCommand,10,true); 
    while((currentMillis-200)<previousMillis){currentMillis = millis(); } //loop for 500ms       
    }
/*          
void moveBackward(int distance) {
    unsigned long currentMillis = 0; 
    unsigned long previousMillis = 0;  
    //Serial.println("backward");
    currentMillis=millis();
    previousMillis=millis();    
    droidCharacteristic.writeValue(eleventhCommand,10,true);
    droidCharacteristic.writeValue(twelthCommand,10,true);
    while((currentMillis-distance)<previousMillis){currentMillis = millis(); } //loop for 30ms
    droidCharacteristic.writeValue(fifthCommand,10,true);  
    droidCharacteristic.writeValue(eighthCommand,10,true); 
    while((currentMillis-200)<previousMillis){currentMillis = millis(); } //loop for 500ms       
    }    


void moveRotate(int direction) {
    unsigned long currentMillis = 0; 
    unsigned long previousMillis = 0;  
    //Serial.println("backward");
    currentMillis=millis();
    previousMillis=millis();    
    if(direction==0) {droidCharacteristic.writeValue(eleventhCommand,10,true);}
    else {droidCharacteristic.writeValue(twelthCommand,10,true);}
    while((currentMillis-300)<previousMillis){currentMillis = millis(); } //loop for 300ms
    droidCharacteristic.writeValue(fifthCommand,10,true);  
    droidCharacteristic.writeValue(eighthCommand,10,true); 
    while((currentMillis-100)<previousMillis){currentMillis = millis(); } //loop for 150ms       
  }     

  void moveHead() {
    unsigned long currentMillis = 0; 
    unsigned long previousMillis = 0;  
    //Serial.println("turn head");
    currentMillis=millis();
    previousMillis=millis();    
    droidCharacteristic.writeValue(sixthCommand,6,true); 
    while((currentMillis-400)<previousMillis){currentMillis = millis(); } //loop for 500ms       
}  
*/
void explorePeripheral(BLEDevice peripherals) {
   if (peripherals.connect()){

   }
   else {Serial.println("unable to connect");return;}
   if (peripherals.discoverAttributes()) {

   }
   else {Serial.println("Discovery failed");peripherals.disconnect();return;} //unknown why this happens frequently
   int services = peripherals.serviceCount();
   Serial.println(services);
   BLEService tempservice=peripherals.service("09b600a0-3e42-41fc-b474-e9c0c8f0c801");
   exploreService(tempservice);
}               

void exploreService(BLEService myservice) {
  droidCharacteristic = myservice.characteristic("09b600b1-3e42-41fc-b474-e9c0c8f0c801");
  connectCharacteristic();
}

void connectCharacteristic (){
       //Serial.println("@@ DROID @@");
       unsigned long currentMillis = 0; 
       unsigned long previousMillis = 0; 
       lightsOn(3);   
       previousMillis = millis(); 
       currentMillis = millis(); 
       droidCharacteristic.writeValue(firstCommand,3,true);
       previousMillis = millis();     
       while((currentMillis-30)<previousMillis){currentMillis = millis(); } //loop for 30ms
       droidCharacteristic.writeValue(firstCommand,3,true);
       previousMillis = millis(); 
       //Serial.println("+");        
       while((currentMillis-25)<previousMillis){currentMillis = millis(); } 
       droidCharacteristic.writeValue(firstCommand,3,true);
       previousMillis = millis(); 
       //Serial.println(previousMillis);    
       while((currentMillis-25)<previousMillis){currentMillis = millis(); } //loop for 25ms
       droidCharacteristic.writeValue(firstCommand,3,true);
       previousMillis = millis(); 
       //Serial.println(previousMillis);    
       while((currentMillis-25)<previousMillis){currentMillis = millis(); } 
       droidCharacteristic.writeValue(secondCommand,8,true);
       previousMillis = millis(); 
       while((currentMillis-25)<previousMillis){currentMillis = millis(); } 
       droidCharacteristic.writeValue(thirdCommand,8,true);
       previousMillis = millis(); 
       while((currentMillis-25)<previousMillis){currentMillis = millis(); } 
       droidCharacteristic.writeValue(secondCommand,8,true);
       previousMillis = millis(); 
       while((currentMillis-25)<previousMillis){currentMillis = millis(); } 
       droidCharacteristic.writeValue(thirdCommand,8,true);
       Serial.println("Initialization commands sent, should hear beeps from droid");
       previousMillis = millis(); 
       while((currentMillis-700)<previousMillis){currentMillis = millis(); }
       paired=true;
}
