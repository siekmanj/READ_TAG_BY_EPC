/*
 * Jonah Siekmann
 * Asynchronous RFID reader that updates RFID and GPS information to a smartphone via Bluetooth
 * 8/31/2017
 * OpenS Lab
 * 
 * UUID
 * Device Address
 */

#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include "SparkFun_UHF_RFID_Reader.h"


SoftwareSerial RFID_port(2, 3);
SoftwareSerial GPS_port(5, 4);
SoftwareSerial BT_port(6, 7);
TinyGPSPlus gps;
RFID nano;



boolean tagFound = false;

struct RFID_tag{
  
  byte EPC[12]; //RFID Tag Identifier
  uint16_t reads; //Number of times this tag has been read
  double lat; //Latitude of tag
  double lng; //Longitude of tag
  float currentAverage; //Current averaged moisture level of tag

  RFID_tag(){
    reads = 0;
    lat = 0.0;
    lng = 0.0;
    currentAverage = 0.0;
  }

  void updateMoisture(uint8_t newVal){
    if(newVal < 25){
      currentAverage = ((float)(currentAverage * reads + newVal))/(reads+1);
      reads++;
    }
  }
};

RFID_tag *tagList[10];
int tagListLength = 0;


void setup() {
  Serial.begin(9600);
  while(!Serial);
  Serial.println("Initializing...");

  /*
   * NANO 6ME Setup
   */
  if (setupNano(38400) == false){ //Configure nano to run at 38400bps
    Serial.println("RFID module failed to respond. Please check wiring.");
    while (1); //Freeze!
  }else{
    nano.setRegion(REGION_NORTHAMERICA); //Set to North America
    nano.setReadPower(700); //5.00 dBm. Higher values may cause USB port to brown out
  }

  /*
   * GPS setup
   */
  GPS_port.begin(9600);
  Serial.print("GPS setup init.");
  while(!GPS_port.available()){
    Serial.print(".");
    delay(100);
  }
  /*while(GPS_port.available()){
    gps.encode(GPS_port.read());
    if(gps.location.isUpdated()){
      Serial.println("GPS fix!");
      break;
    }else{
      Serial.println("No GPS fix.");
    }
  }*/

  /*
   * Bluetooth setup
   */
  BT_port.begin(9600);
  
}

boolean safe = false;
String test = "";
void loop(){
  //longitude, latitude, moisture, time, EPC;

   GPS_port.listen(); //Check for GPS data
   while(GPS_port.available()){
    gps.encode(GPS_port.read());
   }

   RFID_port.listen();
   
   struct RFID_tag *tag = checkForTag();
   
   if(tagFound){ //Found a tag
     tagFound = false;
     Serial.print(" ((Reading moisture)) ");
     
     byte sensorData[2];
     byte sensorDataLength = sizeof(sensorData);

     uint8_t bank = 0x00; //Reserved data bank
     uint8_t address = 0x0B; //Word Address
     
     byte response = nano.readData(bank, address, sensorData, sensorDataLength, 1000);
     tag->updateMoisture(byteArrayToInt(sensorData, sensorDataLength));
     
     Serial.print(tag->currentAverage);
     
     //if(response == RESPONSE_SUCCESS){
      
     //}
   }else{
    Serial.print("No tag found.");
   }
   Serial.println();
   
   
}


struct RFID_tag *checkForTag(){

  byte EPC[12];
  uint8_t EPC_length = sizeof(EPC);
  
  uint8_t response = nano.readTagEPC(EPC, EPC_length, 500);
  
  EPC_length = sizeof(EPC);
  if(response == RESPONSE_SUCCESS){

    Serial.print(" ((TAG FOUND)) ");
    boolean newTag = true;
    
    for(int i = 0; i < tagListLength; i++){ //Check to see if tag list already contains this tag EPC
      if(areEqual(tagList[i]->EPC, EPC)){
        newTag = false;
        Serial.print(" ((OLD TAG!)) ");
        tagFound = true;
        return tagList[i];
      }
    }
    
    if(newTag){
      Serial.print(" ((NEW TAG)) ");
      struct RFID_tag *tag = new RFID_tag;
      tagList[tagListLength] = tag;
      *tag->EPC = new byte[EPC_length];
      for(int i = 0; i < EPC_length; i++){
        tag->EPC[i] = EPC[i];
      }
      tagListLength++;
      tagFound = true;
      return tag;
    }
  }else{
    Serial.print(" ((NO TAG!)) ");
    tagFound = false;
  }
}

boolean setupNano(long baudRate){
  nano.begin(RFID_port); //Tell the library to communicate over software serial port

  //Test to see if we are already connected to a module
  //This would be the case if the Arduino has been reprogrammed and the module has stayed powered
  RFID_port.begin(baudRate); //For this test, assume module is already at our desired baud rate
  while(!RFID_port); //Wait for port to open

  //About 200ms from power on the module will send its firmware version at 115200. We need to ignore this.
  while(RFID_port.available()) RFID_port.read();
  
  nano.getVersion();

  if (nano.msg[0] == ERROR_WRONG_OPCODE_RESPONSE)
  {
    //This happens if the baud rate is correct but the module is doing a ccontinuous read
    nano.stopReading();

    Serial.println(F("Module continuously reading. Asking it to stop..."));

    delay(1500);
  }
  else
  {
    //The module did not respond so assume it's just been powered on and communicating at 115200bps
    RFID_port.begin(115200); //Start software serial at 115200

    nano.setBaud(baudRate); //Tell the module to go to the chosen baud rate. Ignore the response msg

    RFID_port.begin(baudRate); //Start the software serial port, this time at user's chosen baud rate
  }

  //Test the connection
  nano.getVersion();
  Serial.print("SETUP STATUS: "); Serial.println(nano.msg[0]);
  
  if (nano.msg[0] != ALL_GOOD) return (false); //Something is not right

  //The M6E has these settings no matter what
  nano.setTagProtocol(); //Set protocol to GEN2

  nano.setAntennaPort(); //Set TX/RX antenna ports to 1

  return (true); //We are ready to rock
}

boolean areEqual(byte b1[], byte b2[]){  
  for(int i = 0; i < 12; i++){
    if(b1[i] != b2[i]) return false;
  }
  return true;
}
void printTagEPC(byte EPC[]){
  for(int i = 0; i < 12; i++){
    Serial.print(EPC[i]);
  }
}
uint8_t byteArrayToInt(byte arr[], int arrLen){
  uint8_t sum = 0;
  for(int i = 0; i < arrLen; i++){
    sum += arr[i] * pow(10, (arrLen-1)-i);
  }
  return sum;
}

