
#include <SoftwareSerial.h>
#include "SparkFun_UHF_RFID_Reader.h"

#define BUZZER1 9
#define BUZZER2 10

SoftwareSerial softSerial(2, 3); 
RFID nano; 


struct dogbonesTag{
  unsigned long moisture;
  byte *EPC;
};

dogbonesTag tagList[64];
int tagListLength = 0;


void setup() {
    Serial.begin(115200);
    
    pinMode(BUZZER1, OUTPUT);
    pinMode(BUZZER2, OUTPUT);
    digitalWrite(BUZZER2, LOW); //Pull half the buzzer to ground and drive the other half.
    
    while (!Serial);
    Serial.println();
    Serial.println("Initializing...");
    
    if (setupNano(38400) == false) //Configure nano to run at 38400bps
    {
      Serial.println("Module failed to respond. Please check wiring.");
      while (1); //Freeze!
    }else{
      Serial.println("Here we go, boys and girls...");
      nano.setRegion(REGION_NORTHAMERICA); //Set to North America
      nano.setReadPower(500); //5.00 dBm. Higher values may cause USB port to brown out
      //nano.enableDebugging();
    }
    

}

void loop() {

  delay(1000);
  
  byte EPC[12]; //Most EPCs are 12 bytes
  byte EPClength = sizeof(EPC);
  byte response = 0;
  
  while(response != RESPONSE_SUCCESS){
    EPClength = sizeof(EPC);
    response = nano.readTagEPC(EPC, EPClength, 500);
  }
  Serial.print("Scanned a");

  boolean newTag = true;
  int index;
  for(int i = 0; i < tagListLength; i++){
    
    if(areEqual(tagList[i].EPC, EPC)){
      newTag = false;
      index = i;
    }
  }
  
  if(newTag){
    Serial.print(" new tag. Moisture value of ");
    struct dogbonesTag *tag = new dogbonesTag;
    tagList[tagListLength] = *tag;
    *tag->EPC = new byte[EPClength];
    for(int i = 0; i < EPClength; i++){
      tag->EPC[i] = EPC[i];
    }
    index = tagListLength;
    tagListLength++;
  }else{
    Serial.print("n existing tag. Moisture value of ");

  }
  
  byte sensorData[64];
  byte address = 0x00;
  byte bank = 0x03;
  byte sensorDataLength = sizeof(sensorData);
  
  response = 0;
  
  while(response != RESPONSE_SUCCESS){
    sensorDataLength = sizeof(sensorData);
    response = nano.readData(bank, address, sensorData, sensorDataLength);

  }
  
  tagList[index].moisture = byteArrayToLong(sensorData, sensorDataLength);
  Serial.print(tagList[index].moisture); Serial.print(". "); Serial.print(tagListLength); Serial.print(" tags polled.");

  Serial.println();
}



//Gracefully handles a reader that is already configured and already reading continuously
//Because Stream does not have a .begin() we have to do this outside the library
boolean setupNano(long baudRate)
{
  nano.begin(softSerial); //Tell the library to communicate over software serial port

  //Test to see if we are already connected to a module
  //This would be the case if the Arduino has been reprogrammed and the module has stayed powered
  softSerial.begin(baudRate); //For this test, assume module is already at our desired baud rate
  while(!softSerial); //Wait for port to open

  //About 200ms from power on the module will send its firmware version at 115200. We need to ignore this.
  while(softSerial.available()) softSerial.read();
  
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
    softSerial.begin(115200); //Start software serial at 115200

    nano.setBaud(baudRate); //Tell the module to go to the chosen baud rate. Ignore the response msg

    softSerial.begin(baudRate); //Start the software serial port, this time at user's chosen baud rate
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

unsigned long byteArrayToLong(byte arr[], int arrLen){
  unsigned long sum = 0;
  for(int i = 0; i < arrLen; i++){
    sum += arr[i] * pow(10, (arrLen-1)-i);
  }
  return sum;
}

void printTagEPC(byte EPC[]){
  for(int i = 0; i < 12; i++){
    Serial.print(EPC[i]);
  }
}

boolean areEqual(byte b1[], byte b2[]){  
  for(int i = 0; i < 12; i++){
    if(b1[i] != b2[i]) return false;
  }
  return true;
}
  



