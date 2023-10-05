/*
  Simple WebSocket client for ArduinoHttpClient library
  Connects to the WebSocket server, and sends a hello
  message every 5 seconds
  created 28 Jun 2016
  by Sandeep Mistry
  modified 22 Jan 2019
  by Tom Igoe
  this example is in the public domain
*/
#include <Arduino.h>
#include <Wire.h> //Needed for I2C to GNSS
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GNSS myGNSS;

int count = 0;


RF24 radio(4, 5); // CE, CSN

uint8_t addresses[2] = { 0x66, 0x64 };//"Base", "Rover" 

static uint8_t rtk_frame_data[1024] = "";
static uint8_t rtk_frame_size = 0 ;

struct Data_Package {
  byte    data_type = 0;
  byte    data_size = 0;
  byte    data_appendnext = 0;
  uint8_t data[28];
};

Data_Package radio_data;

void setup() {
  Serial.begin(115200);
  while (!Serial); //Wait for user to open terminal
  Serial.println(F("u-blox NEO-M8P-2 Rover"));

  radio.begin();
  radio.openWritingPipe(addresses[0]); // Base
  radio.openReadingPipe(1, addresses[1]); // Rover
  radio.setPALevel(RF24_PA_MIN);

  if(radio.isChipConnected()){
    Serial.println(F("RF24 Initialized!"));
  }else{
    Serial.println(F("Error: RF24 not connected!"));
  }

  Wire.begin();
  Wire.setClock(400000); //Increase I2C clock speed to 400kHz

  if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  myGNSS.setPortInput(COM_PORT_I2C, COM_TYPE_UBX | COM_TYPE_RTCM3); //Enable UBX and RTCM input on I2C
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save the communications port settings to flash and BBR
  myGNSS.setI2CTransactionSize(128);

}

int cycle=0;

void loop() {

  cycle++;
  
  delay(5);
  //radio.stopListening();
  //const char text[] = "Hello World from RTK Rover";
  //radio.write(&text, sizeof(text));
  delay(5);
  radio.startListening();
  if (radio.available()) {
    radio.read(&radio_data, sizeof(Data_Package));

    if(radio_data.data_type==1){
      memcpy(&rtk_frame_data[rtk_frame_size],&radio_data.data[0],radio_data.data_size);
      rtk_frame_size+=radio_data.data_size;
    }

    if(radio_data.data_type==1 && radio_data.data_appendnext == 0){
      Serial.print("RTCM Frame received, size:");
      Serial.print(rtk_frame_size);
      Serial.print(" data:");
      for(int i=0;i<rtk_frame_size;i++){
        if (rtk_frame_data[i] < 0x10) Serial.print(F("0"));  //WTF work around for eroneus HEX printing
        Serial.print(rtk_frame_data[i], HEX);
      }
      Serial.println("");
      myGNSS.pushRawData(((uint8_t *)&rtk_frame_data[0]), rtk_frame_size);
      rtk_frame_size=0;
    }
  }

  if(cycle%2500 == 0){

    long latitude = myGNSS.getLatitude();
    Serial.print(F("Lat: "));
    Serial.print(latitude);

    long longitude = myGNSS.getLongitude();
    Serial.print(F(" Long: "));
    Serial.print(longitude);

    long altitude = myGNSS.getAltitude();
    Serial.print(F(" Alt: "));
    Serial.print(altitude);

    byte fixType = myGNSS.getFixType();
    Serial.print(F(" Fix: "));
    if(fixType == 0) Serial.print(F("No fix"));
    else if(fixType == 1) Serial.print(F("Dead reckoning"));
    else if(fixType == 2) Serial.print(F("2D"));
    else if(fixType == 3) Serial.print(F("3D"));
    else if(fixType == 4) Serial.print(F("GNSS + Dead reckoning"));
    else if(fixType == 5) Serial.print(F("Time only"));

    byte RTK = myGNSS.getCarrierSolutionType();
    Serial.print(" RTK: ");
    Serial.print(RTK);
    if (RTK == 0) Serial.print(F(" (No solution)"));
    else if (RTK == 1) Serial.print(F(" (High precision floating fix)"));
    else if (RTK == 2) Serial.print(F(" (High precision fix)"));
    Serial.println("");
  }
  if(cycle >100000){
    cycle=0;
  }
}

//convert hexstring to len bytes of data
//returns 0 on success, -1 on error
//data is a buffer of at least len bytes
//hexstring is upper or lower case hexadecimal, NOT prepended with "0x"
int hex2data(unsigned char *data, String hexstring, unsigned int len)
{
   
    size_t count = 0;
    size_t pos = 0;
    char *endptr;
    
    if ((hexstring.length() == 0) || (hexstring.length() % 2)) {
        //hexstring contains no data
        //or hexstring has an odd length
        Serial.print("Not pair");
        return -1;
    }

    for(count = 0; count < len; count++) {
        char buf[5] = {'0', 'x', hexstring.charAt(pos), hexstring.charAt(pos+1), 0};
        data[count] = strtol(buf, &endptr, 0);
        pos += 2 * sizeof(char);

        if (endptr[0] != '\0') {
            //non-hexadecimal character encountered
            Serial.print("Not hex");
            return -1;
        }
    }

    return 0;
}


/*
  //pushRawData
  myGNSS.setPortInput(COM_PORT_I2C, COM_TYPE_UBX | COM_TYPE_RTCM3); //Enable UBX and RTCM input on I2C
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save the communications port settings to flash and BBR
  myGNSS.setI2CTransactionSize(128);
  uint8_t store[256];
    while ((Serial1.available()) && (numBytes < 256)) // Check if data has been received
    {
      store[numBytes++] = Serial1.read(); // Read a byte from Serial1 and store it
    }
    if (numBytes > 0) // Check if data was received
    {
      //Serial.print("Pushing ");
      //Serial.print(numBytes);
      //Serial.println(" bytes via I2C");

      //On processors which have large I2C buffers, like the ESP32, we can make the push more efficient by
      //calling setI2CTransactionSize first to increase the maximum I2C transmission size
      //(setI2CTransactionSize only needs to be called once, so it should be in setup, not loop)
      //myGNSS.setI2CTransactionSize(128); // Send up to 128 bytes in one I2C transmission

      //The ESP32 seems to have an issue when using a restarts to break up long RTCM pushes
      //You may need to call pushRawData and set the optional 'stop' argument to true:
      //myGNSS.pushRawData(((uint8_t *)&store), numBytes, true); // Push the RTCM data via I2C - always use stops on long RTCM pushes

      myGNSS.pushRawData(((uint8_t *)&store), numBytes); // Push the RTCM data via I2C - using restarts to break up long I2C pushes
      numBytes = 0; // Reset numBytes
    }
*/