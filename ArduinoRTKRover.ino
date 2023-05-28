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
#include <ArduinoHttpClient.h>
#include <WiFiNINA.h>
#include "arduino_secrets.h"


#include <Wire.h> //Needed for I2C to GNSS
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GNSS myGNSS;

///////please enter your sensitive data in the Secret tab/arduino_secrets.h
/////// WiFi Settings ///////
char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;

char serverAddress[] = "192.168.178.20";  // server address
int port = 8765;

//WiFiClient wifi;
//WebSocketClient client = WebSocketClient(wifi, serverAddress, port);
//int status = WL_IDLE_STATUS;
int count = 0;

RF24 radio(7, 6); // CE, CSN
uint8_t addresses[][6] = { "Base", "Rover" };

void setup() {
  Serial.begin(115200);
  while (!Serial); //Wait for user to open terminal

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

  /*
  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to Network named: ");
    Serial.println(ssid);                   // print the network name (SSID);

    // Connect to WPA/WPA2 network:
    status = WiFi.begin(ssid, pass);
  }*/

  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
}

void loop() {
  //Serial.println("starting WebSocket client");
  //client.begin();

  delay(5);
  radio.stopListening();
  const char text[] = "Hello World from RTK Rover";
  radio.write(&text, sizeof(text));
  delay(5);
  radio.startListening();
  if (radio.available()) {
    char text_incoming[32] = "";
    radio.read(&text_incoming, sizeof(text_incoming));
    Serial.println(text_incoming);
  }

  /*
  while (client.connected()) {
    //Serial.print("Sending hello ");
    //Serial.println(count);

    // send a hello #
    ///client.beginMessage(TYPE_TEXT);
    //client.print("hello ");
    //client.print(count);
    //client.endMessage();

    // increment count for next message
    //count++;

    // check if a message is available to be received
    int messageSize = client.parseMessage();

    if (messageSize > 0) {
      //Serial.println("Received a message:");

      uint8_t store[256];
      arduino::String line = client.readString();

      //Serial.println(line); 
      
      unsigned int numBytes = line.length() / 2;


      //Serial.println(numBytes);
      
      if(hex2data(((uint8_t *)&store), line,numBytes)==0){
        myGNSS.pushRawData(((uint8_t *)&store), numBytes);

        for(count = 0; count < numBytes; count++) {
           Serial.print(store[count]);
           Serial.print(" ");
        }
        Serial.println();
      }
      
      
      
    }

    // wait 5 seconds
    //delay(5000);
  }*/

  //Serial.println("disconnected");
}

//convert hexstring to len bytes of data
//returns 0 on success, -1 on error
//data is a buffer of at least len bytes
//hexstring is upper or lower case hexadecimal, NOT prepended with "0x"
int hex2data(unsigned char *data, arduino::String hexstring, unsigned int len)
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
