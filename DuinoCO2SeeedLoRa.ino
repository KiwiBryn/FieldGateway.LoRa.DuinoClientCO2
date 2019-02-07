/*
    Copyright ® 2019 January devMobile Software, All Rights Reserved
 
    MIT License

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE

    Sample application for CO2 monitoring node based on 
      https://www.seeedstudio.com/Grove-Carbon-Dioxide-Sensor-MH-Z1-p-1863.html
      https://github.com/SandboxElectronics/NDIR/tree/master/NDIR_SoftwareSerial
      https://github.com/sandeepmistry/arduino-LoRa
   
*/
#include <LoRa.h>
#include <SoftwareSerial.h>
#include <NDIR_SoftwareSerial.h>

const unsigned long SensorUploadDelay = 300000;

// LoRa field gateway configuration (these settings must match your field gateway)
const byte FieldGatewayAddress[] = {"LoRaIoT1"};
const byte FieldGatewayAddressLength = strlen( FieldGatewayAddress ) ;
const float FieldGatewayFrequency =  915000000.0;
const byte FieldGatewaySyncWord = 0x12 ;

byte DeviceAddress[] = {"SeeeduinoCO2"};
const byte DeviceAddressLength = strlen( DeviceAddress );

//#define DEBUG_LORA_RADIO
// Dragino LoRa shield radio configuration
const int LoRaChipSelectPin = 10;
const int LoRaResetPin = 9;

// LoRa radio payload configuration
//#define DEBUG_TELEMETRY
const byte PayloadSizeMaximum = 64;
const byte SensorIdValueSeperator = ' ';
const byte SensorReadingSeperator = ',';
byte payload[PayloadSizeMaximum];
byte payloadLength = 0 ;

// CO2 sensor library
#define DEBUG_MHZ16
NDIR_SoftwareSerial CO2Sensor(4, 5);


void setup()
{
  Serial.begin(9600);
  while (!Serial);

  Serial.print("Device address:");
  DisplayHex( DeviceAddress, DeviceAddressLength);
  Serial.println();

  Serial.print("Field gateway:");
  DisplayHex( FieldGatewayAddress, FieldGatewayAddressLength);
  Serial.println();
  Serial.print("Frequency:");
  Serial.print((FieldGatewayFrequency/1000000.0),3 ) ;
  Serial.print("MHz SyncWord:0x");
  Serial.print( FieldGatewaySyncWord, HEX ) ;
  Serial.println();

  Serial.println("LoRa setup start");

  // override the default chip select and reset pins
  LoRa.setPins(LoRaChipSelectPin, LoRaResetPin);
  if (!LoRa.begin(FieldGatewayFrequency))
  {
    Serial.println("LoRa begin failed");
    while (true); // Drop into endless loop requiring restart
  }

  // Need to do this so field gateways pays attention to messsages from this device
  LoRa.enableCrc();
  LoRa.setSyncWord(FieldGatewaySyncWord);

#ifdef DEBUG_LORA_RADIO
  LoRa.dumpRegisters(Serial);
#endif
  Serial.println("LoRa Setup done.");

  PayloadHeader(FieldGatewayAddress,FieldGatewayAddressLength, DeviceAddress, DeviceAddressLength);

  if(!CO2Sensor.begin())
  {
    Serial.println("MZH16 sensor init failed");
    while (true); // Drop into endless loop requiring restart
  }
  delay(10000);
  
  Serial.println("Setup done");
}


void loop()
{
  unsigned long currentMilliseconds = millis();
  unsigned short co2SensorValue = 0 ; 

  PayloadReset();  

  if ( CO2Sensor.measure())
  {
    co2SensorValue = CO2Sensor.ppm;
    
    #ifdef DEBUG_MHZ16
      Serial.print(" CO2:");
      Serial.print( co2SensorValue ) ;
      Serial.println("ppm");
    #endif

    PayloadAdd( "co2", co2SensorValue, true);
  }

#ifdef DEBUG_TELEMETRY
  Serial.println();
  Serial.print( "RFM9X/SX127X Payload length:");
  Serial.print( payloadLength );
  Serial.println( " bytes" );
#endif

  LoRa.beginPacket();
  LoRa.write( payload, payloadLength );
  LoRa.endPacket();
  
  delay(SensorUploadDelay - (millis() - currentMilliseconds ));
}


void PayloadHeader( const byte *to, byte toAddressLength, const byte *from, byte fromAddressLength)
{
  byte addressesLength = toAddressLength + fromAddressLength ;

#ifdef DEBUG_TELEMETRY
  Serial.println("PayloadHeader- ");
  Serial.print( "To Address len:");
  Serial.print( toAddressLength );
  Serial.print( " From Address len:");
  Serial.print( fromAddressLength );
  Serial.print( " Addresses length:");
  Serial.print( addressesLength );
  Serial.println( );
#endif

  payloadLength = 0 ;

  // prepare the payload header with "To" Address length (top nibble) and "From" address length (bottom nibble)
  payload[payloadLength] = (toAddressLength << 4) | fromAddressLength ;
  payloadLength += 1;

  // Copy the "To" address into payload
  memcpy(&payload[payloadLength], to, toAddressLength);
  payloadLength += toAddressLength ;

  // Copy the "From" into payload
  memcpy(&payload[payloadLength], from, fromAddressLength);
  payloadLength += fromAddressLength ;
}


void PayloadAdd( const char *sensorId, float value, byte decimalPlaces, bool lastOne)
{
  byte sensorIdLength = strlen(sensorId ) ;

#ifdef DEBUG_TELEMETRY
  Serial.println("PayloadAdd-float ");
  Serial.print( "SensorId:");
  Serial.print( sensorId );
  Serial.print( " sensorIdLen:");
  Serial.print( sensorIdLength );
  Serial.print( " Value:");
  Serial.print( value, decimalPlaces );
  Serial.print( " payloadLength:");
  Serial.print( payloadLength);
#endif

  memcpy( &payload[payloadLength], sensorId,  sensorIdLength) ;
  payloadLength += sensorIdLength ;
  payload[ payloadLength] = SensorIdValueSeperator;
  payloadLength += 1 ;
  payloadLength += strlen( dtostrf(value, -1, decimalPlaces, (char *)&payload[payloadLength]));
  if ( !lastOne)
  {
    payload[ payloadLength] = SensorReadingSeperator;
    payloadLength += 1 ;
  }
#ifdef DEBUG_TELEMETRY
  Serial.print( " payloadLength:");
  Serial.print( payloadLength);
  Serial.println( );
#endif
}


void PayloadAdd( const char *sensorId, int value, bool lastOne )
{
  byte sensorIdLength = strlen( sensorId ) ;

#ifdef DEBUG_TELEMETRY
  Serial.println("PayloadAdd-int ");
  Serial.print( "SensorId:");
  Serial.print( sensorId );
  Serial.print( " sensorIdLen:");
  Serial.print( sensorIdLength );
  Serial.print( " Value:");
  Serial.print( value );
  Serial.print( " payloadLength:");
  Serial.print( payloadLength);
#endif  

  memcpy( &payload[payloadLength], sensorId,  sensorIdLength) ;
  payloadLength += sensorIdLength ;
  payload[ payloadLength] = SensorIdValueSeperator;
  payloadLength += 1 ;
  payloadLength += strlen( itoa( value,(char *)&payload[payloadLength],10));
  if ( !lastOne)
  {
    payload[ payloadLength] = SensorReadingSeperator;
    payloadLength += 1 ;
  }  
#ifdef DEBUG_TELEMETRY
  Serial.print( " payloadLength:");
  Serial.print( payloadLength);
  Serial.println( );
#endif
}


void PayloadAdd( const char *sensorId, unsigned int value, bool lastOne )
{
  byte sensorIdLength = strlen( sensorId ) ;

#ifdef DEBUG_TELEMETRY
  Serial.println("PayloadAdd-unsigned int ");
  Serial.print( "SensorId:");
  Serial.print( sensorId );
  Serial.print( " sensorIdLen:");
  Serial.print( sensorIdLength );
  Serial.print( " Value:");
  Serial.print( value );
  Serial.print( " payloadLength:");
  Serial.print( payloadLength);
#endif  

  memcpy( &payload[payloadLength], sensorId,  sensorIdLength) ;
  payloadLength += sensorIdLength ;
  payload[ payloadLength] = SensorIdValueSeperator;
  payloadLength += 1 ;
  payloadLength += strlen( utoa( value,(char *)&payload[payloadLength],10));
  if ( !lastOne)
  {
    payload[ payloadLength] = SensorReadingSeperator;
    payloadLength += 1 ;
  }
  
#ifdef DEBUG_TELEMETRY
  Serial.print( " payloadLength:");
  Serial.print( payloadLength);
  Serial.println( );
#endif
}


void PayloadReset()
{
  byte fromAddressLength = payload[0] & 0xf ;
  byte toAddressLength = payload[0] >> 4 ;
  byte addressesLength = toAddressLength + fromAddressLength ;

  payloadLength = addressesLength + 1;

#ifdef DEBUG_TELEMETRY
  Serial.println("PayloadReset");
  Serial.print( "To Address len:");
  Serial.print( toAddressLength );
  Serial.print( " From Address len:");
  Serial.print( fromAddressLength );
  Serial.print( " Addresses length:");
  Serial.print( addressesLength );
  Serial.println( );
#endif
}


void DisplayHex( const byte *byteArray, byte length) 
{
  for (int i = 0; i < length ; i++)
  {
    // Add a leading zero
    if ( byteArray[i] < 16)
    {
      Serial.print("0");
    }
    Serial.print(byteArray[i], HEX);
    if ( i < (length-1)) // Don't put a - after last digit
    {
      Serial.print("-");
    }
  }
}
