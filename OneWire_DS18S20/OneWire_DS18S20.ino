#include <OneWire.h>

// DS18S20 Temperature chip i/o
OneWire ds(2);  // on pin 2

byte present = 0;
byte data[12];
byte addr[8];

void setup(void) {
  // initialize inputs/outputs
  // start serial port
  Serial.begin(9600);
  ds.reset_search();
  if ( !ds.search(addr)) {
    Serial.print("No more addresses.\n");
    ds.reset_search();
    return;
  }

  Serial.print("R=");
  for(int i = 0; i < 8; i++) {
    Serial.print(addr[i], HEX);
    Serial.print(" ");
  }

  if ( OneWire::crc8( addr, 7) != addr[7]) {
    Serial.print("CRC is not valid!\n");
    return;
  }

  if ( addr[0] == 0x10) {
    Serial.print("Device is a DS18S20 family device.\n");
  }
  else if ( addr[0] == 0x28) {
    Serial.print("Device is a DS18B20 family device.\n");
  }
  else {
    Serial.print("Device family is not recognized: 0x");
    Serial.println(addr[0],HEX);
    return;
  }

}

void loop(void) {
  byte i;
  float temperature = 0.0;
  int rawtemp, rawint, rawfract;

  ds.reset();
  ds.select(addr);
  ds.write(0x44,0);         // start conversion

  delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.

  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read Scratchpad

  Serial.print("P=");
  Serial.print(present,HEX);
  Serial.print(" ");
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.print(" CRC=");
  Serial.print( OneWire::crc8( data, 8), HEX);
  Serial.println();
  rawtemp = (data[1] << 8) + data[0];
  rawfract = rawtemp & 0x0F;
  rawint = rawtemp >> 4;
  for(int i = 0; i < 4; i++)
  {
    rawfract = rawfract >> i;
    temperature = temperature + (0.0625 * (i+1)) * (rawfract & 0x01);
  }
  temperature = temperature + (float) rawint;
  Serial.print("Temp = ");
  Serial.println(temperature,4);
}



