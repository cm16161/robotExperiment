#ifndef _Mapping_h
#define _Mapping_h

#include <EEPROM.h>
#include <USBCore.h>    // To fix serial print behaviour bug.
u8 USB_SendSpace(u8 ep);
#define SERIAL_ACTIVE (USB_SendSpace(CDC_TX) >= 50)

const byte MAP_RESOLUTION = 5;
const byte MAP_DEFAULT_FEATURE = '_';
const int MAP_X = 5;
const int MAP_Y = 5;

class Mapper {
  public:
    void resetMap();
    void printMap();
    void updateMapFeature(byte feature, int y, int x);
    void updateMapFeature(byte feature, float y, float x);

    int  indexToPose(int i, int map_size, int resolution);
    int  poseToIndex(int x, int map_size, int resolution);

  private:
    int X_size;
    int Y_size;
};

void Mapper::resetMap() {

  for (int i = 0; i < MAP_RESOLUTION; i++) {
    for (int j = 0; j < MAP_RESOLUTION; j++) {

      int eeprom_address = (i * MAP_RESOLUTION) + j;

      if (eeprom_address > 1023) {
        if ( SERIAL_ACTIVE ) Serial.println(F("Error: EEPROM Address greater than 1023"));
      } else {
        EEPROM.update(eeprom_address, MAP_DEFAULT_FEATURE );
      }
    }
  }
}

void Mapper::printMap() {

  if ( SERIAL_ACTIVE ) {
    Serial.println("Map");
    for (int i = 0; i < MAP_RESOLUTION; i++) {
      for (int j = 0; j < MAP_RESOLUTION; j++) {
        int eeprom_address = (i * MAP_RESOLUTION) + j;
        byte value;
        value = EEPROM.read(eeprom_address);//, value);
        Serial.print( (char)value );
        Serial.print(" ");
      }
      Serial.println("");
    }
  }
}

int Mapper::poseToIndex(int x, int map_size, int resolution)
{
  return x / (map_size / resolution);
}

int Mapper::indexToPose(int i, int map_size, int resolution)
{
  return i * (map_size / resolution);
}


void Mapper::updateMapFeature(byte feature, float y, float x) {
  updateMapFeature( feature, (int)y, (int)x );
}

void Mapper::updateMapFeature(byte feature, int y, int x) {

  if (x > MAP_X || x < 0 || y > MAP_Y || y < 0) {
    if(SERIAL_ACTIVE) {
      Serial.println(F("Error:Invalid co-ordinate"));
      Serial.print("x = ");
      Serial.print(x);
      Serial.print(", MAP_X = ");
      Serial.print(MAP_X);
      Serial.print(", y = ");
      Serial.print(y);
      Serial.print(", MAP_Y = ");
      Serial.println(MAP_Y);
       
      }
    return;
  }

  //int x_index = poseToIndex(x, MAP_X, MAP_RESOLUTION);
  //int y_index = poseToIndex(y, MAP_Y, MAP_RESOLUTION);

  //int eeprom_address = (x_index * MAP_RESOLUTION) + y_index;
  int eeprom_address = (x * MAP_RESOLUTION) + y;

  if (eeprom_address > 1023) {
    if(SERIAL_ACTIVE)Serial.println(F("Error: EEPROM Address greater than 1023"));
  } else {
    EEPROM.update(eeprom_address, feature);
  }
}


#endif
