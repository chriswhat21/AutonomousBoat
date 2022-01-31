#ifndef _BOAT_SIM_H
#define _BOAT_SIM_H

#define PI 3.14159265

/***********
 * Structs *
 ***********/
struct GPS
{
  long Latitude;
  long Longitude;
};

struct sensorData
{
  float Depth;
  float Temp;
};

/******************
 * Define Globals *
 ******************/
// Chip Select Pin on the Arduino Mega
const int chipSelect = 8;

// Equipment error allowance
const float compassError = 7.5;
const float bubble = 20;
const float DPS = 0.10; // Degrees Per Second
const int HALFTIME = 10;
const int FULLTIME = 15;

char inFILENAME[] = "FAKE";
char bufferedInFile[500];
char outFILENAME[10];

// Motor Pins
const int SPDA = 3;
const int SPDB = 4;
const int DIRA1 = 2;
const int DIRA2 = 5;
const int DIRB1 = 6;
const int DIRB2 = 7;


/**************
 * Prototypes *
 **************/
void setupArdumoto();
float getHeading();
float getFloatFromSerialMonitor();
long getLongFromSerialMonitor();
void readFile2Buffer();
struct GPS* getDestination(GPS* destination, int waypoint);
struct GPS* getLocation(GPS* currentLocation);
void recordData(GPS* currentLocation, sensorData* currentData);
float desiredHeading(GPS* currentLocation, GPS* desiredLocation);
float distance2waypoint(GPS* currentLocation, GPS* destination);
void navigate(float destinationHeading, float distance);
void engage(int duration);
void rotateCW(int duration);
void rotateCCW(int duration);

#endif
