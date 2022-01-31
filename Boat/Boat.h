#ifndef _BOAT_H
#define _BOAT_H

#define PI 3.14159265

/***********
 * Structs *
 ***********/
struct GPS {
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
const float bubble = 20.0; // Radius of circle around waypoint
const float DPS = 0.05; // Degrees * DPS = Seconds of Rotation
const int HALFTIME = 7;
const int FULLTIME = 15;
const int HALFSPEED = 180;
const int FULLSPEED = 255;
const int SETTLE_TIME = 500;

// SD card
Sd2Card card;
SdVolume volume;
SdFile root;
SdFile inFile;
SdFile outFile;
char inFILENAME[] = "PONDS";
char bufferedInFile[500];
char outFILENAME[10];

// Motor Pins
const byte SPDA = 3;
const byte SPDB = 4;
const byte DIRA1 = 2;
const byte DIRA2 = 5;
const byte DIRB1 = 6;
const byte DIRB2 = 7;


/**************
 * Prototypes *
 **************/
void setupArdumoto();
int setupSDReader();
int readline(int readch, char *buffer, int len);
float getHeading();
void CreateOutFile();
void readFile2Buffer();
struct GPS* getDestination(GPS* destination, int waypoint);
struct GPS* getLocation(GPS* currentLocation);
struct sensorData* waitOnSensor(sensorData* currentData);
struct sensorData* getData(sensorData* currentData, int haveDepth, int haveTemp);
void recordData(GPS* currentLocation, sensorData* currentData);
float desiredHeading(GPS* currentLocation, GPS* desiredLocation);
float distance2waypoint(GPS* currentLocation, GPS* destination);
void navigate(float destinationHeading, float distance);
void engage(int duration);
void rotateCW(int duration);
void rotateCCW(int duration);


#endif
