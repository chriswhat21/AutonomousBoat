extern HardwareSerial Serial1; // GPS
extern HardwareSerial Serial2; // Transducer
#include <Wire.h> // Compass
#include <SD.h>
#include <math.h>
#include <TinyGPS.h>
#include "Boat.h"


void setup()
{
  delay(2000);
  
  // Initialize Comms
  Serial.begin(9600);// Serial Monitor
  Serial1.begin(57600);// For GPS
  Serial2.begin(4800); // For Transducer
  Wire.begin();// For Compass
  setupArdumoto(); // Set all pins as outputs for motor
  
  // Declare & Initialize variables
  float heading_reqd, distance_reqd;
  int Waypoint = 0, withinLat, withinLon;
  int Valid;
  
  Valid = setupSDReader();
  if(Valid == -1){
    Serial.println("No SD Card Present. Cannot Continue.");
    return;
  }
  CreateOutFile();
  readFile2Buffer();
  Serial.print("The contents of ");Serial.print(inFILENAME);Serial.println(":");
  Serial.println(bufferedInFile);
  GPS* Location;
  Location = (GPS*) malloc (sizeof(GPS));

  GPS* Destination;
  Destination = (GPS*) malloc (sizeof(GPS));

  sensorData* Data;
  Data = (sensorData*) malloc (sizeof(sensorData));

 /********* 
  * Begin *
  *********/
  do{
    Waypoint++;
    Serial.print("Waypoint: ");Serial.println(Waypoint);
    do{
      withinLat = 0;withinLon = 0;
      Location->Latitude = 0;
      Location->Longitude = 0;
      Destination->Latitude = 0;
      Destination->Longitude = 0;
      Data->Depth = -9.9;
      Data->Temp = -9.9;
      
      Location = getLocation(Location);
      Serial.print("Current GPS: ");Serial.print(Location->Latitude);Serial.print(", ");Serial.println(Location->Longitude);
      Data = waitOnSensor(Data);
      recordData(Location, Data);
      
      Destination = getDestination(Destination, Waypoint);
      Serial.print("Dest GPS: ");Serial.print(Destination->Latitude);Serial.print(", ");Serial.println(Destination->Longitude);

      if((Location->Latitude > Destination->Latitude - GPSError) && (Location->Latitude < Destination->Latitude + GPSError)){
        withinLat = 1;
        Serial.println("Within Latitude");
      }
      if((Location->Longitude > Destination->Longitude - GPSError) && (Location->Longitude < Destination->Longitude + GPSError)){
        withinLon = 1;
        Serial.println("Within Longitude");
      }
      if(withinLat == 0 || withinLon == 0){
        heading_reqd = desiredHeading(Location, Destination);
        distance_reqd = distance2waypoint(Location, Destination);
        navigate(heading_reqd, distance_reqd);
      }
    }while(withinLat == 0 || withinLon == 0);
  }while(Destination->Latitude != 0);
}


void loop()
{

}


/****************************
 * Initialize Ardumoto Pins *
 ****************************/
void setupArdumoto()
{
  pinMode(SPDA, OUTPUT);
  pinMode(SPDB, OUTPUT);
  pinMode(DIRA1, OUTPUT);
  pinMode(DIRA2, OUTPUT);
  pinMode(DIRB1, OUTPUT);
  pinMode(DIRB2, OUTPUT);

  digitalWrite(SPDA, LOW);
  digitalWrite(SPDB, LOW);
  digitalWrite(DIRA1, LOW);
  digitalWrite(DIRA2, LOW);
  digitalWrite(DIRB1, LOW);
  digitalWrite(DIRB2, LOW);
}


/************************
 * Initialize SD Reader *
 ************************/
int setupSDReader()
{
  pinMode(chipSelect, OUTPUT);
    if(!SD.begin(chipSelect)){
    return -1;
  }
  card.init();               //Initialize the SD card and configure the I/O pins.
  volume.init(card);         //Initialize a volume on the SD card.
  root.openRoot(volume);     //Open the root directory in the volume.
  
  return 1;
}


/********************************************
 * Reads a line of data from the Transducer *
 *******************************************/
int readline(int readch, char *buffer, int len)
{
  static int pos = 0;
  int rpos;
  
  if(readch > 0){
    switch(readch){
      case '\n': // Ignore new-lines
        break;
      case '\r': // Return on CR
        rpos = pos;
        pos = 0;  // Reset position index ready for next time
        return rpos;
      default:
        if(pos < len-1){
          buffer[pos++] = readch;
          buffer[pos] = 0;
        }
    }
  }
  // No end of line has been found, so return -1.
  return -1;
}


/***************************
 * Returns compass heading *
 ***************************/
float getHeading()
{
  int compassAddress = 0x42 >> 1;
  Wire.beginTransmission(compassAddress);
  Wire.write("A");// 'A' is the Compass "read_data" command
  Wire.endTransmission();
  delay(10);// Wait for the Compass to be ready

  Wire.requestFrom(compassAddress, 2); // Request 2 bytes of data, MSB and LSB
  byte MSB = Wire.read();
  byte LSB = Wire.read();
  float headingSum = (MSB << 8) + LSB; // Combine bytes
  float heading = headingSum / 10; // xxxx->xxx.x

  Serial.print("Current heading: "); Serial.print(heading); Serial.println(" degrees.");
  return heading;
}


/**********************
 * Create Output File *
 **********************/
void CreateOutFile()
{
  File myFile;
  outFILENAME[0] = '\0';
  strcat(outFILENAME, inFILENAME);
  strcat(outFILENAME, "_D");
  if(SD.exists(outFILENAME)){
    Serial.print(outFILENAME);Serial.println(" already exists.");
  }
  else{
    Serial.print(outFILENAME);Serial.println(" doesn't exist.");
    
    // open a new file and immediately close it:
    Serial.print("Creating ");Serial.print(outFILENAME);Serial.print("...");
    myFile = SD.open(outFILENAME, FILE_WRITE);
    myFile.close();
  
    if(SD.exists(outFILENAME)){
      Serial.print(outFILENAME);Serial.println(" created.");
    }
    else{
      Serial.print(outFILENAME);Serial.println(" could not be created.");
    }
  }
  
  char buffer[14];
  String dataString = "";
  unsigned long fix_age;
  TinyGPS gps;
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;
  int year;
  byte month, day, hour, minute, second, hundredths;
  
  for(unsigned long start = millis(); millis() - start < 1000;){
    while(Serial1.available()){
      char c = Serial1.read();
      if(gps.encode(c))
        newData = true;
    }
  }
  if(newData){
    gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &fix_age);
    hour = hour - 5; // UTC to CST
  }
  // For Errors
  gps.stats(&chars, &sentences, &failed);
  if(chars == 0){
    month = 01;
    day = 01;
    year = 2014;
    hour = 00;
    minute = 01;
    second = 01;
  }
  dataString += month;  
  dataString += "/";
  dataString += day;  
  dataString += "/";
  dataString += year;
  dataString += " - ";
  dataString += hour;
  dataString += ":";
  dataString += minute;
  dataString += ":";
  dataString += second;
  File outFILE = SD.open(outFILENAME, FILE_WRITE);
  if(!outFILE){
    Serial.println("Output File Not Found");
  }
  else{
    Serial.println(dataString);
    outFILE.println(dataString);
    outFILE.close();
  }
}


/******************************
 * Reads Input file to buffer *
 ******************************/
void readFile2Buffer()
{
  inFile.open(root, inFILENAME, O_READ);
  int i = 0;
  if(!inFile.isOpen()){
    Serial.println("Input File Not Found");
  }else{
    bufferedInFile[i] = inFile.read();
    
    while(bufferedInFile[i] >= 0 && i < 500){
      i++;
      bufferedInFile[i] = inFile.read();
    }
    inFile.close();
  }
}


/*****************************
 * Returns Waypoint Lat/Long *
 *****************************/
struct GPS* getDestination(GPS* destination, int waypoint)
{
  int i,n = 0;
  int LatSignPos = 25 * (waypoint - 1);
  char Lat[9];
  char Lon[10];
  /** Sample:
       0|+,00000000L,-,000000000L\n|24
      25|+,00000000L,-,000000000L\n|49
      50|+,00000000L,-,000000000L\n|74
  => 25*(Waypoint - 1) = "+" of current Waypoint
  0 = Sign Latitude, 2->10 = Latitude, 
  12 = Sign Longitude, 14->23 = Longitude
  **/
  // Get Latitude
  for(i = LatSignPos + 2; i <= LatSignPos + 10; i++){
    Lat[n] = bufferedInFile[i];
    n++;
  }
  // Get Longitude
  n = 0;
  for(i = LatSignPos + 14; i <= LatSignPos + 23; i++){
    Lon[n] = bufferedInFile[i];
    n++;
  }
  destination->Latitude = atol(Lat);
  destination->Longitude = atol(Lon);
  
  if(bufferedInFile[LatSignPos] == '-'){
    destination->Latitude = destination->Latitude * (-1L);
  }
  if(bufferedInFile[LatSignPos + 12] == '-'){
    destination->Longitude = destination->Longitude * (-1L);
  }
  return destination;
}


/****************************
 * Returns Current GPS Data *
 ****************************/
struct GPS* getLocation(GPS* Location)
{
  TinyGPS gps;
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;
  long lat, lon;

  for(unsigned long start = millis(); millis() - start < 1000;){
    while(Serial1.available()){
      char c = Serial1.read();
      if(gps.encode(c))
        newData = true;
    }
  }
  if(newData){
    unsigned long age;
    gps.get_position(&lat, &lon, &age);
  }
  // For Errors
  gps.stats(&chars, &sentences, &failed);
  if(chars == 0){
    Serial.println("** No characters received from GPS: check wiring **");
    Location->Latitude = -1;
    Location->Longitude = -1;
    return Location;
  }
  Location->Latitude = lat;
  Location->Longitude = lon;
  return Location;
}


/*************************************
 * Waits on Data from the Transducer *
 *************************************/
struct sensorData* waitOnSensor(sensorData* currentData)
{
  int haveTemp = 0, haveDepth = 0;
  while(haveTemp == 0 || haveDepth == 0){
    getData(currentData, haveDepth, haveTemp);
    if(currentData->Depth != -9.9){
      haveDepth = 1;
    }
    if(currentData->Temp != -9.9){
      haveTemp = 1;
    }
  }
  Serial.print("Depth = ");Serial.print(currentData->Depth);Serial.println(" ft");
  Serial.print("Temp = ");Serial.print(currentData->Temp);Serial.println(" C");
  return currentData;
}


/***********************************
 * Returns Current Transducer Data *
 ***********************************/
struct sensorData* getData(sensorData* currentData, int haveDepth, int haveTemp)
{
  static char buffer[80];
  char *p = buffer;
  char *str;
  char *tmp;

  if(readline(Serial2.read(), buffer, 80) > 0){
    if( buffer[0] == '$' ){
      while((str = strtok_r(p, ",", &p)) != NULL ){ // delimiter is the comma
        //Serial.print("Current str = ");Serial.println(str);
        //Serial.print("Current tmp = ");Serial.println(tmp);
        if(strcmp(tmp, "$SDDBT") == 0 && haveDepth == 0){
          if(strcmp(str, "f") == 0){
            currentData->Depth = -1.0;
          }else{
            currentData->Depth = atof(str);
          }
          Serial.print("Depth = ");Serial.print(currentData->Depth);Serial.println(" ft");
          haveDepth = 1;
        }
        if(strcmp(tmp, "$YXMTW") == 0 && haveTemp == 0){
          currentData->Temp = atof(str);
          Serial.print("Temp = ");Serial.print(currentData->Temp);Serial.println(" C");
          haveTemp = 1;
        }
        else{
          
        }
        tmp = str;
      }
    }
  }
  return currentData;
}


/***********************************************************
 * Records given Record#, Lat/Long, Depth, Temp to SD Card *
 ***********************************************************/
void recordData(GPS* currentLocation, sensorData* currentData)
{
  char buffer[14];
  String dataString = "";
  unsigned long fix_age;
  TinyGPS gps;
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;
  int year;
  byte month, day, hour, minute, second, hundredths;
  
  for(unsigned long start = millis(); millis() - start < 1000;){
    while(Serial1.available()){
      char c = Serial1.read();
      if(gps.encode(c))
        newData = true;
    }
  }
  if(newData){
    gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &fix_age);
    hour = hour - 5; // UTC -> Central
  }
  // For Errors
  gps.stats(&chars, &sentences, &failed);
  if(chars == 0){
    hour = 00;
    minute = 01;
    second = 01;
  }
  dataString += hour;  
  dataString += ":";
  dataString += minute;  
  dataString += ":";
  dataString += second;  
  dataString += " - ";
  dataString += ltoa(currentLocation->Latitude, buffer, 10);
  dataString += ", ";
  dataString += ltoa(currentLocation->Longitude, buffer, 11);
  dataString += ", ";
  dataString += dtostrf(currentData->Depth, 6, 2, buffer);
  dataString += ", ";
  dataString += dtostrf(currentData->Temp, 6, 2, buffer);
  
  File outFILE = SD.open(outFILENAME, FILE_WRITE);
  if(!outFILE){
    Serial.println("Output File Not Found");
  }else{
    Serial.println(dataString);
    outFILE.println(dataString);
    outFILE.close();
  }
}


/***************************************************************
 * Returns Desired Heading Given Waypoint and Current Lat/Long *
 ***************************************************************/
float desiredHeading(GPS* currentLocation, GPS* destination){
  float x = (currentLocation->Latitude - destination->Latitude);
  float y = (currentLocation->Longitude - destination->Longitude);
  Serial.print(x,6);Serial.print(", ");Serial.print(y,6);Serial.println(" (x, y)");
  float heading = (atan2f(y, x)) * 180 / PI;
  // Quad I
  if(x > 0 && y < 0){
    Serial.println("I");
    heading = heading += 180; 
  }
  // Quad II
  if(x < 0 && y > 0){
    Serial.println("II");
    heading += 180; 
  }
  // Quad III
  if(x > 0 && y > 0){
    Serial.println("III");
   heading += 180; 
  }
  // Quad IV
  if(x < 0 && y < 0){
    Serial.println("IV");
    heading += 180; 
  }
  
  return heading;
}


/********************************
 * Returns Distance to waypoint *
 ********************************/
float distance2waypoint(GPS* currentLocation, GPS* destination)
{
  float a = (currentLocation->Latitude - destination->Latitude);
  float b = (currentLocation->Longitude - destination->Longitude);
  float c = sqrt(a*a + b*b);
  c = c * (3.0 / 10); // Convert to feet
  return c;
}


/************************************
 * Adjusts heading to Given Heading *
 ************************************/
void navigate(float destinationHeading, float distance)
{
  float currentHeading, delta;
  do{
    currentHeading = getHeading();
    Serial.print("Needed heading: "); Serial.print(destinationHeading); Serial.println(" degrees.");
    delta = currentHeading - destinationHeading;
    if(delta > compassError){
      delta = abs(delta);
      if(abs(delta) <= 180){
        Serial.print("Rotate CCW "); Serial.print(delta); Serial.println(" degrees");
        rotateCCW(delta*DPS);
      }
      else{
        delta = 360 - abs(delta);
        Serial.print("Rotate CW "); Serial.print(delta); Serial.println(" degrees");
        rotateCW(delta*DPS);
      }
    }
    else if(delta < (0 - compassError)){
      delta = abs(delta);
      if(delta <= 180){
        Serial.print("Rotate CW "); Serial.print(delta); Serial.println(" degrees");
        rotateCW(delta*DPS);
      }
      else{
        delta = 360 - delta;
        Serial.print("Rotate CCW "); Serial.print(delta); Serial.println(" degrees");
        rotateCCW(delta*DPS);
      }
    }
    delay(SETTLE_TIME); // Wait for boat to settle
  }
  while(delta < (0 - compassError) || delta > compassError);
  Serial.println("Current Heading is within value");
  Serial.print("Distance: "); Serial.print(distance); Serial.println(" ft");
  if(distance < 10){
    engage(HALFTIME);
  }else{
    engage(FULLTIME);
  }
}


/*************************************************
 * Drives straight for a given number of seconds *
 *************************************************/
void engage(int duration)
{
  // Motor A Forward
  digitalWrite(DIRA1, 1);
  digitalWrite(DIRA2, 0);
  // Motor B Forward
  digitalWrite(DIRB1, 0);
  digitalWrite(DIRB2, 1);
  // Go
  analogWrite(SPDA, FULLSPEED);
  analogWrite(SPDB, FULLSPEED);
  delay(duration*1000);
  // Stop
  analogWrite(SPDA, 0);
  analogWrite(SPDB, 0);
  // Clear
  digitalWrite(DIRA1, 0);
  digitalWrite(DIRA2, 0);
  digitalWrite(DIRB1, 0);
  digitalWrite(DIRB2, 0);
}


/*************************************************
 * Rotates boat CCW for a given number of seconds *
 *************************************************/
void rotateCCW(int duration)
{
  // Motor A Forward
  digitalWrite(DIRA1, 1);
  digitalWrite(DIRA2, 0);
  // Motor B Reverse
  digitalWrite(DIRB1, 1);
  digitalWrite(DIRB2, 0);
  // Go
  analogWrite(SPDA, FULLSPEED);
  analogWrite(SPDB, HALFSPEED);
  delay(duration*1000);
  // Stop
  analogWrite(SPDA, 0);
  analogWrite(SPDB, 0);
  // Clear
  digitalWrite(DIRA1, 0);
  digitalWrite(DIRA2, 0);
  digitalWrite(DIRB1, 0);
  digitalWrite(DIRB2, 0);
}


/**************************************************
 * Rotates boat CW for a given number of seconds *
 **************************************************/
void rotateCW(int duration)
{
  // Motor A Reverse
  digitalWrite(DIRA1, 0);
  digitalWrite(DIRA2, 1);
  // Motor B Forward
  digitalWrite(DIRB1, 0);
  digitalWrite(DIRB2, 1);
  // Go
  analogWrite(SPDA, HALFSPEED);
  analogWrite(SPDB, FULLSPEED);
  delay(duration*1000);
  // Stop
  analogWrite(SPDA, 0);
  analogWrite(SPDB, 0);
  // Clear
  digitalWrite(DIRA1, 0);
  digitalWrite(DIRA2, 0);
  digitalWrite(DIRB1, 0);
  digitalWrite(DIRB2, 0);
}
