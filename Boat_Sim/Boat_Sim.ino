#include <math.h>
#include "Boat_Sim.h"


void setup()
{
  delay(2000);
  
  // Initialize Comms
  Serial.begin(9600);// Serial Monitor
  setupArdumoto(); // Set all pins as outputs for motor
  
  // Declare & Initialize variables
  float heading_reqd, distance_reqd;
  int Waypoint = 0, closeEnough;

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
      closeEnough = 0;
      Location->Latitude = 0;
      Location->Longitude = 0;
      Destination->Latitude = 0;
      Destination->Longitude = 0;
      Data->Depth = -9.9;
      Data->Temp = -9.9;
      
      Location = getLocation(Location);
      recordData(Location, Data);
      
      Destination = getDestination(Destination, Waypoint);
      Serial.print("Dest GPS: ");Serial.print(Destination->Latitude);Serial.print(", ");Serial.println(Destination->Longitude);
 
     distance_reqd = distance2waypoint(Location, Destination);
      if(distance_reqd < bubble){
        closeEnough = 1;
      }
      else{
        heading_reqd = desiredHeading(Location, Destination);
        navigate(heading_reqd, distance_reqd);
      }
      
    }while(closeEnough == 0);
    Serial.println("Close Enough!");
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


/***************************
 * Returns compass heading *
 ***************************/
float getHeading(){
  float heading;
  
  Serial.println("Enter current heading: ");
  heading = getFloatFromSerialMonitor();
  Serial.println();
  Serial.print("Current heading: "); Serial.print(heading); Serial.println(" degrees.");
  return heading;
}

float getFloatFromSerialMonitor(){
  char inData[20];  
  float f=0;    
  int x=0;  
  while (x<1){  
    String str;   
    if (Serial.available()) {
      delay(100);
      int i=0;
      while (Serial.available() > 0) {
       char  inByte = Serial.read();
        str=str+inByte;
        inData[i]=inByte;
        i+=1;
        x=2;
      }
      f = atof(inData);
      memset(inData, 0, sizeof(inData));  
    }
  }//END WHILE X<1  
   return f; 
}

long getLongFromSerialMonitor(){
  char inData[20];  
  long l=0;    
  int x=0;  
  while (x<1){  
    String str;   
    if (Serial.available()) {
      delay(100);
      int i=0;
      while (Serial.available() > 0) {
       char  inByte = Serial.read();
        str=str+inByte;
        inData[i]=inByte;
        i+=1;
        x=2;
      }
      l = atol(inData);
      memset(inData, 0, sizeof(inData));  
    }
  }//END WHILE X<1
  Serial.println(l);
  return l; 
}
/******************************
 * Reads Input file to buffer *
 ******************************/
void readFile2Buffer(){
  char FakeInFile[] = "+,30574175L,-,087253908L\n+,30574109L,-,087254008L\n+,30574175L,-,087253908L\n+,30574109L,-,087254008L\n+,30574175L,-,087253908L\n+,30574109L,-,087254008L";
  int i = 0;

  while(bufferedInFile[i] >= 0 && i < 500){
    bufferedInFile[i] = FakeInFile[i];
    i++;
  }
}


/*****************************
 * Returns Waypoint Lat/Long *
 *****************************/
struct GPS* getDestination(GPS* destination, int waypoint){
  int i,n = 0;
  int LatSignPos = 25 * (waypoint - 1);
  //int LonSignPos = LatSignPos + 12;
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
  
  if (bufferedInFile[LatSignPos] == '-'){
    destination->Latitude = destination->Latitude * (-1L);
  }
  if (bufferedInFile[LatSignPos + 12] == '-'){
    destination->Longitude = destination->Longitude * (-1L);
  }
  
  
  return destination;
}


/****************************
 * Returns Current GPS Data *
 ****************************/
struct GPS* getLocation(GPS* Location){
  long lat, lon;
  Serial.print("Enter Current Lat: ");
  lat = getLongFromSerialMonitor();
  Serial.println();
  Serial.print("Enter Current Lon: ");
  lon = getLongFromSerialMonitor();
  Serial.println();
  Location->Latitude = lat;
  Location->Longitude = lon;
  
  return Location;
}


/***********************************************************
 * Records given Record#, Lat/Long, Depth, Temp to SD Card *
 ***********************************************************/
void recordData(GPS* currentLocation, sensorData* currentData){
  char buffer[14];
  String dataString = "";
  byte hour, minute, second;

  hour = 8;
  minute = 45;
  second = 30;
  
  dataString += hour;  
  dataString += ":";
  dataString += minute;  
  dataString += ":";
  dataString += second;  
  dataString += " - ";
  dataString += ltoa(currentLocation->Latitude, buffer, 10);
  dataString += ", ";
  dataString += ltoa(currentLocation->Longitude, buffer, 10);
  dataString += ", ";
  dataString += dtostrf(currentData->Depth, 6, 2, buffer);
  dataString += ", ";
  dataString += dtostrf(currentData->Temp, 6, 2, buffer);
  Serial.println(dataString);
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
    heading = heading * -1; 
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
float distance2waypoint(GPS* currentLocation, GPS* destination){
  float a = (currentLocation->Latitude - destination->Latitude);
  float b = (currentLocation->Longitude - destination->Longitude);
  float c = sqrt(a*a + b*b);
  c = c * (3.0 / 10); // Convert to feet
  return c;
}


/************************************
 * Adjusts heading to Given Heading *
 ************************************/
void navigate(float destinationHeading, float distance){
  float currentHeading, delta;
  do{
    currentHeading = getHeading();
    Serial.print("Needed heading: "); Serial.print(destinationHeading); Serial.println(" degrees.");
    delta = currentHeading - destinationHeading;
    if(delta > compassError){
      delta = abs(delta);
      if (abs(delta) <= 180){
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
      if (delta <= 180){
        Serial.print("Rotate CW "); Serial.print(delta); Serial.println(" degrees");
        rotateCW(delta*DPS);
      }
      else{
        delta = 360 - delta;
        Serial.print("Rotate CCW "); Serial.print(delta); Serial.println(" degrees");
        rotateCCW(delta*DPS);
      }
    }

    delay(2000); // Wait for boat to settle
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
void engage(int duration){
  // Motor A Forward
  digitalWrite(DIRA1, 1);
  digitalWrite(DIRA2, 0);
  // Motor B Forward
  digitalWrite(DIRB1, 0);
  digitalWrite(DIRB2, 1);
  // Go
  analogWrite(SPDA, 255);
  analogWrite(SPDB, 255);
  delay(duration*1000);
  // Stop
  analogWrite(SPDA, 0);
  analogWrite(SPDB, 0);
  digitalWrite(DIRA1, 0);
  digitalWrite(DIRA2, 0);
  digitalWrite(DIRB1, 0);
  digitalWrite(DIRB2, 0);
}


/*************************************************
 * Rotates boat CCW for a given number of seconds *
 *************************************************/
void rotateCCW(int duration){
  // Motor A Forward
  digitalWrite(DIRA1, 1);
  digitalWrite(DIRA2, 0);
  // Motor B Reverse
  digitalWrite(DIRB1, 1);
  digitalWrite(DIRB2, 0);
  // Go
  analogWrite(SPDA, 127);
  analogWrite(SPDB, 127);
  delay(duration*1000);
  // Stop
  analogWrite(SPDA, 0);
  analogWrite(SPDB, 0);
  digitalWrite(DIRA1, 0);
  digitalWrite(DIRA2, 0);
  digitalWrite(DIRB1, 0);
  digitalWrite(DIRB2, 0);
}


/**************************************************
 * Rotates boat CW for a given number of seconds *
 **************************************************/
void rotateCW(int duration){
  // Motor A Reverse
  digitalWrite(DIRA1, 0);
  digitalWrite(DIRA2, 1);
  // Motor B Forward
  digitalWrite(DIRB1, 0);
  digitalWrite(DIRB2, 1);
  // Go
  analogWrite(SPDA, 127);
  analogWrite(SPDB, 127);
  delay(duration*1000);
  // Stop
  analogWrite(SPDA, 0);
  analogWrite(SPDB, 0);
  digitalWrite(DIRA1, 0);
  digitalWrite(DIRA2, 0);
  digitalWrite(DIRB1, 0);
  digitalWrite(DIRB2, 0);
}
