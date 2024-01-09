#include <stdlib.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <avr/wdt.h>

#include <math.h>

//Experimental rotation factor . will change it later according to practical needs
double rot_factor = 0.0135;
int enA = 9;
int in1 = 8;
int in2 = 7;
// Motor B connections
int enB = 3;
int in3 = 6;
int in4 = 5;
int RXPin = 12;
int TXPin = 11;
int GPSBaud = 9600;
double tolerance = 0.00002;
TinyGPSPlus gps;
SoftwareSerial gpsSerial(RXPin, TXPin);

char num;
double initial_bearing;
double last_bearing;
struct GPS {
  double latitude;
  double longitude;
};
GPS initial_point;
GPS last_point;
GPS target_point;
double target_bearing;

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(GPSBaud);
  // wdt_enable(WDTO_4S);

  // set_speed(9 ,0);-----------------------------------------------------------------------------------------------------
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}



void loop() {
  analogWrite(enA, 225);
  analogWrite(enB, 255);
  if (Serial.available()) {
    num = Serial.read();
    Serial.println(num);
    if (num == 'f') {
      while (!gpsSerial.available() || !gps.encode(gpsSerial.read())) {
        delay(1);  // Adjust the delay based on the expected time to obtain valid data
      }
      if (gps.location.isValid()) {
        target_point = { gps.location.lat(), gps.location.lng() };
        last_point = target_point;
        Serial.print("Target Latitude: ");
        Serial.println(target_point.latitude, 8);
        Serial.print("Target Longitude: ");
        Serial.println(target_point.longitude, 8);
        Serial.println("Target Set");
      } else {
        Serial.println("Invalid location");
      }
    }



    if (num == 'c') { 
      // Code for Calibrating the Robot
      if (target_point.latitude != 0 && target_point.longitude != 0) {
        initial_bearing = calibrate();
        target_bearing = calculateBearing(initial_point, target_point);
        last_point = initial_point;
        Serial.print("Target Bearing is :-");
        Serial.println(target_bearing);
        RotateToBearing(initial_bearing, target_bearing);
        last_bearing = target_bearing;
        Serial.println("Direction Set");
        Serial.println("Calibration Completed");
      } else {
        Serial.println("Set Target First then configure");
      }
    }
    if (num == 's') {
      while (fabs(last_point.latitude - target_point.latitude) > tolerance || fabs(last_point.longitude - target_point.longitude) > tolerance) {
        analogWrite(enA, 225);
        analogWrite(enB, 255);
        if (last_bearing >= target_bearing - 5.00 && last_bearing <= target_bearing + 5.00) {
          // took +-5 as some margin for bearing
          digitalWrite(in1, HIGH);
          digitalWrite(in2, LOW);
          digitalWrite(in3, HIGH);
          digitalWrite(in4, LOW);
          delay(2000);
          while (last_point.latitude == initial_point.latitude && last_point.longitude == initial_point.longitude) {
            while (!gpsSerial.available() || !gps.encode(gpsSerial.read())) {
              delay(1);  // Adjust the delay based on the expected time to obtain valid data
            }
            if (gps.location.isValid()) {
              initial_point = { gps.location.lat(), gps.location.lng() };
            } else {
              Serial.println("Invalid location");
            }
            last_bearing = calculateBearing(last_point, initial_point);
            Serial.println("Robo On line");
          }
          last_point = initial_point;
        } else {
          while (last_point.latitude == initial_point.latitude && last_point.longitude == initial_point.longitude) {
          while (!gpsSerial.available() || !gps.encode(gpsSerial.read())) {
            delay(1);  // Adjust the delay based on the expected time to obtain valid data
          }
          if (gps.location.isValid()) {
            initial_point = { gps.location.lat(), gps.location.lng() };
          } else {
            Serial.println("Invalid location");
          }
          }
          last_bearing = calculateBearing(last_point, initial_point);
          last_point = initial_point;
          target_bearing = calculateBearing(initial_point, target_point);
          RotateToBearing(last_bearing, target_bearing);
          Serial.println("Robo set towards line");
        }
      }
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      digitalWrite(in3, LOW);
      digitalWrite(in4, LOW);
      Serial.println("Location Reached");
      Serial.print("Initial Latitude: ");
      Serial.println(last_point.latitude, 8);
      Serial.print("Initial Longitude: ");
      Serial.println(last_point.longitude, 8);
      Serial.print("Final Longitude: ");
      Serial.println(target_point.latitude, 8);
      Serial.print("Final Longitude: ");
      Serial.println(target_point.longitude, 8);
      
    }
  }
  delay(20);
}





void RotateToBearing(double current_bearing, double target_bearing) {
  analogWrite(enA, 150);
  analogWrite(enB, 150);
  double diff = target_bearing - current_bearing;
  if (diff >0) {
    double time_R = rot_factor * diff;
    Serial.println("on your Right");
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    delay(time_R * 1000);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
  } 
  else {
    double time_L = rot_factor * fabs(diff);
    Serial.println("On your left");
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    delay(time_L * 1000);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
  }
  
  Serial.print("starting Bearing:- ");
  Serial.println(current_bearing);
  Serial.print("ending Bearing:- ");
  Serial.println(target_bearing);
}

// double calibrate() {
//   GPS point1 = { 0.0, 0.0 };
//   GPS point2 = {0.0 , 0.0};  // Initialize with default values

//   analogWrite(enA, 255);
//   analogWrite(enB, 255);
//   Serial.println("Calibration Started");

//   // Wait for valid GPS data
//   while (!gpsSerial.available() || !gps.encode(gpsSerial.read())) {
//     delay(1);  // Adjust the delay based on the expected time to obtain valid data
//   }



//   // Record initial GPS reading
//   if (gps.location.isValid()) {
//     point2 = { gps.location.lat(), gps.location.lng() };
//     Serial.print("Initial Latitude: ");
//     Serial.println(point2.latitude, 8);
//     Serial.print("Initial Longitude: ");
//     Serial.println(point2.longitude, 8);
//   } else {
//     Serial.println("Invalid initial GPS data. Calibration aborted.");
//     return 0.0;  // Return an invalid bearing value
//   }

//   // Move forward for 3 seconds
//   digitalWrite(in1, HIGH);
//   digitalWrite(in2, LOW);
//   digitalWrite(in3, HIGH);
//   digitalWrite(in4, LOW);
//   delay(6000);
//   digitalWrite(in1, LOW);
//   digitalWrite(in2, LOW);
//   digitalWrite(in3, LOW);
//   digitalWrite(in4, LOW);

//   // Wait for valid GPS data after moving forward
//   while (!gpsSerial.available() || !gps.encode(gpsSerial.read())) {
//     delay(1);  // Adjust the delay based on the expected time to obtain valid data
//   }

//   // Record GPS data after moving forward
//   if (gps.location.isValid()) {
//     point1 = { gps.location.lat(), gps.location.lng() };
//     Serial.println("Moved forward, GPS reading:");
//     Serial.print("Latitude: ");
//     Serial.println(point1.latitude, 8);
//     Serial.print("Longitude: ");
//     Serial.println(point1.longitude, 8);
//   } else {
//     Serial.println("Invalid GPS data after moving forward. Calibration aborted.");
//     return 0.0;  // Return an invalid bearing value
//   }
//   delay(1000);
//   // Move back to the starting point
//   digitalWrite(in1, LOW);
//   digitalWrite(in2, HIGH);
//   digitalWrite(in3, LOW);
//   digitalWrite(in4, HIGH);
//   Serial.println("Moving back to starting point...");
//   delay(6000);

//   // Calculate initial bearing and print results
//   initial_bearing = calculateBearing(point2, point1);
//   Serial.print("Initial Bearing: ");
//   Serial.println(initial_bearing, 2);

//   // Stop motors
//   digitalWrite(in1, LOW);
//   digitalWrite(in2, LOW);
//   digitalWrite(in3, LOW);
//   digitalWrite(in4, LOW);

//   Serial.println("Calibration ended");
//   return initial_bearing;
// }

double calibrate() {
  GPS point1 = { 0.0, 0.0 };  // Initialize with default values
  GPS point2 = { 0.0, 0.0 };  // Initialize with default values
  // Wait for valid GPS data
  while (last_point.latitude == target_point.latitude && last_point.longitude == target_point.longitude) {
    while (!gpsSerial.available() || !gps.encode(gpsSerial.read())) {
      delay(1);  // Adjust the delay based on the expected time to obtain valid data
    }

    analogWrite(enA, 225);
    analogWrite(enB, 255);
    Serial.println("Calibration Started");

    // Record initial GPS reading
    if (gps.location.isValid()) {
      point1 = { gps.location.lat(), gps.location.lng()};
      last_point = point1;
      initial_point = point1;
      Serial.print("Initial Latitude: ");
      Serial.println(point1.latitude, 8);
      Serial.print("Initial Longitude: ");
      Serial.println(point1.longitude, 8);
    } else {
      Serial.println("Invalid initial GPS data. Calibration aborted.");
      return 0.0;  // Return an invalid bearing value
    }
  }

  // Move forward for 3 seconds
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  delay(3000);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  while (point1.latitude == last_point.latitude && point1.longitude == last_point.longitude) {
    // Wait for valid GPS data after moving forward
    while (!gpsSerial.available() || !gps.encode(gpsSerial.read())) {
      delay(1);  // Adjust the delay based on the expected time to obtain valid data
    }
    if (gps.location.isValid()) {
      point2 = { gps.location.lat(), gps.location.lng() };
      last_point = point2;
      Serial.println("Moved forward, GPS reading:");
      Serial.print("Latitude: ");
      Serial.println(point2.latitude, 8);
      Serial.print("Longitude: ");
      Serial.println(point2.longitude, 8);
    } else {
      Serial.println("Invalid GPS data after moving forward. Calibration aborted.");
      return 0.0;  // Return an invalid bearing value
    }
    delay(100);
  }

  Serial.println("Got Second Coordinates");
  last_point = point1;
  // Record GPS data after moving forward


  // Move back to the starting point
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  Serial.println("Moving back to starting point...");
  delay(3000);

  // Calculate initial bearing and print results
  double initial_bearing = calculateBearing(point1, point2);
  Serial.print("Initial Bearing: ");
  Serial.println(initial_bearing, 2);

  // Stop motors
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  Serial.println("Calibration ended");
  return initial_bearing;
}



double calculateBearing(GPS point1, GPS point2) {
  double deltaLongitude = point2.longitude - point1.longitude;
  double y = sin(deltaLongitude) * cos(point2.latitude);
  double x = cos(point1.latitude) * sin(point2.latitude) - sin(point1.latitude) * cos(point2.latitude) * cos(deltaLongitude);
  double initialBearing = atan2(y, x) * 180.0 / M_PI;

  // Convert the bearing to a positive value and adjust for Google Maps convention
  initialBearing = fmod((initialBearing + 360.0), 360.0);
  return initialBearing;
}
