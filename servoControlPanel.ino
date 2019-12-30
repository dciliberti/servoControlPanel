/************************************************************************
    Panel to control a servo rotation and read angle on a LCD display,
    with the possibility to set a reference angle and read values
    from that reference. Useful to get the deflection angle of a 
    control surface from a datum. Once set a reference angle, to get
    again the absolute position of the servo, just reset the Arduino.
    There is also the possibility to perform an automatic sweep
    between two given angles, from a given reference, and to change
    the rotational speed of the servo, by adjusting the delay between
    successive angles.

    Copyright (C) 2017 Danilo Ciliberti dancili@gmail.com
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with this program. If not, see <https://www.gnu.org/licenses/>
*************************************************************************/    

#include <LiquidCrystal.h>
#include <Servo.h>
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
Servo myServo;

int const signPin = 6;  // button to switch sign of the angle
int const sweepPin = 7; // button to turn auto angle sweep on/off
int const zeroPin = 8;  // button to set reference
int const pot = A0;     // potentiometer that controls angle/rate
int potVal;             // value of the angle/rate
int angle;              // angle of the servo
int rotDelay;           // speed of the servo (delay between angles)
int zero = 0;           // reference angle of attack
int signSwitch = 1;     // sign of the angle (positive or negative)
int alpha;              // angle of attack
int sweepState = 0;     // auto sweep state (on/off)
int lowLimit = -30;     // low angle limit for auto sweep
int highLimit = 60;     // high angle limit for auto sweep
int newLowLimit;        // low limit angle from the zero
int newHighLimit;       // high limit angle from the zero

void setup() {
  pinMode(zeroPin, INPUT);
  pinMode(signPin, INPUT);

  lcd.begin(16, 2);
  lcd.print("Angle of attack");
  lcd.setCursor(0, 1);
  lcd.print("control panel");
  delay(3000);
  lcd.clear();

  myServo.attach(9);  // servo is attached to pin 9
  Serial.begin(9600);
}

void loop() {

  // if the zero button is pressed the angle of attack is set to 0
  if (digitalRead(zeroPin) == HIGH) {
    zero = angle;
    // centers the auto sweep limits around the new zero
    newLowLimit = lowLimit + zero;
    newHighLimit = highLimit + zero;
    Serial.print("Setting low angle limit: ");
    Serial.println(newLowLimit);
    Serial.print("Setting high angle limit: ");
    Serial.println(newHighLimit);
    delay(100);
  }

  // if the sign button is pressed changes the sign of the angle
  if (digitalRead(signPin) == HIGH) {
    signSwitch = -1 * signSwitch;
    delay(100);
  }

  alpha = signSwitch * (angle - zero);   // angle of attack respect to reference

  // check if auto sweep button has been pressed
  if (digitalRead(sweepPin) == HIGH) {
    sweepState = 1;
    Serial.print("Auto sweep state = ");
    Serial.println(sweepState);
    lcd.clear();
    delay(100);
  }

  // turn on auto sweep if auto sweep button has been pressed
  if (sweepState == 1) {

    // check if alpha range is out of servo range
    if (newLowLimit < 0 || newLowLimit > 179) {
      newLowLimit = 0;
      Serial.print("Actual low angle limit: ");
      Serial.println(newLowLimit);
    }
    if (newHighLimit > 179 || newHighLimit < 0) {
      newHighLimit = 179;
      Serial.print("Actual high angle limit: ");
      Serial.println(newHighLimit);
    }
    
    // if auto sweep button has been released
    if (digitalRead(sweepPin) == LOW) {
      
      lcd.home();
      lcd.print(lowLimit);
      lcd.print(" < AOA < ");
      lcd.print(highLimit);
      lcd.setCursor(0, 1);
      lcd.print("Auto sweep ON");

      // check the angle sign to assign correct sweep angles sequence
      if (signSwitch == 1) {
        angle = zero + alpha;
      }
      else {
        angle = zero - alpha;
        newHighLimit = -lowLimit + zero;
        newLowLimit = -highLimit + zero;
      }

      while (angle < newHighLimit) {
        // assign delay between angles, varying servo rotational speed
        potVal = analogRead(pot);
        Serial.print("potVal: ");
        Serial.print(potVal);
        rotDelay = map(potVal, 0, 1023, 15, 100);
        Serial.print(", rotDelay: ");
        Serial.print(rotDelay);
        Serial.println(" ms");
        myServo.write(angle);
        delay(rotDelay);
        
        // if during loop auto sweep button is pressed exit the loop
        if (digitalRead(sweepPin) == HIGH) {
          sweepState = 0;
          Serial.print("Auto sweep state = ");
          Serial.println(sweepState);
          lcd.clear();
          delay(100);
          goto manual;
        }
        angle++;
      }
      
      while (angle > newLowLimit) {
        // assign delay between angles, varying servo rotational speed
        potVal = analogRead(pot);
        Serial.print("potVal: ");
        Serial.print(potVal);
        rotDelay = map(potVal, 0, 1023, 15, 100);
        Serial.print(", rotDelay: ");
        Serial.print(rotDelay);
        Serial.println(" ms");
        myServo.write(angle);
        delay(rotDelay);
        
        // if during loop auto sweep button is pressed exit the loop
        if (digitalRead(sweepPin) == HIGH) {
          sweepState = 0;
          Serial.print("Auto sweep state = ");
          Serial.println(sweepState);
          lcd.clear();
          delay(100);
          goto manual;
        }
        angle--;
      }
      
    }
  }
  else {  // if auto sweep is turned off
    manual:   // label destination of interrupted loop
    lcd.setCursor(0, 1);
    lcd.print("Manual control");
    
    potVal = analogRead(pot);
    Serial.print("potVal: ");
    Serial.print(potVal);
    angle = map(potVal, 0, 1023, 0, 179);
    Serial.print(", angle: ");
    Serial.print(angle);
    myServo.write(angle);
    delay(15);
    Serial.print(", zero: ");
    Serial.print(zero);
    Serial.print(", AOA: ");
    Serial.println(alpha);
    
    lcd.home();
    lcd.print("AOA = ");
    // To avoid overlapping text among readings I must use blank 
    // characters (space) when needed over the four predicted positions.
    
    // between -179 and -100 four digits are assigned
    if (alpha >= -179 && alpha <= -100) {
      lcd.print(alpha);
      lcd.print(" deg");  // no space after
    }
    // between -99 and -10 three digits are assigned
    else if (alpha >= -99 && alpha <= -10) {
      lcd.print(alpha);
      lcd.print(" deg ");  // one space after
    }
    // between -9 and -1 two digits are assigned
    else if (alpha >= -9 && alpha <= -1) {
      lcd.print(alpha);
      lcd.print(" deg  ");  // two spaces after
    }
    // between 0 and 9 only one digit is assigned
    else if (alpha >= 0 && alpha <= 9) {
      lcd.print(alpha);
      lcd.print(" deg   ");  // three spaces after
    }
    // between 10 and 99 two digits are assigned
    else if (alpha >= 10 && alpha <= 99) {
      lcd.print(alpha);
      lcd.print(" deg  ");  // two spaces after
    }
    // between 100 and 179 three digits are assigned
    else if (alpha >= 100 && alpha <= 179) {
      lcd.print(alpha);
      lcd.print(" deg ");  // one spaces after
    }
    // out of -179,+179 the servo cannot rotate
    else {
      lcd.home();
      lcd.print("Error: signal");
      lcd.setCursor(0,1);
      lcd.print("out of range.");
      delay(1000);
      lcd.clear();
    }
  }
}
