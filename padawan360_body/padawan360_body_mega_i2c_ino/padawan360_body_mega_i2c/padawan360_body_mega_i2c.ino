// =======================================================================================
// /////////////////////////Padawan360 Body Code - Mega I2C v2.0 ////////////////////////////////////
// =======================================================================================
/*
by Dan Kraus
dskraus@gmail.com
Astromech: danomite4047
Project Site: https://github.com/dankraus/padawan360/

v2.0 Changes:
- Makes left analog stick default drive control stick. Configurable between left or right stick via isLeftStickDrive 

Hardware:
***Arduino Mega 2560***
USB Host Shield from circuits@home
Microsoft Xbox 360 Controller
Xbox 360 USB Wireless Reciver (cheap aftermarket versions)
Cytron SmartDriveDuo30 Motor Controller
Syren10 Motor Controller
Sparkfun MP3 Trigger

This sketch supports I2C and calls events on many sound effect actions to control lights and sounds.
It is NOT set up for Dan's method of using the serial packet to transfer data up to the dome
to trigger some light effects.It uses Hardware Serial pins on the Mega to control Sabertooth and Syren

Set Sabertooth 2x25/2x12 Dip Switches 1 and 2 Down, All Others Up
For SyRen Simple Serial Set Switches 1 and 2 Down, All Others Up
For SyRen Simple Serial Set Switchs 2 & 4 Down, All Others Up
Placed a 10K ohm resistor between S1 & GND on the SyRen 10 itself

CHEAP VERSION CHANGES:
by Bret Benziger
Since true Xbox 360 bluetooth controllers are no longer produced, aftermarket cheap knockoffs
are commonly found online. These are paired with a USB receiver typically listed as working
at 2.4GHz. They simulate a wireless controller, and indeed are, but they do not use the 
bluetooth library. Rather, they present as a wired USB controller. This code is updated to 
reflect that.
Also, the Sabertooth motor controllers are typically $130 USD, and a more affordable Cytron
SmartDriveDuo 30 is available for $79 USD. This code only works with the Cytron SmartDriveDuo 
30 in serial mode.

POLOLU MAESTRO and Dome Lift:
Also included are some Pololu Maestro script calls. I have one maestro 24 in the body and 
one 18 channel maestro in the dome. I was not knowledgeable with I2C so I used a 6-wire 
slip-ring for dome communication. 2 wires for power, 2 for I2C to connect to Teeces, 1 serial 
for communicating with the dome Maestro, and 1 for communicating with the dome life by Matt 
Zwarts found at https://www.thingiverse.com/thing:3654411 Dome 

*/

// Padawan360 Script

// ************************** Options, Configurations, and Settings ***********************************


// SPEED AND TURN SPEEDS
//set these 3 to whatever speeds work for you. 0-stop, 100-full speed.
const byte DRIVESPEED1 = 25;
// Recommend beginner: 20 to 50, experienced: 70 to 100, I like 70.
// These may vary based on your drive system and power system
const byte DRIVESPEED2 = 35;
//Set to 0 if you only want 2 speeds.
const byte DRIVESPEED3 = 80;

// Default drive speed at startup
byte drivespeed = DRIVESPEED1;

// the higher this number the faster the droid will spin in place, lower - easier to control.
// Recommend beginner: 40 to 50, experienced: 50 $ up, I like 70
// This may vary based on your drive system and power system
const byte TURNSPEED = 40;

// Set isLeftStickDrive to true for driving  with the left stick
// Set isLeftStickDrive to false for driving with the right stick (legacy and original configuration)
boolean isLeftStickDrive = true;

// If using a speed controller for the dome, sets the top speed. You'll want to vary it potenitally
// depending on your motor. My Pittman is really fast so I dial this down a ways from top speed.
// Use a number up to 127 for serial
const byte DOMESPEED = 85;

// Ramping- the lower this number the longer R2 will take to speedup or slow down,
// change this by incriments of 1
const byte RAMPING = 4;

// Compensation is for deadband/deadzone checking. There's a little play in the neutral zone
// which gets a reading of a value of something other than 0 when you're not moving the stick.
// It may vary a bit across controllers and how broken in they are, sometimex 360 controllers
// develop a little bit of play in the stick at the center position. You can do this with the
// direct method calls against the Syren/Sabertooth library itself but it's not supported in all
// serial modes so just manage and check it in software here
// use the lowest number with no drift
// DOMEDEADZONERANGE for the left stick, DRIVEDEADZONERANGE for the right stick
const byte DOMEDEADZONERANGE = 15;
const byte DRIVEDEADZONERANGE = 15;

// Set the baude rate for the Cytron motor controller (feet)
// 9600 is the default baud rate for Cytron serial.
// for packetized options are: 2400, 9600, 19200 and 38400. I think you need to pick one that works
// and I think it varies across different firmware versions.
const int CYTRONBAUDRATE = 9600;

// Set the baude rate for the Syren motor controller (dome)
// for packetized options are: 2400, 9600, 19200 and 38400. I think you need to pick one that works
// and I think it varies across different firmware versions.
const int DOMEBAUDRATE = 9600;

// Default sound volume at startup
// 0 = full volume, 255 off
byte vol = 42;


// Automation Delays
// set automateDelay to min and max seconds between sounds
byte automateDelay = random(7, 20);
//How much the dome may turn during automation.
int turnDirection = 20;

// Pin number to pull a relay high/low to trigger my upside down compressed air like R2's extinguisher
#define EXTINGUISHERPIN 3

#include <Sabertooth.h>
#include <MP3Trigger.h>
#include <Wire.h>
#include <XBOXUSB.h>
#include <Cytron_SmartDriveDuo.h>
#include <PololuMaestro.h>
#include <SoftwareSerial.h>

// Define the RX and TX pins for SoftwareSerial Dome Maestro
const int RX_PINM = 5;  // Receive Pin
const int TX_PINM = 3;  // Transmit Pin

// Define the RX and TX pins for SoftwareSerial Dome Lift Arduino
const int RX_PINL = 6;  // Receive Pin
const int TX_PINL = 4;  // Transmit Pin

// Create a SoftwareSerial object for dome pololu maestro
SoftwareSerial Serial4(RX_PINM, TX_PINM);

// Create a SoftwareSerial object for dome lift
SoftwareSerial Serial5(RX_PINL, TX_PINL);

// Define Body Maestro on Hardware Serial 3
MiniMaestro maestro(Serial3);

// Define Dome Maestro on Software Serial mySerial
MiniMaestro maestrodome(Serial4);

//Declare variables for Cytron Motors Speeds
signed int speedLeft = 0, speedRight = 0;

//Set pins to connect to Cytron SmartDriveDuo-30 in PWM Independent Mode
//Set dip sticks to 10110100 (on = 1)
#define AN2 13  // Arduino pin 6 is connected to MDDS30 pin AN2.
#define AN1 12  // Arduino pin 5 is connected to MDDS30 pin AN1.
#define IN2 11  // Arduino pin 7 is connected to MDDS30 pin IN2.
#define IN1 10  // Arduino pin 4 is connected to MDDS30 pin IN1.

/////////////////////////////////////////////////////////////////
Cytron_SmartDriveDuo smartDriveDuo30(PWM_INDEPENDENT, IN1, IN2, AN1, AN2);
Sabertooth Syren10(128, Serial2);

// Satisfy IDE, which only needs to see the include statment in the ino.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif

// Set some defaults for start up
// false = drive motors off ( right stick disabled ) at start
boolean isDriveEnabled = false;

// Automated functionality
// Used as a boolean to turn on/off automated functions like periodic random sounds and periodic dome turns
boolean isInAutomationMode = false;
unsigned long automateMillis = 0;
// Action number used to randomly choose a sound effect or a dome turn
byte automateAction = 0;

int driveThrottle = 0;
int throttleStickValue = 0;
int domeThrottle = 0;
int turnThrottle = 0;

boolean firstLoadOnConnect = false;

AnalogHatEnum throttleAxis;
AnalogHatEnum turnAxis;
AnalogHatEnum domeAxis;
ButtonEnum speedSelectButton;
ButtonEnum hpLightToggleButton;

boolean isHPOn = false;

MP3Trigger mp3Trigger;
USB Usb;
XBOXUSB Xbox(&Usb);

// Counter to keep track of button presses for dome lift
int buttonPressCounter_LF = 0;
int buttonPressCounter_LS = 0;
int buttonPressCounter_P = 0;
int buttonPressCounter_Z = 0;
int buttonPressCounter_BM = 0;


// Motor control variables for spinning during animation
unsigned long startMillis = 0;        // Stores the starting time for motor movement
const unsigned long runTime = 12000;  // Desired run time in milliseconds (12 seconds)
int slowSpeed = 50;                   // Speed for slow movement
bool motorRunning = false;            // Tracks if the motor is currently running

void setup() {
  Serial2.begin(DOMEBAUDRATE);
  Serial3.begin(9600);
  Serial4.begin(9600);
  Serial5.begin(9600);

#if defined(SYRENSIMPLE)
  Syren10.motor(0);
#else
  Syren10.autobaud();
#endif

  Syren10.setTimeout(950);

  pinMode(EXTINGUISHERPIN, OUTPUT);
  digitalWrite(EXTINGUISHERPIN, HIGH);

  mp3Trigger.setup();
  mp3Trigger.setVolume(vol);

  randomSeed(analogRead(0));  // Seed randomness using an unconnected analog pin

  if (isLeftStickDrive) {
    throttleAxis = LeftHatY;
    turnAxis = LeftHatX;
    domeAxis = RightHatX;
    speedSelectButton = L3;
    hpLightToggleButton = R3;

  } else {
    throttleAxis = RightHatY;
    turnAxis = RightHatX;
    domeAxis = LeftHatX;
    speedSelectButton = R3;
    hpLightToggleButton = L3;
  }


  // Start I2C Bus. The body is the master.
  Wire.begin();

  //Serial.begin(115200);
  // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
  while (!Serial)
    ;
  if (Usb.Init() == -1) {
    //Serial.print(F("\r\nOSC did not start"));
    while (1)
      ;  //halt
  }
  //Serial.print(F("\r\nXbox Wireless Receiver Library Started"));
  smartDriveDuo30.control(0, 0);
}


void loop() {
  Usb.Task();
  // if we're not connected, return so we don't bother doing anything else.
  // set all movement to 0 so if we lose connection we don't have a runaway droid!
  // a restraining bolt and jawa droid caller won't save us here!
  if (!Xbox.Xbox360Connected) {
    smartDriveDuo30.control(0, 0);
    Syren10.motor(1, 0);
    firstLoadOnConnect = false;
    return;
  }

  // After the controller connects, Blink all the LEDs so we know drives are disengaged at start
  if (!firstLoadOnConnect) {
    firstLoadOnConnect = true;
    mp3Trigger.play(21);
    Xbox.setLedMode(ROTATING);
  }

  //if (Xbox.getButtonClick(XBOX)) {
  //  if(Xbox.getButtonPress(L1) && Xbox.getButtonPress(R1)){
  //    Xbox.disconnect(0);
  //  }
  //}

  // enable / disable right stick (droid movement) & play a sound to signal motor state
  if (Xbox.getButtonClick(START)) {
    if (isDriveEnabled) {
      isDriveEnabled = false;
      Xbox.setLedMode(ROTATING);
      mp3Trigger.play(53);
    } else {
      isDriveEnabled = true;
      mp3Trigger.play(52);
      // //When the drive is enabled, set our LED accordingly to indicate speed
      if (drivespeed == DRIVESPEED1) {
        Xbox.setLedOn(LED1);
      } else if (drivespeed == DRIVESPEED2 && (DRIVESPEED3 != 0)) {
        Xbox.setLedOn(LED2);
      } else {
        Xbox.setLedOn(LED3);
      }
    }
  }

  //Toggle automation mode with the BACK button
  if (Xbox.getButtonClick(BACK)) {
    if (isInAutomationMode) {
      isInAutomationMode = false;
      automateAction = 0;
      mp3Trigger.play(53);
    } else {
      isInAutomationMode = true;
      mp3Trigger.play(52);
    }
  }

  // Plays random sounds or dome movements for automations when in automation mode
  if (isInAutomationMode) {
    unsigned long currentMillis = millis();

    if (currentMillis - automateMillis > (automateDelay * 1000)) {
      automateMillis = millis();
      automateAction = random(1, 5);

      if (automateAction > 1) {
        mp3Trigger.play(random(32, 52));
      }
      if (automateAction < 4) {
#if defined(SYRENSIMPLE)
        Syren10.motor(turnDirection);
#else
        Syren10.motor(1, turnDirection);
#endif

        delay(750);

#if defined(SYRENSIMPLE)
        Syren10.motor(0);
#else
        Syren10.motor(1, 0);
#endif

        if (turnDirection > 0) {
          turnDirection = -45;
        } else {
          turnDirection = 45;
        }
      }

      // sets the mix, max seconds between automation actions - sounds and dome movement
      automateDelay = random(3, 10);
    }
  }

  // Volume Control of MP3 Trigger
  // Hold R1 and Press Up/down on D-pad to increase/decrease volume
  if (Xbox.getButtonClick(UP)) {
    // volume up
    if (Xbox.getButtonPress(R1)) {
      if (vol > 0) {
        vol--;
        vol--;
        vol--;
        mp3Trigger.setVolume(vol);
      }
    }
  }
  if (Xbox.getButtonClick(DOWN)) {
    //volume down
    if (Xbox.getButtonPress(R1)) {
      if (vol < 255) {
        vol++;
        vol++;
        vol++;
        mp3Trigger.setVolume(vol);
      }
    }
  }

  // Logic display brightness.
  // Hold L1 and press up/down on dpad to increase/decrease brightness
  if (Xbox.getButtonClick(UP)) {
    if (Xbox.getButtonPress(L1)) {
      triggerI2C(10, 24);
    }
  }
  if (Xbox.getButtonClick(DOWN)) {
    if (Xbox.getButtonPress(L1)) {
      triggerI2C(10, 25);
    }
  }

  // Lift Mechanism Serial Commands
  // Periscope
  // Handle Lift Mechanism Serial Commands (A + R2 + L1 triggers Periscope)
  if (Xbox.getButtonPress(R2)) {
    if (Xbox.getButtonPress(L1)) {
      if (Xbox.getButtonClick(A)) {
        Serial5.println("4");  // Trigger periscope lift when A + R2 + L1 are pressed
      }
    }
  }

  // Lifeform Scanner lift
  if (Xbox.getButtonPress(R2)) {
    if (Xbox.getButtonClick(X)) {
      Serial5.println("1");
      buttonPressCounter_LF++;

      // Perform actions based on the counter value
      switch (buttonPressCounter_LF) {
        case 1:
          maestrodome.restartScript(3);
          break;
        case 2:
          maestrodome.restartScript(4);
          break;
        case 3:
          buttonPressCounter_LF = 0;  // Reset counter after third press
          break;
      }
    }
  }


  // Bad Motivator lift
  if (Xbox.getButtonPress(R2)) {
    if (Xbox.getButtonClick(Y)) {
      Serial5.println("2");
      buttonPressCounter_BM++;

      // Perform actions based on the counter value
      switch (buttonPressCounter_BM) {
        case 1:
          maestrodome.restartScript(9);
          break;
        case 2:
          maestrodome.restartScript(10);
          break;
        case 3:
          buttonPressCounter_BM = 0;  // Reset counter after third press
          break;
      }
    }
  }

  // Zapper lift
  if (Xbox.getButtonPress(R2)) {
    if (Xbox.getButtonClick(B)) {
      Serial5.println("3");
      buttonPressCounter_Z++;

      // Perform actions based on the counter value
      switch (buttonPressCounter_Z) {
        case 1:
          //maestrodome.restartScript(7);
          break;
        case 2:
          //maestrodome.restartScript(8);
          break;
        case 3:
          buttonPressCounter_Z = 0;  // Reset counter after third press
          break;
      }
    }
  }

  // Light Saber lift
  if (Xbox.getButtonPress(R2)) {
    if (Xbox.getButtonClick(A)) {
      Serial5.println("5");
      buttonPressCounter_LS++;

      // Perform actions based on the counter value
      switch (buttonPressCounter_LS) {
        case 1:
          maestrodome.restartScript(5);
          break;
        case 2:
          maestrodome.restartScript(6);
          break;
        case 3:
          buttonPressCounter_LS = 0;  // Reset counter after third press
          break;
      }
    }
  }

  // Maestro Animations custom to my droid
  //Open utility arms
  if (Xbox.getButtonPress(R2)) {
    if (Xbox.getButtonPress(UP)) {
      maestro.restartScript(0);
    }
  }
  //Close utility arms
  if (Xbox.getButtonPress(R2)) {
    if (Xbox.getButtonPress(DOWN)) {
      maestro.restartScript(1);
    }
  }
  //Open interface arm
  if (Xbox.getButtonPress(R2)) {
    if (Xbox.getButtonPress(RIGHT)) {
      maestro.restartScript(2);
    }
  }
  //Open Gripper Arm
  if (Xbox.getButtonPress(R2)) {
    if (Xbox.getButtonPress(LEFT)) {
      int scriptNumber;

      // Randomly choose between 3, 9, or 10
      int randomChoice = random(3);  // Generates a number between 0 and 2

      if (randomChoice == 0) {
        scriptNumber = 3;
      } else if (randomChoice == 1) {
        scriptNumber = 9;
      } else {
        scriptNumber = 10;
      }

      // Restart the chosen script
      maestro.restartScript(scriptNumber);
    }
  }
  //Open Logic Panel
  if (Xbox.getButtonPress(R2)) {
    if (Xbox.getButtonPress(R1)) {
      if (Xbox.getButtonPress(UP)) {
        maestro.restartScript(4);
      }
    }
  }
  //Close Logic Panel
  if (Xbox.getButtonPress(R2)) {
    if (Xbox.getButtonPress(R1)) {
      if (Xbox.getButtonPress(DOWN)) {
        maestro.restartScript(5);
      }
    }
  }
  //Crazy animation scream
  if (Xbox.getButtonPress(R2)) {
    if (Xbox.getButtonPress(R1)) {
      if (Xbox.getButtonPress(LEFT)) {
        int scriptNumber;

        // Randomly choose between 2, or 11
        int randomChoice = random(2);  // Generates a number between 0 and 1

        if (randomChoice == 0) {
          scriptNumber = 6;
        } else if (randomChoice == 1) {
          scriptNumber = 11;
        }
        maestro.restartScript(scriptNumber);
        maestrodome.restartScript(2);
        mp3Trigger.play(1);
        triggerI2C(10, 1);
      }
    }
  }


  //Parole animation
  if (Xbox.getButtonPress(R2)) {
    if (Xbox.getButtonPress(R1)) {
      if (Xbox.getButtonPress(RIGHT)) {
        maestro.restartScript(8);
        maestrodome.restartScript(1);
        mp3Trigger.setVolume(15);
        mp3Trigger.play(7);
        // Start the motor if it isn't already running
        if (!motorRunning) {
          startMillis = millis();  // Record the start time
          motorRunning = true;     // Set flag to indicate motor is running
        }
      }
    }
  }

  // Maintain the motor at the slow speed for the full 12 seconds
  if (motorRunning) {
    Syren10.motor(1, slowSpeed);  // Keep motor spinning at slow speed

    // Check if the motor has been running for the specified time and stop it
    if (millis() - startMillis >= runTime) {
      Syren10.motor(1, 0);   // Stop the motor
      motorRunning = false;  // Reset flag
      mp3Trigger.setVolume(vol);
    }
  }

  //FIRE EXTINGUISHER
  // When holding L2-UP, extinguisher is spraying. WHen released, stop spraying

  // TODO: ADD SERVO DOOR OPEN FIRST. ONLY ALLOW EXTINGUISHER ONCE IT'S SET TO 'OPENED'
  // THEN CLOSE THE SERVO DOOR
  if (Xbox.getButtonPress(L1)) {
    if (Xbox.getButtonPress(UP)) {
      digitalWrite(EXTINGUISHERPIN, LOW);
    } else {
      digitalWrite(EXTINGUISHERPIN, HIGH);
    }
  }


  // GENERAL SOUND PLAYBACK AND DISPLAY CHANGING

  // Y Button and Y combo buttons
  if (Xbox.getButtonClick(Y)) {
    if (Xbox.getButtonPress(L1)) {
      mp3Trigger.play(8);
      //logic lights, random
      triggerI2C(10, 0);
    } else if (Xbox.getButtonPress(L2)) {
      mp3Trigger.play(2);
      //logic lights, random
      triggerI2C(10, 0);
    } else if (Xbox.getButtonPress(R1)) {
      mp3Trigger.play(9);
      //logic lights, random
      triggerI2C(10, 0);
    } else {
      mp3Trigger.play(random(13, 17));
      //logic lights, random
      triggerI2C(10, 0);
    }
  }

  // A Button and A combo Buttons
  if (Xbox.getButtonClick(A)) {
    if (Xbox.getButtonPress(L1)) {
      mp3Trigger.play(6);
      //logic lights
      triggerI2C(10, 6);
      // HPEvent 11 - SystemFailure - I2C
      triggerI2C(25, 11);
      triggerI2C(26, 11);
      triggerI2C(27, 11);
    } else if (Xbox.getButtonPress(L2)) {
      mp3Trigger.play(1);
      //logic lights, alarm
      triggerI2C(10, 1);
      //  HPEvent 3 - alarm - I2C
      triggerI2C(25, 3);
      triggerI2C(26, 3);
      triggerI2C(27, 3);
    } else if (Xbox.getButtonPress(R1)) {
      mp3Trigger.play(11);
      //logic lights, alarm2Display
      triggerI2C(10, 11);
    } else {
      mp3Trigger.play(random(17, 25));
      //logic lights, random
      triggerI2C(10, 0);
    }
  }

  // B Button and B combo Buttons
  if (Xbox.getButtonClick(B)) {
    if (Xbox.getButtonPress(L1)) {
      mp3Trigger.play(7);
      //logic lights, random
      triggerI2C(10, 0);
    } else if (Xbox.getButtonPress(L2)) {
      mp3Trigger.play(3);
      //logic lights, random
      triggerI2C(10, 0);
    } else if (Xbox.getButtonPress(R1)) {
      mp3Trigger.play(10);
      //March of Pies script 0
      //maestrodome.restartScript(0);
      //logic lights bargrap
      triggerI2C(10, 10);
      // HPEvent 1 - Disco - I2C
      triggerI2C(25, 10);
      triggerI2C(26, 10);
      triggerI2C(27, 10);
    } else {
      mp3Trigger.play(random(32, 52));
      //logic lights, random
      triggerI2C(10, 0);
    }
  }

  // X Button and X combo Buttons
  if (Xbox.getButtonClick(X)) {
    // leia message L1+X
    if (Xbox.getButtonPress(L1)) {
      mp3Trigger.play(5);
      //logic lights, leia message
      triggerI2C(10, 5);
      // Front HPEvent 1 - HoloMessage - I2C -leia message
      triggerI2C(25, 9);
    } else if (Xbox.getButtonPress(L2)) {
      mp3Trigger.play(4);
      //logic lights
      triggerI2C(10, 4);
    } else if (Xbox.getButtonPress(R1)) {
      mp3Trigger.play(12);
      //logic lights, random
      triggerI2C(10, 0);
    } else {
      mp3Trigger.play(random(25, 32));
      //logic lights, random
      triggerI2C(10, 0);
    }
  }

  // turn hp light on & off with Right Analog Stick Press (R3) for left stick drive mode
  // turn hp light on & off with Left Analog Stick Press (L3) for right stick drive mode
  if (Xbox.getButtonClick(hpLightToggleButton)) {
    // if hp light is on, turn it off
    if (isHPOn) {
      isHPOn = false;
      // turn hp light off
      // Front HPEvent 2 - ledOFF - I2C
      triggerI2C(25, 2);
    } else {
      isHPOn = true;
      // turn hp light on
      // Front HPEvent 4 - whiteOn - I2C
      triggerI2C(25, 1);
    }
  }


  // Change drivespeed if drive is enabled
  // Press Left Analog Stick (L3) for left stick drive mode
  // Press Right Analog Stick (R3) for right stick drive mode
  // Set LEDs for speed - 1 LED, Low. 2 LED - Med. 3 LED High
  if (Xbox.getButtonClick(speedSelectButton) && isDriveEnabled) {
    //if in lowest speed
    if (drivespeed == DRIVESPEED1) {
      //change to medium speed and play sound 3-tone
      drivespeed = DRIVESPEED2;
      Xbox.setLedOn(LED2);
      mp3Trigger.play(53);
      triggerI2C(10, 22);
    } else if (drivespeed == DRIVESPEED2 && (DRIVESPEED3 != 0)) {
      //change to high speed and play sound scream
      drivespeed = DRIVESPEED3;
      Xbox.setLedOn(LED3);
      mp3Trigger.play(1);
      triggerI2C(10, 23);
    } else {
      //we must be in high speed
      //change to low speed and play sound 2-tone
      drivespeed = DRIVESPEED1;
      Xbox.setLedOn(LED1);
      mp3Trigger.play(52);
      triggerI2C(10, 21);
    }
  }



  // FOOT DRIVES
  // Xbox 360 analog stick values are signed 16 bit integer value
  // Cytron values for speed -100 to 100 for speed (full speed reverse and  full speed forward)
  // Map the 360 stick values to our min/max current drive speed
  // This code makes the droid safer to run faster as speed increases decrease the output 
  // of the left and right values in a curve

  throttleStickValue = (map(Xbox.getAnalogHat(throttleAxis), -32768, 32767, -drivespeed, drivespeed));
  if (throttleStickValue > -DRIVEDEADZONERANGE && throttleStickValue < DRIVEDEADZONERANGE) {
    // stick is in dead zone - don't drive
    driveThrottle = 0;
  } else {
    if (driveThrottle < throttleStickValue) {
      if (throttleStickValue - driveThrottle < (RAMPING + 1)) {
        driveThrottle += RAMPING;
      } else {
        driveThrottle = throttleStickValue;
      }
    } else if (driveThrottle > throttleStickValue) {
      if (driveThrottle - throttleStickValue < (RAMPING + 1)) {
        driveThrottle -= RAMPING;
      } else {
        driveThrottle = throttleStickValue;
      }
    }
  }

  turnThrottle = map(Xbox.getAnalogHat(turnAxis), -32768, 32767, -TURNSPEED, TURNSPEED);

  // DRIVE!
  // right stick (drive)
  if (isDriveEnabled) {
    // Only do deadzone check for turning here. Our Drive throttle speed has some math applied
    // for RAMPING and stuff, so just keep it separate here
    if (turnThrottle > -DRIVEDEADZONERANGE && turnThrottle < DRIVEDEADZONERANGE) {
      // stick is in dead zone - don't turn
      turnThrottle = 0;
    }
    // OLD METHOD
    // Determine motor speeds and directions based on joystick input
    //int speedLeft = constrain((driveThrottle - turnThrottle), -100, 100);
    //int speedRight = constrain((driveThrottle + turnThrottle), -100, 100);
    //  smartDriveDuo30.control(speedLeft, speedRight);

    int maxDriveThrottle = TURNSPEED;  // Maximum drive throttle value
    int maxTurnThrottle = 100;         // Maximum turn throttle value

    // Linearly map turnThrottle based on driveThrottle
    float driveFactor = 1 - (float(abs(driveThrottle)) / maxDriveThrottle);
    // Calculate scaledTurnThrottle by applying the driveFactor
    int scaledTurnThrottle = turnThrottle * driveFactor;

    int speedLeft = constrain((driveThrottle - scaledTurnThrottle), -100, 100);
    int speedRight = constrain((driveThrottle + scaledTurnThrottle), -100, 100);
    smartDriveDuo30.control(speedLeft, speedRight);
  }

  // DOME DRIVE!

  if (!motorRunning) {
    domeThrottle = (map(Xbox.getAnalogHat(domeAxis), -32768, 32767, DOMESPEED, -DOMESPEED));
    if (domeThrottle > -DOMEDEADZONERANGE && domeThrottle < DOMEDEADZONERANGE) {
      //stick in dead zone - don't spin dome
      domeThrottle = 0;
    }

    Syren10.motor(1, domeThrottle);
  }  // END loop()
}

void triggerI2C(byte deviceID, byte eventID) {
  Wire.beginTransmission(deviceID);
  Wire.write(eventID);
  Wire.endTransmission();
}
