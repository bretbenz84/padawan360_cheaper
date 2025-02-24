# Padawan360

This version of Padawan360 uses less expensive motor controllers for the feet and inexpensive aftermarket wireless xbox controllers. It also uses triggers for Pololu Maestro Minis and a separate arduino lift mechanism by Matt Zwarts.

The R2/Droid Builders have a long history of raising money for groups like [Make-A-Wish](https://wish.org/).

[Donate to Make-A-Wish](https://wish.org/)

## Contents

- [Padawan360](#padawan360)
- [Intro](#intro)
- [Components](#components)
  - [Arduino Mega](#arduino-mega)
  - [USB Shield](#usb-shield)
  - [Xbox 360 Wireless Controller](#xbox-360-wireless-controller)
  - [MP3 Trigger](#mp3-trigger)
  - [Cytron Motor Controller - Feet](#cytron-motor-controller---feet)
  - [Syren Motor Controller - Dome](#syren-motor-controller---dome)
  - [Amp and Speakers](#amp-and-speakers)
  - [Teeces lights](#teeces-lights)
    - [Optional](#optional)
      - [RSeries RGB HPs.](#rseries-rgb-hps)
      - [Slipring](#slipring)
- [Setup](#setup)
  - [Arduino IDE](#arduino-ide)
  - [USB Shield](#usb-shield-1)
  - [Sound](#sound)
  - [Dome](#dome)
    - [Option 1](#option-1)
    - [Option 2 (Best Option)](#option-2--best-option-)
  - [Foot Drive](#foot-drive)
  - [Arduino UNO/MEGA](#arduino-uno-mega)
    - [I2C](#i2c)
  - [Controller Pairing](#controller-pairing)
  - [Options, Configurations, and Settings](#options--configurations--and-settings)
  - [Teeces Logics](#teeces-logics)
  - [HoloProjectors I2C](#holoprojectors-i2c)
  - [Video Guide](#video-guide)
- [Controls](#controls)
  - [Controller LED Status](#controller-led-status)
  - [Button Guide](#button-guide)
- [Troubleshooting and FAQs](#troubleshooting-and-faqs)
  - [General Control Issues](#general-control-issues)
  - [Sound](#sound-1)
  - [Dome](#dome-1)
  - [Drives](#drives)
- [Licensing](#licensing)

## Intro

This is a control system for 1:1 scale remote control R2-D2 powered by Arduinos and controlled with an Xbox 360 Controller. It triggers lights, sounds, and controls foot drive and dome drive motors. It also supports I2C to trigger events in the dome lights or can be extended to interact with anything else that supports I2C

These sketches are heavily based on DanF's Padawan control system that uses Playstation 2 controllers. I found the PS2 controllers to become a bit unreliable and they are increasingly more difficult to come by. I'm also taking advantage of the LEDs around the center Guide button to indicate state of the drive mode (disengaged, engaged w/ speed setting).

The Xbox 360 Controller is over 2.4ghz and uses frequency hopping to avoid interference. At DroidConIII 2014 I was at least over 100ft from the Droid and maintained connection through several walls including two bathrooms with plumbing and tiles. I tested by spinning the dome. When I got out of range, the dome stopped spinning. When I walked a few feet forward, it automatically reconnected and I was able to spin the dome again. Connection status was displayed via LEDs on the controller.

I developed Padawan360 (named with permission from DanF) to use some more easily accessible components with no soldering, wire stripping, etc. It's a bit more plug and play.

A lot of the instructions here are relevant to the original [Padawan PS2](http://astromech.net/droidwiki/index.php?title=PADAWAN) setup instructions. A good chunk of the documentation is reproduced here.

It is strongly recommended that you read this guide completely a few times before plugging things in or trying to run things. You should also review the code to get familiar with some of the options available to adjust.

Note: this does NOT work with an Xbox One controller. Xbox One controllers use different wireless protocols.

[The thread on Astromech for Padawan360 can be found here ](http://astromech.net/forums/showthread.php?t=19263_)

## Contributing

There are quite a few forks either on GitHub, circulating around via email attachments, Google Drive, etc. I'm open to contributions. Please either open an issue, contact me, open a PR, etc. If there's a change you want to see, likely others will benefit as well. The caveat is that the changes need to be generic enough for the average new builder to use and benefit from. If you want to include new and alternate libraries for sound boards, motor drivers, etc, then do so, but wrap them in flags or something so that the compiled sketch is small and effecient still to fit on devices. We already have two Mega and Uno sketches (which arguable could be combined into one), I'd prefer not to add more to manage.

## Components

- ### Arduino Mega

I run a Mega for the body. It uses the hardware serial pins to connect to the motor controllers. Better performance and memory utilization. The Mega has more memory available too so there's more room to expand and do more if you want. With the Mega, I can also support I2C a bit better. I tend to make tweaks to improve readability here more than the Uno as I run a Mega in my droid. It's more performant than the Uno.

- ### USB Shield

  Sourced from [Circuits@Home](https://www.circuitsathome.com/products-page/arduino-shields). They've shuffled their links around. Find it on that link labeled "USB Host Shield 2.0 for Arduino – Assembled". They used to offer it assembled and unassembled but as of this writing, they just have it assembled.

  If Amazon is your thing Sainsmart has a USB Host Shield that has been found to be compatible with Padawan360 (Prime elligble!) SainSmart USB Shield](http://www.amazon.com/SainSmart-Compatible-HOST-Shield-Arduino/dp/B006J4G000/ref=sr_1_1?ie=UTF8&qid=1420994477&sr=8-1&keywords=usb+host+shield+2.0+for+arduino). _NOTE_ I've seen that some SainSmart Shields aren't sending them with the 2x3 headers that connects the shield to the Arduino board together. This connection is absolutely necessary for the Arduino to talk to the shield over serial (ISCP pins). It will NOT work without this connection. Real dumb. The shield is basically useless without the connection. I spent a while a DroidCon trying to help someone it before realizing that was missing and it's absolutely necessary. You can grab 2x3 headers from Sparkfun for 50 cents. The missing headers seem to be a common issue as the Aamazon reviews are rife with that complaint. Check your particular USB Host Shield to see if the headers are there. The production may have been corrected.

Your USB Shield may need voltage pins jumped. See Steve Baudain's YouTube video here walking through it: https://www.youtube.com/watch?v=y9HEeBO3cV0


- ### Xbox 360 Wireless Controller

  Cheap aftermarket wireless controller advertised with a USB dongle operating at 2.4GHz (not real bluetooth) should be used with this script. [Xbox contoller example](https://www.amazon.com/gp/product/B0CC59Z93Q/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1)

- ### MP3 Trigger

  [Sourced from SparkFun](https://www.sparkfun.com/products/11029). Be sure to get a microSD card too. Nothing too big, it's just MP3s.

- ### Cytron Motor Controller - Feet

  Depending on your motors you'll want a Cytron SmartDriveDuo 30A dual motor controller found here: [Cytron Smart Drive Duo 30](https://makermotor.com/pn00218-cyt14-motor-driver-2-channels-30amp-7v-35v-dc-smartdrivedou-mdds30/?setCurrencyId=1&sku=PN00218-CYT14). Be sure to put it in PWM pinout mode.

- ### Syren Motor Controller - Dome

  [Syren 10](https://www.dimensionengineering.com/products/syren10)

- ### Amp and Speakers

  Up to you and how big and loud you want. I have a small speaker and a $20 amp from Pyle. A ground loop isolator might be necessary to protect the MP3 trigger and eliminate buzzing from the speaker.

- ### Teeces lights

  The sketch provided here will work for version 3 of the Teeces lighting system for Logic Lights. Use the regular setup and installation instructions for the Teeces system. To control brightness and changing the lighting animations that match some scenes from the films (like the flicker pattern during the Leia Message), the Arduino for the Teeces system needs to be connected to the body Arduino via I2C. Connect the I2C pins from the Body Arduino to the Teeces Arduino. SDA->SDA pins and SCL->SCL pins. On the Uno these are A4 and A5 respectively and on the Mega they are 20 and 21. Verify the I2C pins on your Teeces Arduino. These can be connected via a slipring.

- #### Optional

  - ##### RSeries RGB HPs.

    Sketches provided are for I2C holoprojector boards. The front uses the one with the servo pinouts although the sketch doesn't servo control of the HPs. The top and rear HP are just the regular I2C controlled boards.

  - ##### Slipring
    Used to pass power up from the body to the dome and also signal for I2C to control dome lights. The slipring allows wires to go from body to dome and allow the dome to spin 360 degrees without tangling the wires.

## Setup

### Arduino IDE

Be sure you have the latest version of the Arduino IDE installed available [here at the Arduino website](https://www.arduino.cc/en/Main/Software) and install the libraries included in this repository from the `libraries` folder.

Installing libraries and using the Arduino IDE is beyond the scope of this documentation. Refer to the Arduino website's documentation for how to install libraries and upload software to your Arduino here https://www.arduino.cc/en/Guide and here https://www.arduino.cc/en/guide/libraries#toc5 . Note, you do not need to include the library references (ex: `#include <Sabertooth.h>`) directly in your sketch as the code to include them is already there. They simply need to be installed so that the Arduino IDE software can find them when compiling the sketch. The libraries are provided as zip files to make for easy installation.

### USB Shield

Solder the headers on the USB Shield if you purchased the unassembled version. Fit the shield over the Arduino UNO by lining up the pins and pushing in. It should fit snugly. Plug the Xbox 360 Wireless Receiver USB cable into the USB port. That was easy.

If you're using the Mega, orient the USB ports to line up over each other.

### Sound

Connect the following pins from the MP3 Trigger to the Body Arduino

| MP3 Trigger Pin | Arduino UNO |
| --------------- | ----------- |
| RX              | 1           |
| USBVCC          | 5v          |
| GND             | GND         |

**There is a small switch at the bottom of the board labled USB - EXT, make sure that it is pointing to the USB side in order to power it via the Arduino**

In order to load the sounds onto the MP3 Trigger, you will need to grab the sounds files either from [here](https://drive.google.com/drive/folders/19-NO-B2NIjAVUvo6hy9pTMNdYRaIFusJ)

Connect the Micro SD card to your computer and upload the files one by one in the numbered order. If you don't do this, you will not be able to control which sound files are triggered. It's picky about file names and ordering. It's annoying but really, do transfer them over one file at a time. You can also use a Windows program called Drivesort to help out here too. Here's a [helpful video from builder Balders on how to load the sounds with Drivesort](https://youtu.be/UsMI2gW7Q40)

Now, insert it into the MP3 Trigger and hook up either the Ground Loop Isolator / Amp / Speaker using the 3.5mm jack on the board.

For anyone with an older version of the MP3Trigger board, you may need to upgrade your firmware before you can use the Arduino to trigger sounds. If your board has the date 3-1-10 on the back (and possibly others) you have the retired version WIG-09715. You can use the board, but it's likely that you'll need updated firmware.

1. Download and unzip the the firmware file available [here](http://astromech.net/droidwiki/images/8/8f/MP3TRIGR.zip).
2. Copy the resulting hex file to a microSD card and rename it to: “MP3TRIGR.HEX”. It does not need to be the only file on the card – it just needs to have that precise filename.
3. Insert the microSD card into your MP3 Trigger and turn the power on while holding down the center navigation switch. Wait for the Status LED to go solid, then cycle the power.
4. You’re now running the new firmware.

### Dome

Review the documentation made available by Dimension Engineering for the Syren Motor Controller.

Connect the pins for the Syren Motor Controller

| Syren10 | Arduino UNO |
| ------- | ----------- |
| S1      | 5           |
| 0v      | GND         |

| Syren10 | Arduino Mega  |
| ------- | ------------- |
| S1      | Serial2 (Tx2) |
| 0v      | GND           |

| Syren10 | Battery  |
| ------- | -------- |
| B+      | Positve  |
| B-      | Negative |

| Syren10 | Dome Motor   |
| ------- | ------------ |
| M1      | 1 (Positve)  |
| M2      | 2 (Negative) |

If you find that the dome spins the opposite direction, flip M1 and M2. When standing behind the droid, moving left on the left analog stick should rotate the dome left.

If using a Syren speed controller for the dome, you will have choose what type of serial connection to use.
Packetized is the best choice for the Syren, but you might need to change the baud rate . This is done by changing the value of int domeBaudeRate at the beginning of the sketch. All supported rates are listed there.
If you can't get packetized to work, (and some people have had problems) you will have 2 options

#### Option 1

Simplified serial- easy to setup, proven to work, BUT a chance that the dome could start spinning if power is lost to the arduino but not the syren. While I haven't seen this happen without physically yanking the power wire or pressing the reset button the risk is there. I would say now that we know of this possibility, do not use this option if people, especially children, will be close enough to the droid to be injured. To use simplified serial, delete the // in front of the line #define SYRENSIMPLE at the beginning of the sketch.

#### Option 2 (Best Option)

Send the syren to Dimension Engineering to be flashed with the Ver. 2 firmware- This allows the syren to be locked in to a set baud rate and eliminates the auto-bauding problem. I have had this done to my own syren and have tested it with several types of arduinos with no problems. All new syrens are being shipped with this new firmware.

The dip switches on the Syren should be set to 1 and 2 off for Packetized, or 2 and 4 off for Simple.

In some cases, we've noticed that the dome may behave eratically after starting up. If this is the case plugging a 10k resister between the S1 and 0V screw terminals. Simply bend the pins and screw them in along with the wires.

Make sure that you use at least a 14 Guage wire between the motor and the Syren 10. Anything hire can cause problems (fire!)

### Foot Drive

Review the documentation made available by Cytron for their Motor Controller. [Cytron Manual](https://makermotor.com/content/cytron/pn00218-cyt14/MDDS30_User_Manual.pdf)

Be sure to set the pins to serial mode.

### Arduino UNO/MEGA

Install the libraries from the Libraries folder. Upload the corresponding padawan360_body sketch from the `padawan360_body` folder for your Arduino (UNO or Mega) sketch to the Arduino. There is one for the UNO and one for the Mega.

Review the sketch as there are some configuration options at the top of the sketch with descriptive comments that you may want to adjust to control speed levels, turn speed,

#### I2C

This is optional. If you want to trigger light effects in the dome via slipring, connect A4 SLC to SLC on the slipring board and A5 SDA on the Arduino to SDA on the slipring board.

Arduino UNO R3 has separate I2C pins which is really nice but the Circuits@Home USB Shield covers them up. The R3 Board still has I2C pins at A4 and A5.

I've had better performance using the Mega with I2C because of using the hardware serial pins.

### Controller Pairing

Since you're using a pre-paired controller, no pairing is required.

### Options, Configurations, and Settings

Review the top of the Arduino Sketch that you will be uploading to your droid. There are a number of options and configurations that you may need to tune for YOUR droid specifically. Provided are defaults and standard that generally work well but you may find that based on your drive system, power system, and personal preferences you may want to adjust them. The top of the sketch documents each of these settings and provides defaults and includes explanations and descriptions of what each of them does and potential changes you may want to change. Review these carefully.

### Teeces Logics

If you didn't install the libraries in preparation of the body sketch, install the libraries found in the library folder. Upload the padawan360_dome sketch on the Teeces arduino. Connect I2C on the dome end of slipring board. SDA to A4 and SLC to A5.

### HoloProjectors I2C

Mike Erwin put out some great boards with Arduino bootloaders on them. Programmable! I wasn't personally thrilled with some of the lighting effects. I thought some of the colored lights didn't look too authentic. Certainly not entirely fil accurate, but that's ok. Some colored lights didn't really seem to fit random idling and blinking. Using an ISP programmer I tweaked the events more to my liking. Alarms flash yellow and red, otherwise everything else is just white or a cyan color for the Leia message. If you don't want to program them with my versions, you can tweak the `triggerI2C(deviceID, eventNumer)` function calls in the body sketch to match the original event numbers. The originals are found on the RSeries code repo on google code.

### Video Guide

Builder Steve Baudains has put together a few videos walking through setup of the main components.

- [Video 1](https://www.youtube.com/watch?v=oZw0zzCgoh8)
- [Video 2](https://www.youtube.com/watch?v=wXRDHJttQbc)

## Controls

### Controller LED Status

| LEDs Around Center Guide Button | Description                                   |
| ------------------------------- | --------------------------------------------- |
| 4 Flashing/Blinking in Unison   | Looking for connection to receiver            |
| Rotating/Spinning pattern       | Connected to receiver. Foot motors disengaged |
| Single LED Top Left - Steady    | Foot Motors Engaged. Speed 1                  |
| Single LED Top Right - Steady   | Foot Motors Engaged. Speed 2                  |
| Single LED Bottom Left - Steady | Foot Motors Engaged. Speed 3                  |

### Button Guide

Press guide button to turn on the controller.

Press Start button to engage motors!

Drive stick is now (as of 2019-11-01) on the LEFT STICK and dome control is on the RIGHT STICK.
They have been reversed from what is seen in the below controller guide.
Set `isLeftStickDrive` in the code to false to drive with the RIGHT STICK as seen in the guide.

![Button Guide](https://github.com/dankraus/padawan360/blob/master/xbox360-controller-guide.jpg)

(Button layout image courtesy of LarryJ on Astromech. Thanks!)

## Troubleshooting and FAQs

### Arduino IDE

_**When I try to upload the sketch I get an error like: sabertooth.h: No such file or directory OR somefile: No such file or directory**_
The libraries included in this repository need to be installed. Instructions on how to install Arduino libraries is beyond the scope of this documentation. Refer to the official Arduino documentation here: https://www.arduino.cc/en/guide/libraries#toc5

### General Control Issues

Not triggering sound or getting any movement? Make sure you're paired. The best way is to use the Xbox Wireless library's example sketch.

1. File>Examples>USB Host Shield>Xbox>Xbox Recv
2. Upload that to your Arduino
3. Look at the code and you'll see `Serial.begin`. Take note of that number.
4. Tools>Serial Monitor. Set the baud rate to that number from step 3.
5. Pair your controller.

Press buttons, do you see the button names output? If you don't, you're not paired. If it's paired, you'll see the button names in the serial monitor as you press them.

### Pairing

_**My controller and receiver won't pair. I don't see any output in the serial monitor in the Xbox Recv test sketch**_

- Confirm that you are using an official Xbox 360 Wireless Controller and Xbox 360 Wireless USB Receiver. If you're not, it's likely the culprit. Get yourself an official one and save yourself some trouble. That said, some people have had luck making some adjustments to get some 3rd Party receivers working. Read up on it here: https://astromech.net/droidwiki/Cheap_XBox_Receivers
- Your USB Shield may need voltage pins jumped. See Steve Baudain's YouTube video here walking through it: https://www.youtube.com/watch?v=y9HEeBO3cV0

### Sound

If the MP3 Trigger isn't functioning as expected, or at all, or just behaving a little "odd" - it would be good to make sure you have the most up to date firmware. Version 2.54 is known to work. It can be downloaded [here ](http://robertsonics.com/mp3-trigger-downloads/). The user manual has instruction for installation on the last page (as of this writing) of the instruction manual under "Bootloader". The manual is available from the previous link and also available directly [here](http://robertsonics.com/resources/mp3trigger/MP3TriggerV2UserGuide_2012-02-04.pdf). As of 2020, you're not likely to run into this unless you happened to buy really old inventory or a spare from another builder that had one laying around for years.

Some users had experienced some issues of sounds freezing up and going a bit haywire. It was resolved by using the barrel jack power connector as the Mp3Trigger was browning out. That means that disconnecting the 5v and Ground pins between the Arduino and MP3Trigger and switching flip the switch to EXT. This is indicative of power brown outs or some other electrical issue.

If the wrong sounds are playing for button presses, they are likely added to the SD card in the wrong order. They must be put on the card in the exact order one at a time or by using Drivesort. Reference [this video](https://youtu.be/UsMI2gW7Q40) for how load the sound files up with Drivesort to ensure they're loaded onto the SD Card in proper order.

Loading the sound files must be done with a Windows PC. Unfortunately, MacOS will leave some helper index files to inprove OS features like Spotlight and other assorted MacOS specific things. If anyone has done this on a Mac, please let me know!! Likely this can be done on Linux, but I've yet to do it personally. If you have, please let me know.

### Dome

_**My Dome just randomly spins! My Dome won't spin! My Dome doesn't spin correctly** etc etc.._

Depending on the firmware on the Syren, sometimes you need to change the serial baud rate it communicates at. Change this line in the code: `const int DOMEBAUDRATE = 9600;` For packetized options are: 2400, 9600, 19200 and 38400

_**The dome spins in the wrong direction when I move the dome control stick!**_

Flip the leads to the dome motor. There is no positive and negative. Orient your controls from the perspective of the droid. Stand behind the droid so that pressing
on the stick makes the dome spin left and right on the stick makes the some spin right.

_**The left analog stick is centered but the dome still spins!**_

You need to just adjust the deadzone `const byte DOMEDEADZONERANGE = 20;` Increase this number until you can let the stick go neutral and nothing moves. The sketch has some more info on that above setting that value to adjust for controller stick drift.

### Drives

_**I've wired up my Sabertooth but it doesn't move!**_

Don't forget to press START on the controller to enable the motors. Still not working? Put a multimeter to the terminals for a motor and push forward on the stick (after enabling the motor controls of course by pressing START). The voltage should go from 0v to something near the voltage of the power supplies from your battery. Ex, Speed 1 on a 12v battery may show 7v when stick is pushed forward all the way. Speed 3 may show closer to 12v. If you're not getting any voltage at the terminals, check the baud rate in the code, dip switches, and for correct wiring.

_**When I press left on the drive stick, it turns to the right (or vice versa)! When I push forward on the drive stick it drives backwards (or vice versa)!**_

People often regularly run into this once they power everything up and find that they push forward on the stick and it drives to left, stick to the left drives drives backwards, etc. Don't fret, motors can be wired positive/negative. It doesn't matter. Start flipping the motor connections to the Sabertooth. First flip M1A and M1B. If that doesn't fix it, flip it back and try the other. If it still isn't right, then try flipping both sets. R2 is driven like a tank. Spin the right motor forward to go left, left motor forward to go right. Both motors forward, drive forward. Keep that in mind as you troubleshoot.

Orient your controls from the perspective of the droid. Stand behind the droid so that pressing left on the drive stick makes the droid turn left, right on the stick makes the droid turn right, up on the stick makes the droid drive forward, down on the stick makes the droid drive backwards.

_**The right analog stick is centered but it still drives(turns, drives forward, drives reverse, etc)!**_

You need to just adjust the deadzone `const byte DRIVEDEADZONERANGE = 20;` Increase this number until you can let the stick go neutral and nothing moves. The sketch has some more info on that above setting that value to adjust for controller stick drift.

## Licensing

See the LICENSE file in this repository.
