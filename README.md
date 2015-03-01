Get Hardware
============
The blimp carries an [RFduino](https://github.com/RFduino/RFduino). The production version uses the [surface-mount](http://www.rfdigital.com/product/rfd22301-rfduino-ble-smt/index.html) version on a custom board, but for development you can also use the [DIP](http://www.rfdigital.com/product/rfd22102-rfduino-dip/index.html) version and the compatible USB shield. The chip is identical, just in different packaging.

Install Arduino + RFduino library
=================================
Install [Arduino 1.5](http://arduino.cc/en/Main/Software) or later. Then get the [RFdunio library](https://github.com/RFduino/RFduino) and follow their install instructions. Note, however, that as of Arduino 1.5.8, the location where you need to put the RFduino folder has changd to `/Applications/Ardunio.app/Contents/Java/hardware/arduino` 
Yes, you do need to install the sketchy FTDI drivers.

Compile to Arduino
==================
Open `rfduino_bleMotorControl/rfduino_bleMotorControl.ino` in the Arduino environment. Choose "Tools > Board > RFDuino" and then "Sketch > Verify/Compile". If no errors, you can load to the board by doing "File > Upload". Be sure you've selected the correct port from "Tools > Port". (Mine was like /dev/cu.usbserial-DC008W6W or something) Once you see "SUCCESS !" you can take the board off the programmer and hook it up to the battery pack. It will automatically run our code when it boots.

Note that the development board motor controller uses pins 2 and 3, whereas the production board uses pins 5 and 6. If you're using a development board, change line 29 to: `Wire.beginOnPins(2,3);`

Install Virtualbox
==================

https://www.virtualbox.org/wiki/Downloads

Get Ubuntu Image
================

Someone can give you the thing, it's like 5GB. All the requisite libraries are in the image. We're using [bluez](http://www.bluez.org/), which provides `hcitool` and `gatttool`. If you can get that thing to boot you're good. The password for the account is `boom`.

Ctrl+Opt+T will give you a terminal. `hindenbergFights/` has this repo checked out already, just `git pull` to update.


Bluetooth Hardware
==================

On your Mac host (to free up the Bluetooth hardware so the guest can use it):

    $ sudo launchctl unload /System/Library/LaunchDaemons/com.apple.blued.plist
    $ sudo kextunload -b com.apple.iokit.IOBluetoothSerialManager
    $ sudo kextunload -b com.apple.iokit.BroadcomBluetoothHostControllerUSBTransport

Then you can click the little icon at the bottom of the Virtualbox window and choose
Apple, Inc. Bluetooth USB Host Controller. Now the guest can use the hardware.

In the Ubuntu guest:

    $ sudo hcitool lescan

Take note of the address of your RFduino.

After you're done blimping and shut down the guest, re-enable Bluetooth on your Mac:

    $ sudo launchctl load /System/Library/LaunchDaemons/com.apple.blued.plist
    $ sudo kextload -b com.apple.iokit.IOBluetoothSerialManager
    $ sudo kextload -b com.apple.iokit.BroadcomBluetoothHostControllerUSBTransport
    
Or copy this blimp script to `/usr/local/bin/blimp`, make sure to `chmod +x /usr/local/bin/blimp`. Then you can run `blimp up` and `blimp down` commands from anywhere to free up or recover the Bluetooth control from MacOS.

    #!/bin/bash

	_usage(){
	    grep "^[^_].\+(){$" $0 | while read line; do
	      echo "  $0 $line" | sed "s/().*//g";
	    done;
	    echo "";
	}

	up(){
	        sudo launchctl unload /System/Library/LaunchDaemons/com.apple.blued.plist
	        sudo kextunload -b com.apple.iokit.IOBluetoothSerialManager
	        sudo kextunload -b com.apple.iokit.BroadcomBluetoothHostControllerUSBTransport
	}

	down(){
	        sudo launchctl load /System/Library/LaunchDaemons/com.apple.blued.plist
	        sudo kextload -b com.apple.iokit.IOBluetoothSerialManager
	        sudo kextload -b com.apple.iokit.BroadcomBluetoothHostControllerUSBTransport
	}

	# run arguments as commands if any, or show Usage
	"$@"
	if [ ${#1} == 0 ]; then echo "$CMDS" | echo "Usage: "; _usage; fi

Remote-Control Software
=======================

There's software that works on linux for connecting to the RFduino over BTLE and sending it commands that go to the motor controller. So your goal is to be able to:

    $ sudo python blimpControl_kbd.py

And/or replace that with something that will send the three byte codes that the RFduino is expecting.

Compiling Pygame
================
The version of pygame that is available from the repos has some annoying printfs that will get in the way of using the terminal for debugging the xbox controller. To get around this, you can remove the repo version and compile from the latest source

### remove repo version, something like:
sudo apt-get remove pygame

### install dependencies.

at least these, but maybe a couple others if the build step below fails:

sudo apt-get install mercurial python-dev python-numpy libsdl-image1.2-dev libsdl-mixer1.2-dev libsdl-ttf2.0-dev libsmpeg-dev libsdl1.2-dev  libportmidi-dev libswscale-dev libavformat-dev libavcodec-dev
 
### Grab Pygame source
hg clone https://bitbucket.org/pygame/pygame
 
### Finally build and install
cd pygame
python setup.py build
sudo python setup.py install
