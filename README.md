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

Install Virtualbox
==================

https://www.virtualbox.org/wiki/Downloads

Get Ubuntu Image
================

Someone can give you the thing, it's like 5GB. All the requisite libraries are in the image. We're using [bluez](http://www.bluez.org/), which provides hcitool and gatttool. If you can get that thing to boot you're good. The password for the account is `boom`.

Ctrl+Opt+T will give you a terminal. `hindenbergFights/` has this repo checked out already.


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


Remote-Control Software
=======================

There's software that works on linux for connecting to the RFduino over BTLE and sending it commands that go to the motor controller. So your goal is to be able to:

    $ sudo python blimpControl_kbd.py

And/or replace that with something that will send the three byte codes that the RFduino is expecting. I haven't found any good library that exposes the OS X bluetooth LE stuff to a scripting language. So I don't think we are going to get this running on OS X in python but here's what I did.
