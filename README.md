### Get Hardware
The blimp carries an [RFduino](https://github.com/RFduino/RFduino). The production version uses the [surface-mount](http://www.rfdigital.com/product/rfd22301-rfduino-ble-smt/index.html) version on a custom board, but for development you can also use the [DIP](http://www.rfdigital.com/product/rfd22102-rfduino-dip/index.html) version and the compatible USB shield. The chip is identical, just in different packaging.

### Install Arduino + RFduino library
Install [Arduino 1.5](http://arduino.cc/en/Main/Software) or later. Then get the [RFdunio library](https://github.com/RFduino/RFduino) and follow their install instructions. Note, however, that as of Arduino 1.5.8, the location where you need to put the RFduino folder has changd to `/Applications/Ardunio.app/Contents/Java/hardware/arduino` 
Yes, you do need to install the sketchy FTDI drivers.

### Compile
Open `rfduino_bleMotorControl/rfduino_bleMotorControl.ino` in the Arduino environment. Choose "Tools > Board > RFDuino" and then "Sketch > Verify/Compile". If no errors, you can load to the board by doing "File > Upload". Be sure you've selected the correct port from "Tools > Port". (Mine was like /dev/cu.usbserial-DC008W6W or something) Once you see "SUCCESS !" you can take the board off the programmer and hook it up to the battery pack. It will automatically run our code when it boots.

### Check Bluetooth
Optional: Get the Hardware IO Tools for Xcode from [developer.apple.com](https://developer.apple.com). In Bluetooth Explorer go "Devices > Low Energy Devices" and click "Start Scanning". You should see "RFduino Blimp" and be able to connect to it.

Optional 2: Get the [LightBlue app](https://itunes.apple.com/us/app/lightblue/id639944780?mt=12) on your iOS device and make sure you see RFduino Blimp, then tap that blimp.

### Remote-Control Software

There's software that works on linux for connecting to the RFduino over BTLE and sending it commands that go to the motor controller. So your goal is to be able to:

    $ python blimpControl_kbd.py

And/or replace that with something that will send the three byte codes that the RFduino is expecting. I haven't found any good library that exposes the OS X bluetooth LE stuff to a scripting language. So I don't think we are going to get this running on OS X in python but here's what I did.

For whatever reason, this is how you install pygame:

    $ brew install libvorbis sdl sdl_image sdl_mixer sdl_ttf portmidi mercurial
    $ sudo pip install hg+http://bitbucket.org/pygame/pygame
  
Also

    $ pip install pexpect 
  
And then how about installing [LightBlue not the app](https://github.com/0-1-0/lightblue-0.4)? Sure. Follow [these directions](http://stackoverflow.com/questions/22279913/how-to-install-either-pybluez-or-lightblue-on-osx-10-9-mavericks) for fun. Nope, that doesn't solve find_library("bluetooth") either.


