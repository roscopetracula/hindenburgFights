#!/usr/bin/python

'''
July 5, 2014 Nancy Ouyang
Usage:
    $ sudo python blescan.py
Copyright:
    public domain
Automation of hcitool lescan to return MAC addresses
Requires sudo privileges
This operates with hcitool v5.20, tested on Ubuntu 14.04
Hardware: Requires a laptop with 4.0 dongle or with built-in bluetooth low energy capabilities
Note: You can check if your dongle is recognized by using hciconfig at a terminal prompt. 

@params None
@returns ble_adrs - A set of MAC addresses of BLE peripherals
'''

import pexpect, time, sys, os
from ctypes.util import find_library
SCAN_TIME = 1 #in seconds


if not os.geteuid() == 0:
    sys.exit("hictool lescan only works as root")

btlib = find_library("bluetooth")
if not btlib:
    raise Exception(
        "Can't find required bluetooth libraries"
    )

def closeall(connection):
    isalive = connection.terminate(force=True)
    connection.close(force = True)

def gatheradr(conn):
    start_time = time.time()
    print 'Scanning for addresses...'
    ble_adrs = set()
    try:
        for line in conn:
            address = line.strip().split(' ')[0]
            #print address
            if address != '':
                ble_adrs.add(address)
            elapsed_time = time.time() - start_time
            if elapsed_time > SCAN_TIME: #in seconds
                closeall(conn)
                break
    except (pexpect.TIMEOUT):
        print 'No BLE addresses found! Are peripherals on and reset?'
        closeall(conn)
    return ble_adrs

def blescan():
    ble_adrs = set()
    # seems like we cannot catch the timeout here, must be inside gatheradr? 
    # pexpect.timeout thrown if no inputs, aka no addresses, are being found with hcitool lescan
    hciout = pexpect.spawn('hcitool lescan', timeout=1)
    i = hciout.expect(['LE Scan ...','File descriptor in bad state','failed', pexpect.TIMEOUT], timeout=1)
    if i == 0:
        ble_adrs = gatheradr(hciout)
    if i == 1:
        c = True
        while c: #code to check if input acceptable
            inp = raw_input('Check if dongle is plugged in. Type "y" to continue, or type "n" to cancel.')
            if inp.lower().startswith('y'):
                hciout.terminate()
                hciout.close()
                print 'Waiting a second for dongle to initialize'
                time.sleep(1)
                hciout = pexpect.spawn('hcitool lescan', timeout=1)
                j = hciout.expect(['LE Scan ...',pexpect.TIMEOUT], timeout=1)
                if j == 0:
                    ble_adrs = gatheradr(hciout)
                    break
                if j == 1:
                    'Dongle not plugged in.'
                    closeall(hciout)
                    break
            elif inp.lower().startswith('n'):
                closeall(hciout)
                break
            else:
                print 'Did not understand command. Try again.'
    if i == 2:
        print 'Is dongle plugged in? Could not scan: ', hciout.before, hciout.after
        closeall(hciout)
    if i == 3:
        print 'Is dongle pluggeded in? Could not scan: ', hciout.before, hciout.after
        closeall(hciout)
    return ble_adrs

        
if __name__ == "__main__":
    ble_adrs = blescan()
    if ble_adrs:
        print 'Addresses found: ', ble_adrs
    else:
        print 'Empty set. No addresses found'


        #self.handle = 'b' #!! this is the TX service on the nRF8001 adafruit breakout with callbackEcho sketch
 #= int(sys.argv[1])
