################################
### USB JP4in1 control example
### The module must be running
### a firmware specifically
### modified to takes its input
### from the USB connector (USART1)
###
### goebish@gmail.com
################################
import serial  # pip3 install pyserial
import time
import os


### protocol constants, do not edit
frskyv = 25  # FrSky older protocol
frskyd = 3   # FrSky "D8" protocol
frskyx = 15  # FrSky ACCST V1 "D16" protocol
###


##############
### settings
##############
protocol = frskyd  # select protocol according to the receiver series (V/D/X)
COM_PORT = 'COM6'  # the COM port the JP4in1 is connected to, eg '/dev/ttyUSB0'
# select stm32flash executable, it must be copied along this file
STM32FLASH = 'stm32flash_windows.exe'
#STM32FLASH = 'stm32flash_osx'
#STM32FLASH = 'stm32flash_linux32'
#STM32FLASH = 'stm32flash_linux64'

### end of settings


### more constants, do not edit
THROTTLE1 = 0  # channel 1
RUDDER1   = 1
THROTTLE2 = 2
RUDDER2   = 3
THROTTLE3 = 4
RUDDER3   = 5
THROTTLE4 = 6
RUDDER4   = 7  # channel 8
BIND_FLAG = 0x80
###


### globals
serial_handle = False
channels = [1000,1500,1000,1500,1000,1500,1000,1500,1000,1500,1000,1500,1000,1500,1000,1500]
packed_channels = bytearray([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])
###


### utilities
def us_to_multi(us):
    # convert servo pwm value to multi value
    if us < 860:
        us = 860
    elif us > 2140:
        us = 2140
    result = int(((us-860)<<3)/5)    
    if result < 2047:
        return result
    else:
        return 2047
    
def pack_channels():
    # pack 16 channels into a 22 byte array (16 x 11bit)
    global packed_channels
    bits = 0
    bitsavailable = 0
    idx = 0
    for ch in range(16):
        val = us_to_multi(channels[ch])
        bits |= val << bitsavailable;
        bitsavailable += 11;
        while bitsavailable >= 8:
            packed_channels[idx] = bits & 0xff
            bits >>= 8
            bitsavailable -= 8;
            idx += 1
###


### module control functions        
def start_module():
    # start JP4in1 firmware while in BOOT0 mode, let stm32flash do its magic
    # we have to do that when the JP4in1 is started from USB since it has an
    # hardwired connection to BOOT0, preventing it to boot the firmware automatically
    global serial_handle
    command = "{} -g 0x8002000 {}".format(STM32FLASH, COM_PORT)
    print("starting JP4in1 firmware")
    print(command)
    try:
        res = os.system(command)
    except:
        print("couldn't run {}".format(STM32FLASH))
        return False
    if res != 0:
        print("stm32flash failed, if the JP4in1 firmware is already running (red LED flashing), just ignore this message")
        print("- is the JP4in1 module connected to the computer ?")
        print("- has the module been flashed with the proper firmware ?")
        print("- is a CP210x USB bridge driver installed ?")
        print("- is the module mapped to the proper COM_PORT ? ({})".format(COM_PORT))
        print("- is STM32FLASH command set properly for this OS ?")
        print("- have you tried turning the JP4in1 module on & off ?\n")
    else:
        print("Multi-Module is running")
    print ("opening {} @ 50000 8E2".format(COM_PORT))
    try:
        serial_handle = serial.Serial(COM_PORT, 50000, serial.EIGHTBITS, serial.PARITY_EVEN, serial.STOPBITS_TWO)
    except:
        print("error opening COM port")
        return False
    print("module waiting for commands")
    return True

def send_bind_packets(timeout):
    # send bind packets for x seconds, several receivers can be bound simultaneously
    header = bytearray([0x55, protocol | BIND_FLAG, 0x00, 0x00])
    stoptime = time.time()+timeout
    # start by sending a dummy protocol change to the module
    # since it cannot enter bind state otherwise
    for loop in range(10):
        serial_handle.write(bytearray([0x55, 0x01, 0x00, 0x00, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]))
        time.sleep(0.05)
    print("sending bind packets for {} seconds".format(timeout))
    while time.time() < stoptime:
        pack_channels()
        packet = (header + packed_channels)
        serial_handle.write(packet)
        time.sleep(0.05)
    
def send_control_packet():
    # this function must be called at least once every 70ms
    # or the module will switch to failsafe mode
    header = bytearray([0x55, protocol, 0x00, 0x00])
    pack_channels()
    packet = (header + packed_channels)
    serial_handle.write(packet)
    
def send_failsafe_packet():
    # set failsafe mode to "no pulse"
    for loop in range(10):
        serial_handle.write(bytearray([0x57, protocol, 0x00, 0x00, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]))
        time.sleep(0.05)
###


##################
### main program
##################
if __name__ == "__main__":
    # start JP4in1 module and open COM port
    if not start_module():
        print("couldn't start JP4in1 module, exiting ...")
        exit(1)
        
    # optional, send bind packets, only required to bind
    # the receiver(s) for the first time, note that the 
    # receiver(s) have to be rebooted after bind
    send_bind_packets(5)
    
    # set failsafe mode
    send_failsafe_packet()
    
    print("starting channels transmition")
    
    # example: set throttle for boat #1
    # 1000 is motor off, 2000 is full throttle
    # ESCs should be calibrated with similar values
    channels[THROTTLE1] = 1000
    # for rudders, 1000 is full left, 1500 is neutral,
    # 2000 is full right (might be reversed ...)
    # if servos allow, absolute maximum range is 860-2140
    # warning, do not damage your servos by going over their limits!
    # example:
    channels[RUDDER1] = 1500
    # finally, send the channels to the JP4in1 module
    # this function must be called at least once every 70ms
    send_control_packet()
    # give the module some time to process the packet
    time.sleep(0.02)
    
    # demo:
    # sweep odd numbered outputs (boat rudders) forever ...
    print("sweeping rudders, press CTRL+C to stop ...")
    speed = 15
    while 1:
        # set channel values
        for ch in range(16):
            if ch & 1:  # only affects odd channel numbers
                channels[ch] += speed
        if channels[1] >= 2000 or channels[1] <=1000:
            speed = -speed
        # send packet to JP4in1 module
        send_control_packet()
        # need a short pause between packets
        time.sleep(0.02)
