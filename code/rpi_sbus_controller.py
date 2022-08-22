#
# Read and decode is a python-written copy of the cpp code by MxFxM https://github.com/MxFxM/Teensy_SBUS_Controller.
#
# This code reads sbus as UART input (here coming from a rc-receiver) and decodes it.
# Will be expanded to either directly pipe the input (in this case to a flight-controller) over UART, or
# manipulate the values before piping them.
#

import serial

ser = serial.Serial(
        port='/dev/ttyAMA0', #Replace ttyS0 with ttyAM0 for Pi1,Pi2,Pi0 or ttyS0 if not using RPI 3B
        baudrate = 100000,
        parity=serial.PARITY_EVEN,
        stopbits=serial.STOPBITS_TWO,
        bytesize=serial.EIGHTBITS
)

serial_buffer = [0] * 30
serial_buffer_index = 0
databits = [0 for _ in range(200)]
channels = [0 for _ in range(17)]
failsafe = False
frame_lost = False
ch18 = False
ch17 = False

while True:
    #print(ser.read())
    
    new_bytes = ser.inWaiting()
    if new_bytes > 0:
        for count in range(new_bytes):
            serial_buffer[serial_buffer_index] = int.from_bytes(ser.read(), "little")
            serial_buffer_index += 1

            if serial_buffer[0] == 0x0F:                    # header byte
                if serial_buffer_index == 25:               # length of one frame
                    if serial_buffer[24] == 0x00:           # footer byte / frame is complete
                        for databyte in range(24):          # for every byte except header and footer
                            for bitnum in range(8):         # for every bit / sort the bit in the list
                                databits[databyte * 8 + bitnum] = (serial_buffer[1 + databyte] >> bitnum) & 0x01
                        for channelno in range(17):         # for every channel
                            # value is received through the bits from the list
                            channels[channelno] = (
                                (databits[channelno * 11 + 0] << 0) +
                                (databits[channelno * 11 + 1] << 1) +
                                (databits[channelno * 11 + 2] << 2) +
                                (databits[channelno * 11 + 3] << 3) +
                                (databits[channelno * 11 + 4] << 4) +
                                (databits[channelno * 11 + 5] << 5) +
                                (databits[channelno * 11 + 6] << 6) +
                                (databits[channelno * 11 + 7] << 7) +
                                (databits[channelno * 11 + 8] << 8) +
                                (databits[channelno * 11 + 9] << 9) +
                                (databits[channelno * 11 + 10] << 10)
                            )

                        # special bits
                        failsafe = channels[16] & 0x08
                        frame_lost = channels[16] & 0x04
                        ch18 = channels[16] & 0x02
                        ch17 = channels[16] & 0x01
                        print(channels)

                    # frame is done
                    serial_buffer_index = 0
                break
            else:
                # not the start byte
                serial_buffer_index = 0
