import os
import time
import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn

# create the spi bus
spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)

# create the cs (chip select)
cs = digitalio.DigitalInOut(board.D22)

# create the mcp object
mcp = MCP.MCP3008(spi, cs)

# create an analog input channel on pin 0
chan0 = AnalogIn(mcp, MCP.P0)

while True:

	print('Raw ADC Value: ', chan0.value)
	print('ADC Voltage: ' + str(chan0.voltage) + 'V')

	last_read = 0       # this keeps track of the last potentiometer value
	tolerance = 250     # to keep from being jittery we'll only change
        	            # volume when the pot has moved a significant amount
                	    # on a 16-bit ADC

	time.sleep(1)
