Blinkatest is the standard setup that checks if I2C and SPI have been set up properly
imutest.py runs the output of the BNO055
instablinka is a shell script to set up the adafruit blinka interface onto the pi
spitest checks to see if data is being received over SPI/The MCP3008 chip is working.
##
For the SPI, need lib as follows 
`sudo pip3 install adafruit-circuitpython-mcp3xxx`
##

##
for IMU need lib as follows:
`sudo pip3 install adafruit-circuitpython-bno055`
##

##
For blinka, ensure
`sudo apt-get update`
`sudo apt-get upgrade`
`sudo pip3 install --upgrade setuptools`
if pip3 doesnt work
`sudo apt-get install python3-pip`
Run instablinka after the above steps
##

