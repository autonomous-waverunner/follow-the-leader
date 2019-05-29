# Ellipse 2D
The Ellipse 2D is a very powerful device capable of gathering positional data. In 
the Ellipse container a USB-drive containing extensive, relevant documentation can
be found. 

## Start up: 
- Follow the Quick Start guide
- The CA-ELI-D-USB-3M cable can be used to connect the device to a computer. This can 
be powerd via battery or the included wall socket.

## Kalman filter: 
The Ellipse uses a Kalman filter to fuse the GPS-data with the inertial measurements to 
improve positional accurancy. The outputs for the Kalamn filter are called EKF in the 
sbgCenter configuration menu. To tune the filter it needs a lot of dynamic input, i.e. 
drive the WR aggresively after GPS reconfiguration for about 15-20 minutes, have fun. 

## Configuration:  
- Increasing the baudrate might cause issues due to the limited baudrate of the Xavier.
- The dimensions for antenna placement was done using less accurate measurements and 
could be improved. 

## Communication with Xavier computer: 
The Ellipse 2D output signal is a 7.7 V rs232 signal. In the Xavier enclosure is a 
max202 IC-chip converting this signal to a TTL 5 V signal. The Xavier I/O pins 
require a 3.3 V signal which is why there is a resistor on the Rx pin, burning off the 
additional voltage. This however means that the Xavier can't communicate with the 
Ellipse. This can be solved by changeing the IC chip to one converting rs232 to TTL 
3.3 V and removing the resistor **(this is higly recommended)**.
