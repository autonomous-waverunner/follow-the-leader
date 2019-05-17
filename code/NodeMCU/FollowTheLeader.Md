
When the code is uploaded on the NodeMCU or any microcontroller that falls within the same category of ESP12/ESP8266, the name of the SSID created will be called "Follow the leader" and apparently its connected to be open to any external connections. The device runs a local access point (AP) and its default IP address is 192.168.1.1. 

The device runs its own local web server on port 80 and a websockets server on port 81. Data is communicated in realtime via a websockets channel towards NodeMCU which transfers them further to the Nucleo-F303
