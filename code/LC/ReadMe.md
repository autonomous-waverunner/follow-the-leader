## Nucleo-F303 

The source code that ran on the leading boat was divided into two components, the main one being found in the **Nucleo-F303** folder. 
The code was implemented for [ST-Nucleo-F303](https://os.mbed.com/platforms/ST-Nucleo-F303K8/) with the help of the [STM32 Standard Peripheral Libraries](https://www.st.com/en/embedded-software/stm32-standard-peripheral-libraries.html)
using [Visual Studio IDE](https://visualstudio.microsoft.com/) with [VisualGDB](https://visualgdb.com/) extension. However, other tools like [OpenSTM32 - OpenSource](http://openstm32.org/HomePage), [IAR Embedded Workbench](https://www.iar.com/iar-embedded-workbench/) and
software driver-libraries like [HAL](http://stm32f4-discovery.net/2015/07/all-stm32-hal-libraries/) or [CMSIS](https://developer.arm.com/tools-and-software/embedded/cmsis) should still produce the desired results with minimal changes. 

We've tried to make the documentation as clear as possible, but otherwise, the macros used are self-documenting. 

## NodeMCU 

The code found in this folder is written for [NodeMCU](https://en.wikipedia.org/wiki/NodeMCU)
 using the [Arduino IDE](https://www.arduino.cc/en/Main/Software). Note however, that
arduino IDE has to be [configured](https://github.com/esp8266/Arduino) to work with esp8266 devices. 

It is important to note that the code configures the device to generate its own local wifi network running with SSID **"Follow the leader"** 
on a default IP-Address of **192.168.1.1**. It also activates the WebServer and WebSockets Server running on ports 80 and 81 respectively. 

Communication with the system crosses over via websockets via a web browser or any other tool with websockets capability. The data is then sent forward the **Nucleo-F303** which redirects the parameters towards the radio link to be sent over the air. 
