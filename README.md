# mbutil
** mbutil ** is a simple C-style library to read data from a modbus RTU slave.
Uses a state machine to parse the response.
Was original developed to read Eastron SDM72D-M Smartmeter. But can be used for any other device to read with function 03 to float (4byte long IEEE).

I just wanted to have a very small and functional limited ways to utilize modbus for this device. There are many more libs out for modbus but they all very large.

Developed for arduino devices and ESP8266. 

Part of restsmart project.

# Disclaimer:
This library is obsolete and will be overtaken by mbparser library.