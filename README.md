# TRAKBOX
Data acquisition system 

Components:
  Main Board:
    LoLin style NodeMCU type esp8266
  GPS:
    GT-U7
  MCU:
    BNO055 10 DOF MCU
  Environment:
    BME280
    
    
 Notes:
    TinyGPSPlus is used for GPS data. All the examples set the baud rate at 4800.
    I found none of the examples worked until I set the baud rate at 9600.
