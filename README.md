# line follower sensor

This documents a simple control board that reads an 8-channel Pololu Reflectance Sensor, converts the readings to a integer that represents the position of the center of a detected line within the sensor space 

Note that the base Pololu sensor library returns a number from 0 - (1000 x (the number of sensors in the array)); 

The software shown here for this board scales that number to a range of 0 - 250 -- if the line is under the middle sensor it would return 125; if under the left-most sensor it returns 0, if under the right-most sensor it regurns 250)

It uses the Arduino library provided by Pololu for getting the raw data, then scales it and returns its to a central robot controller over i2c using a simple protocol.


Video of a simple robot using this sensor being driven by a "main controller" that implements simple bang-bang style controller (someday maybe a PID, but i got distracted...)

https://photos.app.goo.gl/8NZnnZRrZfpAwnC36

Sensor Library and hardware info

 * this code requires Pololu QTRSensors library for Arduino
 * https://pololu.github.io/qtr-sensors-arduino/  library documentation
 * https://github.com/pololu/qtr-sensors-arduino
 * https://www.pololu.com/product/4148    the sensor (8 channel) (as used)

Processor Used 

The software in the .ino file runs on an Adafruit ItsyBitsy M0 Express board, though it could run on almost anything that can be coded in Arduino IDE that has sufficient number of pins and an i2c interface (the PCB described here assumes the ItsyBitsy form factor).
  With minor modifications it could send its output to a serial channel or to a DAC if preferred instead of i2c.

* https://www.adafruit.com/product/3727

