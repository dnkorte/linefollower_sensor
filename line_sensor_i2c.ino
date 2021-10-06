/*
 * line_sensor_i2c.ino
 * 
 * Author(s):  Don Korte
 * Repository: https://github.com/dnkorte/linefollower_sensor 
 * Version: 1.0 Aug 29 2020; 
 * 
 * Used by: https://github.com/dnkorte/linefollower_controller
 * 
 * this code requires Pololu QTRSensors library for Arduino
 * https://pololu.github.io/qtr-sensors-arduino/  library documentation
 * https://github.com/pololu/qtr-sensors-arduino
 * https://www.pololu.com/product/4148    the sensor (8 channel) (as used)
 * 
 * https://www.pololu.com/docs/0J13 Sensor Application Note
 * 
 * Note that per the library documentation, it returns "position value" as an integer
 * from 0 => 1000*(number of sensors-1);  
 * for example, for a 6 sensor array, the sensor values to the serial monitor as
 * numbers from 0 (maximum reflectance) to 1000 (minimum reflectance) followed
 * by the estimated location of the line as a number from 0 to 5000. 1000 means
 * the line is directly under sensor 1, 2000 means directly under sensor 2,
 * etc. 0 means the line is directly under sensor 0 or was last seen by sensor
 * 0 before being lost. 5000 means the line is directly under sensor 5 or was
 * last seen by sensor 5 before being lost.
 *
 * dnk: note when using this board with ItsyBitsy M0, it is important to 
 * power the sensor from the 3v output on the ItsyBitsy, rather than Vbat
 * otherwise the levels are wrong and it doesn't see the sensors... 
 * 
 * Note RingbufCPP library (from Arduino IDE) is documented at
 *     https://github.com/wizard97/ArduinoRingBuffer
 *     https://github.com/wizard97/Embedded_RingBuf_CPP
 *
 * MIT License
 * Copyright (c) 2020 Don Korte
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE. * 
 */
 
#include <QTRSensors.h>
#include <Wire.h>
#include <RingBufHelpers.h>
#include <RingBufCPP.h>
                     

/*
 * ********************************************************************
 * fifo buffer for i2c managed by RingBuf (used to store i2c commands)
 * ********************************************************************
 */
RingBufCPP<uint8_t, 12> commandbuffer;   // buffer has room for 12 characters

// the i2c address that this device will respond to
uint8_t i2c_address = 0x32;

QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

/*
 * array of bytes that represent the device "registers"
 */
#define REG_SENSOR_TYPE 0
#define REG_READ_DELAY 1
#define REG_MODULE_ID 2
#define REG_CALIBRATION_STATUS 3
#define REG_DEVICE_STATUS 4
#define REG_SCALED_POSITION 5
#define REG_LED_STATUS 6
uint8_t myregisters[7];            // array of info (device "registers")

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
  Serial.println("starting...");
  
  // configure the sensors
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){A3, 12, 11, 10, 9, 7, 5, A5}, SensorCount);
  qtr.setEmitterPin(A4);
  
  Wire.begin(i2c_address);                  // join i2c bus as a slave with this address
  Wire.onReceive(isr_i2c_commandreceived);  // register event processing function
  Wire.onRequest(isr_i2c_datarequested);    // register event processing function
  
  myregisters[REG_SENSOR_TYPE] = SensorCount;     // (5=5 led MD pololu, 8=8 led MD pololu, 11=11 led HD pololu)
  myregisters[REG_READ_DELAY] = (SensorCount-1);  // time in mS it takes for a sensor reading to complete for this module
  myregisters[REG_MODULE_ID] = 0x53;        // just to help master verify its really a line follower module out there
  myregisters[REG_CALIBRATION_STATUS] = 0;  // 0=not calibrated, 1=calibrated
  myregisters[REG_DEVICE_STATUS] = 0;       // 0=ready, 1=busy, 3=command queued
  myregisters[REG_SCALED_POSITION] = 125;   // 0 => 250 (125=central, 0=left, 250=right)
  myregisters[REG_LED_STATUS] = 0;          // 0/off 1/on
  
}

void loop() {
  uint8_t thisCommand;

  if (!commandbuffer.isEmpty()) {
    commandbuffer.pull(&thisCommand);

    switch(thisCommand) {
      case 0x01:           // initiate calibration (does not return any value)
        do_calibration();
        break;
        
      case 0x02:           // return position (note master must wait at least 5 mS to look for reading)
        read_sensor();
        break;
        
      case 0x03:           // "uncalibrate" (really just sets flag to say its not calibrated)
        myregisters[REG_CALIBRATION_STATUS] = 0;    // indicate that it has not been calibrated
        break;
        
      case 0x04:           // just turns on LED (this is mostly for testing)
        digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED
        break;
        
      case 0x05:           // just turns off LED (this is mostly for testing)
        digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED
        break;
    }
  }  
  delay(1);
}

/*
 * *********************************************************************************************
 * 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
 * = ~25 ms per calibrate() call.
 * Call calibrate() 400 times to make calibration take about 10 seconds.
 * *********************************************************************************************
 */
void do_calibration() {
  myregisters[REG_CALIBRATION_STATUS] = 0;    // indicate that it has not been calibrated
  myregisters[REG_LED_STATUS] = 1;            // and the LED is on  
  myregisters[REG_DEVICE_STATUS] = 1;         // 0=ready, 1=busy
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode
  
  for (uint16_t i = 0; i < 200; i++) {
    qtr.calibrate();
  }
  
  myregisters[REG_CALIBRATION_STATUS] = 1;   // indicate that it has been calibrated 
  myregisters[REG_LED_STATUS] = 0;           // and the LED is back off
  myregisters[REG_DEVICE_STATUS] = 0;       // 0=ready, 1=busy
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration
}

/*
 * *********************************************************************************************
 * this returns the sensor estimated line position as unsigned int 0-250 (125 means centered)
 * (the actual pololu qtr.readLineBlack returns 0 - 4000)
 * note this takes approx 4090 uS for a 5-channel pololu
 * *********************************************************************************************
 */
uint8_t read_sensor() {
  myregisters[REG_DEVICE_STATUS] = 1;         // 0=ready, 1=busy
  char buffer[80];
  
  // read calibrated sensor values and obtain a measure of the line position
  // from 0 to 4000 (for a white line, use readLineWhite() instead)
  uint16_t position = qtr.readLineBlack(sensorValues);
  
  // convert 0-X000 scale to 0 - 250 (note that the raw position value depends on # of LEDs)
  uint8_t scalefactor = (SensorCount-1) * 1000 / 250;
  uint8_t scaledposition = position / scalefactor;
  //sprintf(buffer, "scalefactor: %d raw: %d scaled: %d  == ", scalefactor, position, scaledposition);
  //Serial.print(buffer);
  //for (uint8_t i = 0; i < SensorCount; i++)
  //{
  //  Serial.print(sensorValues[i]);
  //  Serial.print('\t');
  //}
  //Serial.println();
  myregisters[REG_SCALED_POSITION] = scaledposition;
  myregisters[REG_DEVICE_STATUS] = 0;         // 0=ready, 1=busy
  
}

/*
 * *********************************************************************
 * interrupt service routine to support i2c
 * function that executes whenever data (ie command) is received from master
 * this function is registered as an event          see setup() 
 * *********************************************************************
 */
void isr_i2c_commandreceived(int howMany) {
  int i;
  uint8_t c;
  while (Wire.available() > 0) { // loop through all characters
    c = Wire.read();             // receive byte as character
    commandbuffer.add(c);        // and store it to fifo buffer
  }
}

/*
 * *********************************************************************
 * interrupt service routine to support i2c
 * function that executes whenever request for data is received from master
 * this function is registered as an event          see setup() 
 * *********************************************************************
 */
void isr_i2c_datarequested() { 
  Wire.write(myregisters, 7);
}
