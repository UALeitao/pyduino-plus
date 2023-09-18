/*
 * Sketch to control the pins of Arduino via serial interface
 * by Lekum (https://github.com/lekum)
 * Expanded by Ulisses Leitao in order to use I2C devices: MPC4725 and OLED
 * Expanded by Ulisses Leitão for reading temperature of a DS18B20 sensor.
 * Commands implemented with examples:
 *
 * - RD13 -> Reads the Digital input at pin 13
 * - RA4 - > Reads the Analog input at pin 4
 * - WD13:1 -> Writes 1 (HIGH) to digital output pin 13
 * - WA6:125 -> Writes 125 to analog output pin 6 (PWM)
 * - WC:4095  -> Writes 4095 on MPC4725 DAC 
 * - WO:1     ->  Writes on OLED the msg 1
 * - RT3 -> read the temperature of a DS18B20 sensor at pin 3 (Possibility to chance for readin3 timesand take the media value...)
 *
 *  Implementation of INA219 comands:
 *  RV:1  ->  Read voltage on INA219
 *  RC:1  ->  Read current on INA219 
 * 
 */

char operation; // Holds operation (R, W, ...)
char mode; // Holds the mode (D, A)
int pin_number; // Holds the pin number
int digital_value; // Holds the digital value
int analog_value; // Holds the analog value
int value_to_write; // Holds the value that we want to write
int wait_for_transmission = 5; // Delay in ms in order to receive the serial data
float Voltage;
float Current;
bool ina219_overflow;

/* INA219  */
#include <Wire.h>
#include <INA219_WE.h>
#define INA_ADDRESS 0x40

INA219_WE ina219(INA_ADDRESS);
// INA219_WE ina219 = INA219_WE(); // Alternative: sets default address 0x40

/*  DS18B20  */
#include <OneWire.h>
#include <DallasTemperature.h>
 
// Signal pin for the DS18B20
// Data wire is plugged into digital pin 3 (ONE_WIRE_BUS) on the Arduino
#define ONE_WIRE_BUS 3

// Define an instance of oneWire for the DS18B20 sensor
OneWire sensor_BUS(ONE_WIRE_BUS);
// Pass oneWire reference to DallasTemperature library
DallasTemperature sensors(&sensor_BUS);
DeviceAddress sensorAddress;   // store the sensor address

/* ****************************  *?

/* Including library MCP4725 for DAC     */
#include <SPI.h>
#include <Wire.h>

/* ****************************  */
/* MPC4725    */

#include <Adafruit_MCP4725.h>
Adafruit_MCP4725 dac;

/* ****************************  */

/* Including the library for OLED    */

#include <Adafruit_SSD1306.h> 

#include <Adafruit_GFX.h>
#include <gfxfont.h>
#include <Adafruit_SPITFT_Macros.h>
#include <Adafruit_SPITFT.h>


#define OLED_RESET -1
Adafruit_SSD1306 display(OLED_RESET);

#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2

/* ****************************  */

void set_pin_mode(int pin_number, char mode){
    /*
     * Performs a pinMode() operation depending on the value of the parameter
     * mode :
     * - I: Sets the mode to INPUT
     * - O: Sets the mode to OUTPUT
     * - P: Sets the mode to INPUT_PULLUP
     */

    switch (mode){
        case 'I':
            pinMode(pin_number, INPUT);
            break;
        case 'O':
            pinMode(pin_number, OUTPUT);
            break;
        case 'P':
            pinMode(pin_number, INPUT_PULLUP);
            break;
    }
}

void digital_read(int pin_number){
    /*
     * Performs a digital read on pin_number and returns the value read to serial
     * in this format: D{pin_number}:{value}\n where value can be 0 or 1
     */

    digital_value = digitalRead(pin_number);
    Serial.print('D');
    Serial.print(pin_number);
    Serial.print(':');
    Serial.println(digital_value); // Adds a trailing \n
}

void analog_read(int pin_number){
    /*
     * Performs an analog read on pin_number and returns the value read to serial
     * in this format: A{pin_number}:{value}\n where value ranges from 0 to 1023
     */

    analog_value = analogRead(pin_number);
    delay(10);
    analog_value = analogRead(pin_number);
    delay(10);
    Serial.print('A');
    Serial.print(pin_number);
    Serial.print(':');
    Serial.println(analog_value); // Adds a trailing 
}

void digital_write(int pin_number, int digital_value){
    /*
     * Performs a digital write on pin_number with the digital_value
     * The value must be 1 or 0
     */
	digitalWrite(pin_number, digital_value);
}

void analog_write(int pin_number, int analog_value){
    /*
	 * Performs an analog write on pin_number with the analog_value
	 * The value must be range from 0 to 255
     */
	analogWrite(pin_number, analog_value);
}

void dac_write(int analog_value){
    /*
   * Performs a write on DAC interface with the analog_value
   * The value must be range from 0 to 4095
     */
  dac.setVoltage(analog_value, false);
  delay(wait_for_transmission);
}

void oled_write(int analog_value){
    /*
   * Performs a write a MSG on OLED
   * The value must be range within the predefined msg.
   */
   if (analog_value == 1) {
      // Msg 1
      display.clearDisplay();
      display.setTextSize(2);
      display.setTextColor(WHITE);
      display.setCursor(0,0);
      display.println("HEATER");
      display.println("ON");
      if (pin_number == 1) {
        display.setCursor(85,7);
        display.setTextSize(3);
        display.println("1");
      } else {
        display.setCursor(85,7);
        display.setTextSize(3);
        display.println("2");
      }
      display.display();
      delay(2);                
   } else if (analog_value == 2) {
      display.clearDisplay();
      display.setTextSize(2);
      display.setTextColor(WHITE);
      display.setCursor(0,0);
      display.println("HEATER");
      display.println("OFF");
      if (pin_number == 1) {
        display.setCursor(85,7);
        display.setTextSize(3);
        display.println("1");
      } else {
        display.setCursor(85,7);
        display.setTextSize(3);
        display.println("2");
      }
      display.display();
      delay(2);
   } else if (analog_value == 3) {
      display.clearDisplay();
      display.setTextSize(2);
      display.setTextColor(WHITE);
      display.setCursor(0,0);
      display.println("SETUP 1");
      display.println(" Water ");
      display.display();
      delay(2);
   } else if (analog_value == 4) {
      display.clearDisplay();
      display.setTextSize(2);
      display.setTextColor(WHITE);
      display.setCursor(0,0);
      display.println("SETUP 2");
      display.println("  Oil ");
      display.display();
      delay(2);
    } else if (analog_value == 5) {
      display.clearDisplay();
      display.setTextSize(2);
      display.setTextColor(WHITE);
      display.setCursor(0,0);
      display.println("  Done!");
      display.println("DOWNLOAD");
      display.display();
      delay(2);
    } else if (analog_value == 9) {
      display.clearDisplay();
      display.setTextSize(2);
      display.setTextColor(WHITE);
      display.setCursor(0,0);
      display.println(" OVERHEAT");
      display.println(" WARNING");
      display.display();
      delay(2);
      } else {
      display.clearDisplay();
      display.setTextSize(2);
      display.setTextColor(WHITE);
      display.setCursor(0,0);
      display.println("RLab \n Wellcome!");
      display.display();
      delay(2);
   }
   delay(wait_for_transmission);
}

void temp_read(int pin_number){
    /*
     * Performs the digital read of temperature of a DS18B20 temperature sensor conected in PIN 3 or 4 and returns the value read to serial
     * in this format: A{pin_number}:{value}\n where value is in Celcius
     */
    if (pin_number == 1){
      if (!sensors.getAddress(sensorAddress, 0)){ 
          Serial.println("DS Sensor 1 on PIN 3 not found !");
      }
      // Read the sensor 1
      sensors.requestTemperatures();
      analog_value = 100*sensors.getTempC(sensorAddress);  // transform the float yy.xx in a integer yyxx -> later must be corrected
    }
    else if (pin_number == 2){
      if (!sensors.getAddress(sensorAddress, 1)){ 
          Serial.println("DS Sensor 2 on PIN 3 not found !");
      }
      // Read the sensor 2
      sensors.requestTemperatures();
      analog_value = 100*sensors.getTempC(sensorAddress);  // transform the float yy.xx in a integer yyxx -> later must be corrected
    }
    Serial.print('T');
    Serial.print(pin_number);
    Serial.print(':');
    Serial.println(analog_value); // Adds a trailing 
}

void volt_read(int pin_number){
    /*
     * Performs the readind of bus voltage using a INA219 sensor and returns the value read to serial
     * in this format: V{pin_number}:{value}\n where value is in volts
     */
    Voltage = 0.0;
    ina219_overflow = false;
    ina219.startSingleMeasurement(); // triggers single-shot measurement and waits until completed
    Voltage = ina219.getBusVoltage_V();
    ina219_overflow = ina219.getOverflow();
    if(ina219_overflow){
        Serial.println("Overflow");
    }
    // Sending
    analog_value = 100*Voltage;  // transform the float yy.xx in a integer yyxx -> later must be corrected
    Serial.print('V');
    Serial.print(pin_number);
    Serial.print(':');
    Serial.println(analog_value); // Adds a trailing 
}

void current_read(int pin_number){
    /*
     * Performs the readind of bus voltage using a INA219 sensor and returns the value read to serial
     * in this format: V{pin_number}:{value}\n where value is in volts
     */
    Current = 0.0;
    ina219_overflow = false;
    ina219.startSingleMeasurement(); // triggers single-shot measurement and waits until completed
    Current = ina219.getCurrent_mA();
    ina219_overflow = ina219.getOverflow();
    if(ina219_overflow){
        Serial.println("Overflow");
    }
    // Sending
    analog_value = Current;  // Current given in milliAmpère - Pay attention
    Serial.print('C');
    Serial.print(pin_number);
    Serial.print(':');
    Serial.println(analog_value); // Adds a trailing 
}

void setup() {
    Serial.begin(9600); // Serial Port at 9600 baud
    Serial.setTimeout(100); // Instead of the default 1000ms, in order
                            // to speed up the Serial.parseInt() 
    //sensors.begin();  // initialize the DS18B20 temperature sensor
    sensors.begin();  // Start up the library sensor1
    
    dac.begin(0x62);  // Initialize the DAC MCP4725 on address 0x62 
    dac.setVoltage(2650, false);
    delay(2000);
    dac.setVoltage(0, false);    
    // INA219 settings...
    Wire.begin();
    ina219.init();
    ina219.setADCMode(SAMPLE_MODE_8); // choose mode of measurements: media value of 8 measuments
    //ina219.setADCMode(BIT_MODE_12); // choose mode of measurements 12 bits
    ina219.setMeasureMode(TRIGGERED); // setting Triggered measurements in INA219
    ina219.setPGain(PG_320); // setting gain of INA219
    ina219.setBusRange(BRNG_32); // setting max bus voltage range
    //ina219.setCorrectionFactor(0.98); // insert your correction factor if necessary
    //  Initialize the display
    // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)  // init done
    // Testing the relay
    pinMode(4,OUTPUT);  // Engine to shake Calorimeter 1
    pinMode(5,OUTPUT);  // Engine to shake Calorimeter 2
    pinMode(6,OUTPUT);  // Relay to warm up Calorimeter 1  CUIDADO => Inserir mesma porta no logTemp.py getdata()
    pinMode(7,OUTPUT);  // Relay to warm up Calorimeter 2  CUIDADO => Inserir mesma porta no logTemp.py getdata()
    pinMode(8,OUTPUT);  // Led to show selected  Calorimeter 1
    pinMode(9,OUTPUT);  // Led to show current in Calorimeter 1
    pinMode(10,OUTPUT);  // Led to show selected  Calorimeter 2
    pinMode(11,OUTPUT);  // Led to show current in Calorimeter 2
    digitalWrite(8,1);   
    digitalWrite(9,1);
    digitalWrite(10,1);
    digitalWrite(11,1);
    delay(1000);
    analogWrite(4, 0);  //  stop the shaker
    analogWrite(5, 0);
    digitalWrite(8,0);
    digitalWrite(9,0);
    digitalWrite(10,0);
    digitalWrite(11,0);
    digitalWrite(6,1);  // Inverse logic:  Turn off the heater relay of Calorimeter 1
    digitalWrite(7,1);  // Inverse logic:  Turn off the heater relay of Calorimeter 2
    delay(500);
    // Clear the buffer.
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(0,0);
    display.println("Arduino \n Turned ON!");
    display.display();
    delay(2000);
}

void loop() {
    // Check if characters available in the buffer
    if (Serial.available() > 0) {
        operation = Serial.read();
        delay(wait_for_transmission); // If not delayed, second character is not correctly read
        mode = Serial.read();
        pin_number = Serial.parseInt(); // Waits for an int to be transmitted
        if (Serial.read()==':'){
            value_to_write = Serial.parseInt(); // Collects the value to be written
        }
        switch (operation){
            case 'R': // Read operation, e.g. RD12, RA4
                if (mode == 'D'){ // Digital read
                    digital_read(pin_number);
                } else if (mode == 'A'){ // Analog read
                    analog_read(pin_number);
                } else if (mode == 'T'){ // Temperature read
                    temp_read(pin_number);
                } else if (mode == 'V'){ // INA219 voltage read
                    volt_read(pin_number);
                } else if (mode == 'C'){ // INA219 current read
                    current_read(pin_number);
				} else {
					break; // Unexpected mode
				}
                break;

            case 'W': // Write operation, e.g. WD3:1, WA8:255
                if (mode == 'D'){ // Digital write
                    digital_write(pin_number, value_to_write);
                } else if (mode == 'A'){ // Analog write
                    analog_write(pin_number, value_to_write);
                } else if (mode == 'C'){ // DAC write
                    dac_write(value_to_write);
                } else if (mode == 'O'){ // OLED write
                    oled_write(value_to_write);
                } else {
                    break; // Unexpected mode
                }
                break;

            case 'M': // Pin mode, e.g. MI3, MO3, MP3
                set_pin_mode(pin_number, mode); // Mode contains I, O or P (INPUT, OUTPUT or PULLUP_INPUT)
                break;

            default: // Unexpected char
                break;
        }
    }
}
