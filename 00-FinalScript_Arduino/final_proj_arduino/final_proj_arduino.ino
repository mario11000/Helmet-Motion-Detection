/*
  SoftwareSerial(rxPin, txPin):
      rxPin: the pin on which to receive serial data
      txPin: the pin on which to transmit serial data

      in order to have more than one serial communication enabled, it is required to specify which serial is listening to data at any given time

  Wiring the modules:

  Connection format: module pin => connection pin
  Always connect TX-module to RX-arduino and RX-module to TX-arduino

  Abbreviations:
  - DP = Digital Pin
  - AP = Analog Pin

  1 - HC-06 Module:
    GND => GND
    VCC => 5V
    TX => DP 11
    RX => DP 10
    State => DP 9

  2 - MPU6050 Module:
    GND => GND
    VCC => 5V
    SCL => AP 5
    SDA => AP 4
    INT => DP 2

  3 - NEO-6M Module:
    GND => GND
    VCC => 5V
    TX => DP 4
    RX => DP 3

  4 - LED (for mode control):
    long foot => DP 7
    short foot => GND

  5 - LED (for GPS control):
    long foot => DP 12
    short foot => GND

  6 - LED (for BLE control):
    long foot => DP 8
    short foot => GND

  7 - Push Button: (!Warning, to avoid malfunctions:
                      - Connect the GND leg to a GND different than that of the HC-06 Module
                      - Connect the DP pin in a pin container other than the HC-06 Module!)
    one leg => DP 6
    opposite leg => GND

*/

//include libraries
#include <TinyGPS++.h>
#include <math.h>
#include <Wire.h>
#include <MPU6050_HelmetMotion.h>
#include <SoftwareSerial.h>

//defining led and button pins
#define led_pin_gps 12
#define led_pin_mode 7
#define button_pin_mode 6

//defining pin numbers for TX and RX for arduino => connect the TX and RX pins of the module in reverse
#define Ard_BTRX 11
#define Ard_BTTX 10

#define Ard_GPSRX 4
#define Ard_GPSTX 3

//defining the minimum time needed for the button to be pressed to change modes (in ms)
#define timer_manual 1000
#define timer_auto 3000

//defining the intervals when values from the sensors must be read (in ms)
#define interval_gyro 0
#define interval_gps 20000
#define interval_blink 1000

//defining constant parameters for the conversion of GPS coords
#define prec 5
#define coordBuffSize 12

//initilizing variable using the MPU6050 library to control the gyroscope
MPU6050 mpu6050(Wire);

//initilizing variable using the TinyGPS++ library to control the gps module
TinyGPSPlus gps;

//specifying which DP pins are used for serial communication

//connect TX-module to Ard_BTRX and RX-module to Ard_BTTX
SoftwareSerial BTSerial(Ard_BTRX, Ard_BTTX);

//connect TX-module to Ard_GPSRX and RX-module to Ard_GPSTX
SoftwareSerial GPSSerial(Ard_GPSRX, Ard_GPSTX);

//defining the different kind of motion modes
const int16_t mode_manual = 0;
const int16_t mode_auto = 1;

//variable that helps us keep track on how long the person pressed the button
int btnPressLength = 0;

//variable that helps us in keeping track to what state is the LED in (used for the blinking)
int gps_led_state = LOW;

//x, y to be read from the gyroscope
int16_t x, y;

//specifying initial mode
int16_t mode = mode_manual;

//timer variables that enable the control on the data reading process
long timer_gyro = 0;
long timer_gps = 0;
long timer_blink = 0;
float time_lost = 0;

//lat and lon values to be read from the GPS
double latVal = 0;
double lonVal = 0;

//test if gps coord started to be read
bool gps_valid = false;

//function prototypes
void sendBLEData();
void changeMode();
void changeLedStateMode();
void changeLedStateGPS();
void getGyroVals();
void getGPSCoords();
void activateBLE();
void activateGPS();
int roundAnglex(double);
int roundAngley(double);

//initializing the device
void setup() {

  //specifying that the button pin is an input pin
  //pullup is used to avoid false inputs, a known state (HIGH) is given to the button when not pressed
  pinMode(button_pin_mode, INPUT_PULLUP);
  
  //specifying that the mode led pin is an output pin
  pinMode(led_pin_mode, OUTPUT);

  //specifying that the GPS led pin is an output pin
  pinMode(led_pin_gps, OUTPUT);

  //intializing baud rate of modules
  Serial.begin(9600);
  BTSerial.begin(9600);
  GPSSerial.begin(9600);

  //initiate the Wire library which enables the communication with I2C (inter-intergrated circuits) components
  Wire.begin();

  //start and calibrate the gyroscope
  mpu6050.begin();
  
  //turn on/off mode LED according to the current mode
  changeLedStateMode();
}

//main loop (executes forever)
void loop() {

  Serial.println("---------------------------------------------");
  //check for mode changes
  changeMode();

  //get updated gyroscope values
  getGyroVals();

  //get updated GPS coordinates
  getGPSCoords();

  //send data via HC-06 Module
  sendBLEData();
  Serial.println("\n---------------------------------------------\n");
}

//check for mode change
/*
  - Args: (none)
  - Method:
    - if button not pressed, do nothing
    - if button pressed:
      + calculate the duration of button press (approx. incrementing by 100ms the duration each time)
      + change mode according to the new button press duration value
      + change LED state according to the mode
    - reset button press duration
*/
void changeMode() {
  float startTime = millis();
  bool changing = false;
  
  //since pin mode is input_pullup we should wait for a LOW signal
  while (digitalRead(button_pin_mode) == LOW) {
    changing = true;
    //wait 100ms and increment the duration
    delay(100);
    btnPressLength = btnPressLength + 100;

    //check for changes
    if (btnPressLength >= timer_auto) {
      mode = mode_auto;
    } else if (btnPressLength >= timer_manual) {
      mode = mode_manual;
    }

    changeLedStateMode();
    
    Serial.println(btnPressLength);
  }
  //reset duration
  btnPressLength = 0;
  
  Serial.print("\nMode:");
  Serial.println(mode);
  
  float endTime = millis();
  if(changing)
    time_lost = endTime - startTime;
  else
    time_lost = 0;
}

//change mode LED state
/*
  - Args: (none)
  - Manual Mode: LED ON
  - Automatic Mode: LED OFF
*/
void changeLedStateMode() {

  switch (mode) {
    case mode_manual: digitalWrite(led_pin_mode, HIGH);
      break;
    case mode_auto: digitalWrite(led_pin_mode, LOW);
      break;
  }
}

//get new gyroscope values (x & y)
/*
  - Args: (none)
  - Method:
    - update MPU6050 values
    - get raw X & Y
    - convert and round raw values
*/
void getGyroVals() {  
  //after every interval_gyro calculate the new values
  if (millis() - timer_gyro > interval_gyro) {
    mpu6050.setTimeLoss(time_lost);
    
    //get new gyro values
    mpu6050.update();
    
    x = roundAnglex((double)(mpu6050.getAngleX()));
    y = roundAngley((double)(mpu6050.getAngleY())) - 90;
    
    Serial.print("\nX : "); Serial.print(x);
    Serial.print("\tY : "); Serial.println(y);

    //reset timer_gyro
    timer_gyro = millis();
  }
}

//round raw gyroscope value
/*
  - Args:
      + double [raw value of gyroscope to be rounded]
  - Method:
      -if raw out of range [min, max], return min or max accordingly
      -round raw to the nearest whole number ending with 0 or 5 (i.e. roundAngle(21) = 20
                                                                      roundAngle(23) = 25)
*/
// this where mario made the few changes
int16_t roundAnglex(double raw) {

  /*int minimum = -90;
  int maximum = 90;

  if (raw < minimum)
    return minimum;

  if (raw > maximum)
    return maximum;
    */

  int16_t temp;

  //can modify range to round to greater or less precision
  int16_t range = 5;
  int16_t rawInt = raw;

  temp = rawInt % range;

  if (temp >= range / 2)
    rawInt += (range - temp);
  else
    rawInt -= temp;

  return rawInt;
}

//and also here mario made changes
int16_t roundAngley(double raw) {

  /*int minimum = -180;
  int maximum = 0;

  if (raw < minimum)
    return minimum;

  if (raw > maximum)
    return maximum;*/
    

  int16_t temp;

  //can modify range to round to greater or less precision
  int16_t range = 5;
  int16_t rawInt = raw;

  temp = rawInt % range;

  if (temp >= range / 2)
    rawInt += (range - temp);
  else
    rawInt -= temp;

  return rawInt;
}

//get GPS coordinates (lat, lon)
/*
  - Args: (none)
  - Method:
      -if data is available in the serial read it
      -encode this data in the appropriate format used in the gps library
      -check if location format is valid
      -if valid location, get lat and lon
*/
void getGPSCoords() {
  //get GPS coords after every interval_gps
  if (millis() - timer_gps > interval_gps) {
        
    //test if any data is available
    while (GPSSerial.available() > 0) {
      //encode the data to the appropriate format used in the GPS library
      if (gps.encode(GPSSerial.read())) {
        //if GPS not connected to any satellites
        if (gps.satellites.value() <= 0){
          gps_valid = false;
          break;
        }
        //if there are satellites and location is valid, extract and update lat and lon
        else if (gps.location.isValid()) {

          latVal = gps.location.lat();
          lonVal = gps.location.lng();

          Serial.println("Lat and Lon updated!");

          //specify that the data from the GPS module started being read in order to start blinking the LED to indicate its success
          gps_valid = true;

          break;
        }
      }
    }

    //reset timer_gps
    timer_gps = millis();
  }

  //change led state according to GPS status
  changeLedStateGPS();
}

//change GPS LED state
/*
  - Args: (None)
  - If data read: Blink LED
  - If data not read: LED OFF
*/
void changeLedStateGPS() {

  //if no data read or no satellites, return
  if (!gps_valid) {
    gps_led_state = LOW;
    digitalWrite(led_pin_gps, gps_led_state);
    return;
  }

  //every interval_blink change the LED state to blink it
  if (millis() - timer_blink > interval_blink) {

    if (gps_led_state == LOW)
      gps_led_state = HIGH;
    else
      gps_led_state = LOW;

    //change the LED state
    digitalWrite(led_pin_gps, gps_led_state);

    //reset timer
    timer_blink = millis();
  }
}

//send data via bluetooth
/*
  - Args: (none)
  - Order of transmission:
      a - mode (for motion control)
      b - x & y (if needed)
      c - lat & lon after conversion to const char* for transmission
*/
void sendBLEData() {

  //activate the BLE serial port
  activateBLE();

  //send mode
  BTSerial.write(mode);

  //send x&y if needed
  if(mode == mode_manual){
    //convert x to const char* for transmission
    //coordBuffSize >= number_of_characters_&_numbers(x) + 1(for '\0') + 1(for '-' if needed)
    char resultx[coordBuffSize];
    const char* xConv = dtostrf(x, coordBuffSize, prec, resultx);
    
    BTSerial.write(strlen(xConv));
    BTSerial.write(xConv);
    
    char resulty[coordBuffSize];
    const char* yConv = dtostrf(y, coordBuffSize, prec, resulty);
    
    BTSerial.write(strlen(yConv));
    BTSerial.write(yConv);
  }else if(mode == mode_auto){
    int sendData;
    do{
       sendData = BTSerial.read();
    
      //Serial.println(sendData);
    }while(sendData == -1);  
  }
  
  //convert latVal to const char* for transmission
  //coordBuffSize >= number_of_characters_&_numbers(lat) + 1(for '\0') + 1(for '-' if needed)
  char resultLat[coordBuffSize];
  const char* latConv = dtostrf(latVal, coordBuffSize, prec, resultLat);
  Serial.print("Lat: "); Serial.print(latConv);

  //send lat
  BTSerial.write(strlen(latConv));
  BTSerial.write(latConv);

  //convert lonVal to const char* for transmission
  //coordBuffSize >= number_of_characters_&_numbers(lon) + 1(for '\0') + 1(for '-' if needed)
  char resultLon[coordBuffSize];
  const char* lonConv = dtostrf(lonVal, coordBuffSize, prec, resultLon);
  Serial.print("\tLon: "); Serial.println(lonConv);

  //send lon
  BTSerial.write(strlen(lonConv));
  BTSerial.write(lonConv);

  //activate the GPS serial port (thus deactivating the BLE port)
  activateGPS();
}

//activate bluetooth serial
/*
  - Args: (none)
  - Make the bluetooth port active
*/
void activateBLE() {

  if (BTSerial.listen()) {
    Serial.println("BLE Listening");
  }
}

//activate GPS serial
/*
  - Args: (none)
  - Make the GPS port active
*/
void activateGPS() {

  if (GPSSerial.listen()) {
    Serial.println("GPS Listening");
  }
}
