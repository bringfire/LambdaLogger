
#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
#include <SoftwareSerial.h>
#endif
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "BluefruitConfig.h"
#include "SerialFlashDataLogger.h"
#include <TimeLib.h>
#include <Bounce.h>
#include <Metro.h> //Include Metro library

#include <Wire.h>
#include <avr/io.h>
#include <avr/interrupt.h>

/*==========================================================================
                            IMPORT 9DOF IMU LIBRARIES
============================================================================ */

#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_9DOF.h>
#include <Adafruit_L3GD20_U.h>

/* Assign a unique ID to the sensors */
Adafruit_9DOF                   dof   = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified   accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified     mag   = Adafruit_LSM303_Mag_Unified(30302);

/*==========================================================================
                            IMPORT MS5803 PRESSURE SENSOR LIBRARIES
============================================================================ */

#include <SparkFun_MS5803_I2C.h>

/*==========================================================================
                          INITIALIZE 9DOF IMU SENSORS
============================================================================ */
void initSensors()
{
  if(!accel.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  }
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
}



/*==========================================================================
                            IMPORT TINYGPS LIBRARIES
  ============================================================================ */
#include <TinyGPS++.h>

/*/////////// USER CONFIGURATION////////////*/
#define GMT_OFFSET -5            // conversion from GMT to local time, in hours
#define LED_TIME_NOT_LOGGING  1  // flash LED at this rate if not logging
#define LED_TIME_LOGGING     10  // flash LED at this rate if logging



/*/////////// HARDWARE SETTINGS///////////////*/


const int FlashChipSelect = 20; // digital pin for flash chip CS pin

int BLEmodePin = 6; //BLE Mode Select Pin for switching between UART and Command Modes

#define GPSserial Serial1 //define the Hardware Serial port for GPS unit

#define FACTORYRESET_ENABLE         0
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"
#define BLUEFRUIT_HWSERIAL_NAME      Serial3

const int NorthButtonPin = 21;
const int debouncetime = 10;
Bounce NorthButton = Bounce(NorthButtonPin, debouncetime);


#define N_LED_PIN 3
int N_LED_TIME_ON = 250;
int N_LED_TIME_OFF = 250;
Metro N_LED_INTRVL = Metro(N_LED_TIME_ON);
int state = HIGH;

/////////////////////////////////////////////

// number of bytes at start of block that must be 0xFF to consider it erased
#define SMART_ERASE_SIZE 128

/*/////////// end user congifuration ////////////*/



/////////////////SLEEP CONFIG///////////////

#if SLEEP
#include <Snooze.h>

SnoozeBlock snoozeConfig;
int sleepTime = 0;
#endif

///////////////GLOBAL VARIABLES///////////////

double debugCounter = 0;

bool flashFull = false;
bool logging = false;
uint32_t maxDataLength = 0;
uint32_t nextBlinkTime = 0;
uint32_t nextReadTime = 0;
uint32_t sampleMilliseconds = 2000;  // default is 60 seconds

int sensorValue = 0;                 // value read from the pot
int outputValue = 0;                 // value output to the PWM (analog out)

int asleep;                          // Are we asleep based on our current NorthButton?

double pressure_abs, pressure_relative, altitude_delta, pressure_baseline;

// Create Variable to store altitude in (m) for calculations;
float base_altitude = 183.49;        // Altitude of 1859 Burnell Dr. Los Angeles CA 90065

const size_t LED_COUNT = 4;      // number of LED's connected to Teensy -- used in ISR()


///////////////BLNK ISR SETUP////////////////

struct LedCtrl {
  // LED on time in milliseconds
  volatile uint32_t led_on_time;
  // LED off time in milliseconds
  volatile uint32_t led_off_time;
  // count milliseconds in current on + off cycle
  volatile uint32_t led_timer_counter;
};

//define the different blink modes and their respective blink rates
const LedCtrl LED_BLINK_FAST = { 75, 500, 0 };
const LedCtrl LED_BLINK_SLOW = { 100, 1500, 0 };
const LedCtrl LED_BLINK_ON = { 100, 0, 0 };
const LedCtrl LED_BLINK_OFF = { 0, 100, 0 };
const LedCtrl LED_BLINK_ERASING = { 800, 80, 0 };

typedef LedCtrl LedCtrlArray[LED_COUNT];
LedCtrlArray led_ctrl_array;

const uint8_t led_pins[] = { 3, 23, 5, 4 };

unsigned long previousMillis = 0; // will store last time LED was updated

                  // constants won't change :
const long interval = 1000;     // interval at which to blink (milliseconds)

void systick_isr() {
  // Original ISR code
  systick_millis_count++;

  // Our ISR code
  for (size_t led_nr = 0; led_nr < LED_COUNT; led_nr++) {

    auto& led_ctrl = led_ctrl_array[led_nr];
    auto led_cycle_time = led_ctrl.led_on_time + led_ctrl.led_off_time;

    if (led_ctrl.led_timer_counter < led_ctrl.led_on_time) {
      digitalWrite(led_pins[led_nr], HIGH);
      led_ctrl.led_timer_counter++;
    }
    else if (led_ctrl.led_timer_counter < led_cycle_time) {
      digitalWrite(led_pins[led_nr], LOW);
      led_ctrl.led_timer_counter++;
    }
    else {
      led_ctrl.led_timer_counter = 0;
    }
  }
}


//////////CREATE DATALOGGER OBJECT///////////

SerialFlashDataLogger dataLogger;

/////////////////////////////////////////////


//////////CREATE PRESSURE SENSOR OBJECT///////////

MS5803 Psensor(ADDRESS_HIGH);

/////////////////////////////////////////////

/*==========================================================================
                              CREATE BLE UART OBJECT
  ============================================================================ */
Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);


/*==========================================================================
                              BLE HELPER FUNCTION
  ============================================================================ */
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}


//////////////////////////////TinyGPS Object///////////////////////////////////
// The TinyGPS++ object
TinyGPSPlus gps;

///////////////////////////////////////////////////////////////////////////////






////////////////////////////CUSTOM FUNCTIONS///////////////////////////////////
///////////////////////////////////////////////////////////////////////////////


// Thanks to Mike Grusin for letting me borrow the functions below from
// the BMP180 example code.

 double sealevel(double P, double A)
// Given a pressure P (mbar) taken at a specific altitude (meters),
// return the equivalent pressure (mbar) at sea level.
// This produces pressure readings that can be used for weather measurements.
{
  return(P/pow(1-(A/44330.0),5.255));
}

////////////////////////////////////////////////////////////////////////////////
double altitude(double P, double P0)
// Given a pressure measurement P (mbar) and the pressure at a baseline P0 (mbar),
// return altitude (meters) above baseline.
{
  return(44330.0*(1-pow(P/P0,1/5.255)));
}

////////////////////////////////////////////////////////////////////////////////

void showMenu () {
  Serial.println("");
  Serial.println("User Menu :");
  Serial.println("");
  Serial.println("e : erase Flash");
  Serial.println("l : list (display) the logged data from Flash");
  Serial.println("m : smart erase");
  Serial.println("n# : number of seconds between samples");
  Serial.println("s : start/stop logging");
  Serial.println("t# : set date/time (from epochconverter.com or similar)");
  Serial.println("  Debugging commands:");
  Serial.println("d : directory of files in Flash");
  Serial.println("h : hex dump of flash memory");
  Serial.println("? : show this menu");
  Serial.println("");
}

/////////////////////////////////////////////////////////////////


void startLogging() {

  // check if this file is already on the Flash chip
  if (dataLogger.dataFileExists()) {
    logging = false;
    Serial.println(" Data already exists on the Flash chip - not logging");

    if (!Serial) {
      blinkled();
      blinkled();
      blinkled();
    }
  } else {
    // create the file on the Flash chip
    if (dataLogger.openDataFile (maxDataLength)) {
      Serial.println("  File created");
    } else {
      logging = false;
      Serial.println("  Error creating file");

      if (!Serial) {
        blinkled();
        blinkled();
      }
    }
  }
}


/////////////////////////////////////////////////////////////////

bool areYouSure() {
  uint32_t startTime = millis();
  Serial.println ("Are you sure you want to erase entire SPI Flash? y/[n]");

  // wait up to 5 seconds for a response
  while ((millis() - startTime) < 5000) {
    if (Serial.available()) {
      byte inByte = Serial.read();
      if (inByte == 'y') {
        return true;
        // if a non-character (line feed), ignore it
      } else if (inByte < '0') {
        // if any other character, return false
      } else {
        return false;
      }
    }
  }
  return false;
}


/////////////////////////////////////////////////////////////////

String TimeLocPressTempToString(float pressBaseline) {

  int currentDay = gps.date.day();
  int currentMonth = gps.date.month();
  int currentYear = gps.date.year();
  int currentHour = gps.time.hour();
  int currentMinute = gps.time.minute();
  int currentSecond = gps.time.second();
  float currentCentisecond = gps.time.centisecond();
  double latitude = gps.location.lat();
  double longitude = gps.location.lng();
  double temperature_f = Psensor.getTemperature(FAHRENHEIT, ADC_512);
  double pressure_abs = Psensor.getPressure(ADC_4096);
  double pressure_relative = sealevel(pressure_abs, base_altitude);
  double altitude_delta = altitude(pressure_abs , pressBaseline);

  return String("GPS") + "-" + String(currentMonth) + ":" + String(currentDay) + ":" + String(currentYear) + ";" + String(currentHour) + ":" + String(currentMinute) + ":" + String(currentSecond) + ":" + String(currentCentisecond) + ";" + String(latitude, 6) + "," + String(longitude, 6) + ";" + String(temperature_f, 3) + ";" + String(pressure_relative , 3) + ";" + String(altitude_delta , 4) + "\n";
}

/////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////

String TimeLocPressTempDistCourseToString(float pressBaseline, String DistCourse) {

  int currentDay = gps.date.day();
  int currentMonth = gps.date.month();
  int currentYear = gps.date.year();
  int currentHour = gps.time.hour();
  int currentMinute = gps.time.minute();
  int currentSecond = gps.time.second();
  float currentCentisecond = gps.time.centisecond();
  double latitude = gps.location.lat();
  double longitude = gps.location.lng();
  double temperature_f = Psensor.getTemperature(FAHRENHEIT, ADC_512);
  double pressure_abs = Psensor.getPressure(ADC_4096);
  double pressure_relative = sealevel(pressure_abs, base_altitude);
  double altitude_delta = altitude(pressure_abs , pressBaseline);



  return String("GPS") + String(currentMonth) + ":" + String(currentDay) + ":" + String(currentYear) + ";" + String(currentHour) + ":" + String(currentMinute) + ":" + String(currentSecond) + ":" + String(currentCentisecond) + ";" + String(latitude, 6) + "," + String(longitude, 6) + ";" + String(temperature_f, 3) + ";" + String(pressure_relative , 3) + ";" + String(altitude_delta , 4) + ";" + String(DistCourse) + "\n";
}

/////////////////////////////////////////////////////////////////

String TimeLocationToString() {


  int currentDay = gps.date.day();
  int currentMonth = gps.date.month();
  int currentYear = gps.date.year();
  int currentHour = gps.time.hour();
  int currentMinute = gps.time.minute();
  int currentSecond = gps.time.second();
  int currentCentisecond = gps.time.centisecond();
  double latitude = gps.location.lat();
  double longitude = gps.location.lng();

  return String(currentMonth) + ":" + String(currentDay) + ":" + String(currentYear) + ";" + String(currentHour) + ":" + String(currentMinute) + ":" + String(currentSecond) + ":" + String(currentCentisecond) + ";" + String(latitude, 6) + "," + String(longitude, 6) + "\n";
}

/////////////////////////////////////////////////////////////////

String LocationToString() {

  double latitude = gps.location.lat();
  double longitude = gps.location.lng();
  return String(latitude, 6) + "," + String(longitude, 6) + "\n";
}


/////////////////////////////////////////////////////////////////



void updateSampleRate () {
  dataLogger.readSampleRate (sampleMilliseconds);
}


/////////////////////////////////////////////////////////////////

void processMessage() {
  const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013

  unsigned long pctime;
  byte inByte = Serial.read();

  blinkled ();

  if (inByte == 'd') {
    dataLogger.listDirectory();
  }

  if (inByte == 'e') {
    if (areYouSure ()) {
      led_ctrl_array[0] = LED_BLINK_ERASING;
      dataLogger.eraseAll ();
      flashFull = false;
      led_ctrl_array[0] = LED_BLINK_SLOW;
      Serial.println("  Erase complete");
    } else {
      Serial.println("  Not erased");
    }
  }

  if (inByte == 'm') {
    if (areYouSure ()) {
      updateSampleRate();
      dataLogger.smartEraseAll (SMART_ERASE_SIZE, true);
      flashFull = false;

      // re-write sample rate after erasing
      dataLogger.writeSampleRate (sampleMilliseconds);
    }
  }

  if (inByte == 'n') {
    int seconds = Serial.parseInt();

    if (seconds > 0) {
      sampleMilliseconds = seconds * 1000;

      dataLogger.writeSampleRate (sampleMilliseconds);
    }

    updateSampleRate ();
    Serial.print("seconds = ");
    Serial.println(sampleMilliseconds / 1000);
  }

  if (inByte == 'l') {
    dataLogger.listData();
  }

  if (inByte == 's') {
    logging = !logging;
    if (logging) {
      startLogging();
    } else {
      Serial.println ("Stopped logging");
    }
  }

  if (inByte == 't') {

    // go to epochconverter.com to get current seconds since 1/1/1970

    pctime = Serial.parseInt();

    if (pctime >= DEFAULT_TIME) { // check the integer is a valid time (greater than Jan 1 2013)

      // convert from GMT to local time
      pctime = pctime + GMT_OFFSET * 3600;

      setTime(pctime); // Sync Arduino clock to the time received on the serial port

      dataLogger.writeTime (pctime);

      Serial.print ("Time set to ");
      Serial.println (timeToString (pctime));
    } else {
      Serial.print ("Current time is ");
      Serial.println (timeToString (now ()));
    }
  }

  if (inByte == 'h') {
    dataLogger.hexDumpFlash();
  }

  if (inByte == '?') {
    showMenu();
  }

}

/////////////////////////////////////////////////////////////////

void buttonStartStop(){

   logging = !logging;
    if (logging) {
      startLogging();
      led_ctrl_array[0] = LED_BLINK_FAST;  // LED on pin 3 blinks fast
    } else {
      led_ctrl_array[0] = LED_BLINK_SLOW;  // LED on pin 3 blinks fast
      Serial.println ("Stopped logging");
    }

}


/////////////////////////////////////////////////////////////////

void blinkled () {

  // get current mode of LED pin (may be SPI clock)
  int currentMode = *portConfigRegister (N_LED_PIN);

  pinMode(N_LED_PIN, OUTPUT);

  digitalWriteFast(N_LED_PIN, HIGH);
  delay(150);
  digitalWriteFast(N_LED_PIN, LOW);
  delay(100);

  // restore mode of LED pin
  *portConfigRegister (N_LED_PIN) = currentMode;
}

/////////////////////////////////////////////////////////////////

void updateTime () {
  uint32_t pctime;

  if (dataLogger.readTime (pctime)) {

    setTime (pctime);
    // delete file to indicate time has been used
    dataLogger.eraseTime ();
  }
}

/////////////////////////////////////////////////////////////////

String debugValueToLogString (double counter, double debugValue) {

  return String(counter) + ":  " + String(debugValue / 3) + "\n";
}


/////////////////////////////////////////////////////////////////

String valueToLogString (int value, time_t currentTime) {

  return String(value) + ", " + timeToString (currentTime) + "\n";
}


/////////////////////////////////////////////////////////////////


String timeToString(time_t t) {

  tmElements_t tm;
  String timeString = "";

  breakTime (t, tm);

  // digital clock display of the time
  timeString = String(tm.Hour);
  timeString += ":";
  timeString += digitToString(tm.Minute);
  timeString += ":";
  timeString += digitToString(tm.Second);
  timeString += " ";
  timeString += String(tm.Month);
  timeString += "/";
  timeString += String(tm.Day);
  timeString += "/";
  timeString += String(tmYearToCalendar(tm.Year));

  return timeString;
}


/////////////////////////////////////////////////////////////////

String digitToString(int digits) {
  // utility function for digital clock display: prints preceding colon and leading 0
  String digitString = "";
  if (digits < 10) {
    // add leading 0
    digitString += '0';
  }
  digitString += String(digits);

  return digitString;
}

/////////////////////////////////////////////////////////////////

String IMUtoString() {

  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_vec_t   orientation;

  /* Read the accelerometer and magnetometer */
  accel.getEvent(&accel_event);
  mag.getEvent(&mag_event);

  if (dof.fusionGetOrientation(&accel_event, &mag_event, &orientation))
  {
      return String("IMU") + String(orientation.roll) + ":" + String(orientation.pitch) + ":" + String(orientation.heading) + ";" + "\n";
  }
}


/////////////////////////////////////////////////////////////////

String DistCourse(){
  const double LAT_Destin = 33.982189;
  const double LNG_Destin = -118.414194;
  double latitude = gps.location.lat();
  double longitude = gps.location.lng();

  double distanceKm = gps.distanceBetween(latitude, longitude, LAT_Destin, LNG_Destin) / 1000.0;
  double courseToDestination = gps.courseTo(latitude, longitude, LAT_Destin, LNG_Destin);

  String CardinalDirection = String (gps.cardinal(courseToDestination));
  return CardinalDirection;

}

///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

void setup() {

  Serial.begin(115200);  //Serial Monitor .begin and BAUD Rate
  GPSserial.begin(9600); //GPS Serial Port .begin and BAUD Rate

  /* Initialise the sensors */
  initSensors();


  // wait up to 2 seconds for Arduino Serial Monitor
  unsigned long startMillis = millis();
  while (!Serial && (millis() - startMillis < 2000));

  // set the button pinMode to INPUT using the internal pullup
  pinMode(NorthButtonPin, INPUT_PULLUP);

  // initialize all of the LED pins as OUTPUT pins
  for (auto pin : led_pins) pinMode(pin, OUTPUT);

  // set the initial states for the LED's
  led_ctrl_array[0] = LED_BLINK_SLOW;  // LED on pin 3 blinks slowly
  led_ctrl_array[1] = LED_BLINK_OFF;  // LED on pin 23 turned off
  led_ctrl_array[2] = LED_BLINK_OFF;  // LED on pin 5 turned off
  led_ctrl_array[3] = LED_BLINK_OFF;   // LED on pin 4 turned off

  //pinMode(N_LED_PIN, OUTPUT);

  // intialize BLEmodePin as an OUTPUT pin and then write it LOW
  pinMode (BLEmodePin, OUTPUT);
  digitalWrite(BLEmodePin, LOW);


  // set the initial button press state "asleep" to true for detecting long press.
  asleep = true;
  logging = false;

    //Retrieve calibration constants for conversion math.
    Psensor.reset();
    Psensor.begin();

    pressure_baseline = Psensor.getPressure(ADC_4096);


  if (dataLogger.begin(FlashChipSelect)) {
    // maximum data size is all of chip except for 1 MByte for directory info, time, and sample rate
    maxDataLength = dataLogger.getCapacity () - 0x100000;
  } else {
    Serial.println(" *** Unable to access SPI Flash chip");
  }

  showMenu ();

  // read stuff from Flash files

  updateSampleRate ();

  delay(1000);
}


///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

void loop() {

  unsigned long currentTime;
  
  String GPSlogString;
  String IMUlogString;
  String DistCourseString;



  NorthButton.update();

  if (NorthButton.read() != asleep && NorthButton.duration() > 2000) {
    asleep = NorthButton.read();

    if (asleep) {
      buttonStartStop();
    }

  }



  if (Serial.available()) {
    processMessage();
  }


  if (logging) {


      IMUlogString = IMUtoString();
      Serial.print(IMUlogString);
      if (!dataLogger.writeData (IMUlogString.c_str(), IMUlogString.length())) {
        if (!flashFull) {
          Serial.print ("Flash data is full - stopped writing\n");
          flashFull = true;
        }
      }

  while (GPSserial.available() > 0) {

      if (gps.encode(GPSserial.read())){
        DistCourseString = DistCourse();
        GPSlogString = TimeLocPressTempDistCourseToString(pressure_baseline, DistCourseString);
        Serial.print(GPSlogString);
        if (!dataLogger.writeData (GPSlogString.c_str(), GPSlogString.length())) {
        if (!flashFull) {
          Serial.print ("Flash data is full - stopped writing\n");
          flashFull = true;
        }
      }
     }
    }




   }

delay(100);
}

