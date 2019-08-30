/* MPU9250 Basic Example Code
 by: Kris Winer
 date: April 1, 2014
 license: Beerware - Use this code however you'd like. If you 
 find it useful you can buy me a beer some time.
 
 Demonstrate basic MPU-9250 functionality including parameterizing the register addresses, initializing the sensor, 
 getting properly scaled accelerometer, gyroscope, and magnetometer data out. Added //display functions to 
 allow //display to on breadboard monitor. Addition of 9 DoF sensor fusion using open source Madgwick and 
 Mahony filter algorithms. Sketch runs on the 3.3 V 8 MHz Pro Mini and the Teensy 3.1.
 
 SDA and SCL should have external pull-up resistors (to 3.3V).
 10k resistors are on the EMSENSR-9250 breakout board.
 
 Hardware setup:
 MPU9250 Breakout --------- Arduino
 VDD ---------------------- 3.3V
 VDDI --------------------- 3.3V
 SDA ----------------------- A4
 SCL ----------------------- A5
 GND ---------------------- GND
 
 Note: The MPU9250 is an I2C sensor and uses the Arduino Wire library. 
 Because the sensor is not 5V tolerant, we are using a 3.3 V 8 MHz Pro Mini or a 3.3 V Teensy 3.1.
 We have disabled the internal pull-ups used by the Wire library in the Wire.h/twi.c utility file.
 We are also using the 400 kHz fast I2C mode by setting the TWI_FREQ  to 400000L /twi.h utility file.

Add this to platformio.ini and place mopdified ESP8266HTTPClient.h (line 49) with 5000 delay changed to 50 for Geiger 
lib_deps =
  D:\Googledrive\Manuals\GitKraken\Rubber duck\ESP8266HTTPClient\
 */

#define xstr(s) str(s)
#define str(s) #s

//#include <SPI.h>
#include <Arduino.h>
#include <GY91.h>
#include "Adafruit_BMP280.h"
#include <DFPlayer_Mini_Mp3.h> //Library for TF Sound module
#include <SoftwareSerial.h>    //Library for serial comms to TF Sound module
#include <CapacitiveSensor.h>
#include <FastLED.h>
#include <ESP8266HTTPClient.h>
#include <WiFiManager.h>
#include <ESP8266WiFiMulti.h>
#include <ArduinoOTA.h>
#include <DNSServer.h>
#include <ArduinoJson.h>
#include "FS.h"
#include "ESP8266FtpServer.h"
#include <BlynkSimpleEsp8266.h>
#include <TimeLib.h> //https://github.com/PaulStoffregen/Time.git

#define BLYNK_PRINT Serial
#define DBLYNKCERT_NAME "H3reyRYu6lEjFVRLbgwMF9JwLVMd8Lff"
const char auth[] = xstr(BLYNKCERT_NAME); // your BLYNK Cert from build flags

// See also MPU-9250 Register Map and Descriptions, Revision 4.0, RM-MPU-9250A-00, Rev. 1.4, 9/9/2013 for registers not listed in
// above document; the MPU9250 and MPU9150 are virtually identical but the latter has a different register map

//Magnetometer Registers
#define AK8963_WHO_AM_I 0x00 // should return 0x48

// NTP
#define NTP_SERVER "ch.pool.ntp.org"

// Set initial input parameters
enum Ascale
{
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

enum Gscale
{
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

enum Mscale
{
  MFS_14BITS = 0, // 0.6 mG per LSB
  MFS_16BITS      // 0.15 mG per LSB
};

//Tweaks
#define TEMP_CORR (-1)  //Manual correction of temp sensor (mine reads 1 degree too high)
#define ELEVATION (100) //Enter your elevation in m ASL to calculate rel pressure (ASL/QNH) at your place

// Specify sensor full scale
uint8_t Gscale = GFS_250DPS;
uint8_t Ascale = AFS_2G;
uint8_t Mscale = MFS_16BITS;                                 // Choose either 14-bit or 16-bit magnetometer resolution
uint8_t Mmode = 0x02;                                        // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
float aRes, gRes, mRes;                                      // scale resolutions per LSB for the sensors
int16_t accelCount[3];                                       // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];                                        // Stores the 16-bit signed gyro sensor output
int16_t magCount[3];                                         // Stores the 16-bit signed magnetometer sensor output
float magCalibration[3] = {0, 0, 0}, magbias[3] = {0, 0, 0}; // Factory mag calibration and mag bias
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0};     // Bias corrections for gyro and accelerometer
int16_t tempCount;                                           // temperature raw count output
float temperature;                                           // Stores the real internal chip temperature in degrees Celsius
float SelfTest[6];                                           // holds results of gyro and accelerometer self test

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float GyroMeasError = PI * (40.0f / 180.0f); // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = PI * (0.0f / 180.0f);  // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense;
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy.
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
float beta = sqrt(3.0f / 4.0f) * GyroMeasError; // compute beta
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift; // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f                          // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

uint32_t delt_t = 0;                                                            // used to control display output rate
uint32_t count = 0, sumCount = 0, timecount = 0, NSWEcount, zambretticount = 0; // used to control display output rate
float pitch, yaw, roll;
float oldpitch, oldyaw, oldroll;
float deltat = 0.0f, sum = 0.0f;          // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;                         // used to calculate integration interval

float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values
float qq[4] = {1.0f, 0.0f, 0.0f, 0.0f};   // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method
int heading;

//Pressure
int pressure;
int minpressure = 100000;
int maxpressure = 0;
int Zambrettiarraysize = 36;
int Zambretti_array[36];  //Store readings for 3hr period
long Zambretti_count = 1; //cycle from 1 to 36 for each 5min reading
long pressure_read_millis;
int write_timestamp;
int accuracy_in_percent;
int accuracygate = 12; //Must reach this level (12 x 30min) to give a forecast
float measured_temp;
float measured_humi;
float measured_pres;
float SLpressure_hPa; // needed for rel pressure calculation
float HeatIndex;      // Heat Index in °C
float volt;
int rel_pressure_rounded;
double DewpointTemperature;
float DewPointSpread; // Difference between actual temperature and dewpoint

//Touch sensor
int touchthreshold = 500; //Sensor min for touch
uint32_t touchmillis;
const int cap1 = 2;  // D4 470k resistor between pins with 22pf cap in parallel
const int cap2 = 12; // D6 Capacitive Sensor
CapacitiveSensor csensy = CapacitiveSensor(cap1, cap2);
long touchsensor = csensy.capacitiveSensor(30);
uint32_t touchmax = 5000;
int touch1 = 500;

//DF Player
int mp3vol = 20;                //Volume for DF card player.  Keep at 0, used as a flag to skip functions if not wifimanager credentials (no sound option)
int mp3_selected = 1;           //Default mp3 to play ("mp3/0001.mp3" on SDcard)
SoftwareSerial mySerial(0, 14); // Declare pin RX & TX pins for TF Sound module.
long MP3millis;

//LED details
#define NUM_LEDS_PER_STRIP 29 //Number of LEDs per strip
#define PIN_LED D7            //I.O pin on ESP2866 device going to LEDs
#define COLOR_ORDER GRB       // LED stips aren't all in the same RGB order.  If colours are wrong change this  e.g  RBG > GRB.   :RBG=TARDIS
#define brightness 64
#define brightness1 64
#define brightness2 255
struct CRGB leds[NUM_LEDS_PER_STRIP]; //initiate FastLED with number of LEDs
int LEDpick = 0;
long LEDmillis;
int red = 128;
int green = 128;
int blue = 128;

int body = 16;
int head = 24;
int beak = 26;
int eye = 27;

//Wifi and internet variables
const unsigned int localPort = 2390; // local port to listen for UDP packets
const char *geiger = "http://192.168.1.105/j/";
String geigerresponse;
String recovered_ssid;
String recovered_pass;

//Duck touch & sound vars
uint32_t Duckaction_millis = millis();
int Duck_quack_mp3, Duck_north_mp3, Duck_north_LED, Duck_north_mp3_flag = 0;
int Touch_play, Touch_play1 = 0;
int absyaw;

//Millis Unix_timestmap update
unsigned long millis_unix_timestamp;
char UTC[3];
int RequestedTime = 0, TimeCheckLoop = 0, NTPdelaycounter = 0;
int hour_actual = 200, dia_actual = 0, anyo = 0;
int timeout = 0, timeout_prev = 0;
int vera = 0, night = 0;                                                                                //1 = Night, 0 = Day
const int NTP_PACKET_SIZE = 48;                                                                         // NTP time stamp is in the first 48 bytes of the message
byte packetBuffer[NTP_PACKET_SIZE];                                                                     //buffer to hold incoming and outgoing packets
unsigned long epoch = 0, lastepoch = 0, Last_NTP_millis = 0, LastAPI, LastLED, epochstart, startmillis; //Unix time in seconds
int lastepochcount = 1, totalfailepoch = 0;
int hour_UTC, minute_UTC, second_UTC;                               //UTC time
int clock_minutes_from_midnight, local_clock_minutes_from_midnight; //Minutes from midnight
int NTP_Seconds_to_wait = 1;                                        //Initial wait time between NTP/Sunrise pulls (1 sec)
String clock_AMPM;                                                  //AM/PM from NTP Server
int printNTP = 0;                                                   //Set to 1 when a NTP is pulled.  The decode_epoch function used for both NTP epoch and millis epoch.  printNTP=1 in this fucnction only print new NTP results (time).
int Seconds_SinceLast_NTP_millis;                                   //Counts seconds since last NTP pull
int retryNTP = 0;                                                   //Counts the number of times the NTP Server request has had to retry
int UTC_Cycle = 53;
int timefactor = 1;                              //Used for testing to accelerate time
int Display_data_duck;                           //Flag to enable/disable printing on informations
int NTPSecondstowait = 1 * 60 * 60;              //Wait between NTP pulls (sec)
int working_hourtomin = 0;                       //Used to convert hours into total minutes
int localUTC = 12;                               //Country UTC offset, needed for UTC for day/night calc  (+12 for NZ)  don't need to change for daylight saving as no needed for day/night
int UTCoffset = 0;                               //Set my user with touch button +1, -1, 0
const char *NTPServerName = "0.nz.pool.ntp.org"; //local NTP server

//Time delays
uint32_t delayamount = 2000;                     //LED update delay
uint32_t zambretti_delayamount = 30 * 60 * 1000; //Update Zambretti
uint32_t showtime_delayamount = 60 * 1000;       //Display local every 60s
uint32_t pressure_read_interval = 5 * 60 * 1000; //5mins x 60 sec x 1000 millis
uint32_t NSWE_delayamount = 4 * 1000;            //Speak if North every 5s
unsigned long currentMillis = millis();

//Geiger vars
long previousMillis;
long interval = 1000;
int rangehops = 10;
int maxrange = rangehops * 6; //Max CPM count to reach max range (RED) after that LEDs go bright
int percenttemp = 0;          //used by each range as a temp variable
int percentrange = 0;         //used by each range as a temp variable.  Top of range
int percentrangeprev = 0;     //used by each range as a temp variable.  Botton of range
int cpm = 0;                  //Counts per min
int temp = 0;                 //Temperature
int fails = 0;                //Failed times to connect to Geiger counter
int tempmin = 10000;          //Min temp - set high initially
int tempmax = 0;              //Max temp
int cpmmin = 10000;           //Min cpm - set high initially
int cpmmax = 0;               //Max cpm
int restmaxmin = 0;
int APICount = 0;
String JSON_Extract(String lookfor);

//Print var
int Display_data = 0; // 0 = No serial print, 1 = serial print

//Reset - format
String resetfilename = "/reset.txt"; //Filename for triggering restart in SPIFFS

// FORECAST CALCULATION
unsigned long current_timestamp; // Actual timestamp read from NTPtime_t now;
unsigned long saved_timestamp;   // Timestamp stored in SPIFFS
unsigned long millis_unix_timestamp_baseline;
float pressure_value[12];      // Array for the historical pressure values (6 hours, all 30 mins)
float pressure_difference[12]; // Array to calculate trend with pressure differences

// FORECAST RESULT
int accuracy;           // Counter, if enough values for accurate forecasting
String ZambrettisWords; // Final statement about weather forecast
String trend_in_words;  // Trend in words
char ZambrettiLetter();
String ZambrettiSays(char code);
int CalculateTrend();
int16_t readTempData();

int Zambretti_mp3 = 0;
int Zambretti_trend_mp3 = 0;
int Zambretti_LED = 0; //1=Stormy (X>Z), 2=Rain (T>W), 3=Unsettled (P>S), 4=Showery (I>O), 5=Fine (A>H)
const char TEXT_RISING_FAST[] = "Rising fast";
const char TEXT_RISING[] = "Rising";
const char TEXT_RISING_SLOW[] = "Rising slow";
const char TEXT_STEADY[] = "Steady";
const char TEXT_FALLING_SLOW[] = "Falling slow";
const char TEXT_FALLING[] = "Falling";
const char TEXT_FALLING_FAST[] = "Falling fast";
const char TEXT_ZAMBRETTI_A[] = "Settled Fine Weather";
const char TEXT_ZAMBRETTI_B[] = "Fine Weather";
const char TEXT_ZAMBRETTI_C[] = "Becoming Fine";
const char TEXT_ZAMBRETTI_D[] = "Fine, Becoming Less Settled";
const char TEXT_ZAMBRETTI_E[] = "Fine, Possibly showers";
const char TEXT_ZAMBRETTI_F[] = "Fairly Fine, Improving";
const char TEXT_ZAMBRETTI_G[] = "Fairly Fine, Possibly showers early";
const char TEXT_ZAMBRETTI_H[] = "Fairly Fine, Showers Later";
const char TEXT_ZAMBRETTI_I[] = "Showery Early, Improving";
const char TEXT_ZAMBRETTI_J[] = "Changeable Improving";
const char TEXT_ZAMBRETTI_K[] = "Fairly Fine, Showers likely";
const char TEXT_ZAMBRETTI_L[] = "Rather Unsettled Clearing Later";
const char TEXT_ZAMBRETTI_M[] = "Unsettled, Probably Improving";
const char TEXT_ZAMBRETTI_N[] = "Showery Bright Intervals";
const char TEXT_ZAMBRETTI_O[] = "Showery Becoming Unsettled";
const char TEXT_ZAMBRETTI_P[] = "Changeable some rain";
const char TEXT_ZAMBRETTI_Q[] = "Unsettled, short fine Intervals";
const char TEXT_ZAMBRETTI_R[] = "Unsettled, Rain later";
const char TEXT_ZAMBRETTI_S[] = "Unsettled, rain at times";
const char TEXT_ZAMBRETTI_T[] = "Very Unsettled, Finer at times";
const char TEXT_ZAMBRETTI_U[] = "Rain at times, Worse later";
const char TEXT_ZAMBRETTI_V[] = "Rain at times, becoming very unsettled";
const char TEXT_ZAMBRETTI_W[] = "Rain at Frequent Intervals";
const char TEXT_ZAMBRETTI_X[] = "Very Unsettled, Rain";
const char TEXT_ZAMBRETTI_Y[] = "Stormy, possibly improving";
const char TEXT_ZAMBRETTI_Z[] = "Stormy, much rain";
const char TEXT_ZAMBRETTI_DEFAULT[] = "Sorry, no forecast for the moment";

//Declare functions
uint8_t readByte(uint8_t address, uint8_t subAddress);
void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);
void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t *dest);
void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);
void MPU9250SelfTest(float *destination);
void calibrateMPU9250(float *dest1, float *dest2);
void initMPU9250();
void initAK8963(float *destination);
void readMagData(int16_t *destination);
void readGyroData(int16_t *destination);
void readAccelData(int16_t *destination);
void getAres();
void getGres();
void getMres();
void initAK8963(float *destination);
void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
void MPU9250_Dataupdate();
void Timekeeping();
void Touchsensor_check();
void StartOTA();
void Startsensor();
void WiFi_start();
void Test_LEDs();
void DotheLEDs();
void Duck_movement();
void Duck_alignment();
void Check_Geiger();
void LEDrange();
void API_Request();
void Zambretti_calc();
void (*resetFunc)(void) = 0; // declare reset function @ address 0
void measurementEvent();
void ReadFromSPIFFS();
void WriteToSPIFFS(int write_timestamp);
void do_blynk();
void SPIFFS_init();
void Zambretti_nocalc();
void UpdateSPIFFS();
void FirstTimeRun();
void Request_Time();
void sendNTPpacket(const IPAddress &address);
void update_epoch_time();
void decode_epoch(unsigned long currentTime);
void initiate_time();
void LocalClock();
bool Check_Time(); //Check time is correct and ok

//Classes
WiFiUDP udp;                // A UDP instance to let us send and receive packets over UDP
Adafruit_BMP280 myBMP280;   //Adafruit_BMP280 myBMP280;
ESP8266WiFiMulti wifiMulti; // Create an instance of the ESP8266WiFiMulti class, called 'wifiMulti'
IPAddress timeServer;
FtpServer ftpSrv;

uint32_t getmillis1, getmillis2;

//BLYNK Definition to capture virtual pin to start prank
BLYNK_WRITE(V1)
{
  if (param.asInt() == 1)
  {
    // assigning incoming value from pin V1 to a variable
    Serial.println("Formatting SPIFFs");
    SPIFFS.format();
    delay(2000);
    //FirstTimeRun();
    ESP.restart();
  }
}

BLYNK_WRITE(V3)
{
  if (param.asInt() == 1)
  {
    // assigning incoming value from pin V3 to a variable

    if (accuracy >= accuracygate)
    {
      mp3_play(Zambretti_trend_mp3); //only one of these will have a value
      delay(1000);
      yield();
    }
    mp3_play(Zambretti_mp3);
  }
}

void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("Quack Quack..");
  Serial.println();

  WiFi_start();
  StartOTA();

  //Blynk.begin(auth, ssid, pass);  //Blynk setup (if being used).
  //Blynk.begin(auth, WiFi.SSID().c_str(), pass);
  Blynk.config(auth);
  Blynk.connect();

  FastLED.addLeds<WS2811, PIN_LED, COLOR_ORDER>(leds, NUM_LEDS_PER_STRIP); //Initialise the LEDs

  LEDmillis = millis();
  MP3millis = millis();

  csensy.set_CS_AutocaL_Millis(0xFFFFFFFF); //Touch sensor Initialisation                               //Touch sensor Initialisation

  //DF Sound player setup
  mySerial.begin(9600);     //Initiate comms to TF Sound module
  mp3_set_serial(mySerial); //set softwareSerial for DFPlayer-mini mp3 module
  mp3_set_volume(mp3vol);   //Set default volume
  mp3_stop();               //soft-Reset module DFPlayer.  Make sure nothing is playing on start up

  Wire.begin(); //Initial I2C bus
  delay(1000);
  Startsensor();
  Test_LEDs();

  //Initiate array with 0 (no value = no prediction)
  for (int x = 1; x <= Zambrettiarraysize; x++)
  {
    Zambretti_array[x] = 990;
  }

  pressure_read_millis = millis();

  //******** GETTING THE TIME FROM NTP SERVER  ***********************************
  initiate_time(); //Get NTP and time set up for the first time

  //current_timestamp = ntpClient.getUnixTime(); // get UNIX timestamp (seconds from 1.1.1970 on)
  saved_timestamp = current_timestamp;
  millis_unix_timestamp = millis(); //millis time tracking reset to current millis when getting new NTP time
  millis_unix_timestamp_baseline = current_timestamp;

  Serial.print("Current UNIX Timestamp: ");
  Serial.println(current_timestamp);
  Serial.print("Time & Date: ");
  Serial.print(hour(current_timestamp));
  Serial.print(":");
  Serial.print(minute(current_timestamp));
  Serial.print(":");
  Serial.print(second(current_timestamp));
  Serial.print("; ");
  Serial.print(day(current_timestamp));
  Serial.print(".");
  Serial.print(month(current_timestamp)); // needed later: month as integer for Zambretti calcualtion
  Serial.print(".");
  Serial.println(year(current_timestamp));

  SPIFFS_init();

  if (SPIFFS.begin())
  {
    Serial.println("SPIFFS opened!");
    Serial.println("");
  }

  ftpSrv.begin(recovered_ssid, recovered_pass); // username, password for ftp. Set ports in ESP8266FtpServer.h (default 21, 50009 for PASV)

  //Initial run - Do this now so if rebooted gets old data and starts with history, otherwise we wait for 30mins for this to happen
  measurementEvent(); //Get BMP280 Pressure data
  yield();
  ReadFromSPIFFS(); //Read the previous SPIFFs
  yield();
  UpdateSPIFFS(); //Update the SPIFFs
  Zambretti_calc();

  zambretticount = millis(); //Initial count for Zambretti update
  timecount = millis();      //Initial count for NTP update
  count = millis();          //Initial count for Geiger LED update update
  NSWEcount = millis();

  //This requires changes to WiFiManager.cpp and WiFiManager.h

  //Un-comment from WiFiManager.cpp
  // void WiFiManager::startWPS() {
  //   DEBUG_WM(F("START WPS"));
  //   WiFi.beginWPSConfig();
  //   DEBUG_WM(F("END WPS"));
  // }

  //   String WiFiManager::getSSID() {
  //   if (_ssid == "") {
  //     DEBUG_WM(F("Reading SSID"));
  //     _ssid = WiFi.SSID();
  //     DEBUG_WM(F("SSID: "));
  //     DEBUG_WM(_ssid);
  //   }
  //   return _ssid;
  //   }

  //   String WiFiManager::getPassword() {
  //   if (_pass == "") {
  //     DEBUG_WM(F("Reading Password"));
  //     _pass = WiFi.psk();
  //     DEBUG_WM("Password: " + _pass);
  //     //DEBUG_WM(_pass);
  //   }
  //   return _pass;
  //   }

  //Add last 2 lines into WiFiManager.h
  // class WiFiManager
  // {
  //   public:
  //     WiFiManager();
  //     ~WiFiManager();

  // 	String          getSSID();
  // 	String          getPassword();

  Serial.println("");
  Serial.println("********************************************************* START LOOP **************************************************************");
  Serial.println("");
  Serial.println("");
}

void loop()
{
  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of the magnetometer;
  // the magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
  // We have to make some allowance for this orientationmismatch in feeding the output to the quaternion filter.
  // For the MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis just like
  // in the LSM9DS0 sensor. This rotation can be modified to allow any convenient orientation convention.
  // This is ok by aircraft orientation standards!
  // Pass gyro rate as rad/s

  //Handlers
  ftpSrv.handleFTP();
  yield();
  ArduinoOTA.handle();
  yield();
  Blynk.run(); //If Blynk being used
  yield();
  MPU9250_Dataupdate(); //Check for new data MPU9250 and update vars
  yield();
  Timekeeping(); //Keep track ing micros() and use for calculation adjustments
  yield();
  MahonyQuaternionUpdate(ax, ay, az, gx * PI / 180.0f, gy * PI / 180.0f, gz * PI / 180.0f, my, mx, mz); //  MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);
  yield();
  Duck_movement(); //Check for movement and make a sound
  yield();
  Duck_alignment(); //Update yaw, pitch, roll and work out compass heading
  yield();
  Touchsensor_check(); //Any touchs on sensor - speak the forecast
  yield();
  update_epoch_time(); //update epoch time by millis update or NTP request
  yield();
  decode_epoch(epoch); //epoch has been updated, Now turn this into UTC clock_minutes_from_midnight
  yield();

  //heading = atan2(my, mx)*180./PI; //in degrees, magnetometer held level, Z straight down.

  // Serial.println();
  // Serial.print("Heading: ");
  // Serial.print(heading);
  // Serial.print("    yaw: ");
  // Serial.print(yaw);
  // Serial.print("    roll: ");
  // Serial.print(roll);
  // Serial.print("    pitch: ");
  // Serial.print(pitch);
  // Serial.print("    duck_north: ");
  // Serial.print(Duck_north);
  // Serial.print("    Duck_north_LED: ");
  // Serial.print(Duck_north_LED);
  // Serial.print("    NSWE count: ");
  // Serial.println(NSWEcount);

  //*** Display local time
  delt_t = millis() - timecount;
  if (delt_t > showtime_delayamount)
  {
    if (Display_data == 1)
    {
      Serial.println("");
      Serial.println("********************************************************");
    }
    LocalClock();         //Turn minutes from midnight into local H:M
    timecount = millis(); //Reset the count

    if (Display_data == 1)
    {
      Serial.println("********************************************************");
      Serial.println("");
    }
  }

  yield();

  //*** Display Geiger / Zambretti and update LEDs
  delt_t = millis() - count;
  if (delt_t > delayamount)
  {
    Check_Geiger();
    yield();
    Display_data_duck = 1; //Serial prints for this function (need this and global display_data == 1)
    //Duck_alignment(); //Update yaw, pitch, roll
    do_blynk();
    yield();
    Zambretti_calc(); //Calculate weather forecast based on data and print
    count = millis(); //Reset the count
  }

  yield();

  //*** Do the Zambretti - Get data, update SPIFFs array
  delt_t = millis() - zambretticount;
  if (delt_t > zambretti_delayamount)
  {
    measurementEvent(); //Get BMP280 Pressure data
    yield();
    ReadFromSPIFFS(); //Read the previous SPIFFs
    yield();
    UpdateSPIFFS(); //Update the SPIFFs

    //do_blynk();

    zambretticount = millis(); //Reset the count
  }

  //If accuracy less than 6hours then make no foreacast
  if (accuracy < accuracygate && Zambretti_mp3 > 0)
  {
    ZambrettisWords = TEXT_ZAMBRETTI_DEFAULT;
    Zambretti_mp3 = 126;
  }

  yield();

  //*** Check and sat North South West East
  //1-Up, 2-Down, 3-Left, 4-Right, 5-Quack, 6-North, 7-South, 8-East, 9-West, 10-Radiation, 11-

  // // Set NSWE millis only when north first triggered
  // if (abs(yaw) <= 20 && Duck_north_LED == 0)
  // {                          // yaw is -20 to 20
  //   Duck_north_LED = 1;      //LED flag
  //   Duck_north_mp3_flag = 1; //North mp3 flag
  //   NSWEcount = millis();    //Reset the count
  // }

  // yield();

  // //Not north, everything reset
  // if (abs(yaw) > 20)
  // { // yaw is -20 to 20
  //   Duck_north_mp3 = 0;
  //   Duck_north_LED = 0;      //LED flag
  //   Duck_north_mp3_flag = 0; //LED flag
  //   NSWEcount = millis();    //Reset the count
  // }

  yield();

  //Test for playing sound, north for more than 5s
  // delt_t = millis() - NSWEcount;
  // if (delt_t > NSWE_delayamount && Duck_north_mp3_flag > 0) {
  // Duck_north_mp3 = 200;        //Keep this as 5 for LEDs
  // NSWEcount = millis();       //Reset the count so only plays once every 5s
  // }

  yield();

  //Manage Touch sensor or Movement priority
  //If both movement and north, then priortise movement
  // if (Duck_north_mp3 > 0 && Duck_quack_mp3 > 0)
  // {
  //   Duck_north_mp3 = 0;
  // }

  yield();

  //If both movement/north and touch plays, then only play touch
  if (Touch_play > 0 && Duck_quack_mp3 > 0)
  {
    Duck_quack_mp3 = 0;
  }

  yield();

  if (Touch_play + Duck_quack_mp3 > 0) //Anything touched, play a sound
  {
    int mp3_total = (Touch_play + Duck_quack_mp3);
    Serial.println(mp3_total);

    if (mp3_total < 50)
    {
      mp3_set_volume(30); //This is a quack, make it loud
    }
    else
    {
      mp3_set_volume(mp3vol); //Set default volume
    }

    if (accuracy >= accuracygate || Duck_quack_mp3 > 0)
    {
      mp3_play(mp3_total); //only one of these will have a value
      delay(1000);
      yield();
    }
    mp3_play(Touch_play1);
    yield();
    delay(2500);
    yield();

    Touch_play = 0;          //Zambretti trend mp3
    Touch_play1 = 0;         //Zambretti forecast mp3
    Duck_north_mp3 = 0;      //North mp3
    Duck_north_mp3_flag = 0; //North flag
  }

  yield();
}

void Zambretti_calc()
{

  //**************************Calculate Zambretti Forecast*******************************************

  accuracy_in_percent = accuracy * 94 / 12; // 94% is the max predicion accuracy of Zambretti

  ZambrettisWords = ZambrettiSays(char(ZambrettiLetter()));

  if (Display_data == 1)
  {
    Serial.print("Zambretti says: ");
    Serial.print(ZambrettisWords);
    Serial.print(", ");
    Serial.println(trend_in_words);
    Serial.print("Prediction accuracy: ");
    Serial.print(accuracy_in_percent);
    Serial.println("%");
    Serial.print("Zambretti mp3 = ");
    Serial.println(Zambretti_mp3);
    if (accuracy < 12)
    {
      Serial.println("Reason: Not enough weather data yet.");
      Serial.print("We need ");
      Serial.print((12 - accuracy) / 2);
      Serial.println(" hours more to get sufficient data.");
    }
  }
}

char ZambrettiLetter()
{
  //Serial.println("---> Calculating Zambretti letter");
  char z_letter;
  int(z_trend) = CalculateTrend();
  // Case trend is falling
  if (z_trend == -1)
  {
    float zambretti = 0.0009746 * rel_pressure_rounded * rel_pressure_rounded - 2.1068 * rel_pressure_rounded + 1138.7019;
    if (month(current_timestamp) < 4 || month(current_timestamp) > 9)
      zambretti = zambretti + 1;

    if (Display_data == 1)
    {
      Serial.print("Calculated and rounded Zambretti in numbers: ");
      Serial.println(round(zambretti));
    }

    switch (int(round(zambretti)))
    {
    case 0:
      z_letter = 'A';
      break; //Settled Fine
    case 1:
      z_letter = 'A';
      break; //Settled Fine
    case 2:
      z_letter = 'B';
      break; //Fine Weather
    case 3:
      z_letter = 'D';
      break; //Fine Becoming Less Settled
    case 4:
      z_letter = 'H';
      break; //Fairly Fine Showers Later
    case 5:
      z_letter = 'O';
      break; //Showery Becoming unsettled
    case 6:
      z_letter = 'R';
      break; //Unsettled, Rain later
    case 7:
      z_letter = 'U';
      break; //Rain at times, worse later
    case 8:
      z_letter = 'V';
      break; //Rain at times, becoming very unsettled
    case 9:
      z_letter = 'X';
      break; //Very Unsettled, Rain
    }
  }
  // Case trend is steady
  if (z_trend == 0)
  {
    float zambretti = 138.24 - 0.133 * rel_pressure_rounded;

    if (Display_data == 1)
    {
      Serial.print("Calculated and rounded Zambretti in numbers: ");
      Serial.println(round(zambretti));
    }
    switch (int(round(zambretti)))
    {
    case 0:
      z_letter = 'A';
      break; //Settled Fine
    case 1:
      z_letter = 'A';
      break; //Settled Fine
    case 2:
      z_letter = 'B';
      break; //Fine Weather
    case 3:
      z_letter = 'E';
      break; //Fine, Possibly showers
    case 4:
      z_letter = 'K';
      break; //Fairly Fine, Showers likely
    case 5:
      z_letter = 'N';
      break; //Showery Bright Intervals
    case 6:
      z_letter = 'P';
      break; //Changeable some rain
    case 7:
      z_letter = 'S';
      break; //Unsettled, rain at times
    case 8:
      z_letter = 'W';
      break; //Rain at Frequent Intervals
    case 9:
      z_letter = 'X';
      break; //Very Unsettled, Rain
    case 10:
      z_letter = 'Z';
      break; //Stormy, much rain
    }
  }
  // Case trend is rising
  if (z_trend == 1)
  {
    float zambretti = 142.57 - 0.1376 * rel_pressure_rounded;
    //A Summer rising, improves the prospects by 1 unit over a Winter rising
    if (month(current_timestamp) < 4 || month(current_timestamp) > 9)
      zambretti = zambretti + 1;

    if (Display_data == 1)
    {
      Serial.print("Calculated and rounded Zambretti in numbers: ");
      Serial.println(round(zambretti));
    }

    switch (int(round(zambretti)))
    {
    case 0:
      z_letter = 'A';
      break; //Settled Fine
    case 1:
      z_letter = 'A';
      break; //Settled Fine
    case 2:
      z_letter = 'B';
      break; //Fine Weather
    case 3:
      z_letter = 'C';
      break; //Becoming Fine
    case 4:
      z_letter = 'F';
      break; //Fairly Fine, Improving
    case 5:
      z_letter = 'G';
      break; //Fairly Fine, Possibly showers, early
    case 6:
      z_letter = 'I';
      break; //Showery Early, Improving
    case 7:
      z_letter = 'J';
      break; //Changeable, Improving
    case 8:
      z_letter = 'L';
      break; //Rather Unsettled Clearing Later
    case 9:
      z_letter = 'M';
      break; //Unsettled, Probably Improving
    case 10:
      z_letter = 'Q';
      break; //Unsettled, short fine Intervals
    case 11:
      z_letter = 'T';
      break; //Very Unsettled, Finer at times
    case 12:
      z_letter = 'Y';
      break; //Stormy, possibly improving
    case 13:
      z_letter = 'Z';
      break;
      ; //Stormy, much rain
    }
  }
  if (Display_data == 1)
  {
    Serial.print("This is Zambretti's famous letter: ");
    Serial.println(z_letter);
    Serial.println("");
  }
  return z_letter;
}

int CalculateTrend()
{
  int trend; // -1 falling; 0 steady; 1 raising
  //Serial.println("---> Calculating trend");

  //--> giving the most recent pressure reads more weight
  pressure_difference[0] = (pressure_value[0] - pressure_value[1]) * 1.5;
  pressure_difference[1] = (pressure_value[0] - pressure_value[2]);
  pressure_difference[2] = (pressure_value[0] - pressure_value[3]) / 1.5;
  pressure_difference[3] = (pressure_value[0] - pressure_value[4]) / 2;
  pressure_difference[4] = (pressure_value[0] - pressure_value[5]) / 2.5;
  pressure_difference[5] = (pressure_value[0] - pressure_value[6]) / 3;
  pressure_difference[6] = (pressure_value[0] - pressure_value[7]) / 3.5;
  pressure_difference[7] = (pressure_value[0] - pressure_value[8]) / 4;
  pressure_difference[8] = (pressure_value[0] - pressure_value[9]) / 4.5;
  pressure_difference[9] = (pressure_value[0] - pressure_value[10]) / 5;
  pressure_difference[10] = (pressure_value[0] - pressure_value[11]) / 5.5;

  //--> calculating the average and storing it into [11]
  pressure_difference[11] = (pressure_difference[0] + pressure_difference[1] + pressure_difference[2] + pressure_difference[3] + pressure_difference[4] + pressure_difference[5] + pressure_difference[6] + pressure_difference[7] + pressure_difference[8] + pressure_difference[9] + pressure_difference[10]) / 11;

  if (Display_data == 1)
  {
    Serial.print("Current trend: ");
    Serial.print(pressure_difference[11]);
    Serial.print(" -->  ");
  }

  if (pressure_difference[11] > 3.5)
  {
    trend_in_words = TEXT_RISING_FAST;
    Zambretti_trend_mp3 = 127;
    trend = 1;
  }
  else if (pressure_difference[11] > 1.5 && pressure_difference[11] <= 3.5)
  {
    trend_in_words = TEXT_RISING;
    Zambretti_trend_mp3 = 128;
    trend = 1;
  }
  else if (pressure_difference[11] > 0.25 && pressure_difference[11] <= 1.5)
  {
    trend_in_words = TEXT_RISING_SLOW;
    Zambretti_trend_mp3 = 129;
    trend = 1;
  }
  else if (pressure_difference[11] > -0.25 && pressure_difference[11] < 0.25)
  {
    trend_in_words = TEXT_STEADY;
    Zambretti_trend_mp3 = 130;
    trend = 0;
  }
  else if (pressure_difference[11] >= -1.5 && pressure_difference[11] < -0.25)
  {
    trend_in_words = TEXT_FALLING_SLOW;
    Zambretti_trend_mp3 = 131;
    trend = -1;
  }
  else if (pressure_difference[11] >= -3.5 && pressure_difference[11] < -1.5)
  {
    trend_in_words = TEXT_FALLING;
    Zambretti_trend_mp3 = 132;
    trend = -1;
  }
  else if (pressure_difference[11] <= -3.5)
  {
    trend_in_words = TEXT_FALLING_FAST;
    Zambretti_trend_mp3 = 133;
    trend = -1;
  }

  if (Display_data == 1)
  {
    Serial.println(trend_in_words);
  }

  return trend;
}

String ZambrettiSays(char code)
{
  Zambretti_LED = 0;
  String zambrettis_words = "";
  switch (code)
  {
  case 'A':
    zambrettis_words = TEXT_ZAMBRETTI_A;
    Zambretti_mp3 = 100;
    Zambretti_LED = 5;
    break; //see Tranlation.h
  case 'B':
    zambrettis_words = TEXT_ZAMBRETTI_B;
    Zambretti_mp3 = 101;
    Zambretti_LED = 5;
    break;
  case 'C':
    zambrettis_words = TEXT_ZAMBRETTI_C;
    Zambretti_mp3 = 102;
    Zambretti_LED = 5;
    break;
  case 'D':
    zambrettis_words = TEXT_ZAMBRETTI_D;
    Zambretti_mp3 = 103;
    Zambretti_LED = 5;
    break;
  case 'E':
    zambrettis_words = TEXT_ZAMBRETTI_E;
    Zambretti_mp3 = 104;
    Zambretti_LED = 5;
    break;
  case 'F':
    zambrettis_words = TEXT_ZAMBRETTI_F;
    Zambretti_mp3 = 105;
    Zambretti_LED = 5;
    break;
  case 'G':
    zambrettis_words = TEXT_ZAMBRETTI_G;
    Zambretti_mp3 = 106;
    Zambretti_LED = 5;
    break;
  case 'H':
    zambrettis_words = TEXT_ZAMBRETTI_H;
    Zambretti_mp3 = 107;
    Zambretti_LED = 5;
    break;
  case 'I':
    zambrettis_words = TEXT_ZAMBRETTI_I;
    Zambretti_mp3 = 108;
    Zambretti_LED = 4;
    break;
  case 'J':
    zambrettis_words = TEXT_ZAMBRETTI_J;
    Zambretti_mp3 = 109;
    Zambretti_LED = 4;
    break;
  case 'K':
    zambrettis_words = TEXT_ZAMBRETTI_K;
    Zambretti_mp3 = 110;
    Zambretti_LED = 4;
    break;
  case 'L':
    zambrettis_words = TEXT_ZAMBRETTI_L;
    Zambretti_mp3 = 111;
    Zambretti_LED = 4;
    break;
  case 'M':
    zambrettis_words = TEXT_ZAMBRETTI_M;
    Zambretti_mp3 = 112;
    Zambretti_LED = 4;
    break;
  case 'N':
    zambrettis_words = TEXT_ZAMBRETTI_N;
    Zambretti_mp3 = 113;
    Zambretti_LED = 4;
    break;
  case 'O':
    zambrettis_words = TEXT_ZAMBRETTI_O;
    Zambretti_mp3 = 114;
    Zambretti_LED = 4;
    break;
  case 'P':
    zambrettis_words = TEXT_ZAMBRETTI_P;
    Zambretti_mp3 = 115;
    Zambretti_LED = 3;
    break;
  case 'Q':
    zambrettis_words = TEXT_ZAMBRETTI_Q;
    Zambretti_mp3 = 116;
    Zambretti_LED = 3;
    break;
  case 'R':
    zambrettis_words = TEXT_ZAMBRETTI_R;
    Zambretti_mp3 = 117;
    Zambretti_LED = 3;
    break;
  case 'S':
    zambrettis_words = TEXT_ZAMBRETTI_S;
    Zambretti_mp3 = 118;
    Zambretti_LED = 3;
    break;
  case 'T':
    zambrettis_words = TEXT_ZAMBRETTI_T;
    Zambretti_mp3 = 119;
    Zambretti_LED = 2;
    break;
  case 'U':
    zambrettis_words = TEXT_ZAMBRETTI_U;
    Zambretti_mp3 = 120;
    Zambretti_LED = 2;
    break;
  case 'V':
    zambrettis_words = TEXT_ZAMBRETTI_V;
    Zambretti_mp3 = 121;
    Zambretti_LED = 2;
    break;
  case 'W':
    zambrettis_words = TEXT_ZAMBRETTI_W;
    Zambretti_mp3 = 122;
    Zambretti_LED = 2;
    break;
  case 'X':
    zambrettis_words = TEXT_ZAMBRETTI_X;
    Zambretti_mp3 = 123;
    Zambretti_LED = 1;
    break;
  case 'Y':
    zambrettis_words = TEXT_ZAMBRETTI_Y;
    Zambretti_mp3 = 124;
    Zambretti_LED = 1;
    break;
  case 'Z':
    zambrettis_words = TEXT_ZAMBRETTI_Z;
    Zambretti_mp3 = 125;
    Zambretti_LED = 1;
    break;
  default:
    zambrettis_words = TEXT_ZAMBRETTI_DEFAULT;
    Zambretti_mp3 = 126;
    break;
  }
  return zambrettis_words;
}

void Duck_movement()
{
  int movement = abs(gx) + abs(gy) + abs(gz);
  if (movement >= 50)
  {
    Duck_quack_mp3 = 5; //Quack quack
    Duckaction_millis = millis();
  }
  else
  {
    Duck_quack_mp3 = 0;
  }
}

void Duck_alignment()
{

  yaw = atan2(2.0f * (qq[1] * qq[2] + qq[0] * qq[3]), qq[0] * qq[0] + qq[1] * qq[1] - qq[2] * qq[2] - qq[3] * qq[3]);
  pitch = -asin(2.0f * (qq[1] * qq[3] - qq[0] * qq[2]));
  roll = atan2(2.0f * (qq[0] * qq[1] + qq[2] * qq[3]), qq[0] * qq[0] - qq[1] * qq[1] - qq[2] * qq[2] + qq[3] * qq[3]);
  pitch *= 180.0f / PI;
  yaw *= 180.0f / PI;
  //yaw -= 13.8; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
  yaw -= 22.7; // Declination at Wellington
  roll *= 180.0f / PI;

  oldpitch = pitch;
  oldyaw = yaw;
  oldroll = roll;

  pressure = myBMP280.readPressure() / 100;

  if (pressure < minpressure)
  {
    minpressure = pressure;
  }

  if (pressure > maxpressure)
  {
    maxpressure = pressure;
  }

  if (Display_data == 1 && Display_data_duck == 1)
  {
    // Print gyro values in degree/sec
    Serial.print("X-gyro rate: ");
    Serial.print(gx, 3);
    Serial.print(" degrees/sec ");
    Serial.print("Y-gyro rate: ");
    Serial.print(gy, 3);
    Serial.print(" degrees/sec ");
    Serial.print("Z-gyro rate: ");
    Serial.print(gz, 3);
    Serial.println(" degrees/sec");

    Serial.println();
    Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(yaw, 2);
    Serial.print(", ");
    Serial.print(pitch, 2);
    Serial.print(", ");
    Serial.println(roll, 2);

    Serial.print("Dif Yaw, Pitch, Roll: ");
    Serial.print(yaw - oldyaw, 2);
    Serial.print(", ");
    Serial.print(pitch - oldpitch, 2);
    Serial.print(", ");
    Serial.print(roll - oldroll, 2);

    Serial.print(",  altitude: ");
    Serial.print(myBMP280.readAltitude(1017.23), 1);
    Serial.println(" meters");

    Serial.print("Min / Max pressure: ");
    Serial.print(minpressure);
    Serial.print(" / ");
    Serial.println(maxpressure);

    Serial.println();
    Serial.println("********************************************************\n");

    //From BMP280 Sensor
    Serial.print("temperature, pressure: ");
    Serial.print(myBMP280.readTemperature(), 1);
    Serial.print(" C,  ");

    Serial.print(pressure, 1);
    Serial.println(" milli bar");

    Serial.print("Current UNIX Timestamp: ");
    Serial.println(current_timestamp);

    Serial.println();
    Serial.println("********************************************************\n");

    Serial.print("Radiation cpm = ");
    Serial.print(cpm);

    Serial.print(",  Outside temp = ");
    Serial.println(temp);

    Serial.println();
    Serial.println("********************************************************\n");

    Display_data_duck = 0;
  }

  sumCount = 0;
  sum = 0;
}

void Timekeeping()
{
  //time keeping adjustments for loop
  Now = micros();
  deltat = ((Now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = Now;
  sum += deltat; // sum for averaging filter update rate
  sumCount++;
}

void MPU9250_Dataupdate()
{
  // If intPin goes high, all data registers have new data
  if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {                            // On interrupt, check if data ready interrupt
    readAccelData(accelCount); // Read the x/y/z adc values
    getAres();

    // Now we'll calculate the accleration value into actual g's
    ax = (float)accelCount[0] * aRes; // - accelBias[0];  // get actual g value, this depends on scale being set
    ay = (float)accelCount[1] * aRes; // - accelBias[1];
    az = (float)accelCount[2] * aRes; // - accelBias[2];

    readGyroData(gyroCount); // Read the x/y/z adc values
    getGres();

    // Calculate the gyro value into actual degrees per second
    gx = (float)gyroCount[0] * gRes; // get actual gyro value, this depends on scale being set
    gy = (float)gyroCount[1] * gRes;
    gz = (float)gyroCount[2] * gRes;

    readMagData(magCount); // Read the x/y/z adc values
    getMres();
    magbias[0] = +470.; // User environmental x-axis correction in milliGauss, should be automatically calculated
    magbias[1] = +120.; // User environmental x-axis correction in milliGauss
    magbias[2] = +125.; // User environmental x-axis correction in milliGauss

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental corrections
    mx = (float)magCount[0] * mRes * magCalibration[0] - magbias[0]; // get actual magnetometer value, this depends on scale being set
    my = (float)magCount[1] * mRes * magCalibration[1] - magbias[1];
    mz = (float)magCount[2] * mRes * magCalibration[2] - magbias[2];
  }
}

//===================================================================================================================
//====== Set of useful function to access acceleration. gyroscope, magnetometer, and temperature data
//===================================================================================================================

void getMres()
{
  switch (Mscale)
  {
    // Possible magnetometer scales (and their register bit settings) are:
    // 14 bit resolution (0) and 16 bit resolution (1)
  case MFS_14BITS:
    mRes = 10. * 4912. / 8190.; // Proper scale to return milliGauss
    break;
  case MFS_16BITS:
    mRes = 10. * 4912. / 32760.0; // Proper scale to return milliGauss
    break;
  }
}

void getGres()
{
  switch (Gscale)
  {
    // Possible gyro scales (and their register bit settings) are:
    // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
  case GFS_250DPS:
    gRes = 250.0 / 32768.0;
    break;
  case GFS_500DPS:
    gRes = 500.0 / 32768.0;
    break;
  case GFS_1000DPS:
    gRes = 1000.0 / 32768.0;
    break;
  case GFS_2000DPS:
    gRes = 2000.0 / 32768.0;
    break;
  }
}

void getAres()
{
  switch (Ascale)
  {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
  case AFS_2G:
    aRes = 2.0 / 32768.0;
    break;
  case AFS_4G:
    aRes = 4.0 / 32768.0;
    break;
  case AFS_8G:
    aRes = 8.0 / 32768.0;
    break;
  case AFS_16G:
    aRes = 16.0 / 32768.0;
    break;
  }
}

void readAccelData(int16_t *destination)
{
  uint8_t rawData[6];                                       // x/y/z accel register data stored here
  readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]); // Read the six raw data registers into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1]; // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3];
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5];
}

void readGyroData(int16_t *destination)
{
  uint8_t rawData[6];                                       // x/y/z gyro register data stored here
  readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1]; // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3];
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5];
}

void readMagData(int16_t *destination)
{
  uint8_t rawData[7]; // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
  if (readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01)
  {                                                           // wait for magnetometer data ready bit to be set
    readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]); // Read the six raw data and ST2 registers sequentially into data array
    uint8_t c = rawData[6];                                   // End data read by reading ST2 register
    if (!(c & 0x08))
    {                                                           // Check if magnetic sensor overflow set, if not then report data
      destination[0] = ((int16_t)rawData[1] << 8) | rawData[0]; // Turn the MSB and LSB into a signed 16-bit value
      destination[1] = ((int16_t)rawData[3] << 8) | rawData[2]; // Data stored as little Endian
      destination[2] = ((int16_t)rawData[5] << 8) | rawData[4];
    }
  }
}

int16_t readTempData()
{
  uint8_t rawData[2];                                     // x/y/z gyro register data stored here
  readBytes(MPU9250_ADDRESS, TEMP_OUT_H, 2, &rawData[0]); // Read the two raw data registers sequentially into data array
  return ((int16_t)rawData[0] << 8) | rawData[1];         // Turn the MSB and LSB into a 16-bit value
}

void initAK8963(float *destination)
{
  // First extract the factory calibration for each magnetometer axis
  uint8_t rawData[3];                           // x/y/z gyro calibration data stored here
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
  delay(10);
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
  delay(10);
  readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]); // Read the x-, y-, and z-axis calibration values
  destination[0] = (float)(rawData[0] - 128) / 256. + 1.; // Return x-axis sensitivity adjustment values, etc.
  destination[1] = (float)(rawData[1] - 128) / 256. + 1.;
  destination[2] = (float)(rawData[2] - 128) / 256. + 1.;
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
  delay(10);
  // Configure the magnetometer for continuous read and highest resolution
  // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
  // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
  writeByte(AK8963_ADDRESS, AK8963_CNTL, Mscale << 4 | Mmode); // Set magnetometer data resolution and sample ODR
  delay(10);
}

void initMPU9250()
{
  // wake up device
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
  delay(100);                                   // Wait for all registers to reset

  // get stable time source
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01); // Auto select clock source to be PLL gyroscope reference if ready else
  delay(200);

  // Configure Gyro and Thermometer
  // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
  // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
  // be higher than 1 / 0.0059 = 170 Hz
  // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
  // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
  writeByte(MPU9250_ADDRESS, CONFIG, 0x03);

  // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04); // Use a 200 Hz rate; a rate consistent with the filter update rate
                                                // determined inset in CONFIG above

  // Set gyroscope full scale range
  // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  uint8_t c = readByte(MPU9250_ADDRESS, GYRO_CONFIG); // get current GYRO_CONFIG register value
                                                      // c = c & ~0xE0; // Clear self-test bits [7:5]
  c = c & ~0x02;                                      // Clear Fchoice bits [1:0]
  c = c & ~0x18;                                      // Clear AFS bits [4:3]
  c = c | Gscale << 3;                                // Set full scale range for the gyro
                                                      // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c);         // Write new GYRO_CONFIG value to register

  // Set accelerometer full-scale range configuration
  c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG); // get current ACCEL_CONFIG register value
                                               // c = c & ~0xE0; // Clear self-test bits [7:5]
  c = c & ~0x18;                               // Clear AFS bits [4:3]
  c = c | Ascale << 3;                         // Set full scale range for the accelerometer
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

  // Set accelerometer sample rate configuration
  // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
  // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
  c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
  c = c & ~0x0F;                                // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
  c = c | 0x03;                                 // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value
                                                // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
                                                // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
  // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
  // can join the I2C bus and all can be controlled by the Arduino as master
  writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
  writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01); // Enable data ready (bit 0) interrupt
  delay(100);
}

void calibrateMPU9250(float *dest1, float *dest2)
{
  // Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
  // of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

  // reset device
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  delay(100);

  // get stable time source; Auto select clock source to be PLL gyroscope reference if ready
  // else use the internal oscillator, bits 2:0 = 001
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
  writeByte(MPU9250_ADDRESS, PWR_MGMT_2, 0x00);
  delay(200);

  // Configure device for bias calculation
  writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
  writeByte(MPU9250_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
  delay(15);

  // Configure MPU6050 gyro and accelerometer for bias calculation
  writeByte(MPU9250_ADDRESS, CONFIG, 0x01);       // Set low-pass filter to 188 Hz
  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);   // Set sample rate to 1 kHz
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

  uint16_t gyrosensitivity = 131;    // = 131 LSB/degrees/sec
  uint16_t accelsensitivity = 16384; // = 16384 LSB/g

  // Configure FIFO to capture accelerometer and gyro data for bias calculation
  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x40); // Enable FIFO
  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x78);   // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
  delay(40);                                   // accumulate 40 samples in 40 milliseconds = 480 bytes

  // At end of sample accumulation, turn off FIFO sensor read
  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);            // Disable gyro and accelerometer sensors for FIFO
  readBytes(MPU9250_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count / 12; // How many sets of full gyro and accelerometer data for averaging

  for (ii = 0; ii < packet_count; ii++)
  {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    readBytes(MPU9250_ADDRESS, FIFO_R_W, 12, &data[0]);           // read data for averaging
    accel_temp[0] = (int16_t)(((int16_t)data[0] << 8) | data[1]); // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t)(((int16_t)data[2] << 8) | data[3]);
    accel_temp[2] = (int16_t)(((int16_t)data[4] << 8) | data[5]);
    gyro_temp[0] = (int16_t)(((int16_t)data[6] << 8) | data[7]);
    gyro_temp[1] = (int16_t)(((int16_t)data[8] << 8) | data[9]);
    gyro_temp[2] = (int16_t)(((int16_t)data[10] << 8) | data[11]);

    accel_bias[0] += (int32_t)accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t)accel_temp[1];
    accel_bias[2] += (int32_t)accel_temp[2];
    gyro_bias[0] += (int32_t)gyro_temp[0];
    gyro_bias[1] += (int32_t)gyro_temp[1];
    gyro_bias[2] += (int32_t)gyro_temp[2];
  }
  accel_bias[0] /= (int32_t)packet_count; // Normalize sums to get average count biases
  accel_bias[1] /= (int32_t)packet_count;
  accel_bias[2] /= (int32_t)packet_count;
  gyro_bias[0] /= (int32_t)packet_count;
  gyro_bias[1] /= (int32_t)packet_count;
  gyro_bias[2] /= (int32_t)packet_count;

  if (accel_bias[2] > 0L)
  {
    accel_bias[2] -= (int32_t)accelsensitivity;
  } // Remove gravity from the z-axis accelerometer bias calculation
  else
  {
    accel_bias[2] += (int32_t)accelsensitivity;
  }

  // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0] / 4 >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0] / 4) & 0xFF;      // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1] / 4 >> 8) & 0xFF;
  data[3] = (-gyro_bias[1] / 4) & 0xFF;
  data[4] = (-gyro_bias[2] / 4 >> 8) & 0xFF;
  data[5] = (-gyro_bias[2] / 4) & 0xFF;

  // Push gyro biases to hardware registers
  writeByte(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
  writeByte(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
  writeByte(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
  writeByte(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
  writeByte(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
  writeByte(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);

  // Output scaled gyro biases for //display in the main program
  dest1[0] = (float)gyro_bias[0] / (float)gyrosensitivity;
  dest1[1] = (float)gyro_bias[1] / (float)gyrosensitivity;
  dest1[2] = (float)gyro_bias[2] / (float)gyrosensitivity;

  // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
  // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
  // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
  // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
  // the accelerometer biases calculated above must be divided by 8.

  int32_t accel_bias_reg[3] = {0, 0, 0};                // A place to hold the factory accelerometer trim biases
  readBytes(MPU9250_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
  accel_bias_reg[0] = (int32_t)(((int16_t)data[0] << 8) | data[1]);
  readBytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[1] = (int32_t)(((int16_t)data[0] << 8) | data[1]);
  readBytes(MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[2] = (int32_t)(((int16_t)data[0] << 8) | data[1]);

  uint32_t mask = 1uL;             // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

  for (ii = 0; ii < 3; ii++)
  {
    if ((accel_bias_reg[ii] & mask))
      mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
  }

  // Construct total accelerometer bias, including calculated average accelerometer bias from above
  accel_bias_reg[0] -= (accel_bias[0] / 8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accel_bias_reg[1] -= (accel_bias[1] / 8);
  accel_bias_reg[2] -= (accel_bias[2] / 8);

  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0]) & 0xFF;
  data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1]) & 0xFF;
  data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2]) & 0xFF;
  data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

  // Apparently this is not working for the acceleration biases in the MPU-9250
  // Are we handling the temperature correction bit properly?
  // Push accelerometer biases to hardware registers
  writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
  writeByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
  writeByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
  writeByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
  writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
  writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);

  // Output scaled accelerometer biases for //display in the main program
  dest2[0] = (float)accel_bias[0] / (float)accelsensitivity;
  dest2[1] = (float)accel_bias[1] / (float)accelsensitivity;
  dest2[2] = (float)accel_bias[2] / (float)accelsensitivity;
}

// Accelerometer and gyroscope self test; check calibration wrt factory settings
void MPU9250SelfTest(float *destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
  uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
  uint8_t selfTest[6];
  int32_t gAvg[3] = {0}, aAvg[3] = {0}, aSTAvg[3] = {0}, gSTAvg[3] = {0};
  float factoryTrim[6];
  uint8_t FS = 0;

  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);      // Set gyro sample rate to 1 kHz
  writeByte(MPU9250_ADDRESS, CONFIG, 0x02);          // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, FS << 3);  // Set full scale range for the gyro to 250 dps
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x02);   // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, FS << 3); // Set full scale range for the accelerometer to 2 g

  for (int ii = 0; ii < 200; ii++)
  { // get average current values of gyro and acclerometer

    readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);      // Read the six raw data registers into data array
    aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
    aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
    aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);

    readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);       // Read the six raw data registers sequentially into data array
    gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
    gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
    gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);
  }

  for (int ii = 0; ii < 3; ii++)
  { // Get average of 200 values and store as average current readings
    aAvg[ii] /= 200;
    gAvg[ii] /= 200;
  }

  // Configure the accelerometer for self-test
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0xE0);  // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
  delay(25);                                      // Delay a while to let the device stabilize

  for (int ii = 0; ii < 200; ii++)
  { // get average self-test values of gyro and acclerometer

    readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);        // Read the six raw data registers into data array
    aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
    aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
    aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);

    readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);         // Read the six raw data registers sequentially into data array
    gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
    gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
    gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);
  }

  for (int ii = 0; ii < 3; ii++)
  { // Get average of 200 values and store as average self-test readings
    aSTAvg[ii] /= 200;
    gSTAvg[ii] /= 200;
  }

  // Configure the gyro and accelerometer for normal operation
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00);
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);
  delay(25); // Delay a while to let the device stabilize

  // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
  selfTest[0] = readByte(MPU9250_ADDRESS, SELF_TEST_X_ACCEL); // X-axis accel self-test results
  selfTest[1] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_ACCEL); // Y-axis accel self-test results
  selfTest[2] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
  selfTest[3] = readByte(MPU9250_ADDRESS, SELF_TEST_X_GYRO);  // X-axis gyro self-test results
  selfTest[4] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_GYRO);  // Y-axis gyro self-test results
  selfTest[5] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_GYRO);  // Z-axis gyro self-test results

  // Retrieve factory self-test value from self-test code reads
  factoryTrim[0] = (float)(2620 / 1 << FS) * (pow(1.01, ((float)selfTest[0] - 1.0))); // FT[Xa] factory trim calculation
  factoryTrim[1] = (float)(2620 / 1 << FS) * (pow(1.01, ((float)selfTest[1] - 1.0))); // FT[Ya] factory trim calculation
  factoryTrim[2] = (float)(2620 / 1 << FS) * (pow(1.01, ((float)selfTest[2] - 1.0))); // FT[Za] factory trim calculation
  factoryTrim[3] = (float)(2620 / 1 << FS) * (pow(1.01, ((float)selfTest[3] - 1.0))); // FT[Xg] factory trim calculation
  factoryTrim[4] = (float)(2620 / 1 << FS) * (pow(1.01, ((float)selfTest[4] - 1.0))); // FT[Yg] factory trim calculation
  factoryTrim[5] = (float)(2620 / 1 << FS) * (pow(1.01, ((float)selfTest[5] - 1.0))); // FT[Zg] factory trim calculation

  // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
  // To get percent, must multiply by 100
  for (int i = 0; i < 3; i++)
  {
    destination[i] = 100.0 * ((float)(aSTAvg[i] - aAvg[i])) / factoryTrim[i] - 100.;         // Report percent differences
    destination[i + 3] = 100.0 * ((float)(gSTAvg[i] - gAvg[i])) / factoryTrim[i + 3] - 100.; // Report percent differences
  }
}

void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  // Wire.h read and write protocols
  Wire.beginTransmission(address); // Initialize the Tx buffer
  Wire.write(subAddress);          // Put slave register address in Tx buffer
  Wire.write(data);                // Put data in Tx buffer
  Wire.endTransmission();          // Send the Tx buffer
}

uint8_t readByte(uint8_t address, uint8_t subAddress)
{
  uint8_t data;                          // `data` will store the register data
  Wire.beginTransmission(address);       // Initialize the Tx buffer
  Wire.write(subAddress);                // Put slave register address in Tx buffer
  Wire.endTransmission(false);           // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom(address, (uint8_t)1); // Read one byte from slave register address
  data = Wire.read();                    // Fill Rx buffer with result
  return data;                           // Return data read from slave register
}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t *dest)
{
  Wire.beginTransmission(address); // Initialize the Tx buffer
  Wire.write(subAddress);          // Put slave register address in Tx buffer
  Wire.endTransmission(false);     // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
  Wire.requestFrom(address, count); // Read bytes from slave register address
  while (Wire.available())
  {
    dest[i++] = Wire.read();
  } // Put read results in the Rx buffer
}

void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
  // Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
  // (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
  // which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
  // device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
  // The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
  // but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!

  float q1 = qq[0], q2 = qq[1], q3 = qq[2], q4 = qq[3]; // short name local variable for readability
  float norm;
  float hx, hy, _2bx, _2bz;
  float s1, s2, s3, s4;
  float qDot1, qDot2, qDot3, qDot4;

  // Auxiliary variables to avoid repeated arithmetic
  float _2q1mx;
  float _2q1my;
  float _2q1mz;
  float _2q2mx;
  float _4bx;
  float _4bz;
  float _2q1 = 2.0f * q1;
  float _2q2 = 2.0f * q2;
  float _2q3 = 2.0f * q3;
  float _2q4 = 2.0f * q4;
  float _2q1q3 = 2.0f * q1 * q3;
  float _2q3q4 = 2.0f * q3 * q4;
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  // Normalise accelerometer measurement
  norm = sqrtf(ax * ax + ay * ay + az * az);
  if (norm == 0.0f)
    return; // handle NaN
  norm = 1.0f / norm;
  ax *= norm;
  ay *= norm;
  az *= norm;

  // Normalise magnetometer measurement
  norm = sqrtf(mx * mx + my * my + mz * mz);
  if (norm == 0.0f)
    return; // handle NaN
  norm = 1.0f / norm;
  mx *= norm;
  my *= norm;
  mz *= norm;

  // Reference direction of Earth's magnetic field
  _2q1mx = 2.0f * q1 * mx;
  _2q1my = 2.0f * q1 * my;
  _2q1mz = 2.0f * q1 * mz;
  _2q2mx = 2.0f * q2 * mx;
  hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
  hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
  _2bx = sqrtf(hx * hx + hy * hy);
  _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
  _4bx = 2.0f * _2bx;
  _4bz = 2.0f * _2bz;

  // Gradient decent algorithm corrective step
  s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  norm = sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4); // normalise step magnitude
  norm = 1.0f / norm;
  s1 *= norm;
  s2 *= norm;
  s3 *= norm;
  s4 *= norm;

  // Compute rate of change of quaternion
  qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
  qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
  qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
  qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

  // Integrate to yield quaternion
  q1 += qDot1 * deltat;
  q2 += qDot2 * deltat;
  q3 += qDot3 * deltat;
  q4 += qDot4 * deltat;
  norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4); // normalise quaternion
  norm = 1.0f / norm;
  qq[0] = q1 * norm;
  qq[1] = q2 * norm;
  qq[2] = q3 * norm;
  qq[3] = q4 * norm;
}

// Similar to Madgwick scheme but uses proportional and integral filtering on the error between estimated reference vectors and
// measured ones.
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
  float q1 = qq[0], q2 = qq[1], q3 = qq[2], q4 = qq[3]; // short name local variable for readability
  float norm;
  float hx, hy, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez;
  float pa, pb, pc;

  // Auxiliary variables to avoid repeated arithmetic
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  // Normalise accelerometer measurement
  norm = sqrtf(ax * ax + ay * ay + az * az);
  if (norm == 0.0f)
    return;           // handle NaN
  norm = 1.0f / norm; // use reciprocal for division
  ax *= norm;
  ay *= norm;
  az *= norm;

  // Normalise magnetometer measurement
  norm = sqrtf(mx * mx + my * my + mz * mz);
  if (norm == 0.0f)
    return;           // handle NaN
  norm = 1.0f / norm; // use reciprocal for division
  mx *= norm;
  my *= norm;
  mz *= norm;

  // Reference direction of Earth's magnetic field
  hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
  hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
  bx = sqrtf((hx * hx) + (hy * hy));
  bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

  // Estimated direction of gravity and magnetic field
  vx = 2.0f * (q2q4 - q1q3);
  vy = 2.0f * (q1q2 + q3q4);
  vz = q1q1 - q2q2 - q3q3 + q4q4;
  wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
  wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
  wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

  // Error is cross product between estimated direction and measured direction of gravity
  ex = (ay * vz - az * vy) + (my * wz - mz * wy);
  ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
  ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
  if (Ki > 0.0f)
  {
    eInt[0] += ex; // accumulate integral error
    eInt[1] += ey;
    eInt[2] += ez;
  }
  else
  {
    eInt[0] = 0.0f; // prevent integral wind up
    eInt[1] = 0.0f;
    eInt[2] = 0.0f;
  }

  // Apply feedback terms
  gx = gx + Kp * ex + Ki * eInt[0];
  gy = gy + Kp * ey + Ki * eInt[1];
  gz = gz + Kp * ez + Ki * eInt[2];

  // Integrate rate of change of quaternion
  pa = q2;
  pb = q3;
  pc = q4;
  q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
  q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
  q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
  q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

  // Normalise quaternion
  norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
  norm = 1.0f / norm;
  qq[0] = q1 * norm;
  qq[1] = q2 * norm;
  qq[2] = q3 * norm;
  qq[3] = q4 * norm;
}

void Startsensor()
{

  // Read the WHO_AM_I register, this is a good test of communication
  byte c = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250); // Read WHO_AM_I register for MPU-9250
  Serial.print("MPU9250 ");
  Serial.print("I AM ");
  Serial.print(c, HEX);
  Serial.print(" I should be ");
  Serial.println(0x71, HEX);

  delay(1000);

  if (c == 0x71) // WHO_AM_I should always be 0x68
  {
    Serial.println("MPU9250 is online...");
    Serial.println();

    MPU9250SelfTest(SelfTest); // Start by performing self test and reporting values
    Serial.print("x-axis self test: acceleration trim within : ");
    Serial.print(SelfTest[0], 1);
    Serial.println("% of factory value");
    Serial.print("y-axis self test: acceleration trim within : ");
    Serial.print(SelfTest[1], 1);
    Serial.println("% of factory value");
    Serial.print("z-axis self test: acceleration trim within : ");
    Serial.print(SelfTest[2], 1);
    Serial.println("% of factory value");
    Serial.print("x-axis self test: gyration trim within : ");
    Serial.print(SelfTest[3], 1);
    Serial.println("% of factory value");
    Serial.print("y-axis self test: gyration trim within : ");
    Serial.print(SelfTest[4], 1);
    Serial.println("% of factory value");
    Serial.print("z-axis self test: gyration trim within : ");
    Serial.print(SelfTest[5], 1);
    Serial.println("% of factory value");

    calibrateMPU9250(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers

    delay(1000);

    initMPU9250();
    Serial.println("MPU9250 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
    Serial.println();
    // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
    byte d = readByte(AK8963_ADDRESS, AK8963_WHO_AM_I); // Read WHO_AM_I register for AK8963
    Serial.print("AK8963 ");
    Serial.print("I AM ");
    Serial.print(d, HEX);
    Serial.print(" I should be ");
    Serial.println(0x48, HEX);

    delay(1000);

    // Get magnetometer calibration from AK8963 ROM
    initAK8963(magCalibration);
    Serial.println("AK8963 initialized for active data mode...."); // Initialize device for active mode read of magnetometer

    if (Display_data)
    {
      //  Serial.println("Calibration values: ");
      Serial.print("X-Axis sensitivity adjustment value ");
      Serial.println(magCalibration[0], 2);
      Serial.print("Y-Axis sensitivity adjustment value ");
      Serial.println(magCalibration[1], 2);
      Serial.print("Z-Axis sensitivity adjustment value ");
      Serial.println(magCalibration[2], 2);
    }

    delay(1000);
  }
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);
    while (1)
      ; // Loop forever if communication doesn't happen
  }
  //Setup BMP280 Sensor
  myBMP280.begin(BMP280_ADDRESS);
}

void WiFi_start()
{
  WiFiManager wifiManager;
  wifiManager.autoConnect("WiFi_RubberDuck");

  recovered_ssid = wifiManager.getSSID();
  recovered_pass = wifiManager.getPassword();
}

void StartOTA()
{
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
    {
      type = "sketch";
    }
    else
    { // U_SPIFFS
      type = "filesystem";
    }

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR)
    {
      Serial.println("Auth Failed");
    }
    else if (error == OTA_BEGIN_ERROR)
    {
      Serial.println("Begin Failed");
    }
    else if (error == OTA_CONNECT_ERROR)
    {
      Serial.println("Connect Failed");
    }
    else if (error == OTA_RECEIVE_ERROR)
    {
      Serial.println("Receive Failed");
    }
    else if (error == OTA_END_ERROR)
    {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();

  Serial.println();
  Serial.println("OTA Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("");
}

void Test_LEDs()
{
  //Test the LEDs in RGB order
  fill_solid(leds, NUM_LEDS_PER_STRIP, CRGB(255, 0, 0));
  FastLED.setBrightness(brightness);
  FastLED.show();
  Serial.println("");
  Serial.println("TEST:  Red");
  delay(1000);

  fill_solid(leds, NUM_LEDS_PER_STRIP, CRGB(0, 255, 0));
  FastLED.setBrightness(brightness);
  FastLED.show();
  Serial.println("TEST:  Green");
  delay(1000);

  fill_solid(leds, NUM_LEDS_PER_STRIP, CRGB(0, 0, 255));
  FastLED.setBrightness(brightness);
  FastLED.show();
  Serial.println("TEST:  Blue");
  Serial.println();
  delay(1000);

  fill_solid(leds, NUM_LEDS_PER_STRIP, CRGB(0, 0, 0));
  FastLED.setBrightness(brightness);
  FastLED.show();
}

void DotheLEDs()
{

  //Clear LEDs
  fill_solid(&(leds[0]), NUM_LEDS_PER_STRIP /*number of leds*/, CRGB(0, 0, 0));

  //Zambretti fill  (Body / head)
  //1=Stormy (X>Z), 2=Rain (T>W), 3=Unsettled (P>S), 4=Showery (I>O), 5=Fine (A>H)

  //Stormy
  if (Zambretti_LED == 1)
  {
    fill_solid(&(leds[0]), beak /*number of leds*/, CRGB(255, 0, 0));
    FastLED.setBrightness(brightness2);
  }

  //Rain
  if (Zambretti_LED == 2)
  {
    fill_solid(&(leds[0]), beak /*number of leds*/, CRGB(255, 0, 0));
    FastLED.setBrightness(brightness1);
  }

  //Unsettled
  if (Zambretti_LED == 3)
  {
    fill_solid(&(leds[0]), beak /*number of leds*/, CRGB(255, 0, 0));
    FastLED.setBrightness(brightness);
  }

  //Showery
  if (Zambretti_LED == 4)
  {
    fill_solid(&(leds[0]), beak /*number of leds*/, CRGB(255, 255, 0));
    FastLED.setBrightness(brightness);
  }

  //Fine
  if (Zambretti_LED == 5)
  {
    fill_solid(&(leds[0]), beak /*number of leds*/, CRGB(255, 255, 255));
    FastLED.setBrightness(brightness);
  }

  //Geiger fill  (beak)
  for (int i = head; i < beak; i++)
  {
    leds[i] = CRGB(255, 0, 0);
    //leds[i] = CRGB(red, green, blue);
  }

  leds[eye] = CRGB(red, green, blue);
  leds[eye + 1] = CRGB(red, green, blue);

  FastLED.show();
}

void Touchsensor_check()
{
  Touch_play = 0;
  touchsensor = csensy.capacitiveSensor(30);

  if (touchsensor > 0)
  {
    touchmillis = millis();
    touchsensor = csensy.capacitiveSensor(30);

    if (touchsensor > touchthreshold)
    {
      //How long has it been held down
      while (csensy.capacitiveSensor(30) > touchthreshold)
      {
        if (millis() - touchmillis > touchmax)
        { //More than 10s then escape the while
          break;
        }
        yield(); //Tight loop, yield to avoid WTC crashes
      }

      int press_period = millis() - touchmillis;

      //Between min time and the spoken time min
      if (press_period >= touch1)
      {

        Serial.println();
        Serial.println("********************************************************\n");

        Serial.println("* Touch *");

        Serial.println();
        Serial.println("********************************************************\n");

        Touch_play = Zambretti_trend_mp3;
        Touch_play1 = Zambretti_mp3;
      }
    }
  }
}

void Check_Geiger()
{
  // if (currentMillis - previousMillis > interval)
  //   // save the last time you blinked the LED
  //   previousMillis = currentMillis;

  API_Request(); // Get data from Geiger counter

  cpm = JSON_Extract("cpm").toInt();

  if (cpm < cpmmin && cpm > 0)
  {
    cpmmin = cpm;
  }

  if (cpm > cpmmax)
  {
    cpmmax = cpm;
  }

  temp = JSON_Extract("temperature").toInt();

  if (temp < tempmin && temp > 0)
  {
    tempmin = temp;
  }

  if (temp > tempmax)
  {
    tempmax = temp;
  }

  //Display
  LEDrange();  // Calculate LED colour range based on CPM
  DotheLEDs(); //Update LEDs

  // Serial.print("cpm = ");
  // Serial.println(cpm);

  // Serial.print("temp = ");
  // Serial.println(temp);

  // Serial.print("cpm min / max = ");
  // Serial.print(cpmmin);
  // Serial.print("  /  ");
  // Serial.print(cpmmax);
  // Serial.println();
  // Serial.print("temp min / max = ");
  // Serial.print(tempmin);
  // Serial.print("  /  ");
  // Serial.println(tempmax);
  // Serial.print("maxrange = ");
  // Serial.println(maxrange);
  // Serial.println();
  // Serial.print("fails = ");
  // Serial.println(fails);
  // Serial.println();
  // Serial.print("red = ");
  // Serial.println(red);
  // Serial.print("blue = ");
  // Serial.println(blue);
  // Serial.print("green = ");
  // Serial.println(green);
  // Serial.println();
  // Serial.println("****************");
  // Serial.println();

  //displayLEDs();
}

//Get data from Geiger counter
void API_Request()
{
  if ((WiFi.status() == WL_CONNECTED))
  {
    yield();
    HTTPClient http;
    char buff[400];

    //Serial.print("Getting Geiger data...");
    //Serial.println();

    http.begin(geiger);

    //Serial.print("[HTTP] GET...\n");
    // start connection and send HTTP header
    int httpCode = http.GET();

    // httpCode will be negative on error
    if (httpCode > 0)
    {
      // HTTP header has been send and Server response header has been handled
      //Serial.printf("[HTTP] GET... code: %d\n", httpCode);

      // file found at server
      if (httpCode == HTTP_CODE_OK)
      {
        // Serial.print("Start - ");
        // getmillis1 = millis();

        String payload = http.getString();

        // getmillis2 = millis();
        // Serial.print("Stop - ");
        //Serial.println(getmillis2 - getmillis1);

        payload.toCharArray(buff, 400);
        geigerresponse = payload;
      }
    }
    else
    {
      Serial.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
      //geigerresponse = "";
      fails = fails + 1;
    }
    http.end();
  }
}

void LEDrange()
{
  //R 0>0, G 255>255, B0>255
  percentrange = rangehops * 1;     //top of this range
  percentrangeprev = rangehops * 0; //bottom of this range

  if (cpm <= percentrange)
  {
    red = 255;
    green = 255;
    blue = 0;

    percenttemp = ((cpm - percentrangeprev) * 100) / (percentrange - percentrangeprev); //% of full range / this subrange = % within this range to apply to chaning LED
    red = (255 - (percenttemp * 255 / 100));                                            //100 - percenttemp = declining

    // Serial.print("1 percenttemp = ");
    // Serial.println(percenttemp);
  }

  //R 0>0, G 255>255, B0>255
  percentrange = rangehops * 2;           //top of this range
  percentrangeprev = (rangehops * 1) - 1; //bottom of this range

  if (cpm <= percentrange && cpm > percentrangeprev)
  {
    red = 0;
    green = 255;
    blue = 255;
    percenttemp = ((cpm - percentrangeprev) * 100) / (percentrange - percentrangeprev); //% of full range / this subrange = % within this range to apply to chaning LED
    blue = percenttemp * 255 / 100;
    // Serial.print("2 percenttemp = ");
    // Serial.println(percenttemp);
  }

  //R 0>0, G 255>0, B255>255
  percentrange = rangehops * 3;           //top of this range
  percentrangeprev = (rangehops * 2) - 1; //bottom of this range

  if (cpm <= percentrange && cpm > percentrangeprev)
  {
    red = 0;
    green = 255;
    blue = 255;

    percenttemp = ((cpm - percentrangeprev) * 100) / (percentrange - percentrangeprev); //% of full range / this subrange = % within this range to apply to chaning LED
    green = (255 - (percenttemp * 255 / 100));                                          //100 - percenttemp = declining
    // Serial.print("3 percenttemp = ");
    // Serial.println(percenttemp);
  }

  //R 0>255, G 0>0, B255>255
  percentrange = rangehops * 4;           //top of this range
  percentrangeprev = (rangehops * 3) - 1; //bottom of this range

  if (cpm <= percentrange && cpm > percentrangeprev)
  {
    red = 255;
    green = 0;
    blue = 255;

    percenttemp = ((cpm - percentrangeprev) * 100) / (percentrange - percentrangeprev); //% of full range / this subrange = % within this range to apply to chaning LED
    red = percenttemp * 255 / 100;
    // Serial.print("4 percenttemp = ");
    // Serial.println(percenttemp);
  }

  //R 255>255, G 0>0, B255>0
  percentrange = rangehops * 5;           //top of this range
  percentrangeprev = (rangehops * 4) - 1; //bottom of this range

  if (cpm > percentrangeprev)
  {
    red = 255;
    green = 0;
    blue = 255;

    percenttemp = ((cpm - percentrangeprev) * 100) / (percentrange - percentrangeprev); //% of full range / this subrange = % within this range to apply to chaning LED
    blue = (255 - (percenttemp * 255 / 100));                                           //100 - percenttemp = declining
    if (blue < 0)
    {
      blue = 0;
    }

    // Serial.print("5 percenttemp = ");
    // Serial.println(percenttemp);
  }

  if (cpm > rangehops * 5)
  {
    red = 255;
    green = 0;
    blue = 0;

    FastLED.setBrightness(brightness);
  }
  else
  {
    FastLED.setBrightness(brightness);
  }
}

//JSON Function
String JSON_Extract(String lookfor)
{
  DynamicJsonBuffer jsonBuffer;
  JsonObject &root = jsonBuffer.parseObject(geigerresponse);
  JsonObject &data = root["data"];
  return data[lookfor];
}

void measurementEvent()
{

  //Measures absolute Pressure, Temperature, Humidity, Voltage, calculate relative pressure,
  //Dewpoint, Dewpoint Spread, Heat Index

  //myBMP280.takeForcedMeasurement();

  // Get temperature
  measured_temp = myBMP280.readTemperature();
  measured_temp = measured_temp + TEMP_CORR;
  // print on serial monitor
  Serial.print("Temp: ");
  Serial.print(measured_temp);
  Serial.print("°C; ");

  // // Get humidity
  // measured_humi = myBMP280.readHumidity();
  // // print on serial monitor
  // Serial.print("Humidity: ");
  // Serial.print(measured_humi);
  // Serial.print("%; ");

  // Get pressure
  measured_pres = myBMP280.readPressure() / 100.0F;
  // print on serial monitor
  Serial.print("Pressure: ");
  Serial.print(measured_pres);
  Serial.print("hPa; ");

  // Calculate and print relative pressure
  SLpressure_hPa = (((measured_pres * 100.0) / pow((1 - ((float)(ELEVATION)) / 44330), 5.255)) / 100.0);
  rel_pressure_rounded = (int)(SLpressure_hPa + .5);
  // print on serial monitor
  Serial.print("Pressure rel: ");
  Serial.print(rel_pressure_rounded);
  Serial.print("hPa; ");

  // Calculate dewpoint
  double a = 17.271;
  double b = 237.7;
  double tempcalc = (a * measured_temp) / (b + measured_temp) + log(measured_humi * 0.01);
  DewpointTemperature = (b * tempcalc) / (a - tempcalc);
  Serial.print("Dewpoint: ");
  Serial.print(DewpointTemperature);
  Serial.println("°C; ");

  // Calculate dewpoint spread (difference between actual temp and dewpoint -> the smaller the number: rain or fog

  DewPointSpread = measured_temp - DewpointTemperature;
  Serial.print("Dewpoint Spread: ");
  Serial.print(DewPointSpread);
  Serial.println("°C; ");

  // Calculate HI (heatindex in °C) --> HI starts working above 26,7 °C
  if (measured_temp > 26.7)
  {
    double c1 = -8.784, c2 = 1.611, c3 = 2.338, c4 = -0.146, c5 = -1.230e-2, c6 = -1.642e-2, c7 = 2.211e-3, c8 = 7.254e-4, c9 = -2.582e-6;
    double T = measured_temp;
    double R = measured_humi;

    double A = ((c5 * T) + c2) * T + c1;
    double B = ((c7 * T) + c4) * T + c3;
    double C = ((c9 * T) + c8) * T + c6;
    HeatIndex = (C * R + B) * R + A;
  }
  else
  {
    HeatIndex = measured_temp;
    Serial.println("Not warm enough (less than 26.7 °C) for Heatindex");
  }
  Serial.print("HeatIndex: ");
  Serial.print(HeatIndex);
  Serial.print("°C; ");

} // end of void measurementEvent()

void UpdateSPIFFS()
{

  Serial.print("Timestamp difference: ");
  Serial.println(current_timestamp - saved_timestamp);

  if (current_timestamp - saved_timestamp > 21600)
  { // last save older than 6 hours -> re-initialize values
    FirstTimeRun();
  }
  else if (current_timestamp - saved_timestamp > 1800)
  { // it is time for pressure update (1800 sec = 30 min)

    for (int i = 11; i >= 1; i = i - 1)
    {
      pressure_value[i] = pressure_value[i - 1]; // shifting values one to the right
    }

    pressure_value[0] = rel_pressure_rounded; // updating with acutal rel pressure (newest value)

    if (accuracy < 12)
    {
      accuracy = accuracy + 1; // one value more -> accuracy rises (up to 12 = 100%)
    }
    WriteToSPIFFS(current_timestamp); // update timestamp on storage
    Serial.println("writing current_timestamp");
  }
  else
  {
    WriteToSPIFFS(saved_timestamp); // do not update timestamp on storage
    Serial.println("writing saved_timestamp");
  }
}

void FirstTimeRun()
{
  Serial.println("---> Starting initializing process.");
  accuracy = 1;
  char filename[] = "/data.txt";
  File myDataFile = SPIFFS.open(filename, "w"); // Open a file for writing
  if (!myDataFile)
  {
    Serial.println("Failed to open file");
    Serial.println("Stopping process - maybe flash size not set (SPIFFS).");
    exit(0);
  }
  myDataFile.println(current_timestamp); // Saving timestamp to /data.txt
  Serial.print("*!* current_timestamp = ");
  Serial.println(current_timestamp);

  myDataFile.println(accuracy); // Saving accuracy value to /data.txt
  for (int i = 0; i < 12; i++)
  {
    myDataFile.println(rel_pressure_rounded); // Filling pressure array with current pressure
  }
  Serial.println("** Saved initial pressure data. **");
  myDataFile.close();
}

void ReadFromSPIFFS()
{
  char filename[] = "/data.txt";
  File myDataFile = SPIFFS.open(filename, "r"); // Open file for reading
  if (!myDataFile)
  {
    Serial.println("Failed to open file");
    FirstTimeRun(); // no file there -> initializing
  }

  Serial.println("---> Now reading from SPIFFS");

  String temp_data;

  temp_data = myDataFile.readStringUntil('\n');
  saved_timestamp = temp_data.toInt();
  Serial.print("Timestamp from SPIFFS: ");
  Serial.println(saved_timestamp);

  temp_data = myDataFile.readStringUntil('\n');
  accuracy = temp_data.toInt();
  Serial.print("Accuracy value read from SPIFFS: ");
  Serial.println(accuracy);

  Serial.print("Last 12 saved pressure values: ");
  for (int i = 0; i <= 11; i++)
  {
    temp_data = myDataFile.readStringUntil('\n');
    pressure_value[i] = temp_data.toInt();
    Serial.print(pressure_value[i]);
    Serial.print("; ");
  }
  myDataFile.close();
  Serial.println();
}

void WriteToSPIFFS(int write_timestamp)
{
  char filename[] = "/data.txt";
  File myDataFile = SPIFFS.open(filename, "w"); // Open file for writing (appending)
  if (!myDataFile)
  {
    Serial.println("Failed to open file");
  }

  Serial.println("---> Now writing to SPIFFS");

  myDataFile.println(write_timestamp); // Saving timestamp to /data.txt
  myDataFile.println(accuracy);        // Saving accuracy value to /data.txt

  for (int i = 0; i <= 11; i++)
  {
    myDataFile.println(pressure_value[i]); // Filling pressure array with updated values
  }
  myDataFile.close();

  Serial.println("File written. Now reading file again.");
  myDataFile = SPIFFS.open(filename, "r"); // Open file for reading
  Serial.print("Found in /data.txt = ");
  while (myDataFile.available())
  {
    Serial.print(myDataFile.readStringUntil('\n'));
    Serial.print("; ");
  }
  Serial.println();
  myDataFile.close();
}

void do_blynk()
{
  //**************************Sending Data to Blynk and ThingSpeak*********************************
  // code block for uploading data to BLYNK App

  // if (App1 == "BLYNK") {
  //  Blynk.virtualWrite(0, measured_temp);            // virtual pin 0
  //  Blynk.virtualWrite(1, measured_humi);            // virtual pin 1
  Blynk.virtualWrite(2, measured_pres);        // virtual pin 2
  Blynk.virtualWrite(3, rel_pressure_rounded); // virtual pin 3
  Blynk.virtualWrite(4, temp);                 // virtual pin 4  - outside temp
                                               //   Blynk.virtualWrite(5, DewpointTemperature);      // virtual pin 5
                                               // Blynk.virtualWrite(6, HeatIndex);                // virtual pin 6
  Blynk.virtualWrite(7, ZambrettisWords);      // virtual pin 7
  Blynk.virtualWrite(10, cpm);                 // virtual pin 10
  Blynk.virtualWrite(8, accuracy_in_percent);  // virtual pin 8

  if (accuracy < accuracygate)
  {
    Blynk.virtualWrite(9, "No trend"); // virtual pin 9
  }
  else
  {
    Blynk.virtualWrite(9, trend_in_words); // virtual pin 9
  }
}

void SPIFFS_init()
{
  //*****************Checking if SPIFFS available********************************
  Serial.println("SPIFFS Initialization: (First time run can last up to 30 sec - be patient)");

  boolean mounted = SPIFFS.begin(); // load config if it exists. Otherwise use defaults.
  if (!mounted)
  {
    Serial.println("FS not formatted. Doing that now... (can last up to 30 sec).");
    SPIFFS.format();
    Serial.println("FS formatted...");
    Serial.println("");
    SPIFFS.begin();
  }
}

void update_epoch_time()
{
  //update current time with millis
  current_timestamp = millis_unix_timestamp_baseline + ((millis() - millis_unix_timestamp) / 1000);

  epoch = epochstart + (((millis() - startmillis) / 1000) * timefactor); //Get epoch from millis count.  May get over writtem by NTP pull.  timefactor is for testing to accellerate time for testing
  printNTP = 0;                                                          //Flag to state time was not from an NTP request.

  //Update the time.  NTP pull is only done periodically based on NTP_Seconds_to_wait, we count millis (pretty accurate) when not getting NTP time
  Seconds_SinceLast_NTP_millis = (millis() - Last_NTP_millis) / 1000; //How many seconds since Last_NTP_millis pull

  if (Seconds_SinceLast_NTP_millis > NTP_Seconds_to_wait) //Don't go to NTP during flash phase as it causes flicker
  {
    if (Display_data == 1)
    {
      Serial.print("millis = ");
      Serial.println(millis());
      Serial.print("start millis = ");
      Serial.println(startmillis);
      Serial.print("epochstart = ");
      Serial.println(epochstart);
      Serial.print("epoch = ");
      Serial.println(epoch);
      Serial.println("");
    }

    Request_Time(); //Get the timedata
    printNTP = 1;   //1 is a flag to serialprint the time (only used for NTP pull not for millis updates)
    delay(2000);
    NTPdelaycounter++; //Count how many delay functions after request time

    while (!Check_Time()) //Converts to Epoch, returns a False if not data Rxd
    {                     //If no time recieved then do this
      delay(2000);
      Serial.println("No packets, NTP Wait...");
      NTPdelaycounter++; //+1 for another delay
      TimeCheckLoop++;

      //If after 5 tries, Give up and exit the NTP function, reset the loop counter.  epoch already updated from Millis()
      if (TimeCheckLoop >= 5)
      {
        TimeCheckLoop = 0;
        NTPdelaycounter = 0; //Reset counter on exit
        break;
      }
      else if (TimeCheckLoop > 2)
      {
        //If after 2 tried then try re-requesting the time
        retryNTP += 1; //Update the counter for informational only, not used in the program
        Request_Time();
      }
    }

    NTPdelaycounter = 0; //Time recieved, reset counter

    //Time confirmed received and more than wait period to pull NTP / Sunrise time
    Last_NTP_millis = millis(); //Set the Last_NTP_millis time to now - resets the wait time

    if (Display_data == 1)
    {
      Serial.println();
      Serial.println("********************************************************");
      Serial.println();
    }

    yield();
    NTP_Seconds_to_wait = NTPSecondstowait; //Over write the initial wait period (1 sec) to the ongoing period (e.g 600 sec)
  }
  current_timestamp = epoch;
}

void initiate_time()
{
  //Initiate time
  udp.begin(localPort);
  Serial.print("Local port: ");
  Serial.println(udp.localPort());

  NTPdelaycounter = 0; //Reset counter on exit
  Request_Time();      //Get the time
  delay(2000);
  NTPdelaycounter++; //Add 1 to delay counter

  while (!Check_Time())
  { //If no time recieved then do this
    delay(2000);
    TimeCheckLoop++;
    NTPdelaycounter++;

    if (TimeCheckLoop > 5)
    {                      //If not time received even after 5x 2sec delays, then try re-getting time
      Request_Time();      //Get the time
      NTPdelaycounter = 0; //Reset delay counter on new request
    }
  }

  NTPdelaycounter = 0;    //Reset delay counter on exit
  epochstart = epoch;     //epoch pulled from NTP server, use initial epoch to set starting point for epochmillis
  startmillis = millis(); //get starting point for millis
  current_timestamp = epoch;
}

//Update the time
void decode_epoch(unsigned long currentTime)
{
  // print the raw epoch time from NTP server
  if (printNTP == 1 && Display_data == 1)
  {
    Serial.print("The epoch UTC time is ");
    Serial.print(epoch);
    Serial.println();

    // print the hour, minute and second:
    Serial.print("The UTC time is ");      // UTC is the time at Greenwich Meridian (GMT)
    Serial.print((epoch % 86400L) / 3600); // print the hour (86400 equals secs per day)
    Serial.print(':');

    if (((epoch % 3600) / 60) < 10)
    {
      // In the first 10 minutes of each hour, we'll want a leading '0'
      Serial.print('0');
    }

    Serial.print((epoch % 3600) / 60); // print the minute (3600 equals secs per minute)
    Serial.print(':');

    if ((epoch % 60) < 10)
    {
      // In the first 10 seconds of each minute, we'll want a leading '0'
      Serial.print('0');
    }

    Serial.println(epoch % 60); // print the second
  }
  hour_UTC = (currentTime % 86400L) / 3600;

  minute_UTC = (currentTime % 3600) / 60;
  second_UTC = currentTime % 60;

  clock_AMPM = "AM"; //Default to AM

  //If it's 12 or greater (e.g 12 > 23) go to PM
  if (hour_UTC >= 12)
  {
    clock_AMPM = "PM";
  }

  //If it's greater than 12 (e.g 13 > 23) deduct 12 to make 1 > 11
  if (hour_UTC > 12)
  {
    hour_UTC = hour_UTC - 12;
  }

  if (printNTP == 1 && Display_data == 1)
  {
    Serial.print("UTC Hour: ");
    Serial.print(hour_UTC);
    Serial.print(",   Minute: ");
    Serial.print(minute_UTC);
    Serial.print(",   Second: ");
    Serial.print(second_UTC);
    Serial.print(",   clock_AMPM: ");
    Serial.println(clock_AMPM);
    Serial.println();
  }

  //Work out Hours/min into minutes from midnight to Calculate if it's AM or PM time
  working_hourtomin = hour_UTC;

  //PM add 12.  e,g 1PM = 13:00
  if (clock_AMPM == "PM")
  {
    working_hourtomin = hour_UTC + 12;
  }

  //Midnight = 0
  if (clock_AMPM == "AM" && hour_UTC == 12)
  {
    working_hourtomin = 0;
  }

  //Noon = 12
  if (clock_AMPM == "PM" && hour_UTC == 12)
  {
    working_hourtomin = 12;
  }

  clock_minutes_from_midnight = ((working_hourtomin * 60) + minute_UTC);

  //Get local minutes for day/night calc
  local_clock_minutes_from_midnight = clock_minutes_from_midnight + ((localUTC + UTCoffset) * 60);

  //If local_minutes is negative (e.g Negative UTC)
  if (local_clock_minutes_from_midnight > 1440)
  {
    local_clock_minutes_from_midnight = local_clock_minutes_from_midnight - 1440;
  }

  //If local_minutes is negative (e.g Negative UTC)
  if (local_clock_minutes_from_midnight < 0)
  {
    local_clock_minutes_from_midnight = local_clock_minutes_from_midnight + 1440;
  }

  if (printNTP == 1 && Display_data == 1)
  {
    Serial.print("UTC Clock - Mins from midnight = ");
    Serial.print(clock_minutes_from_midnight);
    Serial.print(",   Local - Clock - Mins from midnight = ");
    Serial.println(local_clock_minutes_from_midnight);
    Serial.println();
    Serial.println("********************************************************");
    Serial.println();
  }
}

//Get time from NTP Server
void Request_Time()
{
  Serial.println("Getting Time");
  WiFi.hostByName(NTPServerName, timeServer);
  sendNTPpacket(timeServer); // send an NTP packet to a time server
}

//This returns a bool value based on UDP time being received and placed in epoch variable
bool Check_Time()
{
  bool ret_val = false;

  // //Test only:  Simulate NTP packets not Rxd
  // srand (millis());

  // int blah=rand() % 100;
  // Serial.print("RAND = ");
  // Serial.println(blah);

  // if (blah >= 80){
  //   return ret_val;
  // }

  //Clear the buffer, keep looping and pulling packets until no more NTP packets are available.
  //Put here in scenario that if multiple retires to NTP then multiple packets eventually Rxd, this clears them all and takes the latest
  for (int cb = udp.parsePacket(); cb > 0; cb = udp.parsePacket())
  {

    Serial.print("packet received, length=");
    Serial.println(cb);
    Serial.println();
    Serial.println("****************");
    Serial.println();

    // We've received a packet, read the data from it
    udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

    //the timestamp starts at byte 40 of the received packet and is four bytes,
    // or two words, long. First, extract the two words:

    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    // combine the four bytes (two words) into a long integer
    // this is NTP time (seconds since Jan 1 1900):
    unsigned long secsSince1900 = highWord << 16 | lowWord;
    //Serial.print("Seconds since Jan 1 1900 = " );
    // Serial.println(secsSince1900);

    // now convert NTP time into everyday time:
    //Serial.print("Unix time = ");
    // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    const unsigned long seventyYears = 2208988800UL;
    lastepoch = epoch; //Used to compare last known epoch time with new NTP

    // subtract seventy years:
    epoch = secsSince1900 - seventyYears;

    //Add seconds to factor for delay(2000)
    epoch = epoch + (2 * NTPdelaycounter);

    //Check if there are lost packets (epoch is wildly different from last time).  If yes, use last epoch
    //if first epoch time is wrong this is constantly fail.

    Serial.print("secsSince1900: ");
    Serial.print(secsSince1900);
    Serial.print(",  epoch: ");
    Serial.println(epoch);

    if (abs(epoch - lastepoch) > 3600 && lastepoch != 0) //Check if the old and new epoch times are more than 60s x 60 (1hr) and lastepoch isn't 0 (not had a time before)
    {
      Serial.println("epoch vs oldepoch > 1hr");
      if (lastepochcount <= 3)
      {
        epoch = lastepoch; //If more than 1hr difference, and old/new different less than 'N' times
        lastepochcount = lastepochcount + 1;
        totalfailepoch = totalfailepoch + 1;

        Serial.println("Using oldepoch from millis");
        Serial.print("previously failed = ");
        Serial.println(totalfailepoch);
        Serial.println();
      }
      else
      {
        lastepochcount = 0; //It's different more than 'N' times, inital NTP must have been wrong.  Stay with last recieved epoch.
        lastepoch = epoch;
        Serial.println("Using new epoch even though different.  It's been different too many times - resetting");
        Serial.print("previously failed = ");
        Serial.print(totalfailepoch);
        Serial.println(",  Making internal clock = new NTP time");
        Serial.println();

        epochstart = epoch;     //Using NTP epoch time.  Reset the millis time variables to use this as new starting point
        startmillis = millis(); //Using NTP epoch time.  Reset the millis time variables to use this as new starting point
      }
    }
    else
    {
      Serial.print("epoch is good");
      Serial.print(",   previously failed = ");
      Serial.println(totalfailepoch);
      Serial.println("Making internal clock = new NTP time");
      Serial.println();

      lastepochcount = 0;     //With a good epoch reset the bad epoch counter to zero
      lastepoch = epoch;      //With a good epoch make lastepoch the new good one for next loop
      epochstart = epoch;     //Using NTP epoch time.  Reset the millis time variables to use this as new starting point
      startmillis = millis(); //Using NTP epoch time.  Reset the millis time variables to use this as new starting point
    }

    Serial.print("new NTP epoch = ");
    Serial.print(epoch);
    Serial.print(",   Millis epoch = ");
    Serial.println(lastepoch);

    Last_NTP_millis = millis(); //Set the last millis time the NTP time was attempted
    RequestedTime = 0;
    TimeCheckLoop = 0;

    ret_val = true;
  }

  if (!ret_val)
  {
    Serial.println("no packet yet");
  }

  return ret_val;
}

// send an NTP request to the time server at the given address
void sendNTPpacket(const IPAddress &address)
{
  Serial.println("sending NTP packet");
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011; // LI, Version, Mode
  packetBuffer[1] = 0;          // Stratum, or type of clock
  packetBuffer[2] = 6;          // Polling Interval
  packetBuffer[3] = 0xEC;       // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  udp.beginPacket(address, 123); //NTP requests are to port 123
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket();
}

void LocalClock()
{

  int local_hour = local_clock_minutes_from_midnight / 60; //Turn minutes into the hour, needed for chime check
  String AMPM = "AM";

  if (local_hour > 12)
  {
    AMPM = "PM";
    local_hour -= 12;
  }

  if (Display_data == 1)
  {
    Serial.println();
    Serial.print("Local time: ");
    Serial.print(local_hour);
    Serial.print(":");

    if (minute_UTC < 10)
    {
      Serial.print("0");
    }

    Serial.print(minute_UTC);
    Serial.print(" ");
    Serial.println(AMPM);
    Serial.println();
  }
}