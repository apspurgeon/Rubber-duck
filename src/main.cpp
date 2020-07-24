
// ****************************************************************************************************************************************************************
// ****************************************************************************************************************************************************************
//
//  Touch briefly for time, longer for forecast, longer for weather/nightlight mode for LEDs, longest for UTC change
//  Setup using Wifi manager website.  Enter Long/Latitude, UTC and chime
//  
//  Chime = 25 chars  First char is Volume (0>9), then chime MP3 file name or action for each hour starting from midnight, 1am, 2am etc > 11pm
//  0=No chime, 1-7 are 001.mp3 > 007.mp3 files in SDcard folder 03.  8 & 9 are reserved where 8 will speak forecast and 9 will speak time.
//
// ****************************************************************************************************************************************************************
// ****************************************************************************************************************************************************************


#define xstr(s) str(s)
#define str(s) #s

#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"
#include <ESP8266HTTPClient.h>
#include <FastLED.h>
#include <ArduinoJson.h>
#include <WiFiManager.h>
#include <FS.h>
#include <CapacitiveSensor.h>
#include <ESP8266WiFiMulti.h>
#include <ArduinoOTA.h>
#include "ESP8266FtpServer.h"
#include <BlynkSimpleEsp8266.h>
#include <TimeLib.h> 
#include "BMP280.h"
#include "Wire.h"


//******** <Things to change>  ********
#define hemisphere 2       //1=North, 2=South, 0=Equator
#define P0 1013.25
#define ELEVATION (100)             //Enter your elevation in m ASL to calculate rel pressure (ASL/QNH) at your place

#define NUM_LEDS_PER_STRIP 30 //Number of LEDs per strip
#define PIN_LED D7            //I.O pin on ESP2866 device going to LEDs
#define COLOR_ORDER GRB       // LED stips aren't all in the same RGB order.  If colours are wrong change this  e.g  RBG > GRB.

//******** <Things to change>  ********


#define NTP_SERVER "ch.pool.ntp.org"
#define BLYNK_PRINT Serial

#ifndef BLYNKCERT_NAME
#define BLYNKCERT_NAME "1234567890" //Default BLYNK Cert if not build flag from PlatformIO doesn't work
#endif

#ifndef SSID_NAME
#define SSID_NAME "WIFI_SSID" //Default SSID if not build flag from PlatformIO doesn't work
#endif

#ifndef PASSWORD_NAME
#define PASSWORD_NAME "WIFI_PASSWORD" //Default WiFi Password if not build flag from PlatformIO doesn't work
#endif

const char auth[] = xstr(BLYNKCERT_NAME);       // your BLYNK Cert from build flags


//Brightness of weather and night light set by user.  Except for stormy weather where set to 255
int brightness = 128;           //for nightlight display
int brightness1 = 128;          //for weather display
#define brightness2 255         //for stormy weather display

//Mode  true=Night light, false=Weather
bool working_mode = false;

uint32_t delt_t = 0;                                                            // used to control display output rate
uint32_t count = 0, sumCount = 0, timecount = 0, working_modecount = 0, zambretticount = 0; // used to control display output rate
float deltat = 0.0f, sum = 0.0f;          // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;                         // used to calculate integration interval

//SPIFFS Filenames
String HTTPfilename = "/APIaddress.txt"; //Filename for storing Sunrise API HTTP address in SPIFFS
String UTCfilename = "/UTC.txt";         //Filename for storing Sunrise UTC in SPIFFS
String chimefilename = "/chime.txt";     //Filename for storing Sunrise chime in SPIFFS
String restartfilename = "/restart.txt"; //Filename for triggering restart in SPIFFS
String resetfilename = "/reset.txt";     //Filename for triggering restart in SPIFFS

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
uint32_t touchmax = 3000;
int touch1 = 500;
long press_period;

//DF Player
int mp3vol = 20;                //Volume for DF card player.  Keep at 0, used as a flag to skip functions if not wifimanager credentials (no sound option)
long MP3millis;

//LEDs
struct CRGB leds[NUM_LEDS_PER_STRIP]; //initiate FastLED with number of LEDs
long LEDmillis;

//Max values for LEDs
int green_max = 128; //green (128 for yellow / 255 for orange)
int blue_max = 255;
int red_max = 255;

//What lamp looks like for day time (sunshine)
int green_day = 128; //green (128 for yellow / 255 for orange)
int blue_day = 0;
int red_day = 255;

//What lamp looks like for night
const int green_night = 0;
const int blue_night = 255;
const int red_night = 0;

//Wifi and internet variables
const unsigned int localPort = 2390; // local port to listen for UDP packets
String recovered_ssid;
String recovered_pass;

unsigned long touchUTCmin = 10000;     //this value or more to go into UTC change
unsigned long touchUTCmax = 20000;     //this value or more to go into UTC change
int touchstopmin = 300;   //Min touch to trigger a mp3_stop()
int touchForecastmin = 1000;  //Min touch to trigger spoken clock_minutes
int touchSwapmin = 4000;     //this value or more to go into UTC change

//Millis Unix_timestmap update
unsigned long millis_unix_timestamp;
char UTC[3];
int RequestedTime = 0, TimeCheckLoop = 0, NTPdelaycounter = 0;
int hour_UTC, minute_UTC, second_UTC;   
int hour_actual = 200, dia_actual = 0, anyo = 0;
int timeout = 0, timeout_prev = 0;
int vera = 0, night = 0;                                                                                //1 = Night, 0 = Day
const int NTP_PACKET_SIZE = 48;                                                                         // NTP time stamp is in the first 48 bytes of the message
byte packetBuffer[NTP_PACKET_SIZE];                                                                     //buffer to hold incoming and outgoing packets
unsigned long epoch = 0, lastepoch = 0, Last_NTP_millis = 0, LastAPI, LastLED, epochstart, startmillis; //Unix time in seconds
int lastepochcount = 1, totalfailepoch = 0;
int clock_minutes_from_midnight, local_clock_minutes_from_midnight; //Minutes from midnight
//int hour, minute, second;                                           //UTC time
int NTP_Seconds_to_wait = 1;                                        //Initial wait time between NTP/Sunrise pulls (1 sec)
String clock_AMPM;                                                  //AM/PM from NTP Server
int printNTP = 0;                                                   //Set to 1 when a NTP is pulled.  The decode_epoch function used for both NTP epoch and millis epoch.  printNTP=1 in this fucnction only print new NTP results (time).
int Seconds_SinceLast_NTP_millis;                                   //Counts seconds since last NTP pull
int retryNTP = 0;                                                   //Counts the number of times the NTP Server request has had to retry
int UTC_Cycle = 152;
unsigned long currentMillis = millis();
const char *NTPServerName = "0.nz.pool.ntp.org"; //local NTP server

//Time delays
uint32_t delayamount = 2000;                     //LED update delay
uint32_t zambretti_delayamount = 30 * 60 * 1000; //Update Zambretti
uint32_t showtime_delayamount = 60 * 1000;       //Display local every 60s
uint32_t pressure_read_interval = 5 * 60 * 1000; //5mins x 60 sec x 1000 millis
const int minswithin = 60;      //Minutes within sunrise / sunset to begin the LED colour change sequence  (60 = phase starts 30mins before sunrise/set and end 30mins after)
const int change = 1;           //Speed of LED change in tones.  Recommend = 1
int NTPSecondstowait = 1 * 60 * 60; //Wait between NTP pulls (sec)
int APISecondstowait = 6 * 60 * 60; //Wait between Sunrise API pulls (sec)
int SecondsSinceLastAPI = 0;
int timefactor = 1; //Used for testing to accelerate time
uint32_t getmillis1, getmillis2;

//Sunrise - Sunset API variables
char sunrise_api_request[150]; //It should end up containing an adress like this "http://api.sunrise-sunset.org/json?lat=-41.2865&lng=174.7762";
int h_sunrise, hour_sunrise, minute_sunrise, sunrise_minutes_from_midnight, local_sunrise_minutes_from_midnight;
int SR_Phase = 0; //1 = in Sunrise phase (30 mins either side if minwithin = 60mins)
int h_sunset, hour_sunset, minute_sunset, sunset_minutes_from_midnight, local_sunset_minutes_from_midnight;
int SS_Phase = 0;            //1 = in Sunset phase (30 mins either side if minwithin = 60mins)
int working_hourtomin = 0;   //Used to convert hours into total minutes
float LED_phase;             //0-255 in the phase of sunrise/set   0=begining 255=end
char SR_AMPM[1], SS_AMPM[1]; //Sunrise/set AMPM
String AMPM, sunAPIresponse;
String JSON_Extract(String);
char mode[4];      //Used to get input from webpage
int SRSS_Flip = 0; //Used to manipulate SR and SS varible if in nightlight mode

//Longitude/Latitude and UTC are stated here but overwritten when webpage credentials are entered (if using WiFi Manager)
int localUTC = 12;                               //Country UTC offset, needed for UTC for day/night calc  (+12 for NZ)  don't need to change for daylight saving as no needed for day/night
int UTCoffset = 0;                               //Set my user with touch button +1, -1, 0

//Chime variables
int chime_phase = 0;   //If top of the hour 
int Lastchimemillis;   //Used for tracking delay
int chime_working = 0; //working var for sinearray (ranges 0-31)
int chime_start_millis;
int chime_phase_complete = 0; //Flag to ensure only 1 chime cycle per 0 minutes
int completed_chimes = 0;    //Counts the number of completed chimes
int chime = 0;
char chime_array[26];

//LED Variables. Hold the value (0-255) of each primary colour
int green = 0;
int blue = 0;
int red = 0;

//Print var
int verbose_output = 0; // 0 = No serial print, 1 = serial print

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
void Timekeeping();
void Touchsensor_check();
void StartOTA();
void WiFi_start();
void Test_LEDs();
void weather_DotheLEDs();
void nightday_DoTheLEDs();
void Pressure_handle();
void API_Request();
void Zambretti_calc();
void measurementEvent();
void ReadFromSPIFFS();
void WriteToSPIFFS(int write_timestamp);
void do_blynk();
void SPIFFS_init();
void UpdateSPIFFS();
void FirstTimeRun();
void Request_Time();
void sendNTPpacket(const IPAddress &address);
void update_epoch_time();
void decode_epoch(unsigned long currentTime);
void initiate_time();
void LocalClock();
bool Check_Time(); //Check time is correct and ok
void sunrise_sunset();
void API_check(); 
void WiFi_and_Credentials();
void Flip_modes();
void SpeakClock();
void daynight();
void checkchime();


//Classes
WiFiUDP udp;                // A UDP instance to let us send and receive packets over UDP
ESP8266WiFiMulti wifiMulti; // Create an instance of the ESP8266WiFiMulti class, called 'wifiMulti'
IPAddress timeServer;
FtpServer ftpSrv;
BMP280 bmp;
SoftwareSerial mySoftwareSerial(0, 14); // Declare pin RX & TX pins for TF Sound module.
DFRobotDFPlayerMini myDFPlayer;


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
      myDFPlayer.playFolder(2, Zambretti_trend_mp3); //only one of these will have a value
      delay(1000);
      yield();
    }
    myDFPlayer.playFolder(2, Zambretti_mp3);
  }
}

void setup()
{
  Serial.begin(9600);

  //Touch sensor Initialisation
  csensy.set_CS_AutocaL_Millis(0xFFFFFFFF); //Touch sensor Initialisation                               

  WiFi_and_Credentials();

  //Get initial sunrise/set times
  API_Request(); //Get sunrise/sunset times.  Do this after Test section in case API HTTP overridden

  // SPIFFS.format();
  // WiFiManager wifiManager;
  // wifiManager.resetSettings();

  //DF Sound player setup
  mySoftwareSerial.begin(9600);
  myDFPlayer.setTimeOut (2000);

  Serial.println();
  Serial.println(F("DFRobot DFPlayer Mini Demo"));
  Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));
  
  if (!myDFPlayer.begin(mySoftwareSerial, false)) {  //Use softwareSerial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    while(true){
      delay(0); // Code to compatible with ESP8266 watch dog.
    }
  }
  Serial.println(F("DFPlayer Mini online."));
  
  myDFPlayer.volume(20);  //Set volume value. From 0 to 30

  delay(1000);

  // //Get initial sunrise/set times
  // API_Request(); //Get sunrise/sunset times.  Do this after Test section in case API HTTP overridden
  StartOTA();

  Blynk.config(auth); 
  Blynk.connect();

  FastLED.addLeds<WS2811, PIN_LED, COLOR_ORDER>(leds, NUM_LEDS_PER_STRIP); //Initialise the LEDs

  LEDmillis = millis();
  MP3millis = millis();


  //BMP setup
  if(!bmp.begin()){
    Serial.println("BMP init failed!");
    while(1);
  }
  else Serial.println("BMP init success!");
  
  bmp.setOversampling(4);

  Wire.begin(); //Initial I2C bus
  delay(1000);
  //Startsensor();
  Test_LEDs();

  //Initiate array with 0 (no value = no prediction)
  for (int x = 1; x <= Zambrettiarraysize; x++)
  {
    Zambretti_array[x] = 990;
  }

  pressure_read_millis = millis();

  //******** GETTING THE TIME FROM NTP SERVER  ***********************************
  initiate_time(); //Get NTP and time set up for the first time
  decode_epoch(epoch); //epoch has been updated, Now turn this into UTC clock_minutes_from_midnight
  
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
  count = millis();          //Initial count for LED update
  working_modecount = millis();

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

  SpeakClock();
    delay(1500);
  if (working_mode == true){
    myDFPlayer.playFolder(2, 150);
    }
  else
  {
    myDFPlayer.playFolder(2, 151);
  }
  
  Serial.println("");
  Serial.println("********************************************************* START LOOP **************************************************************");
  Serial.println("");
  Serial.println("");
 }

void loop()
{

  ftpSrv.handleFTP();
    yield();
  ArduinoOTA.handle();
    yield();
  Blynk.run(); //If Blynk being used
    yield();
  Timekeeping(); //Keep track ing micros() and use for calculation adjustments
    yield();
  Pressure_handle(); //Update yaw, pitch, roll and work out compass heading
    yield();
  Touchsensor_check(); //Any touchs on sensor - speak the forecast
    yield();
  update_epoch_time(); //update epoch time by millis update or NTP request
    yield();
  decode_epoch(epoch); //epoch has been updated, Now turn this into UTC clock_minutes_from_midnight
    yield();
  API_check();         //Check if it's time to get Sunrise/Set times and action
    yield();
  measurementEvent();
    yield();
  checkchime(); 

  if (SPIFFS.exists(restartfilename) == true)
  {
    SPIFFS.remove(restartfilename);
    delay(5000);
    ESP.restart();
  }


  //*** Display local time
  delt_t = millis() - timecount;
  if (delt_t > showtime_delayamount)
  {
    if (verbose_output == 1)
    {
      Serial.println("");
      Serial.println("********************************************************");
    }
    LocalClock();         //Turn minutes from midnight into local H:M
    timecount = millis(); //Reset the count

    if (verbose_output == 1)
    {
      Serial.println("********************************************************");
      Serial.println("");
    }
  }

  yield();

  //*** Display Zambretti and update LEDs
  delt_t = millis() - count;
  if (delt_t > delayamount)
  {
    
    if (working_mode == true ){
      nightday_DoTheLEDs(); //Update LEDs
    }
    
    if (working_mode == false){
      weather_DotheLEDs(); //Update LEDs
    }


    yield();
    Pressure_handle(); 
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

  yield();
}

void Zambretti_calc()
{

  //**************************Calculate Zambretti Forecast*******************************************

  accuracy_in_percent = accuracy * 94 / 12; // 94% is the max predicion accuracy of Zambretti

  ZambrettisWords = ZambrettiSays(char(ZambrettiLetter()));

  if (verbose_output == 1)
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
    
    // if (month(current_timestamp) > 4 || month(current_timestamp) < 9)
    //   zambretti = zambretti + 1;

    //A Winter falling generally results in a Z value lower by 1 unit
    if (hemisphere == 1 && (month(current_timestamp) < 4 || month(current_timestamp) > 9)) zambretti = zambretti + 1;  //Northern hemisphere
    if (hemisphere == 2 && (month(current_timestamp) > 4 || month(current_timestamp) < 9)) zambretti = zambretti + 1;  //Southern hemisphere

    if (zambretti > 9) zambretti = 9;

    if (verbose_output == 1)
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

    if (verbose_output == 1)
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

    if (verbose_output == 1)
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
  if (verbose_output == 1)
  {
    Serial.print("This is Zambretti's famous letter: ");
    Serial.println(z_letter);
    Serial.print("Zambretti LED = ");
    Serial.println(Zambretti_LED);
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

  if (verbose_output == 1)
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

  if (verbose_output == 1)
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
    Zambretti_LED = 4;
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


void Pressure_handle()
{

  if (pressure < minpressure)
  {
    minpressure = pressure;
  }

  if (pressure > maxpressure)
  {
    maxpressure = pressure;
  }

  if (verbose_output == 2)
  {

    Serial.print("Current / Min / Max pressure: ");
    Serial.print(pressure);
    Serial.print(" / ");
    Serial.print(minpressure);
    Serial.print(" / ");
    Serial.println(maxpressure);

    Serial.println();
    Serial.println("********************************************************\n");

    //From BMP280 Sensor
    Serial.print("temperature, pressure: ");
    //Serial.print(myBMP280.readTemperature(), 1);
    Serial.print(" C,  ");

    Serial.print(pressure, 1);
    Serial.println(" milli bar");

    Serial.print("Current UNIX Timestamp: ");
    Serial.println(current_timestamp);

    Serial.println();
    Serial.println("********************************************************\n");

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

//===================================================================================================================
//====== Set of useful function to access acceleration. gyroscope, magnetometer, and temperature data
//===================================================================================================================

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

void weather_DotheLEDs()
{

  //Red or Oarnge is Unsettled/stormy
  //Blue or Light blue is rain or showers
  //Yellow is fine

  //Clear LEDs
  fill_solid(leds, NUM_LEDS_PER_STRIP, CRGB(0, 0, 0));

  //1=Stormy (X>Z), 2=Rain (T>W), 3=Unsettled (P>S), 4=Showery (I>O), 5=Fine (A>H)

  //Stormy
  if (Zambretti_LED == 1)
  {
    //Red
    fill_solid(leds, NUM_LEDS_PER_STRIP, CRGB(255, 0, 0));
    FastLED.setBrightness(brightness2);
  }

  //Rain
  if (Zambretti_LED == 2)
  {
    //Green
    fill_solid(leds, NUM_LEDS_PER_STRIP, CRGB(0, 255, 0));
    FastLED.setBrightness(brightness1);
  }

  //Unsettled
  if (Zambretti_LED == 3)
  {
    //Orange
    fill_solid(leds, NUM_LEDS_PER_STRIP, CRGB(255, 50, 0));
    FastLED.setBrightness(brightness2);
  }

  //Showery
  if (Zambretti_LED == 4)
  {
    //Light Green
    fill_solid(leds, NUM_LEDS_PER_STRIP, CRGB(170, 200, 0));
    FastLED.setBrightness(brightness1);
  }

  //Fine
  if (Zambretti_LED == 5)
  {
    //Yellow
    fill_solid(leds, NUM_LEDS_PER_STRIP, CRGB(255, 128, 0));
    FastLED.setBrightness(brightness1);
  }

  FastLED.show();

}

void Touchsensor_check()
{
  touchsensor = csensy.capacitiveSensor(30);

  if (touchsensor > 0)
  {
    touchmillis = millis();
    //delay(200);
    touchsensor = csensy.capacitiveSensor(30);


    if (touchsensor > touchthreshold)
    {
      //How long has it been held down
      while (csensy.capacitiveSensor(30) > touchthreshold)
      {
        if (millis() - touchmillis > touchUTCmax)
        { //More than 10s then escape the while
          break;
        }
        yield(); //Tight loop, yield to avoid WTC crashes
      }

      press_period = millis() - touchmillis;

      //Between min time and the spoken time min then stop playing mp3
      if (press_period >= touchstopmin && press_period < touchForecastmin)
      {
        SpeakClock();
        Serial.println();
        Serial.println("*** Speak time ***");
        Serial.println();      
      }

      //Between spoken forecast min and the mode swap
      if (press_period >= touchForecastmin && press_period <= touchSwapmin)
      {

        //If accuracy less than 6hours then make no foreacast
        if (accuracy < accuracygate && Zambretti_mp3 > 0)
        {
          ZambrettisWords = TEXT_ZAMBRETTI_DEFAULT;
          Zambretti_mp3 = 126;
        }  
        Serial.println();
        Serial.println("*** Speak forecast ***");
        Serial.println();

        //SpeakClock();
        // delay(1500);
        myDFPlayer.playFolder(2, Zambretti_trend_mp3); //only one of these will have a value
          delay(1000);
          yield();
        myDFPlayer.playFolder(2, Zambretti_mp3);
          yield();
          delay(2500);
      }

      //Between spoken forecast min and the mode swap
      if (press_period > touchSwapmin && press_period < touchUTCmin)
      {
          Flip_modes();
          yield();
          delay (1000);
          working_modecount = millis();       
      }

      //Greater than UTC min
      if (press_period >= touchUTCmin)
      {
        UTC_Cycle++;

        if (UTC_Cycle > 154)
        {
          UTC_Cycle = 152;

        }

        if (UTC_Cycle == 153)
        {
          UTCoffset = -1;
                Serial.println();
                Serial.println("*** UTC -1 ***");
                Serial.println();
        }

        if (UTC_Cycle == 152)
        {
          UTCoffset = 0;
                Serial.println();
                Serial.println("*** UTC ***");
                Serial.println();
        }

        if (UTC_Cycle == 154)
        {
          UTCoffset = 1;
                Serial.println();
                Serial.println("*** UTC +1 ***");
                Serial.println();
        }

        myDFPlayer.playFolder(2, UTC_Cycle); //Play selected mp3 in folder mp3
        delay(1000);
      }
    }
  }
}


void measurementEvent()
{

  //Measures absolute Pressure, Temperature, Humidity, Voltage, calculate relative pressure,
  //Dewpoint, Dewpoint Spread, Heat Index


  double T,P;
  char result = bmp.startMeasurment();

if(result!=0){
    delay(result);
    result = bmp.getTemperatureAndPressure(T,P);
    
      if(result!=0)
      {
        double A = bmp.altitude(P,P0);     
      }
      else {
        Serial.println("Error.");
      }
  }
  else {
    Serial.println("Error.");
  }

  // Get temperature
  measured_temp = T;

  // Get pressure
  measured_pres = P;
  pressure = P;

  // Calculate and print relative pressure
  SLpressure_hPa = (((measured_pres * 100.0)/pow((1-((float)(ELEVATION))/44330), 5.255))/100.0);
  rel_pressure_rounded=(int)(SLpressure_hPa+.5);

  // Calculate dewpoint
  double a = 17.271;
  double b = 237.7;
  double tempcalc = (a * measured_temp) / (b + measured_temp) + log(measured_humi * 0.01);
  DewpointTemperature = (b * tempcalc) / (a - tempcalc);

  // Calculate dewpoint spread (difference between actual temp and dewpoint -> the smaller the number: rain or fog
  DewPointSpread = measured_temp - DewpointTemperature;

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
  }
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
  Blynk.virtualWrite(7, ZambrettisWords);      // virtual pin 7
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

  if (Seconds_SinceLast_NTP_millis > NTP_Seconds_to_wait) //Don't go to NTP during chime phase as it causes flicker
  {
    if (verbose_output == 1)
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

    if (verbose_output == 1)
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

  if (verbose_output == 1)
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

//Calculate LED colours phases on the phase of the sun using sunrise API data and NTP time converted to local time (using UTC offset)
void nightday_DoTheLEDs()
{

  //Turn hours into H:H
  int local_hour = local_clock_minutes_from_midnight / 60; //Turn minutes into the hour, needed for chime check

  if (local_hour > 12)
  {
    local_hour = local_hour - 12;
  }


  if (verbose_output == 1)
  {
    // Serial.print("UTC Hour: ");
    // Serial.print(hour);
    // Serial.print(",   Minute: ");
    // Serial.print(minute);
    // Serial.print(",   Second: ");
    // Serial.print(second);
    // Serial.print(",   clock_AMPM: ");
    // Serial.println(clock_AMPM);
    // Serial.println();
  }
  //Check for sunrise.  clock_minutes_from_midnight is time in minutes from midnight.  Sunrise/set minutes and clock are both UTC
  //Only compare UTC with UTC as local time (using UTC offset can change with daylight savings).  Local only for figuring out if it's night or day

  //Corrected Sunrise/Set and time variables
  int sunrise_minutes_from_midnight_corrected = sunrise_minutes_from_midnight;
  int sunset_minutes_from_midnight_corrected = sunset_minutes_from_midnight;
  int clock_minutes_from_midnight_corrected = clock_minutes_from_midnight;

  //e.g SR 0020 means 30mins before and 30 after would be 1430:0050.  Different timelines are difficult to compare.  Make 0020 = 1460 (1440 + 0020) then 30mins before/after:  1430:1490
  //Need to correct time
  if (sunrise_minutes_from_midnight < (minswithin / 2))
  {
    sunrise_minutes_from_midnight_corrected = sunrise_minutes_from_midnight + 1440;

    //if Sunrise corrected then correct time is also in the same way
    if (clock_minutes_from_midnight < (minswithin / 2))
    {
      clock_minutes_from_midnight_corrected = clock_minutes_from_midnight + 1440;

      Serial.println("*sunrise period spans midnight UTC - corrected");
    }
  }

  if (sunset_minutes_from_midnight < (minswithin / 2))
  {
    sunset_minutes_from_midnight_corrected = sunset_minutes_from_midnight + 1440;

    //if Sunrise corrected then correct time is also in the same way
    if (clock_minutes_from_midnight < (minswithin / 2))
    {
      clock_minutes_from_midnight_corrected = clock_minutes_from_midnight + 1440;

      Serial.println("*sunset period spans midnight UTC - corrected");
    }
  }

  //Check for Sunrise phase
  if (clock_minutes_from_midnight_corrected >= (sunrise_minutes_from_midnight_corrected - (minswithin / 2)) && clock_minutes_from_midnight_corrected <= (sunrise_minutes_from_midnight_corrected + (minswithin / 2)))
  {
    SR_Phase = 1;
    LED_phase = ((clock_minutes_from_midnight_corrected - sunrise_minutes_from_midnight_corrected) + (minswithin / 2)) / (float)minswithin * 255;
  }
  else
  {
    SR_Phase = 0;
  }

  //Check for sunset.  clock_minutes_from_midnight is time in minutes from midnight.  Sunrise/set minutes is LOCAL time from API
  if (clock_minutes_from_midnight_corrected >= (sunset_minutes_from_midnight_corrected - (minswithin / 2)) && clock_minutes_from_midnight_corrected <= (sunset_minutes_from_midnight_corrected + (minswithin / 2)))
  {
    SS_Phase = 1;
    LED_phase = ((clock_minutes_from_midnight_corrected - sunset_minutes_from_midnight_corrected) + (minswithin / 2)) / (float)minswithin * 255;
  }
  else
  {
    SS_Phase = 0;
  }

  //if it's not in sunrise or sunset sequence then find out if it's day (yellow) or night (blue) and set colour
  //Using Local UTC (don't care about daylight saving) for day or night
  if (local_clock_minutes_from_midnight > local_sunrise_minutes_from_midnight && local_clock_minutes_from_midnight < local_sunset_minutes_from_midnight)
  {
    night = 0;
  }
  else
  {
    night = 1;
  }

  if (verbose_output == 1)
  {

    Serial.print("clock_minutes_from_midnight (UTC) = ");
    Serial.print(clock_minutes_from_midnight);
    Serial.print(",   local_clock_minutes_from_midnight = ");
    Serial.println(local_clock_minutes_from_midnight);
    Serial.print("sunrise minutes_from_midnight (UTC) = ");
    Serial.print(sunrise_minutes_from_midnight);
    Serial.print(",   sunset_minutes_from_midnight (UTC) = ");
    Serial.println(sunset_minutes_from_midnight);
    Serial.print("Seconds_SinceLast_NTP_millis: ");
    Serial.print(Seconds_SinceLast_NTP_millis);
    Serial.print(",   Startmillis: ");
    Serial.print(startmillis);
    Serial.print(",   epochstart: ");
    Serial.print("SecondsSinceLastAPI: ");
    Serial.print(SecondsSinceLastAPI);
    Serial.println(epochstart);
    Serial.println();
    Serial.print("chime_phase = ");
    Serial.print(chime_phase);
    Serial.print(",   night = ");
    Serial.print(night);
    Serial.print(",   Sunrise phase = ");
    Serial.print(SR_Phase);
    Serial.print(",   Sunset phase = ");
    Serial.println(SS_Phase);
    Serial.print("Mins to Sunset phase = ");
    Serial.print(sunset_minutes_from_midnight - clock_minutes_from_midnight - int(minswithin / 2));
    Serial.print(",   Mins to Sunrise phase = ");
    Serial.print(sunrise_minutes_from_midnight - clock_minutes_from_midnight - int(minswithin / 2));
    Serial.print(", retryNTP = ");
    Serial.println(retryNTP);
  }
  
  //calculate the RGB for LEDs
  daynight();

  fill_solid(leds, NUM_LEDS_PER_STRIP, CRGB(red, green, blue));

  FastLED.setBrightness(brightness);
  FastLED.show();

  if (verbose_output == 1)
  {
    Serial.print("LED_phase = ");
    Serial.print(LED_phase);
    Serial.print(",   red = ");
    Serial.print(red);
    Serial.print(",   blue = ");
    Serial.print(blue);
    Serial.print(",   green = ");
    Serial.println(green);
    Serial.println();
    Serial.println("********************************************************");
    Serial.println();
    Serial.print("Working mode = ");
    Serial.print(working_mode);
    
    if (working_mode == true){
      Serial.println(" : Night light mode");
    }
    else
    {
      Serial.println(" : Weather mode");
    }
    
    Serial.println();
    Serial.println("********************************************************");
    Serial.println("");
  }
}

//daunight mode:  yellow during day, blue at night
void daynight()

{
  SRSS_Flip = 0; //0=Normal sunrise/sunset, 1=Inverted sunrise/sunset colour change order (only used in nightlight mode 1 & 2)
  sunrise_sunset();

  //It's night and not in sunrise or sunset mode
  if (SR_Phase == 0 && SS_Phase == 0 && night == 1)
  {
    blue = blue_night;
    green = green_night;
    red = red_night;
  }

  //It's day and not in sunrise or sunset mode
  if (SR_Phase == 0 && SS_Phase == 0 && night == 0)
  {
    blue = blue_day;
    green = green_day;
    red = red_day;
  }
}

//Calculate sunrise/sunset LED colours
void sunrise_sunset()
{

  //This piece flips SR and SS phases if requested by use of SRSS_Flip (for nightlight)
  int SR_Phase_use;
  int SS_Phase_use;

  if (SR_Phase == 1 && SRSS_Flip == 0)
  {
    SR_Phase_use = 1;
    SS_Phase_use = 0;
  }

  if (SR_Phase == 1 && SRSS_Flip == 1)
  {
    SR_Phase_use = 0;
    SS_Phase_use = 1;
  }

  if (SS_Phase == 1 && SRSS_Flip == 0)
  {
    SR_Phase_use = 0;
    SS_Phase_use = 1;
  }

  if (SS_Phase == 1 && SRSS_Flip == 1)
  {
    SR_Phase_use = 1;
    SS_Phase_use = 0;
  }

  if (verbose_output == 1)
  {
    Serial.print("SR_Phase_use = ");
    Serial.print(SR_Phase_use);
    Serial.print(",   SS_Phase_use = ");
    Serial.println(SS_Phase_use);
    Serial.println();
  }

  //sunrise:  Start with blue reducing to zero, and red increasing, when blue 0 increase green
  if (SR_Phase_use == 1 && blue >= 0)
  {
    //in first part of sunrise (reduce blue / increase red)
    blue = blue_max - (LED_phase * 2); //LED Phase 0-128,  *2 as in 1st phase we need 0-255 of LED movement for blue & red
    red = 0 + (LED_phase * 2);

    if (blue < 0)
    {
      blue = 0;
    }
    if (red > red_max)
    {
      red = red_max;
    }
    green = 0; //force in case lamp started during phase
  }

  if (SR_Phase_use == 1 && blue == 0)
  {
    //in second part of sunrise (increase green)
    green = 0 + abs(LED_phase - 128); //LED Phase 128-255, need to be 0 to start green LED movement (0-128)

    if (green > green_max)
    {
      green = green_max;
    }
  }

  //sunset:  Start with green reducing to zero, then reducing red to 0 and increasing blue
  if (SS_Phase_use == 1 && green > 0)
  {
    green = green_max - (LED_phase); //LED Phase 0-128, reduce green from 128 > 0

    if (green < 0)
    {
      green = 0;
    }

    blue = 0; //force in case lamp started during phase
  }

  if (SS_Phase_use == 1 && green == 0)
  {
    //in second part of sunset increase blue and rdecrease red
    blue = 0 + (abs(LED_phase - 128) * 2);      //LED Phase 128-255, subtract 128 (gives range 0-128) then x2 (range 0-255)
    red = red_max - (abs(LED_phase - 128) * 2); //LED Phase 128-255, subtract 128 (gives range 0-128) then x2 (range 0-255)

    if (blue > blue_max)
    {
      blue = blue_max;
    }

    if (red < 0)
    {
      red = 0;
    }
  }
}
//Check if reset button pressed.  D3 / GPIO0 held to ground.
void checkreset(int ClearSPIFFS)
{
  if (digitalRead(0) == 0 || ClearSPIFFS == 1)
  {
    delay(500); //500ms for button bounce
    if (digitalRead(0) == 0 || ClearSPIFFS == 1)
    {

      //LEDs off
      fill_solid(leds, NUM_LEDS_PER_STRIP, CRGB(0, 0, 0));
      FastLED.setBrightness(255);
      FastLED.show();

      Serial.println("** RESET **");
      Serial.println("** RESET **");
      Serial.println("** RESET **");

      SPIFFS.remove(HTTPfilename);
      SPIFFS.remove(UTCfilename);
      SPIFFS.remove(chimefilename);
      SPIFFS.format();
      WiFi.disconnect();

      delay(2500);
      ESP.restart();
    }
  }
}
void API_check()
{
  //Check if it's time to get Sunrise/Set times
  SecondsSinceLastAPI = (millis() - LastAPI) / 1000;              //How many seconds since Last API pull
  if (SecondsSinceLastAPI > APISecondstowait && chime_phase == 0) //Don't go to sunride API during chime phase as it causes flicker)
  {
    LastAPI = millis();
    API_Request(); //get sunrise/sunset data
    yield();
  }
}


// Get data from Sunrise API HTTP address
void API_Request()

{
  HTTPClient http;
  char buff[400];

  Serial.print("Getting Sunrise API data with: ");
  Serial.println(sunrise_api_request);
  http.begin(sunrise_api_request);

  int httpCode = http.GET(); //Serial.print("[HTTP] GET...\n");  Start connection and send HTTP header
  if (httpCode > 0)          // httpCode will be negative on error
  {
    // HTTP header has been send and Server response header has been handled
    //Serial.printf("[HTTP] GET... code: %d\n", httpCode);
    if (httpCode == HTTP_CODE_OK)
    {
      String payload = http.getString();
      payload.toCharArray(buff, 400);
      sunAPIresponse = payload; //Sunresponse is used by JSON function
      Serial.println("API response received");
      Serial.println();

      sscanf(JSON_Extract("sunrise").c_str(), "%d:%d:%*d %c", &hour_sunrise, &minute_sunrise, SR_AMPM); //Get JSON for sunrise (string) convert to const char and search for hours:minutes
      sscanf(JSON_Extract("sunset").c_str(), "%d:%d:%*d %c", &hour_sunset, &minute_sunset, SS_AMPM);    //Get JSON for sunset (string) convert to const char and search for hours:minutes

      Serial.print("hour_sunrise = ");
      Serial.print(hour_sunrise);
      Serial.print(",   minute_sunrise = ");
      Serial.print(minute_sunrise);
      Serial.print(",   SR AMPM = ");
      Serial.println(SR_AMPM);
      Serial.print("hour_sunset = ");
      Serial.print(hour_sunset);
      Serial.print(",   minute_sunset = ");
      Serial.print(minute_sunset);
      Serial.print(",   SS AMPM = ");
      Serial.println(SS_AMPM);
      Serial.println();
      Serial.println("****************");
      Serial.println();
    }
  }
  else
  {
    Serial.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
  }
  http.end();
}

//JSON Function
String JSON_Extract(String lookfor)
{
  DynamicJsonBuffer jsonBuffer;
  JsonObject &root = jsonBuffer.parseObject(sunAPIresponse);
  JsonObject &data = root["results"];
  return data[lookfor];
}

//Connect to the WiFi and manage credentials
void WiFi_and_Credentials()
{
  //2 ways to get WiFi.  ConnectToAP uses variables from build flags (platformio) or hard coded.
  //WiFiManager will check if WiFi credentials are known (in Flash memory).  If not it will stary a webserver to collect details from user

  //Use one of the follow lines depending on approach (build flags vs WiFiManager)
  //ConnectToAP();           //Connect to Wifi (if not using Wifi Manager approach)
  WiFiManager wifiManager;

  //SPIFFs section to Read and Write the saved credentials in Flash memory
  //Check if APIaddress.txt exists.  If not create it and store the http address for sunrise API, if yes read it.

  if (SPIFFS.begin())
  {
    Serial.println("SPIFFS Initialize....ok");
  }
  else
  {
    Serial.println("SPIFFS Initialization...failed");
    SPIFFS.end();
  }

  Serial.println("Trying files...");

  if (SPIFFS.exists(HTTPfilename) == true)
  {
    Serial.println("File already exisits.  Read stored data.");

    //Read HTTP addresss file data
    File f = SPIFFS.open(HTTPfilename, "r");

    if (!f)
    {
      Serial.println("file open failed");
    }
    else
    {
      Serial.println("Reading Data from HTTP file:");
      //Data from file

      size_t size = f.size();

      f.readBytes(sunrise_api_request, size);

      f.close(); //Close file
      Serial.printf("Read http adress for Sunrise/set API = %s\n", sunrise_api_request);
      Serial.println("File Closed");


      //Read UTC file data
      File h = SPIFFS.open(UTCfilename, "r");

      if (!h)
      {
        Serial.println("file open failed");
      }
      else
      {
        Serial.println("Reading Data from UTC file:");
        //Data from file

        size_t size = h.size();

        h.readBytes(UTC, size);

        h.close(); //Close file
        Serial.printf("Read UTC = %s\n", UTC);
        Serial.println("File Closed");
      }

      //Read chime file data
      File i = SPIFFS.open(chimefilename, "r");

      if (!i)
      {
        Serial.println("file open failed");
      }
      else
      {
        Serial.println("Reading Data from chime file:");
        //Data from file

        size_t size = i.size();

        i.readBytes(chime_array, size);

        i.close(); //Close file
        Serial.printf("Read chime = %s\n", chime_array);
        Serial.println("File Closed");
      }

      //WiFiManager will read stored WiFi Data, if it can't connect it will create a website to get new credentials.
      wifiManager.autoConnect("Rubber_duck");
    }
  }

  else
  {
    Serial.println("Filename DOESN'T exisit");

    //If file doesn't exist, get details from the user with wifimanager website
    //create http address and store in APIaddress.txt file

    WiFiManagerParameter custom_longitude("Longitude", "Longitude", "", 10);
    WiFiManagerParameter custom_latitude("Latitude", "Latitude", "", 10);
    WiFiManagerParameter custom_UTC("UTC", "UTC", "", 3);
    WiFiManagerParameter custom_chime("chime", "chime", "", 25);

    wifiManager.addParameter(&custom_longitude);
    wifiManager.addParameter(&custom_latitude);
    wifiManager.addParameter(&custom_UTC);
    wifiManager.addParameter(&custom_chime);

    //WiFiManager will read stored WiFi Data, if it can't connect it will create a website to get new credentials
    wifiManager.autoConnect("Duck_lamp");

    //Check if new http address needed to be written to file.  If yes, create and write.
    //Example: sunrise_api_request = "http://api.sunrise-sunset.org/json?lat=-41.2865&lng=174.7762"
    //Mode of the Lamp (e.g 0, 1, 2) as separate files in SPIFFS
    sprintf(sunrise_api_request, "http://api.sunrise-sunset.org/json?lat=%s&lng=%s", +custom_latitude.getValue(), custom_longitude.getValue());
    sprintf(UTC, "%s", +custom_UTC.getValue());
    sprintf(chime_array, "%s", +custom_chime.getValue());
    
    //Create New HTTP File And Write Data to It
    //w=Write Open file for writing
    File f = SPIFFS.open(HTTPfilename, "w");

    if (!f)
    {
      Serial.println("HTTP file open failed");
    }
    else
    {
      //Write data to file
      Serial.println("Writing Data to HTTP File");
      f.print(sunrise_api_request);
      Serial.println("New file written");
      f.close(); //Close file
    }

    //Create New UTC File And Write Data to It
    //w=Write Open file for writing
    File h = SPIFFS.open(UTCfilename, "w");

    if (!h)
    {
      Serial.println("UTC file open failed");
    }
    else
    {
      //Write data to file
      Serial.println("Writing Data to UTC File");
      h.print(UTC);
      Serial.println("New file written");
      h.close(); //Close file
    }

    //Create New chime File And Write Data to It
    //w=Write Open file for writing

    File i = SPIFFS.open(chimefilename, "w");

    if (!i)
    {
      Serial.println("chime file open failed");
    }
    else
    {
      //Write data to file
      Serial.println("Writing Data to chime File");
      i.print(chime);
      Serial.println("New file written");
      i.close(); //Close file
    }
  }


  //Get volume from chime array
  char buffer0[1];
  buffer0[0] = chime_array[0];
  mp3vol = atoi(buffer0);

  //Set the mp3 volume form the chime array, this is used as a flag to run/no run sound functions in loop
  if (mp3vol < 0 || mp3vol > 9)
  {
    mp3vol = 0; //default to 0 in error state - no sound
    Serial.println("mp3 volume incorrect - overriding");
  }
  else
  {
    mp3vol = mp3vol * 3.3; //30 is max volume, 9 x 3.3 = 29.7

    if (mp3vol >= 29)
    { //29 is close enough to 30, make it 30
      mp3vol = 30;
    }
  }

  //mp3_set_volume(mp3vol); //Set the mp3 volume
  Serial.print("mp3 Volume = ");
  Serial.println(mp3vol);

  //Get UTC from saved file and turn into an Int and check it's between 0-24
  char buffer4[3];
  buffer4[0] = UTC[0];
  buffer4[1] = UTC[1];
  buffer4[2] = UTC[2];
  localUTC = atoi(buffer4);

  if (localUTC < -13 || localUTC > 13)
  {
    Serial.println("UTC mode incorrect - overriding");
    localUTC = 12; //Default to NZ (winter)
  }
  Serial.print("UTC used = ");
  Serial.println(localUTC);

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

  recovered_ssid = wifiManager.getSSID();
  recovered_pass = wifiManager.getPassword();
}

//Update the time
void decode_epoch(unsigned long currentTime)
{

  // print the raw epoch time from NTP server
  if (printNTP == 1 && verbose_output == 1)
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

  if (printNTP == 1 && verbose_output == 1)
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

  if (printNTP == 1 && verbose_output == 1)
  {
    Serial.print("UTC Clock - Mins from midnight = ");
    Serial.print(clock_minutes_from_midnight);
    Serial.print(",   Local - Clock - Mins from midnight = ");
    Serial.println(local_clock_minutes_from_midnight);
    Serial.println();
    Serial.println("****************");
    Serial.println();
  }

  //Work out Hours/min into minutes from midnight
  working_hourtomin = hour_sunrise;

  //PM add 12
  if (strcmp(SR_AMPM, "P") == 0)
  {
    working_hourtomin = hour_sunrise + 12;
  }

  //Midnight = 0
  if (strcmp(SR_AMPM, "A") == 0 && hour_sunrise == 12)
  {
    working_hourtomin = 0;
  }

  //Noon = 12
  if (strcmp(SR_AMPM, "P") == 0 && hour_sunrise == 12)
  {
    working_hourtomin = 12;
  }

  //UTC number of minutes from midnight until sunrise qwerty
  sunrise_minutes_from_midnight = ((working_hourtomin * 60) + minute_sunrise);

  //Convert UTC sunrise_minutes_from_midnight into local_sunrise_minutes_from_midnight with UTC
  local_sunrise_minutes_from_midnight = sunrise_minutes_from_midnight + ((localUTC + UTCoffset) * 60);

  //If local_minutes is greater than 1 day (e.g large postive UTC)
  if (local_sunrise_minutes_from_midnight > 1440)
  {
    local_sunrise_minutes_from_midnight = local_sunrise_minutes_from_midnight - 1440;
  }

  //If local_minutes is negative (e.g Negative UTC)
  if (local_sunrise_minutes_from_midnight < 0)
  {
    local_sunrise_minutes_from_midnight = local_sunrise_minutes_from_midnight + 1440;
  }

  //Work out Hours/min into minutes from midnight
  working_hourtomin = hour_sunset;

  //PM add 12
  if (strcmp(SS_AMPM, "P") == 0)
  {
    working_hourtomin = hour_sunset + 12;
  }

  //Midnight = 0
  if (strcmp(SS_AMPM, "A") == 0 && hour_sunset == 12)
  {
    working_hourtomin = 0;
  }

  //Noon = 12
  if (strcmp(SS_AMPM, "P") == 0 && hour_sunset == 12)
  {
    working_hourtomin = 12;
    Serial.println();
  }

  sunset_minutes_from_midnight = ((working_hourtomin * 60) + minute_sunset);

  //Convert UTC sunrise_minutes_from_midnight into local_sunrise_minutes_from_midnight with UTC
  local_sunset_minutes_from_midnight = sunset_minutes_from_midnight + ((localUTC + UTCoffset) * 60);

  //If local_minutes is greater than 1 day (e.g large postive UTC)
  if (local_sunset_minutes_from_midnight > 1440)
  {
    local_sunset_minutes_from_midnight = local_sunset_minutes_from_midnight - 1440;
  }

  //If local_minutes is negative (e.g Negative UTC)
  if (local_sunset_minutes_from_midnight < 0)
  {
    local_sunset_minutes_from_midnight = local_sunset_minutes_from_midnight + 1440;
  }

  if (verbose_output == 1 && printNTP == 1)
  {
    Serial.print("UTC Hour: ");
    Serial.print(hour_UTC);
    Serial.print(",   Minute: ");
    Serial.print(minute_UTC);
    Serial.print(",   Second: ");
    Serial.println(second_UTC);
    Serial.println();

    Serial.print("local_sunrise_minutes_from_midnight = ");
    Serial.print(local_sunrise_minutes_from_midnight);
    Serial.print(",   local_sunset_minutes_from_midnight = ");
    Serial.println(local_sunset_minutes_from_midnight);
    Serial.print("sunrise_minutes_from_midnight = ");
    Serial.print(sunrise_minutes_from_midnight);
    Serial.print(",   sunset_minutes_from_midnight = ");
    Serial.println(sunset_minutes_from_midnight);
    Serial.println();
  }
}

void Flip_modes(){

    if (working_mode == true){
      yield();
      working_mode = false;
      myDFPlayer.playFolder(2, 151);
      Serial.println();
      Serial.println("*** Changed to Weather mode ***");
      Serial.println();
    }
  else 
    {
      yield();
      working_mode = true;
      myDFPlayer.playFolder(2, 150);
      Serial.println();
      Serial.println("*** Changed to Night light mode ***");
      Serial.println();
   }
}

void SpeakClock()
{

  int hour_mp3 = 0;
  int minute_mp3 = 0;
  int minute_mp3b = 999;
  int AMPMmp3 = 50;
  int local_hour = local_clock_minutes_from_midnight / 60; //Turn minutes into the hour, needed for chime check

  //Set to PM is 12pm or later
  if (local_hour >= 12)
  {
    AMPMmp3 = 51;
  }

  //Convert 24hr into 12hr clock
  if (local_hour > 12)
  {
    local_hour -= 12;
  }

  //Hours mp3.  Ranges from 30 (00 midnight) to 42 (Twelve)
  hour_mp3 = 30 + local_hour;

  //Minute mp3.  Specific words for 00 (OClock), 01, 02, 03, 04, 05, 06, 07, 08, 09, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 30, 40, 50
  if (minute_UTC <= 19)
  {
    minute_mp3 = 100 + minute_UTC;
  }
  else
  {
    if (minute_UTC % 10 == 0) //If it's MOD 0, this means it's at the top of the hour (O'Clock)
    {
      minute_mp3 = 100 + minute_UTC;
    }
    else
    {
      minute_mp3 = 100 + (minute_UTC - (minute_UTC % 10));
      minute_mp3b = 30 + (minute_UTC % 10);
    }
  }

  Serial.println();
  Serial.println("****************");
  Serial.print("Local time: ");
  Serial.print(local_hour);
  Serial.print(":");
  Serial.println(minute_UTC);
  Serial.print("Speaking mp3  Hour: ");
  Serial.print(hour_mp3);
  myDFPlayer.playFolder(1, hour_mp3); //Play selected mp3 in folder mp3
  delay(1000);
  Serial.print(",  Minute: ");
  Serial.print(1, minute_mp3);
  myDFPlayer.playFolder(1, minute_mp3); //Play selected mp3 in folder mp3
  delay(1000);
  Serial.print(" / ");
  Serial.print(minute_mp3b);
  Serial.print(" AMPM: ");

  if (minute_mp3b != 999)
  {
    myDFPlayer.playFolder(1, minute_mp3b);
    delay(1000);
  }

  Serial.println(AMPMmp3);
  Serial.println("****************");
  Serial.println();
  myDFPlayer.playFolder(1, AMPMmp3); //Play selected mp3 in folder mp3

  delay(1000);
  Serial.println();
}


//Check if chime is required and manipulate brightness
void checkchime()
{
  //if chime >= 1 (enabled) and minutes = 0 (top of hour) and LEDs are on (e.g not off during day) then set chime_phase = 1
  if (minute_UTC == 0 && chime_phase_complete == 0)
  {
    chime_phase_complete = 1;

      int local_hour = local_clock_minutes_from_midnight / 60; //Turn mines into the hour, needed for chime check
      int chimehour;
      char buffer8[0]; //Buffer from chime hour

      //Get chime on/off for hour
      buffer8[0] = chime_array[local_hour + 1];
      chimehour = atoi(buffer8);

      //Check for chime
      Serial.print("local clock_minutes from midnight = ");
      Serial.print(local_clock_minutes_from_midnight);
      Serial.print(",  Local hour = ");
      Serial.print(local_hour);
      Serial.print(",  chime array for hour = ");
      Serial.print(chimehour);

      if (chimehour > 0)
      {
        Serial.print(",  chime ON, Volume = "); //chime is on, the chime array didn't have 0 for this hour
        Serial.print(mp3vol);
        Serial.print(", ");

        if (chimehour >= 1 && chimehour <= 7)
        {
           myDFPlayer.playFolder(3, chimehour); //Play selected mp3 in folder mp3  (e.g if the chime sequence is 1-8, play 0001.mp3 > 0008.mp3 )
        }

        if (chimehour == 9)
        { //if chime sequence = 9 then don't play 0009.mp3, speak the clock time instead
          SpeakClock();
        }
        if (chimehour == 8)
        { 
        //if chime sequence = 8 then don't play 0008.mp3, speak the forecast instead
        //If accuracy less than 6hours then make no foreacast
        if (accuracy < accuracygate && Zambretti_mp3 > 0)
        {
          ZambrettisWords = TEXT_ZAMBRETTI_DEFAULT;
          Zambretti_mp3 = 126;
        }  
        Serial.println();
        Serial.println("*** Speak forecast ***");
        Serial.println();

        myDFPlayer.playFolder(2, Zambretti_trend_mp3); //only one of these will have a value
          delay(1000);
          yield();
        myDFPlayer.playFolder(2, Zambretti_mp3);
          yield();
          delay(2500);
        }
      }
      else
      {
        Serial.print(",  No chime, "); //The chime array had 0, no chime for this hour
      }
    }

    //Reset the flag to zero after 1min passed
    if (minute_UTC == 1 && chime_phase_complete == 1)
    {
      chime_phase_complete = 0;    
    }
  }


//Check if reset button pressed.  D3 / GPIO0 held to ground.
void checkreset()
{
      //LEDs off
      fill_solid(leds, NUM_LEDS_PER_STRIP, CRGB(0, 0, 0));
      FastLED.show();

      Serial.println("** RESET **");
      Serial.println("** RESET **");
      Serial.println("** RESET **");

      SPIFFS.remove(HTTPfilename);
      SPIFFS.remove(UTCfilename);
      SPIFFS.remove(chimefilename);
      SPIFFS.format();
      WiFi.disconnect();

      delay(2500);
      ESP.restart();
    }
