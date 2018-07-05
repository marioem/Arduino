/*
 * SD circuit:
 * SD card attached to SPI bus as follows UNO (MEGA):
 ** MOSI - pin 11 (51)
 ** MISO - pin 12 (50)
 ** CLK - pin 13 (52)
 ** CS - pin 10 (53)
 * 
 * WiFi circuit:
 * ESP8266 connected via UART UNO (MEGA):
 ** ESP RX - pin 1 (19)
 ** ESP TX - pin 0 (18)
 * 
 * Display module attached to I2C via PFC8574 (address 0x39)
 * Heater control attached to I2C via PFC8574 (address 0x38)
 * RTC (DS1307) attached to I2C (address xx)
 * 
 * SW Serial Monitor via FTDI Serial-USB board (9600 8-N-1) - if using Arduino Uno
 ** RX - pin 9
 ** TX - pin 8
 * 
 * LCD wiring to PCF8574:
 ** D0 - RS (3)
 ** D1 - R/W (4)
 ** D2 - EN (5)
 ** D3 - BACKLIGHT (6)
 ** D5...D7 - D5...D7 (11..14)
 * 
 */
#define DEBUGMODE // enables printing out diagnostic info to terminal

// Select platform (if not defined A_UNO Arduino MEGA is assumed)
// Proper board has to be selected, too.
//#define A_UNO

#include <avr/pgmspace.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <LiquidCrystal_PCF8574.h>
#ifdef A_UNO
  #include <SoftwareSerial.h>
#endif
#include <DS1307RTC.h>

// LCD stuff
//
#define LCDADDR  0x39
#define LCD_ROWS 2
#define LCD_COLS 16

#define CNTCOL 14
#define CNTROW  1

#define HEATERCOL 1
#define HEATERROW 0

#define TALCOL    0
#define TALROW    1

#define SALCOL    8    
#define SALROW    1

#define TIMECOL   8
#define TIMEROW   0

LiquidCrystal_PCF8574 lcd(LCDADDR);

// Heater controller stuff
#define HEATERADDR 0x38

const byte tempLimits[6] =  {15, 20, 25, 30, 35, 40};  // values of the temperatures to maintain in each box
byte heater = 0;  // global var sent to PCF controlling the state of the heaters.

#define HYSTHIGH    0.35  // hysteresis for temperature control, upper
#define HYSTLOW    -0.14  // hysteresis for temperature control, lower. Due to thermal inertia we need to control always above the required temperature
#define TEMPMARGIN  2     // +/- temp limit above/below which the temperature is deemed unreliable or not correctly controlled (e.g. sensor has bad contact or delivers start-up temperature)
#define TEMPLIM    85

// SD stuff
//
#if defined(A_UNO)
  #define CHIPSELECT 10 // for SPI interface - SD card
#else
  #define CHIPSELECT 53
#endif

// Temperature Sensors stuff
//
// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 2
#define TEMPERATURE_PRECISION 11
#define SENSOR_COUNT 7

byte tAlarms = 0;   // temperature alarms for the heater thermometers (0-5)

#define T7AL 6

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// arrays to hold device addresses
DeviceAddress Thermometer[SENSOR_COUNT] = { { 0x28, 0xF9, 0x34, 0x4A, 0x8, 0x0, 0x0, 0xBF },
                              { 0x28, 0x5D, 0xCB, 0x8F, 0x8, 0x0, 0x0, 0x4E },
                              { 0x28, 0xF4, 0x4D, 0x92, 0x8, 0x0, 0x0, 0x76 },
                              { 0x28, 0x3E, 0xED, 0x54, 0x8, 0x0, 0x0, 0x25 },
                              { 0x28, 0xFF, 0x71, 0xBD, 0x60, 0x16, 0x3, 0xE5 },
                              { 0x28, 0xFF, 0xC8, 0x60, 0x61, 0x16, 0x3, 0x6E },
                              { 0x28, 0x8E, 0x94, 0x4A, 0x8, 0x0, 0x0, 0x91 }
                            };


// SW monitor via SoftwareSerial stuff
//
#ifdef A_UNO
  SoftwareSerial monitor(9, 8); // RX, TX
#endif

// WiFi stuff
//
#define WIFISETUP   0
#define WIFILOOP    1
#define WIFIRSTPIN  3   // pin number to which WiFi module RST input is connected

// General stuff
//
#define THINGSPEAKINTERVAL 60000   // ThingSpeak updae interval in ms
#define SDINTERVAL         60000   // SD file update interval in ms
#define DELAY               1000   // Delay in ms between temp readouts 
#define BUZZERINTERVAL     29000   // Alarm buzzer every 29s for 1s
#define BUZZERPIN              4

// PROGMEM strings

const char string_0[] PROGMEM = "Hello!";
const char string_1[] PROGMEM = "Error opening datalog file";
const char string_2[] PROGMEM = "WiFi init error";
const char string_3[] PROGMEM = "GET /update?key=8ABPRZYZOPS96160";
const char string_4[] PROGMEM = "AT";
const char string_5[] PROGMEM = "AT+CIPSTART=\"TCP\",\"184.106.153.149\",80";
const char string_6[] PROGMEM = "SEND: ";
const char string_7[] PROGMEM = "AT+CIPSEND=";
const char string_8[] PROGMEM = "AT+CIPCLOSE";
const char string_9[] PROGMEM = "AT+CWMODE=1";
const char string_10[] PROGMEM = "AT+CWJAP=\"ASUS_Guest1\",\"Czech3'limbo\"";
const char string_11[] PROGMEM = "RECEIVE: Error";
const char string_12[] PROGMEM = "SEND: Error";
const char string_13[] PROGMEM = "Error";
const char string_14[] PROGMEM = "OK";
const char string_15[] PROGMEM = "DATALOG.CSV";
const char string_16[] PROGMEM = "Date,Time,Temp1,Temp2,Temp3,Temp4,Temp5,Temp6,Temp7,H1,H2,H3,H4,H5,H6\n";
const char string_17[] PROGMEM = "2016.11.11,22:24:43";
const char string_18[] PROGMEM = "&field";
const char string_19[] PROGMEM = "RECEIVE: OK";
const char string_20[] PROGMEM = "Init WiFi...";
const char string_21[] PROGMEM = "Init LCD...";
const char string_22[] PROGMEM = "Init SD...";
const char string_23[] PROGMEM = "Init Sensors...";
const char string_24[] PROGMEM = "Failed to initialize LCD...";
const char string_25[] PROGMEM = "Failed to init SD...";
const char string_26[] PROGMEM = "Temp C: ";
const char string_27[] PROGMEM = "Dev Addr: ";
const char string_28[] PROGMEM = "Requestinging temperatures...";
const char string_29[] PROGMEM = "Done";
const char string_30[] PROGMEM = "Conversion time: ";
const char string_31[] PROGMEM = "Starting...";
const char string_32[] PROGMEM = "System ready";
const char string_33[] PROGMEM = "Init Heater...";
const char string_34[] PROGMEM = "Heater error";
const char string_35[] PROGMEM = "Heater OK";
const char string_36[] PROGMEM = "WiFi OK";
const char string_37[] PROGMEM = "SD OK";


const char* const string_table[] PROGMEM = {string_0, string_1, string_2, string_3, string_4, string_5,
                                            string_6, string_7, string_8, string_9, string_10, string_11,
                                            string_12, string_13, string_14, string_15, string_16, string_17,
                                            string_18, string_19, string_20, string_21, string_22, string_23,
                                            string_24, string_25, string_26, string_27, string_28, string_29,
                                            string_30, string_31, string_32, string_33, string_34, string_35,
                                            string_36, string_37};
char buffer[128], buffer2[100], buffer3[8];

#define STR_HELLO       0
#define STR_DATALOGERR  1
#define STR_WIFIERR     2
#define STR_GET         3
#define STR_AT          4
#define STR_CIPSTART    5
#define STR_SEND        6
#define STR_CIPSEND     7
#define STR_CIPCLOSE    8
#define STR_CWMODE      9
#define STR_CWJAP      10
#define STR_RECERR     11
#define STR_SENDERR    12
#define STR_ERROR      13
#define STR_OK         14
#define STR_DATALOG    15
#define STR_HEADERS    16
#define STR_DATETIME   17
#define STR_FIELD      18
#define STR_RECOK      19
#define STR_WIFIINIT   20
#define STR_LCDINIT    21
#define STR_SDINIT     22
#define STR_SENSINIT   23
#define STR_LCDFAILED  24
#define STR_SDFAILED   25
#define STR_TEMPC      26
#define STR_DEVADDR    27
#define STR_REQTEMPC   28
#define STR_DONE       29
#define STR_CONVTIME   30
#define STR_START      31
#define STR_INITDONE   32
#define STR_HEATERINIT 33
#define STR_HEATERFAILED 34
#define STR_HEATEROK   35
#define STR_WIFIOK     36
#define STR_SDOK       37

#define ALARMSET   1
#define ALARMCLEAR 0

// Define system alarm bit positions
#define SALLCD    4
#define SALWIFI   3
#define SALHEATER 2
#define SALSD     1
#define SALRTC    0

byte cnt = 0, logdata = 0;

byte sAlarms = 0;      // system alarms flags.
                       // Bits:
                       // 4 - LCD
                       // 3 - WiFi
                       // 2 - Heaters
                       // 1 - SD
                       // 0 - RTC
char alarmStr[8];
char rtcTime[9], rtcDate[11];

byte wifiDelayCnt = 0; // counter used to count 1 minute intervals to update ThingSpeak
byte buzzDelayCnt = 0; // counter used to count 29s of pause between alarm beeps.

void setup(void)
{
  byte i = 0;

  pinMode(BUZZERPIN, OUTPUT);
  digitalWrite(BUZZERPIN, LOW);
  
  // start serial port
  Serial.begin(115200);
#if !defined(A_UNO)
  Serial1.begin(115200);
#endif

#ifdef DEBUGMODE
  #ifdef A_UNO
    monitor.begin(57600);
  #endif
#endif

  // initialize LCD with number of columns and rows: 
#ifdef DEBUGMODE
  printMonit(STR_LCDINIT);
#endif
  Wire.begin();
  Wire.beginTransmission(LCDADDR);

  if (Wire.endTransmission() != 0) {
    Alarm(&sAlarms, SALLCD, ALARMSET);
#ifdef DEBUGMODE
    printMonit(STR_LCDFAILED);
#endif
  }

  lcd.begin(LCD_COLS, LCD_ROWS); // initialize the lcd
  
  // Print a message to the LCD
  lcdMonit(STR_START, 0, 0, true);
  delay(700);

#ifdef DEBUGMODE
  printMonit(STR_HEATERINIT);
#endif
  Wire.begin();
  Wire.beginTransmission(HEATERADDR);

  if (Wire.endTransmission() != 0) {
    Alarm(&sAlarms, SALHEATER, ALARMSET);
#ifdef DEBUGMODE
    printMonit(STR_HEATERFAILED);
#endif
  } else {
    heater = 0;             // all heaters off
    Wire.beginTransmission(HEATERADDR);
    Wire.write(heater);
    Wire.endTransmission();
    
    delay(700);
    lcdMonit(STR_HEATEROK, 0, 0, true);
    delay(700);
  }
    
  // initialize and conenct wifi
#ifdef DEBUGMODE
  printMonit(STR_WIFIINIT);
#endif
  lcdMonit(STR_WIFIINIT, 0, 0, true);

  wifiInit(WIFISETUP);

  delay(700);
  
#ifdef DEBUGMODE
  printMonit(STR_SDINIT);
#endif
  lcdMonit(STR_SDINIT, 0, 0, true);

  // see if the card is present and can be initialized:
  if (!SD.begin(CHIPSELECT)) {
    Alarm(&sAlarms, SALSD, ALARMSET);
#ifdef DEBUGMODE
    printMonit(STR_SDFAILED);
#endif
    lcdMonit(STR_SDFAILED, 0, 0, true);
  } else {
#ifdef DEBUGMODE
    printMonit(STR_SDOK);
#endif
    lcdMonit(STR_SDOK, 0, 0, true);
  }
  delay(700);
  
#ifdef DEBUGMODE
  printMonit(STR_SENSINIT);
#endif
  lcdMonit(STR_SENSINIT, 0, 0, true);

  // Start up the library
  sensors.begin();

  // set the resolution
  for(i = 0; i < SENSOR_COUNT; i++) {
    sensors.setResolution(Thermometer[i], TEMPERATURE_PRECISION);
  }

  delay(700);

  lcd.clear();
  lcdMonit(STR_INITDONE, 0, 0, true);

  delay(1000);

  lcdPrint(heaterString(), HEATERCOL, HEATERROW, true);
  tAlarmPrint();
  sAlarmPrint();
}

void lcdMonit(int strIdx, byte row, byte col, bool clr) {
  if(clr)
    lcd.clear();
  lcd.setCursor(row, col);
  strcpy_P(buffer, (char*)pgm_read_word(&(string_table[strIdx])));
  lcd.print(buffer);
}

void lcdPrint(char* msg, byte col, byte row, bool clr) {
  if(clr)
    lcd.clear();
  lcd.setCursor(col, row);
  lcd.print(msg);
}

#ifdef DEBUGMODE

void printMonit(int strIdx) {
  strcpy_P(buffer, (char*)pgm_read_word(&(string_table[strIdx])));
#if defined(A_UNO)
  monitor.println(buffer);
#else
  Serial.println(buffer);
#endif
}

void printAddress(DeviceAddress deviceAddress)
{
  for (byte i = 0; i < 8; i++) {
    // zero pad the address if necessary
    if (deviceAddress[i] < 16)
#if defined(A_UNO)
      monitor.print("0");
#else
      Serial.print("0");
#endif

#if defined(A_UNO)
    monitor.print(deviceAddress[i], HEX);
#else
    Serial.print(deviceAddress[i], HEX);
#endif
  }
}

// function to print the temperature for a device
// Only in debug mode
void printTemperature(DeviceAddress deviceAddress)
{
  float tempC = sensors.getTempC(deviceAddress);

  strcpy_P(buffer, (char*)pgm_read_word(&(string_table[STR_TEMPC])));
#if defined(A_UNO)
  monitor.print(buffer);
  monitor.print(tempC);
#else
  Serial.print(buffer);
  Serial.print(tempC);
#endif
}

// main function to print information about a device
void printData(DeviceAddress deviceAddress)
{
  strcpy_P(buffer, (char*)pgm_read_word(&(string_table[STR_DEVADDR])));
#if defined(A_UNO)
  monitor.print(buffer);
#else
  Serial.print(buffer);
#endif
  printAddress(deviceAddress);
#if defined(A_UNO)
  monitor.print(" ");
#else
  Serial.print(" ");
#endif
  printTemperature(deviceAddress);
#if defined(A_UNO)
  monitor.println();
#else
  Serial.println();
#endif
}

#endif

void updateTemp(char* fields){
  strcpy_P(buffer2, (char*)pgm_read_word(&(string_table[STR_CIPSTART])));
  sendDebug1(buffer2);
  delay(2000);
  strcpy_P(buffer2, (char*)pgm_read_word(&(string_table[STR_ERROR])));
#if defined(A_UNO)
  if(Serial.find(buffer2)){
#else
  if(Serial1.find(buffer2)){
#endif
    Alarm(&sAlarms, SALWIFI, ALARMSET);
#ifdef DEBUGMODE
    printMonit(STR_RECERR);
#endif
    return;
  }
  strcpy_P(buffer2, (char*)pgm_read_word(&(string_table[STR_CIPSEND])));
#if defined(A_UNO)
  Serial.print(buffer2);
  Serial.println(strlen(fields));
  if(Serial.find(">")){
#else
  Serial1.print(buffer2);
  Serial1.println(strlen(fields));
  if(Serial1.find(">")){
#endif
#ifdef DEBUGMODE
  #if defined(A_UNO)
    monitor.print(">");
    monitor.print(fields);
  #else
    Serial.print(">");
    Serial.print(fields);
  #endif
#endif
#if defined(A_UNO)
    Serial.print(fields);
#else
    Serial1.print(fields);
#endif
  }else{
    strcpy_P(buffer2, (char*)pgm_read_word(&(string_table[STR_CIPCLOSE])));
    sendDebug1(buffer2);
  }
  strcpy_P(buffer2, (char*)pgm_read_word(&(string_table[STR_OK])));
#if defined(A_UNO)
  if(Serial.find(buffer2)){
#else
  if(Serial1.find(buffer2)){
#endif
    Alarm(&sAlarms, SALWIFI, ALARMCLEAR);
#ifdef DEBUGMODE
    printMonit(STR_RECOK);
#endif
  }else{
    Alarm(&sAlarms, SALWIFI, ALARMSET);
#ifdef DEBUGMODE
    printMonit(STR_RECERR);
#endif
  }
}
 
 void sendDebug1(char* cmd){
#ifdef DEBUGMODE
  strcpy_P(buffer3, (char*)pgm_read_word(&(string_table[STR_SEND])));
  #if defined(A_UNO)
    monitor.print(buffer3);
    monitor.println(cmd);
  #else
    Serial.print(buffer3);
    Serial.println(cmd);
  #endif
#endif
#if defined(A_UNO)
  Serial.println(cmd);
#else
  Serial1.println(cmd);
#endif
} 
 
boolean connectWiFi(){
  strcpy_P(buffer, (char*)pgm_read_word(&(string_table[STR_CWMODE])));
#if defined(A_UNO)
  Serial.println(buffer);
#else
  Serial1.println(buffer);
#endif
#ifdef DEBUGMODE
  #if defined(A_UNO)
    monitor.println(buffer);
  #else
    Serial.println(buffer);
  #endif
#endif
  delay(2000);
  strcpy_P(buffer, (char*)pgm_read_word(&(string_table[STR_CWJAP])));
#if defined(A_UNO)
  Serial.println(buffer);
#else
  Serial1.println(buffer);
#endif
#ifdef DEBUGMODE
  #if defined(A_UNO)
    monitor.println(buffer);
  #else
    Serial.println(buffer);
  #endif
#endif
  delay(5000);
  strcpy_P(buffer, (char*)pgm_read_word(&(string_table[STR_OK])));
#if defined(A_UNO)
  if(Serial.find(buffer)){
#else
  if(Serial1.find(buffer)){
#endif
    return true;
  }else{
    return false;
  }
}

void loop(void)
{ 
#ifdef DEBUGMODE
  unsigned long time1, timediff;
#endif
  float temps[SENSOR_COUNT];
  byte i;
  File dataFile;
  tmElements_t tm;

  if (RTC.read(tm)) {
    strcpy(rtcTime, "");
    print2digits(rtcTime, tm.Hour);
    strcat(rtcTime, ":");
    print2digits(rtcTime, tm.Minute);
    strcat(rtcTime, ":");
    print2digits(rtcTime, tm.Second);

    strcpy(rtcDate, ""); 
    print2digits(rtcDate, tm.Day);
    strcat(rtcDate, ".");
    print2digits(rtcDate, tm.Month);
    strcat(rtcDate, ".");
    print2digits(rtcDate, tmYearToCalendar(tm.Year));

#ifdef DEBUGMODE
  #if defined(A_UNO)
    monitor.print(F("Ok, Time = "));
    monitor.print(rtcTime);
    monitor.print(F(", Date (D.M.Y) = "));
    monitor.println(rtcDate);
  #else
    Serial.print(F("Ok, Time = "));
    Serial.print(rtcTime);
    Serial.print(F(", Date (D.M.Y) = "));
    Serial.println(rtcDate);
  #endif
#endif
    lcdPrint(rtcTime, TIMECOL, TIMEROW, false);
    
    Alarm(&sAlarms, SALRTC, ALARMCLEAR);
  } else {
    if (RTC.chipPresent()) {
#ifdef DEBUGMODE
  #if defined(A_UNO)
      monitor.println(F("The DS1307 is stopped.  Please run the SetTime"));
      monitor.println(F("example to initialize the time and begin running."));
      monitor.println();
  #else
      Serial.println(F("The DS1307 is stopped.  Please run the SetTime"));
      Serial.println(F("example to initialize the time and begin running."));
      Serial.println();
  #endif
#endif
    } else {
#ifdef DEBUGMODE
  #if defined(A_UNO)
      monitor.println(F("DS1307 read error!  Please check the circuitry."));
      monitor.println();
  #else
      Serial.println(F("DS1307 read error!  Please check the circuitry."));
      Serial.println();
  #endif
#endif
    }
    Alarm(&sAlarms, SALRTC, ALARMSET);
    //delay(9000);
  }

  lcd.setCursor(CNTCOL, CNTROW);
  if(wifiDelayCnt < 10) {
    lcd.print(" ");
  }
  lcd.print(wifiDelayCnt);
  
  // call sensors.requestTemperatures() to issue a global temperature 
  // request to all devices on the bus

#ifdef DEBUGMODE
  printMonit(STR_REQTEMPC);
  time1 = millis();
#endif

  sensors.requestTemperatures();

#ifdef DEBUGMODE
  timediff = millis() - time1;  
  printMonit(STR_DONE);
  strcpy_P(buffer, (char*)pgm_read_word(&(string_table[STR_CONVTIME])));
  #if defined(A_UNO)
    monitor.print(buffer);
    monitor.println(timediff);
  #else
    Serial.print(buffer);
    Serial.println(timediff);
  #endif
#endif

#ifdef DEBUGMODE
  // print the device information -- debug only
  for(i = 0; i < SENSOR_COUNT; i++)
    printData(Thermometer[i]);
#endif

  for(i = 0; i < SENSOR_COUNT; i++) {
    temps[i] = sensors.getTempC(Thermometer[i]);
    if(i < SENSOR_COUNT - 1) {           // Box 1 to 6 sensors
      if(temps[i] > tempLimits[i] + TEMPMARGIN || temps[i] < tempLimits[i] - TEMPMARGIN)
        Alarm(&tAlarms, i, ALARMSET);
      else
        Alarm(&tAlarms, i, ALARMCLEAR);
    } else {                          // ext temp sensor
      if(abs(temps[i]) > TEMPLIM)
        Alarm(&tAlarms, i, ALARMSET);
      else
        Alarm(&tAlarms, i, ALARMCLEAR);
    }
  }

  heaterControl(temps);
  lcdPrint(heaterString(), HEATERCOL, HEATERROW, false);

  strcpy_P(buffer, (char*)pgm_read_word(&(string_table[STR_DATALOG])));
  if(!SD.exists(buffer)) {
    strcpy_P(buffer, (char*)pgm_read_word(&(string_table[STR_HEADERS])));
  }
  else {
    strcpy(buffer, "");
  }
  
//  strcpy_P(buffer2, (char*)pgm_read_word(&(string_table[STR_DATETIME])));
//  strcat(buffer, buffer2);

  strcat(buffer, rtcDate);
  strcat(buffer, ",");
  strcat(buffer, rtcTime);
  
  // write temperatures to the log
  //
  for(i = 0; i < SENSOR_COUNT; i++) {
    strcat(buffer, ",");
    strcat(buffer, dtostrf(temps[i], -5, 2, buffer2));
  }

  // write heater states to the log
  //
  for(i = 0; i < SENSOR_COUNT - 1; i++) {
    strcat(buffer, ",");
    strcat(buffer, itoa((heater >> i) & 0x01, buffer2, 10));
  }
  
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.

  if(logdata > 0) { // log every 2 * DELAY ms
    logdata = 0;
    strcpy_P(buffer2, (char*)pgm_read_word(&(string_table[STR_DATALOG])));
  
    dataFile = SD.open(buffer2, FILE_WRITE);
    // if the file is available, write to it:
    if (dataFile) {
      dataFile.println(buffer);
      dataFile.close();
      Alarm(&sAlarms, SALSD, ALARMCLEAR);
#ifdef DEBUGMODE
  #if defined(A_UNO)
      monitor.println(buffer);
  #else 
      Serial.println(buffer);
  #endif
#endif
    } else {  // if the file isn't open, pop up an error:
      Alarm(&sAlarms, SALSD, ALARMSET);
#ifdef DEBUGMODE
      printMonit(STR_DATALOGERR);
#endif
    }
  } else {
    logdata++;
  }

  if(bitRead(sAlarms, SALWIFI) == ALARMSET) {
    wifiInit(WIFILOOP);   // try to recover wifi
  }
  // Update ThingsSpeak every minute
  if(wifiDelayCnt >= (THINGSPEAKINTERVAL - 2000) / DELAY && bitRead(sAlarms, SALWIFI) == ALARMCLEAR) {
    wifiDelayCnt = 0;

    strcpy_P(buffer, (char*)pgm_read_word(&(string_table[STR_GET])));
    strcpy_P(buffer2, (char*)pgm_read_word(&(string_table[STR_FIELD])));
    
    for(i = 0; i < SENSOR_COUNT; i++) {
      strcat(buffer, buffer2);
      strcat(buffer, itoa(i + 1, buffer3, 10));
      strcat(buffer, "=");
      strcat(buffer, dtostrf(temps[i], -5, 2, buffer3));
    }
    strcat(buffer, "\r\n");
#ifdef DEBUGMODE
  #if defined(A_UNO)
    monitor.println(buffer);
  #else
    Serial.println(buffer);
  #endif
#endif    
    updateTemp(buffer);
  }

  tAlarmPrint();
  sAlarmPrint();

  if(buzzDelayCnt == 0 && (tAlarms != 0 || sAlarms != 0)) {
    digitalWrite(BUZZERPIN, HIGH);
  }
  if(tAlarms != 0 || sAlarms != 0) {
    buzzDelayCnt++;         // increment counter only if any of the alarms is active
    if(buzzDelayCnt == BUZZERINTERVAL / DELAY)
      buzzDelayCnt = 0;
  } else {
    buzzDelayCnt = 0;
  }
  
  delay(DELAY);

  digitalWrite(BUZZERPIN, LOW);
    
  wifiDelayCnt++;
  cnt++;
}

void heaterControl(float temps[]) {
  byte i;
  bool result;
  
  if((tAlarms & 0x7f) != 0x7f) {
    for(i = 0; i < SENSOR_COUNT - 1; i++) {
      if(temps[i] < (float) tempLimits[i] - HYSTLOW) {
        bitSet(heater, i);
      } else if(temps[i] > (float) tempLimits[i] + HYSTHIGH) {
        bitClear(heater, i);
      }
      // otherwise the value for the heater is unchanged
    }
  } else {          // if all termometers indicate alarm, this may be the connection issue over 1-Wire
    heater = 0;     // it's better to disconnect the heating than to bake the bichos
  }
#ifdef DEBUGMODE
  #if defined(A_UNO)
    monitor.println(heater, BIN);
  #else
    Serial.println(heater, BIN);
  #endif
#endif

  Wire.beginTransmission(HEATERADDR); // transmit to device
  Wire.write(heater);                 // sends one byte
  Wire.endTransmission();             // stop transmitting    

  if (Wire.endTransmission() != 0) {
    Alarm(&sAlarms, SALHEATER, ALARMSET);
  } else {
    Alarm(&sAlarms, SALHEATER, ALARMCLEAR);
  }
}

char* heaterString() {
  byte i, heater2;

  heater2 = heater;
  strcpy(buffer3, "");
  heater2 = heater2 << 2; // shift out two MSBs
  for(i = 0; i < SENSOR_COUNT - 1; i++) {
    if((heater2 & 0x80) > 0) {
      strcat(buffer3, "*");
    } else {
      strcat(buffer3, "_");
    }
    heater2 = heater2 << 1; // get next bit
  }  
  return buffer3;
}

void Alarm(byte* alarmvar, byte bitpos, byte action) {
  if(action == ALARMSET) {
    bitSet(*alarmvar, bitpos);
  } else {
    bitClear(*alarmvar, bitpos);
  }
}

void tAlarmPrint() {
  strcpy(alarmStr, "");
  for(byte i = 0; i <= T7AL; i++) {
    if(bitRead(tAlarms, T7AL - i))
      strcat(alarmStr, "A");
    else
      strcat(alarmStr, "_");
  }
  lcd.setCursor(TALCOL, TALROW);
  lcd.print(alarmStr);
}

void sAlarmPrint() {
  strcpy(alarmStr, "");
  if(bitRead(sAlarms, SALLCD))
    strcat(alarmStr, "L");
  else
    strcat(alarmStr, "l");
  if(bitRead(sAlarms, SALWIFI))
    strcat(alarmStr, "W");
  else
    strcat(alarmStr, "w");
  if(bitRead(sAlarms, SALHEATER))
    strcat(alarmStr, "H");
  else
    strcat(alarmStr, "h");
  if(bitRead(sAlarms, SALSD))
    strcat(alarmStr, "S");
  else
    strcat(alarmStr, "s");
  if(bitRead(sAlarms, SALRTC))
    strcat(alarmStr, "R");
  else
    strcat(alarmStr, "r");
  lcd.setCursor(SALCOL, SALROW);
  lcd.print(alarmStr);  
}

void wifiInit(byte mode) {

//  wifiRST();
  
  strcpy_P(buffer, (char*)pgm_read_word(&(string_table[STR_AT])));
  sendDebug1(buffer);
  
  delay(7000);

  strcpy_P(buffer, (char*)pgm_read_word(&(string_table[STR_OK])));
#if defined(A_UNO)
  if(Serial.find("OK")){
#else
  if(Serial1.find("OK")){
#endif
#ifdef DEBUGMODE
    printMonit(STR_RECOK);
#endif
    if(connectWiFi()) {
      if(mode == WIFISETUP)
        lcdMonit(STR_WIFIOK, 0, 0, true);
      Alarm(&sAlarms, SALWIFI, ALARMCLEAR);
#ifdef DEBUGMODE
      printMonit(STR_WIFIOK);
#endif  
    } else {
      if(mode == WIFISETUP)
        lcdMonit(STR_WIFIERR, 0, 0, true);
      Alarm(&sAlarms, SALWIFI, ALARMSET);
#ifdef DEBUGMODE
      printMonit(STR_WIFIERR);
#endif  
    }
  } else {
    if(mode == WIFISETUP)
      lcdMonit(STR_WIFIERR, 0, 0, true);
    Alarm(&sAlarms, SALWIFI, ALARMSET);
#ifdef DEBUGMODE
    printMonit(STR_WIFIERR);
#endif
  }  
}

void wifiRST() {
  pinMode(WIFIRSTPIN, OUTPUT);
  digitalWrite(WIFIRSTPIN, LOW);
  delay(500);
  digitalWrite(WIFIRSTPIN, HIGH);
  delay(2000);
}

void print2digits(char* buf, int number) {
  if (number >= 0 && number < 10) {
    strcat(buf, "0");
  }
  strcat(buf, itoa(number, buffer3, 10));
}

