
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
 * For UNO it is Serial port, for MEGA it is Serial1.
 * Because on UNO this port is also used by USB to download code and to connect Monitor,
 * so in order to be able to have a debugging monitor a SW Serial via FTDI is used, see below.
 * 
 * Display module attached to I2C via PFC8574 (address 0x39)
 * Heater control attached to I2C via PFC8574 (address 0x38)
 * RTC (DS1307) attached to I2C (address 0x68)
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
 *  LCD information layout:
 *  
 *  Row 0: [Heaters status] [Time]
 *  Row 1: [Temp. alarms]   [System alarms] [ThingSpeak update interval counter]
 *  
 *  Example:
 *  Row 0:  ***__* HH:MM:SS
 *  Row 1: _AAA__A lwhsr 26
 *  Cols:  0123456789012345
 *         0         1 
 *         
 *  Heaters status:
 *    * - heater on
 *    _ - heater off
 *    
 *  Temperature alarms:
 *    _ - No alarm
 *    A - Temperaure out of range alarm
 *    
 *  System alarms:
 *    l - LCD no alarm
 *    L - LCD error
 *    w - WiFi no alarm
 *    W - WiFi error
 *    h - Heaters no alarm
 *    H - Heaters error
 *    s - SD card no alarm
 *    S - SD card error
 *    r - RTC no alarm
 *    R - RTC error
 *    
 * Beeper 
 ** pin 3
 *
 * HW setup
 * ========
 * Main controller Arduino MEGA due to more SRAM memory available. With UNO it was frequent that the data memory and stack
 * started to overwrite each other causing program to crash.
 * 
 * Arduino MEGA controlling:
 *    WiFi ESP8266 module over UART1, using AT commands
 *    LCD display over I2C and PCF8574
 *    Heaters over I2C and PCF8574 and steering heaters through ULN2803A
 *    RTC module over I2C
 *    SD card module over SPI interface
 *    7 DS18B20 One Wire digital termometers
 *    Buzzer over digital out pin
 *    
 * SW structure
 * ============
 * Due to limited data memory size on UNO program memory is used to store a number of string constants. Those constants are used
 * to:
 *  - build AT commands for WiFi module
 *  - provide debug info over Monitor interface
 *  - build info strings send to ThingSpeak
 *  - build data strings for logging to SD card
 *  
 *  Code is writen with preprocessor directives conditional on two #define constants: A_UNO and DEBUGMODE.
 *  If A_UNO is defined it means the program is compiled fro UNO board, consequently as Serial us used to interface WiFi module, to connect monitor
 *  for debugging purposes a SW Serial over FTDI is used. If it is undefined then program is compiled for MEGA board and Serial is available for monitor.
 *  If DEBUGMODE is defined then code printing debug info to the monitor is enabled/included.
 *  
 *  The entire process is handled within the main loop function. See its comments for details.
 *  
 *  The SQW-triggered interrupt, occurring every 1 second is used to update the time display on the LCD.
 *  
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

#include <Time.h>
#include <TimeLib.h>


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
byte heater = 0;  // global var sent to PCF controlling the state of the heaters. Each bit controls one heater. 1 - heater on, 0 - heater off

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
#define SENSOR_COUNT 7  // six boxes and the external temp sensor

byte tAlarms = 0;   // temperature alarms for the heater thermometers (0-5)

#define T7AL 6      // bit number for external temperature sensor alarm

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

// RTC stuff
//
#define RTCADDR    0x68
#define CONTROLREG 0x07
#define RTC_SQWE      4
#define SQ_1HZ     0x00
#define SQW_INT_PIN   3   // SQ will drive the INT1 interrupt (pin 3)


char rtcTime[9], rtcDate[11];    // updated in the int_sqw() interrupt routine

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

byte sAlarms = 0;      // system alarms flags. Updated also in the int_sqw() interrupt routine. This is why the Alarm() is surrounded by cli/sei
                       // Bits:
                       // 4 - LCD
                       // 3 - WiFi
                       // 2 - Heaters
                       // 1 - SD
                       // 0 - RTC
char alarmStr[8];

byte wifiDelayCnt = 0; // counter used to count 1 minute intervals to update ThingSpeak
byte buzzDelayCnt = 0; // counter used to count 29s of pause between alarm beeps.



//*****************************************************************
//      ##   #####  #####  #   #  ####
//     #  #  #        #    #   #  #   #
//      #    #        #    #   #  #   #
//       #   ###      #    #   #  ####
//        #  #        #    #   #  #
//    #  #   #        #    #   #  #
//     ##    #####    #     ###   #
//*****************************************************************

void setup(void)
{
  byte i = 0;
  //****************************Buzzer*****************************
  // Setup Buzzer pin
  //
  pinMode(BUZZERPIN, OUTPUT);
  digitalWrite(BUZZERPIN, LOW);
  
  //****************************Serial****************************
  // start serial port
  // Used for communication with WiFi module
  //
  Serial.begin(115200);   // MEGA - USB, Monitor; UNO - USB & WIFI
#if !defined(A_UNO)
  Serial1.begin(115200);  // MEGA - WIFI
#endif

#ifdef DEBUGMODE
  #ifdef A_UNO
    monitor.begin(57600); // UNO - Monitor
  #endif
#endif

  //****************************LCD*******************************
  //
  // initialize LCD with number of columns and rows: 
  //
#ifdef DEBUGMODE
  printMonit(STR_LCDINIT);
#endif
  Wire.begin();
  Wire.beginTransmission(LCDADDR);

  if (Wire.endTransmission() != 0) {
    cli();
    Alarm(&sAlarms, SALLCD, ALARMSET);
    sei();
#ifdef DEBUGMODE
    printMonit(STR_LCDFAILED);
#endif
  }

  lcd.begin(LCD_COLS, LCD_ROWS); // initialize the lcd
  
  // Print a message to the LCD
  lcdMonit(STR_START, 0, 0, true);
  delay(700);

  //*****************************RTC SQW***************************
  // Init SQW on RTC
  //
  RTCSquareWave(SQ_1HZ);  // Initialize 1 Hz square wave output from RTC, so that it can drive the external interrupt

  //*****************************Heaters***************************
  // Init Heaters
  //
#ifdef DEBUGMODE
  printMonit(STR_HEATERINIT);
#endif
  Wire.begin();
  Wire.beginTransmission(HEATERADDR);

  if (Wire.endTransmission() != 0) {
    cli();
    Alarm(&sAlarms, SALHEATER, ALARMSET);
    sei();
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

  //******************************WIFI****************************
  // Initialize and conenct wifi
  //
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

  //*****************************SD Card***************************
  // see if the card is present and can be initialized:
  //
  if (!SD.begin(CHIPSELECT)) {
    cli();
    Alarm(&sAlarms, SALSD, ALARMSET);
    sei();
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

  //**************************Thermometers*************************
  // Init temperature sensors
  //
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

  //************************INIT COMPLETE***************************
  // Init completed, print information to LCD
  //
  lcd.clear();
  lcdMonit(STR_INITDONE, 0, 0, true);

  delay(1000);

  //*****************Display status info to LCD*********************
  // Display temp and system alarm information
  //
  lcdPrint(heaterString(), HEATERCOL, HEATERROW, true);
  tAlarmPrint();
  sAlarmPrint();

  //*************************RTC SQW INT init***********************
  // Init interrupt driven by RTC SQ output
  //
  pinMode(SQW_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SQW_INT_PIN), int_sqw, FALLING);

}

//*****************************************************************
//                     #       ###    ###   ####
//                     #      #   #  #   #  #   #
//                     #      #   #  #   #  #   #
//                     #      #   #  #   #  ####
//                     #      #   #  #   #  #
//                     #      #   #  #   #  #
//                     #####   ###    ###   #
//
//*****************************************************************
//
// loop tasks:
//  - increment and print wifi update delay counter
//  - read temperatures
//  - raise temp alarms if temperatres outside allowed ranges
//  - update heater controls
//  - read date and time
//  - log temps and heaters status to file on SD every 2 seconds
//  - Update ThingSpeak every minute
//  - update alarm information on LCD
//  - activate buzzer if any alarm active
//
//  The loop function is timed to be spaced each run at least 1s apart
//*****************************************************************
void loop(void)
{ 
#ifdef DEBUGMODE
  unsigned long time1, timediff;
#endif
  float temps[SENSOR_COUNT];
  byte i;
  File dataFile;

  //*****************************************************************
  // Update wifi delay counter on LCD
  //
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

  //*****************************************************************
  // initiate temps reading
  //
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

  //*****************************************************************
  // Read temps and update temp alarm statuses
  //
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

  //*****************************************************************
  // update heater controls in function of the temp readings
  //
  heaterControl(temps);
  lcdPrint(heaterString(), HEATERCOL, HEATERROW, false);

  //*****************************************************************
  // build strings for data logging
  //
  strcpy_P(buffer, (char*)pgm_read_word(&(string_table[STR_DATALOG])));
  if(!SD.exists(buffer)) {
    strcpy_P(buffer, (char*)pgm_read_word(&(string_table[STR_HEADERS])));
  }
  else {
    strcpy(buffer, "");
  }
  
//  strcpy_P(buffer2, (char*)pgm_read_word(&(string_table[STR_DATETIME])));
//  strcat(buffer, buffer2);

  //*****************************************************************
  // Read date and time
  //
  cli();
  strcat(buffer, rtcDate);
  sei();
  strcat(buffer, ",");
  cli();
  strcat(buffer, rtcTime);
  sei();
  
  //*****************************************************************
  // prepare temperatures for the log
  //
  for(i = 0; i < SENSOR_COUNT; i++) {
    strcat(buffer, ",");
    strcat(buffer, dtostrf(temps[i], -5, 2, buffer2));
  }

  //*****************************************************************
  // prepare heater states for the log
  //
  for(i = 0; i < SENSOR_COUNT - 1; i++) {
    strcat(buffer, ",");
    strcat(buffer, itoa((heater >> i) & 0x01, buffer2, 10));
  }
  
  //*****************************************************************
  // Write data to the log. Every 2 seconds, approximately.
  // open the file. Note that only one file can be open at a time,
  // so you have to close this one before opening another.
  //
  if(logdata > 0) { // log every 2 * DELAY ms
    logdata = 0;
    strcpy_P(buffer2, (char*)pgm_read_word(&(string_table[STR_DATALOG])));
  
    dataFile = SD.open(buffer2, FILE_WRITE);
    // if the file is available, write to it:
    if (dataFile) {
      dataFile.println(buffer);
      dataFile.close();
      cli();
      Alarm(&sAlarms, SALSD, ALARMCLEAR);
      sei();
#ifdef DEBUGMODE
  #if defined(A_UNO)
      monitor.println(buffer);
  #else
      Serial.println(buffer);
  #endif
#endif
    } else {  // if the file isn't open, pop up an error:
      cli();
      Alarm(&sAlarms, SALSD, ALARMSET);
      sei();
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
  
  //*****************************************************************
  // Update ThingsSpeak every minute
  //
  if(wifiDelayCnt >= (THINGSPEAKINTERVAL - 2000) / DELAY && bitRead(sAlarms, SALWIFI) == ALARMCLEAR) {
    wifiDelayCnt = 0;

    // Build update string. String format;
    // "GET /update?key=8ABPRZYZOPS96160&field0=temp0&field1=temp1&field2=temp2&field3=temp3&field4=temp4&field5=temp5&field6=temp6&field7=temp7\r\n"
    //
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
    updateTemp(buffer);       // Takes the payload string, initiates the trasmission and sends it
  }


  //*****************************************************************
  // Update alarm info on LCD
  //
  tAlarmPrint();
  sAlarmPrint();
  
  //*****************************************************************
  // If any alarm active activate buzzer for 1 second every 30 seconds
  //
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
// End of loop function


//************************************************************************* LCD routines ******************

//**************************** lcdMonit ***************************
// Print the progress info to the LCD.
// Used during initialization phase. The index to a string to be
// printed is passed as an argument. String is located in progmem
//
void lcdMonit(int strIdx, byte row, byte col, bool clr) {
  if(clr)
    lcd.clear();
  lcd.setCursor(row, col);
  strcpy_P(buffer, (char*)pgm_read_word(&(string_table[strIdx])));
  lcd.print(buffer);
}

//**************************** lcdPrint ***************************
// A general LCD print routine. String to be printed is passed as
// an argument.
//
void lcdPrint(char* msg, byte col, byte row, bool clr) {
  if(clr)
    lcd.clear();
  lcd.setCursor(col, row);
  lcd.print(msg);
}





#ifdef DEBUGMODE //************************************************ DEBUG code start ****************

//**************************** printMonit ***************************
// Print a diagnostic string to serial monitor. String index is passed
// as an argument.
//
void printMonit(int strIdx) {
  strcpy_P(buffer, (char*)pgm_read_word(&(string_table[strIdx])));
#if defined(A_UNO)
  monitor.println(buffer);
#else
  Serial.println(buffer);
#endif
}

//******************* Print temp sensors info ***********************
// Three funtions used to print the sensors reading and addresses to the
// serial monitor:
//      printData
//          printAddress
//          printTemperature
//
//************************* printAddress ****************************
// 
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

#if defined(A_UNO)                              // merge with above
    monitor.print(deviceAddress[i], HEX);
#else
    Serial.print(deviceAddress[i], HEX);
#endif
  }
}

//********************* printTemperature ***************************
// Function to print the temperature for a device
// Only in debug mode
//
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

//**************************** printData ***************************
// main function to print information about a device
//
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

#endif    //************************************************ DEBUG code end ******************



//************************************************************************* WiFi routines *****************


//************************** wifiInit ********************************
// Initialize Wifi module
//********************************************************************

void wifiInit(byte mode) {

//  wifiRST();
  
  strcpy_P(buffer, (char*)pgm_read_word(&(string_table[STR_AT])));
  sendWiFi(buffer);
  
  delay(5000);

  strcpy_P(buffer, (char*)pgm_read_word(&(string_table[STR_OK])));
#if defined(A_UNO)
  if(Serial.find((char*)"OK")){
#else
  if(Serial1.find((char*)"OK")){
#endif
#ifdef DEBUGMODE
    printMonit(STR_RECOK);
#endif
    if(connectWiFi()) {
      if(mode == WIFISETUP)
        lcdMonit(STR_WIFIOK, 0, 0, true);
      cli();
      Alarm(&sAlarms, SALWIFI, ALARMCLEAR);
      sei();
#ifdef DEBUGMODE
      printMonit(STR_WIFIOK);
#endif  
    } else {
      if(mode == WIFISETUP)
        lcdMonit(STR_WIFIERR, 0, 0, true);
      cli();
      Alarm(&sAlarms, SALWIFI, ALARMSET);
      sei();
#ifdef DEBUGMODE
      printMonit(STR_WIFIERR);
#endif  
    }
  } else {
    if(mode == WIFISETUP)
      lcdMonit(STR_WIFIERR, 0, 0, true);
    cli();
    Alarm(&sAlarms, SALWIFI, ALARMSET);
    sei();
#ifdef DEBUGMODE
    printMonit(STR_WIFIERR);
#endif
  }  
}


//*************************** wifiRST ********************************
// HW-reset Wifi module
//********************************************************************

void wifiRST() {
  pinMode(WIFIRSTPIN, OUTPUT);
  digitalWrite(WIFIRSTPIN, LOW);
  delay(500);
  digitalWrite(WIFIRSTPIN, HIGH);
  delay(2000);
}


//*************************** connectWiFi ****************************
// Connect Wifi module to the AP/Router
//********************************************************************

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


//****************************** sendWiFi ****************************
// Send buffer to Wifi module
//********************************************************************

void sendWiFi(char* cmd){
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
 

//**************************** updateTemp ***************************
//
// Update temp on ThingSpeak over WiFi
//
//********************************************************************

void updateTemp(char* fields){
  strcpy_P(buffer2, (char*)pgm_read_word(&(string_table[STR_CIPSTART])));
  sendWiFi(buffer2);                                                    // Initiate transmission
  delay(2000);
  strcpy_P(buffer2, (char*)pgm_read_word(&(string_table[STR_ERROR])));    // Check for possible ERROR answer from WiFi module
#if defined(A_UNO)
  if(Serial.find(buffer2)){
#else
  if(Serial1.find(buffer2)){
#endif
    cli();
    Alarm(&sAlarms, SALWIFI, ALARMSET);                                    // If ERROR at transmission start - raise an alarm
    sei();
#ifdef DEBUGMODE
    printMonit(STR_RECERR);
#endif
    return;
  }
  strcpy_P(buffer2, (char*)pgm_read_word(&(string_table[STR_CIPSEND])));    // If OK build and send the temps update string
#if defined(A_UNO)
  Serial.print(buffer2);
  Serial.println(strlen(fields));
  if(Serial.find((char*)">")){
#else
  Serial1.print(buffer2);
  Serial1.println(strlen(fields));
  if(Serial1.find((char*)">")){
#endif
#ifdef DEBUGMODE
  #if defined(A_UNO)
    monitor.print((char*)">");
    monitor.print(fields);
  #else
    Serial.print((char*)">");
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
    sendWiFi(buffer2);
  }
  strcpy_P(buffer2, (char*)pgm_read_word(&(string_table[STR_OK])));
#if defined(A_UNO)
  if(Serial.find(buffer2)){
#else
  if(Serial1.find(buffer2)){
#endif
    cli();
    Alarm(&sAlarms, SALWIFI, ALARMCLEAR);
    sei();
#ifdef DEBUGMODE
    printMonit(STR_RECOK);
#endif
  }else{
    cli();
    Alarm(&sAlarms, SALWIFI, ALARMSET);
    sei();
#ifdef DEBUGMODE
    printMonit(STR_RECERR);
#endif
  }
}
 


//*********************************************************************** Heater routines *****************

//****************************** heaterControl ************************
// 
void heaterControl(float temps[]) {
  byte i;
  bool result;
  Serial.print("tAlarms: ");
  Serial.println(tAlarms & 0x7f, HEX);
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
    heater = 0;    // it's better to disconnect the heating than to bake the bichos
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
    cli();
    Alarm(&sAlarms, SALHEATER, ALARMSET);
    sei();
  } else {
    cli();
    Alarm(&sAlarms, SALHEATER, ALARMCLEAR);
    sei();
  }
}

//****************************** heaterString *************************
// 
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



//************************************************************************ Alarm routines *****************


//****************************** Alarm ********************************
// 
void Alarm(byte* alarmvar, byte bitpos, byte action) {
  if(action == ALARMSET) {
    bitSet(*alarmvar, bitpos);
  } else {
    bitClear(*alarmvar, bitpos);
  }
}


//****************************** tAlarmPrint **************************
// 
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

//****************************** sAlarmPrint **************************
// 
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


//************************************************************************** RTC routines *****************

//************************ RTCSquareWave *****************************
// Program the RTC's SQW output
//
void RTCSquareWave(byte rate) {
  byte ctrlreg;

#ifdef DEBUGMODE
  #if defined(A_UNO)
      monitor.println(F("RTC: Initializing SQW"));
  #else
      Serial.println(F("RTC: Initializing SQW"));
  #endif
#endif

   Wire.beginTransmission(RTCADDR);
   Wire.write(CONTROLREG);
   if(Wire.endTransmission() != 0) {
     cli();
     Alarm(&sAlarms, SALRTC, ALARMSET);
     sei();
#ifdef DEBUGMODE
  #if defined(A_UNO)
      monitor.println(F("RTC: Failed to address control register"));
  #else
      Serial.println(F("RTC: Failed to address control register"));
  #endif
#endif
     return;
   }
   Wire.requestFrom(RTCADDR, 1);
   ctrlreg = Wire.read();
   bitSet(ctrlreg, RTC_SQWE);
   rate |= 0xFC;              // set bits 3-7 to 1
   ctrlreg &= rate;           // set RS1, RS0 to the value requested by the parameter

   // put the value of the register back
   Wire.beginTransmission(RTCADDR);
   Wire.write(CONTROLREG);
   Wire.write(ctrlreg);
   if(Wire.endTransmission() != 0) {
     cli();
     Alarm(&sAlarms, SALRTC, ALARMSET);
     sei();
#ifdef DEBUGMODE
  #if defined(A_UNO)
      monitor.println(F("RTC: Failed to write to control register"));
  #else
      Serial.println(F("RTC: Failed to write to control register"));
  #endif
#endif
     return;
   }
}


//**************************************************************** SQW Interrupt routine ******************

//*************************** int_sqw *********************************
// SQW-triggered interrupt handling routine
// triggered every 1 second
//
void int_sqw() {
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

    lcdPrint(rtcTime, TIMECOL, TIMEROW, false);
    Alarm(&sAlarms, SALRTC, ALARMCLEAR);
  } else {      // time not set
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
    } else {      // don't see the RTC module
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
    Alarm(&sAlarms, SALRTC, ALARMSET);      // Display RTC alarm if time not set
  }
}


//****************************** print2digits *************************
// 
void print2digits(char* buf, int number) {
  if (number >= 0 && number < 10) {
    strcat(buf, "0");
  }
  strcat(buf, itoa(number, buffer3, 10));
}

