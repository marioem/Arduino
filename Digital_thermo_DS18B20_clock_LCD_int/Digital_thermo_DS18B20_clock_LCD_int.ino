/******************************************************************************
 * Digital thermometer and clock using LCD display and DS18B20 as a sensor.
 * Display is an LCD 16x2 character display in 4-bit mode.
 *
 * Temperature sampling and display is handled via Timer 1 interrupt (generated every 1 s).
 *
 * Clock works in 24h mode. It is possible to switch on/off beep at o'clock hour.
 * 
 * Timer 1 prescaler setting: 1024
 * Timer 1 compare match register: 15624.
 * 
 * (C) 2014, 2015 Mariusz Musiał
 *
 * 2015.01.18  Added LCD backlight brightness control in function of ambient light intensity (LDR-based)
 ******************************************************************************/

// Definition of pins to which clock adjustment buttons are connected
#define SETPIN     8
#define UPPIN      7

// Define the analog input pin for reading ambient light level
#define LDR        0
#define LDRMIN   200               // lower limit of LDR analog reading, will result in lowest brightness of LCD
#define LDRMAX   650               // higer limit of LDR analog reading, will result in highest brightness of LCD
#define LDRHYST   10               // Hysteresis value for LDR ambient light reading
#define BLMIN     32               // PWM value for min backlight brightness
#define BLMAX    255               // PWM value for max backlight brightness

#define LCDBL      6               // PWM pin cntrolling the LCD backlight brightness

// define clock setup modes (states)
#define SMIDLE     0
#define SMTHOUR    1               // setting tens of hours
#define SMUHOUR    2               // setting units of hours
#define SMTMINUTE  3               // setting tens of minutes
#define SMUMINUTE  4               // setting units of minutes

#define SEC        2
#define MIN        1
#define HOUR       0

#define CLOCKSTARTCOL 2            // first column for clock display

// Definition of buttons pressed
#define BNONE      0               // no button pressed
#define BSETUP     1               // SETUP button pressed
#define BUP        2               // UP button pressed

#define BEEP      13               // the onboard LED pin
#define BEEPTIME 200               // time in ms for beep function

#define ONEWIREBUS 10              // pin number where the OneWire device is connected to

#include <math.h>
#include <OneWire.h>
#include <Wire.h>                  // required by New LiquidCrystal
#include <LiquidCrystal.h>         // New LiquidCrystal

// DS18B20 Temperature chip i/o
OneWire ds(ONEWIREBUS);            // on pin ONEWIREBUS
// global variables for handling DS18B20 device
byte addr[8];                      // array for holding the device address on the OneWire bus
boolean dsfound = false;
boolean startConversion = true;    // if set then the conversion is called, otherwise readout. It is used in the timer 1 interrupt routine

// clock variables
byte clock[3] = {
  0,0,0};                          // array holding curent time for display: hour, minute, seconds (in binary)
byte tmpClock[3] = {
  0,0,0};
boolean beepflag = false;          // flags if sharp hour beeps or not
boolean notestate = false;         // note displayed in off state

// global variables for handling clock setup/adjustmen
byte setupMode = SMIDLE;           // state variable for handling time addjustment procedure
char timeString[] = {
  "00:00:00"};                     // here the string for current time is constructed
int buttonSetupState;              // the current reading from the input pin
int lastButtonSetupState = LOW;    // the previous reading from the input pin
int buttonUpState;                 // the current reading from the input pin
int lastButtonUpState = LOW;       // the previous reading from the input pin
long lastSetupDebounceTime = 0;    // the last time the output pin for Setuo button was toggled
long lastUpDebounceTime = 0;       // the last time the output pin for Up button was toggled
long debounceDelay = 50;           // the debounce time; increase if the output flickers

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
//                 |   |  |  |  |  |
//                 |   |  |  |  |  \-- D7
//                 |   |  |  |  \----- D6
//                 |   |  |  \-------- D5
//                 |   |  \----------- D4
//                 |   \-------------- Enable
//                 \------------------ RS

byte noteon[8] = {
  0b00100,0b00110,0b00101,0b00101,0b00100,0b11100,0b11100,0b00000};
byte noteoff[8] = {
  0b00100,0b00110,0b10101,0b01101,0b00110,0b11101,0b11100,0b00000};

int currentLDR, lastLDR;           // reading of the ambient light value. last is updated only if current is greater or lower more than hysteresis value

void setup() {

  // Serial.begin(9600);

  // Configure pins for setting the clock
  pinMode(BEEP, OUTPUT);
  digitalWrite(BEEP, LOW);         // switch off the onboard LED

  pinMode(SETPIN,INPUT);           // button for entering and cycling through setup modes
  pinMode(UPPIN,INPUT);            // button for scrolling up values of clock digits

  pinMode(LDR,INPUT);              // analog in for LDR divider
  pinMode(LCDBL,OUTPUT);           // PWM control of LCD backlight brightness
  
  lastLDR = analogRead(LDR);
  // clamp the reading
  lastLDR = lastLDR < LDRMIN ? LDRMIN : lastLDR;
  lastLDR = lastLDR > LDRMAX ? LDRMAX : lastLDR;
  // set initial value of backlight brightness
  analogWrite(LCDBL,map(lastLDR,LDRMIN,LDRMAX,BLMIN,BLMAX));

  cli();                           //stop interrupts

  //set timer1 interrupt at 1Hz
  TCCR1A = 0;                      // set entire TCCR1A register to 0
  TCCR1B = 0;                      // same for TCCR1B
  TCNT1  = 0;                      //initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 15624;                   // = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  sei();                          //allow interrupts

  // look for DS18B20 device
  ds.reset_search();
  if (ds.search(addr)) {
    if (OneWire::crc8(addr, 7) == addr[7])
      if (addr[0] == 0x28)
        dsfound = true;
  }
  else
    ds.reset_search();

  // set up the LCD's number of columns and rows: 
  lcd.begin(16, 2);

  lcd.createChar (0, noteon);     // load character to the LCD
  lcd.createChar (1, noteoff);    // load character to the LCD  

  lcd.setCursor(0,0);
  lcd.print(char(1));             // print note off
}

void loop() {
  byte button = BNONE;

  if(!dsfound) {                   // if DS18B20 was not foud at setup try to find it here
    ds.reset_search();
    if (ds.search(addr)) {
      if (OneWire::crc8(addr, 7) == addr[7])
        if (addr[0] == 0x28)
          dsfound = true;
    }
    else
      ds.reset_search();
  }

  currentLDR = analogRead(LDR);
  // clamp the reading
  currentLDR = currentLDR < LDRMIN ? LDRMIN : currentLDR;
  currentLDR = currentLDR > LDRMAX ? LDRMAX : currentLDR;
  
  if((lastLDR - currentLDR) > LDRHYST || (currentLDR - lastLDR) > LDRHYST)
    lastLDR = currentLDR;

  // check if the any of the clock adjustment buttons is pressed
  button = readClockButtons(SETPIN, UPPIN);
  if(button == BSETUP)                    // if SETUP button is pressed
  {
    setupMode += 1;                       // cycle trough setup states
    if(setupMode > SMUMINUTE)             // if we cycled through all states
    {
      setupMode = SMIDLE;                 // we return do idle
      clock[HOUR] = tmpClock[HOUR];       // and copy adjusted clock values to main clock array
      clock[MIN] = tmpClock[MIN];         // and copy adjusted clock values to main clock array
      clock[SEC] = 0;                     // reset seconds counter
      lcd.noBlink();
    }
    if(setupMode != SMIDLE)               // probably it could be 'else' here; if not idle state
    {
      clockSetup(setupMode);              // prepare for adjusting clock digit
    }
  }
  if(button == BUP)                       // if UP button is pressed
  {
    if(setupMode != SMIDLE)               // and we are in the adjustment mode
    {
      clockAdjust(setupMode);             // cycle through allowed values at each button press
    }
    else                                  // beep is toggled outside normal adjust mode
    {
      lcd.setCursor(0,0);
      if(notestate)
      {
        notestate = false;
        lcd.print(char(1));
      }
      else
      {
        notestate = true;
        lcd.print(char(0));
        beep();
      }
    }
  }

  if(beepflag && notestate)               // if beep-on-sharp is on (notestate = true) and timer 1 ISR orders beep
  {
    beep();
    beepflag = false;
  }
}

byte readClockButtons(byte setupPin, byte upPin) {
  byte buttonPressed = BNONE;
  byte testSetup, testUp;

  testSetup = digitalRead(setupPin);
  // check to see if you just pressed the button 
  // (i.e. the input went from LOW to HIGH),  and you've waited 
  // long enough since the last press to ignore any noise:  

  // If the switch changed, due to noise or pressing:
  if (testSetup != lastButtonSetupState)
  {
    // reset the debouncing timer
    lastSetupDebounceTime = millis();
  } 

  if ((millis() - lastSetupDebounceTime) > debounceDelay)
  {
    // whatever the reading is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:

    // if the button state has changed:
    if (testSetup != buttonSetupState)
    {
      buttonSetupState = testSetup;

      if (buttonSetupState == HIGH)
        buttonPressed = BSETUP;
      else
        buttonPressed = BNONE;
    }
  }
  lastButtonSetupState = testSetup;

  if(buttonPressed == BNONE)
  {
    testUp = digitalRead(upPin);
    if (testUp != lastButtonUpState)
    {
      // reset the debouncing timer
      lastUpDebounceTime = millis();
    } 

    if ((millis() - lastUpDebounceTime) > debounceDelay)
    {
      // whatever the reading is at, it's been there for longer
      // than the debounce delay, so take it as the actual current state:

      // if the button state has changed:
      if (testUp != buttonUpState)
      {
        buttonUpState = testUp;

        if (buttonUpState == HIGH)
          buttonPressed = BUP;
        else
          buttonPressed = BNONE;
      }
    }
    lastButtonUpState = testUp;
  }

  return buttonPressed;
}

void clockSetup(byte setupmode) {
  int col;

  switch(setupmode) {
  case SMTHOUR:              // we are beginning adjustment
    for(int i = 0; i <= SEC; i++)
      tmpClock[i] = clock[i];
    col = CLOCKSTARTCOL;
    break;
  case SMUHOUR:              // during addjustment the temp time is displayed and addusted
    col = CLOCKSTARTCOL + 1;
    break;
  case SMTMINUTE:
    col = CLOCKSTARTCOL + 3;
    break;
  case SMUMINUTE:
    col = CLOCKSTARTCOL + 4;
    break;
  default:
    break;
  }
  lcd.setCursor(col,0);
  lcd.blink();
}

void clockAdjust(byte setupmode) {
  int thour, tmin, col;

  switch(setupmode) {
  case SMTHOUR:
    tmpClock[HOUR] += 10;                           // ----- this method clears hour units which may be present in clock
    if(tmpClock[HOUR] > 20)
      tmpClock[HOUR] = 0;
    col = CLOCKSTARTCOL;                            // col holds column number where the adjustment blinking cursor will be displayed
    break;
  case SMUHOUR:
    thour = tmpClock[HOUR] / 10;                    // remember tens of hour
    tmpClock[HOUR] = tmpClock[HOUR] % 10;           // leave only units of hour in temp variable
    tmpClock[HOUR] += 1;                            // we are working on units position, so increment units
    if(thour == 2)                                  // if tens of hour is 20, then units are limited to range 0-3
      if(tmpClock[HOUR] > 3)
        tmpClock[HOUR] = 0;
      else if(tmpClock[HOUR] > 9)                   // otherwise units are in the range of 0-9
          tmpClock[HOUR] = 0;
    tmpClock[HOUR] = 10 * thour + tmpClock[HOUR];   // recreate the full value of hour with new value of units
    col = CLOCKSTARTCOL + 1;
    break;
  case SMTMINUTE:
    tmpClock[MIN] += 10;                            // ----- this method clears minute units which may be present in clock
    if(tmpClock[MIN] > 50)
      tmpClock[MIN] = 0;
    col = CLOCKSTARTCOL + 3;
    break;
  case SMUMINUTE:
    tmin = tmpClock[MIN] / 10;                      // remember tens of minute
    tmpClock[MIN] = tmpClock[MIN] % 10;             // leave only units of minute in temp variable
    tmpClock[MIN] += 1;                             // we are working on units position, so increment units
    if(tmpClock[MIN] > 9)                           // otherwise units are in the range of 0-9
        tmpClock[MIN] = 0;
    tmpClock[MIN] = 10 * tmin + tmpClock[MIN];      // recreate the full value of hour with new value of units
    col = CLOCKSTARTCOL + 4;
    break;
  default:
    break;   
  }
  timeString[6] = (tmpClock[SEC] / 10) + '0';
  timeString[7] = (tmpClock[SEC] % 10) + '0';
  timeString[3] = (tmpClock[MIN] / 10) + '0';
  timeString[4] = (tmpClock[MIN] % 10) + '0';
  timeString[0] = (tmpClock[HOUR] / 10) + '0';
  timeString[1] = (tmpClock[HOUR] % 10) + '0';
  lcd.setCursor(CLOCKSTARTCOL,0);
  lcd.print(timeString);
  lcd.setCursor(col,0);
  lcd.blink();
}

ISR(TIMER1_COMPA_vect) {

  // This function is called every second
  if(!setupMode)              // if clock setting is not in progress
  {
    clock[SEC] += 1;
    if(clock[SEC] == 60) {
      clock[SEC] = 0;
      clock[MIN] += 1;
      if(clock[MIN] == 60){
        clock[MIN] = 0;
        clock[HOUR] += 1;
        if(clock[HOUR] == 24)
          clock[HOUR] = 0;
      }
    }

    if((clock[MIN] == 0) && (clock[SEC] == 0) && notestate)  // if it's xx o'clock sharp and beep mode is on
      beepflag = true;                                       // order beep outside interrupt routine - beep requires delay

    timeString[6] = (clock[SEC] / 10) + '0';
    timeString[7] = (clock[SEC] % 10) + '0';
    timeString[3] = (clock[MIN] / 10) + '0';
    timeString[4] = (clock[MIN] % 10) + '0';
    timeString[0] = (clock[HOUR] / 10) + '0';
    timeString[1] = (clock[HOUR] % 10) + '0';

    // After updating time, the display of required data is handled.
    lcd.setCursor(CLOCKSTARTCOL,0);
    lcd.print(timeString);  // display mem is only written when the temperature is read
    
    // set LCD backlight brightness according to last LDR reading
    analogWrite(LCDBL,map(lastLDR,LDRMIN,LDRMAX,BLMIN,BLMAX));

    if(dsfound) {                                  // if DS18B20 has been found on the bus
      if(startConversion) {                        // and it's time for requesting conversion of temperature
        startDSConversion();                       // send conversion order to the device
        startConversion = false;                   // and change state to temperature readout
      }
      else                                         // it's time to read the temperature from the sensor
      {
        lcd.setCursor(2,1);
        lcd.print(readTempDS(),4);                 // read it and display
        lcd.print((char)223);
        lcd.write('C');
        startConversion = true;                    // and change state to temperature conversion
      }
    }
    else                                           // otherwise, if device is not found
    { 
      lcd.setCursor(0,1);
      lcd.print("Missing sensor");   // display indication of missing temperature
    }
  }
}


/************************************************************************
 * startDSConversion
 * Input:
 * - nothing
 *
 * Output:
 * - nothing
 * 
 * Globals:
 * - addr - array holding address of the DS18B20 device
 *
 * This function orders temperature conversion.
 * Some 750 ms is needed for a 12-bit conversion result, so the readout
 * of the result is called during the consecutive timer 1 interrupt.
 ************************************************************************/

void startDSConversion() {

  ds.reset();
  ds.select(addr);
  ds.write(0x44,0);         // start conversion
}

/************************************************************************
 * readTempDS
 * Input:
 * - nothing
 *
 * Output:
 * - value fo the temperature
 * 
 * Globals:
 * - addr - array holding address of the DS18B20 device
 *
 * This function reads the temperature conversion result, ordered by 
 * calling startDSConversion().
 * Some 750 ms is needed for a 12-bit conversion result, so the readout
 * of the result is called during the consecutive timer 1 interrupt after
 * the conversion request.
 *
 * 12-bit resolution is used, so the value is up to 4th decimal place.
 ************************************************************************/

float readTempDS()
{
  float temperature = 0.0;
  int rawtemp, rawint, rawfract;
  byte data[12];

  ds.reset();
  ds.select(addr);    
  ds.write(0xBE);                         // Read Scratchpad

  for (int i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
  }

  rawtemp = (data[1] << 8) + data[0];     // we put the temperature conversion MSB and LSB into a single variable
  rawfract = rawtemp & 0x0F;              // and then split it into iteger
  rawint = rawtemp >> 4;                  // and fractional part.

  for(int i = 0; i < 4; i++)              // Then we convert it to float value
  {
    rawfract = rawfract >> i;
    temperature = temperature + (0.0625 * (i+1)) * (rawfract & 0x01);
  }
  temperature = temperature + (float) rawint;

  return temperature;
}

/************************************************************************
 * convTemp
 * Input:
 * - temperature - float value of the temperature to convert to string
 *
 * Output:
 * - tempString - char array holding the result of the conversion to string.
 *                The format is ' '/-ddg.
 *
 * The string is always 4 character long. If the temperature is positive, then
 * the first character is a space, otherwise it is a '-'. Second and third
 * characters represent the value of the temperature and fourth character
 * is a letter 'g' which is displayed as a grade symbol (small superscript circle).
 ************************************************************************/

char *convTemp(float temperature)
{
  char *tempString = "  00";
  int tens = 0, units = 0, tenths = 0;
  int i = 0;

  temperature = round(temperature);
  if(temperature < 100)
  {
    if(temperature < 0)
      tempString[i] = '-';
    else
      tempString[i] = ' ';
    temperature = fabs(temperature);
    i++;
    if(temperature >= 10.0)
    {
      tens = temperature / 10;
      tempString[i] = tens + '0';
      temperature = temperature - tens * 10;
    }
    else
      tempString[i] = ' ';
    i++;
    if(temperature >= 1.0)
    {
      units = temperature;
      tempString[i] = units + '0';
      temperature = temperature - units;
    }
    else
      tempString[i] = '0';
    tempString[++i] = 'g';
  }

  return tempString;
}

void beep() {
  digitalWrite(BEEP, HIGH);
  delay(BEEPTIME);
  digitalWrite(BEEP, LOW);
}






































































































































































