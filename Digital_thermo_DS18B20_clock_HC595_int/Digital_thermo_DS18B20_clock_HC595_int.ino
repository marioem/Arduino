/******************************************************************************
 * Digital thermometer and clock using HC595 shift registers and DS18B20 as a sensor.
 * Display on 4-digit 7-segment multiplexed display module.
 * Registers are connected in series. Bits 3-0 of MSB drive digit enable,
 * LSB drives segments (a at MSb, h at LSb)
 *
 * Temperature sampling and display is handled via Timer 1 interrupt (generated every 1 s).
 * Display is refresh is handled via Timer 2 on every interrupt, every 5 ms. 
 * 
 * Timer 1 prescaler setting: 1024
 * Timer 1 compare match register: 15624.
 * 
 * Timer 2 prescaler setting: 1024
 * Timer 2 compare match register: 77.
 * 
 * Display procedure split into separate, asynchronous write to display memory
 * and regularly called display function.
 * (C) 2014 Mariusz Musiał
 ******************************************************************************/

// Definition of 7-segment, active low characters
#define ZERO        0x03 // 0 - a, b, c, d, e, f
#define ONE         0x9F // 1 - b, c
#define TWO         0x25 // 2 - a, b, d, e, g 00100101
#define THREE       0x0D // 3 - a, b, c, d, g
#define FOUR        0x99 // 4 - b, c, f, g
#define FIVE        0x49 // 5 - a, c, d, f, g
#define SIX         0x41 // 6 - a, c, d, e, f, g
#define SEVEN       0x1F // 7 - a, b, c
#define EIGHT       0x01 // 8 - a, b, c, d, e, f, g
#define NINE        0x09 // 9 - a, b, c, d, f, g
#define MINUS       0xFD // "minus" - g
#define BLANK       0xFF // blank
#define DEGREE      0x39 // ° - a, b, f, g
#define ACHAR       0x11 // A - a, b, c, e, f, g
#define BCHAR       0xC1 // b - c, d, e, f, g
#define CCHAR       0x63 // C - a, d, e, f
#define DCHAR       0x85 // d - b, c, d, e, g
#define ECHAR       0x61 // E - a, d, e, f, g
#define FCHAR       0x71 // F - a, e, f, g
#define HCHARSMALL  0xD1 // h - c, e, f, g
#define HCHARBIG    0x91 // H - b, c, e, f, g
#define JCHAR       0x07 // J - a, b, c, d, e
#define LCHAR       0xE3 // L - d, e, f
#define NCHAR       0xD5 // n - c, e, g 11010101
#define OCHAR       0xC5 // o - c, d, e, g
#define PCHAR       0x31 // P - a, b, e, f, g
#define RCHAR       0xF5 // r - e, g
#define TCHAR       0xE1 // t - d, e, f, g
#define UCHARSMALL  0xC7 // u - c, d, e
#define UCHARBIG    0x83 // U - b, c, d, e, f
#define YCHAR       0x89 // Y - b, c, d, f, g
#define QUESTION    0x35 // ? - a, b, e, g + DP on previous position
#define DP          0xFE // . - h

// Definition of pins interfacing with HC595
#define LATCH      13
#define DATA       12
#define CLOCK      11

// for how many timer 1 interrupt periods given information (time or temperature) is displayed
#define TREADDELAY 5

// Definition of pins to which clock adjustment buttons are connected
#define SETPIN     8
#define UPPIN      7

// define clock setup modes (states)
#define SMIDLE     0
#define SMTHOUR    1  // setting tens of hours
#define SMUHOUR    2  // setting units of hours
#define SMTMINUTE  3  // setting tens of minutes
#define SMUMINUTE  4  // setting units of minutes

// Definition of buttons pressed
#define BNONE      0  // no button pressed
#define BSETUP     1  // SETUP button pressed
#define BUP        2  // UP button pressed

#define DSTEMP     true  // displaySwitch -> temperature; ~DSTEMP -> clock

#define ONEWIREBUS 2     // pin number where the OneWire device is connected to

#include <math.h>
#include <OneWire.h>

// DS18B20 Temperature chip i/o
OneWire ds(ONEWIREBUS);   // on pin 2
// global variables for handling DS18B20 device
byte addr[8];    // address of the device on OneWire bus
boolean dsfound = false;
boolean startConversion = true;   // if set then the conversion is called, otherwise readout. It is used in the timer 1 interrupt routine

// this is display memory and digit pointer into that memory
byte displayBuf[4], digit = 0;

//float temperature;
// display controlling global variables
long readDelay = TREADDELAY;  // counter which upon expiration switches the info display type from time to temperature and vice versa
boolean displaySwitch = DSTEMP;  // flag indicating which info to display; temperature or time
boolean tick = false;    // changes state every second. If 1 then DP is displayed

// clock variables
byte clock[5] = {
  0,0,0,0,0};        // array holding curent time for display, tens of Hours, units of Hours, tens of minutes, units of minutes, seconds (in binary)
byte tmpClock[4] = {
  0,0,0,0};

// global variables for handling clock setup/adjustmen
byte setupMode = SMIDLE;           // state variable for handling time addjustment procedure
char timeString[] = {
  '0','0','.','0','0',NULL,NULL};  // here the string for current time is constructed
int buttonSetupState;              // the current reading from the input pin
int lastButtonSetupState = LOW;    // the previous reading from the input pin
int buttonUpState;                 // the current reading from the input pin
int lastButtonUpState = LOW;       // the previous reading from the input pin
long lastSetupDebounceTime = 0;    // the last time the output pin for Setuo button was toggled
long lastUpDebounceTime = 0;       // the last time the output pin for Up button was toggled
long debounceDelay = 50;           // the debounce time; increase if the output flickers


void setup() {

  // first pins controlling HC595 are set up
  pinMode(LATCH,OUTPUT);
  digitalWrite(LATCH,HIGH);
  pinMode(DATA,OUTPUT);
  digitalWrite(DATA,HIGH);
  pinMode(CLOCK,OUTPUT);
  digitalWrite(CLOCK,HIGH);

  Serial.begin(9600);

  // Configure pins for setting the clock
  pinMode(SETPIN,INPUT);  // button for entering and cycling through setup modes
  pinMode(UPPIN,INPUT);   // button for scrolling up values of clock digits

  cli();                  //stop interrupts

  //set timer1 interrupt at 1Hz
  TCCR1A = 0;            // set entire TCCR1A register to 0
  TCCR1B = 0;            // same for TCCR1B
  TCNT1  = 0;            //initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 15624;         // = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  //set timer2 interrupt at 200Hz
  TCCR2A = 0;            // set entire TCCR2A register to 0
  TCCR2B = 0;            // same for TCCR2B
  TCNT2  = 0;            //initialize counter value to 0
  // set compare match register for 8khz increments
  OCR2A = 77;            // = (16*10^6) / (1024*200) - 1 (must be <256)
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);
  // Set CS22, CS21 and CS20 bits for 1024 prescaler
  TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);   
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);

  sei();                  //allow interrupts

  // look for DS18B20 device
  ds.reset_search();
  if (ds.search(addr)) {
    if (OneWire::crc8(addr, 7) == addr[7])
      if (addr[0] == 0x28)
        dsfound = true;
  }
  else
    ds.reset_search();
}

void loop() {
  byte button = BNONE;

  if(!dsfound) {            // if DS18B20 was not foud at setup try to find it here
    ds.reset_search();
    if (ds.search(addr)) {
      if (OneWire::crc8(addr, 7) == addr[7])
        if (addr[0] == 0x28)
          dsfound = true;
    }
    else
      ds.reset_search();
  }

  // check if the any of the clock adjustment buttons is pressed
  button = readClockButtons(SETPIN, UPPIN);
  if(button == BSETUP)                    // if SETUP button is pressed
  {
    setupMode += 1;                       // cycle trough setup states
    if(setupMode > SMUMINUTE)             // if we cycled through all states
    {
      setupMode = SMIDLE;                 // we return do idle
      for(int i = 0; i < 4; i++)
        clock[i] = tmpClock[i];           // and copy adjusted clock values to main clock array
      clock[4] = 0;                       // reset seconds counter
      if(!displaySwitch)                  // if we entered into setup during temperature display timeslot
        displaySwitch = !displaySwitch;   // we force time display timeslot in order to improve feeling of responsiveness
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

  if(setupmode == SMTHOUR)                // we are beginning adjustment, so copy actual time to temp variable
    for(int i = 0; i < 4; i++)
      tmpClock[i] = clock[i];
  timeString[5] = NULL;                   // write string terminator
  if(setupmode == SMUMINUTE)              // during addjustment the temp time is displayed and addusted
    timeString[4] = tmpClock[3] + '0';    // it is later copied to main clock variable and displayed
  else
    timeString[4] = ' ';
  if(setupmode >= SMTMINUTE)
    timeString[3] = tmpClock[2] + '0';
  else
    timeString[3] = ' ';
  timeString[2] = '.';
  if(setupmode >= SMUHOUR)
    timeString[1] = tmpClock[1] + '0';
  else
    timeString[1] = ' ';
  if(setupmode >= SMTHOUR)
    timeString[0] = tmpClock[0] + '0';
  LC4LEDDisplayMemWrite(timeString, displayBuf);
}

void clockAdjust(byte setupmode) {
  int i, j;

  switch(setupmode) {
  case SMTHOUR:
    tmpClock[0] += 1;
    if(tmpClock[0] >= 3)
      tmpClock[0] = 0;
    i = 0;
    j = 0;
    break;
  case SMUHOUR:
    tmpClock[1] += 1;
    if((tmpClock[0] < 2 && tmpClock[1] >= 10) || (tmpClock[0] == 2 && tmpClock[1] >= 4))
      tmpClock[1] = 0;
    i = 1;
    j = 1;
    break;
  case SMTMINUTE:
    tmpClock[2] += 1;
    if(tmpClock[2] >= 6)
      tmpClock[2] = 0;
    i = 2;
    j = 3;
    break;
  case SMUMINUTE:
    tmpClock[3] += 1;
    if(tmpClock[3] >= 10)
      tmpClock[3] = 0;
    i = 3;
    j = 4;
    break;
  default:
    break;   
  }
  timeString[j] = tmpClock[i] + '0';
  LC4LEDDisplayMemWrite(timeString, displayBuf);
}

ISR(TIMER1_COMPA_vect) {

  // This function is called every second
  // Firs, it handles time update every 60th call (every minute)
  clock[4] += 1;
  if(clock[4] == 60)
  {   
    clock[4] = 0;
    clock[3] += 1;
    if(clock[3] == 10)
    {   
      clock[3] = 0;
      clock[2] += 1;
    }
    if(clock[2] == 6)
    {   
      clock[2] = 0;
      clock[1] += 1;
    }
    if((clock[0] < 2 && clock[1] == 10) || (clock[0] == 2 && clock[1] == 4))
    {   
      clock[1] = 0;
      clock[0] += 1;
    }
    if(clock[0] == 3)
      clock[0] = 0;
    if(!setupMode)          // if clock setting is not in progress
      for(int i = 0, j = 0; i < 4; i++, j++)
      {
        if(i == 2)
          j++;
        timeString[j] = clock[i] + '0';
      }
  }

  // After updating time, the display of required data is handled.
  // It alternates between time and temperature.
  if(!setupMode)              // if clock setting is not in progress
  {
    if(tick)                  // if DP should be displayed
      timeString[5] = '.';
    else                      // if DP should be cleared
    {
      timeString[5] = NULL;
    }
    tick = !tick;

    if(displaySwitch)
      LC4LEDDisplayMemWrite(timeString, displayBuf);  // display mem is only written when the temperature is read
    else
    {    
      if(dsfound)                                    // if DS18B20 has been found on the bus
        if(startConversion) {                        // and it's time for requesting conversion of temperature
          startDSConversion();                       // send conversion order to the device
          startConversion = false;                   // and change state to temperature readout
        }
        else                                         // it's time to read the temperature from the sensor
      {
        LC4LEDDisplayMemWrite(convTemp(readTempDS()), displayBuf);  // read it and display
        startConversion = true;                    // and change state to temperature conversion
      }
      else                                           // otherwise, if device is not found
      LC4LEDDisplayMemWrite("t---", displayBuf);   // display indication of missing temperature
    }
    readDelay--;                                     // decrease counter
    if(readDelay == 0)                               // it counter expired, reload it and change display flag
    {                                                // so that we alternate between time and temperature every TREADDELAY second
      readDelay = TREADDELAY;
      displaySwitch = !displaySwitch;
    }
  }
}

ISR(TIMER2_COMPA_vect){
  LC4LEDDisplay(displayBuf, digit);  // content of the display memory is put on display in every loop iteration
  digit += 1;                        // every interrupt one digit is displayed - this way the multiplexing is done
  digit &= 0x3;                      // digit is a two-bit counter
}

/************************************************************************
 * LC4LEDDisplayMemWrite - display string on a 4 LED 7 segmend LED display
 * module from LC Technology.
 * 
 * Input:
 * - digits - string to display
 * - displayBuf - pointer to display memory (a byte array)
 * 
 * Output:
 * - nothing
 *
 * Function accepts the string table of length between 4 (min value, use 
 * spaces for padding, if necessary) and 8 characters.
 * If there is no Decimal Point to display, then the allowed length is 4.
 * Thece can be up to 4 DPs, only on even positions. In this case the max
 * length of the input string can be up to 8.
 * 
 * Displayed character set (case sensitive):
 * "0123456789- °AbCdEFhHJLnoPrtuUY."
 * Degree (°) character is passed to this function by means of 'g' character
 * 
 * Dots can be only appended to other chars. So in order to display 4 dots
 * the string passed has to be the following: " . . . ."
 * 
 * In case of any error at input parameter handling the string "??" is displayed
 * on the two middle positions.
 * Positions are counted from left (1) to right (4).
 ************************************************************************/

void LC4LEDDisplayMemWrite(char *digits, byte *displayBuf)
{
  int bufIndex = 0, len = 0, dots = 0, dotsPos[4] = {
    0,0,0,0                              };
  boolean error = false;

  while(digits[len++] != 0x0)  // calculate the length of the string to display
    ;
  len--;
  if(len > 8 || len < 4)
  {
    error = true;
    goto error;
  }
  for(int i = 0; i < len; i++)
  {
    if(digits[i] == '.')
      dotsPos[dots++] = i;
  }
  // dots can be only appended to other chars
  if(len - dots != 4)
  {
    error = true;
    goto error;
  }
  if(dots > 0)
    if(dotsPos[0] == 0)                      // string cannot start with a dot
    {
      error = true;
      goto error;
    }
  for(int i = 1; i < dots; i++)
    if(dotsPos[i] == (dotsPos[i - 1] + 1))    // dots cannot be on consecutive positions
    {
      error = true;
      goto error;
    }
  // encode input string in display buffer
  for(int i = 0; i < len; i++)
  {
    if(bufIndex > 3 && digits[i] != '.')
    {
      error = true;
      goto error;
    }
    switch(digits[i]) {
    case '0':
      displayBuf[bufIndex++] = ZERO;
      break;
    case '1':
      displayBuf[bufIndex++] = ONE;
      break;
    case '2':
      displayBuf[bufIndex++] = TWO;
      break;
    case '3':
      displayBuf[bufIndex++] = THREE;
      break;
    case '4':
      displayBuf[bufIndex++] = FOUR;
      break;
    case '5':
      displayBuf[bufIndex++] = FIVE;
      break;
    case '6':
      displayBuf[bufIndex++] = SIX;
      break;
    case '7':
      displayBuf[bufIndex++] = SEVEN;
      break;
    case '8':
      displayBuf[bufIndex++] = EIGHT;
      break;
    case '9':
      displayBuf[bufIndex++] = NINE;
      break;
    case '-':
      displayBuf[bufIndex++] = MINUS;
      break;
    case ' ':
      displayBuf[bufIndex++] = BLANK;
      break;
    case 'g':
      displayBuf[bufIndex++] = DEGREE;
      break;
    case 'A':
      displayBuf[bufIndex++] = ACHAR;
      break;
    case 'b':
      displayBuf[bufIndex++] = BCHAR;
      break;
    case 'C':
      displayBuf[bufIndex++] = CCHAR;
      break;
    case 'd':
      displayBuf[bufIndex++] = DCHAR;
      break;
    case 'E':
      displayBuf[bufIndex++] = ECHAR;
      break;
    case 'F':
      displayBuf[bufIndex++] = FCHAR;
      break;
    case 'h':
      displayBuf[bufIndex++] = HCHARSMALL;
      break;
    case 'H':
      displayBuf[bufIndex++] = HCHARBIG;
      break;
    case 'J':
      displayBuf[bufIndex++] = JCHAR;
      break;
    case 'L':
      displayBuf[bufIndex++] = LCHAR;
      break;
    case 'n':
      displayBuf[bufIndex++] = NCHAR;
      break;
    case 'o':
      displayBuf[bufIndex++] = OCHAR;
      break;
    case 'P':
      displayBuf[bufIndex++] = PCHAR;
      break;
    case 'r':
      displayBuf[bufIndex++] = RCHAR;
      break;
    case 't':
      displayBuf[bufIndex++] = TCHAR;
      break;
    case 'u':
      displayBuf[bufIndex++] = UCHARSMALL;
      break;
    case 'U':
      displayBuf[bufIndex++] = UCHARBIG;
      break;
    case 'Y':
      displayBuf[bufIndex++] = YCHAR;
      break;
    case '.':
      displayBuf[bufIndex - 1] = displayBuf[bufIndex - 1] & DP;
      break;
    default:
      error = true;
    }
  }
error:
  if(error)
  {
    displayBuf[0] = DP;               // DP
    displayBuf[1] = (QUESTION & DP);  // '?' and DP for next char
    displayBuf[2] = QUESTION;
    displayBuf[3] = BLANK;
  }
}


/************************************************************************
 * LC4LEDDisplay
 * Input:
 * - displayBuf - pointer to display memory (a byte array)
 * 
 * Output:
 * - nothing
 * 
 * Function puts content of the display memory pointed to by the LCDisplayBuf
 * parameter to a pair of HC595 shift registers. MSB, bits 3-0 low enable digit
 * and LSB low drives the segments. Display module is multiplexed common anode.
 ************************************************************************/

void LC4LEDDisplay(byte *LCdisplayBuf, byte digit) {

  digitalWrite(LATCH, LOW);
  shiftOut(DATA, CLOCK, MSBFIRST, (0xF7 >> digit) & 0x0F);          // set low the digit enable bit
  shiftOut(DATA, CLOCK, MSBFIRST, LCdisplayBuf[digit]);
  digitalWrite(LATCH, HIGH);
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

