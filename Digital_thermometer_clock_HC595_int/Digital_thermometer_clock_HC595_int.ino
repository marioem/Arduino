/******************************************************************************
 * Digital thermometer and clock using HC595 shift registers.
 * Display on 4-digit 7-segment multiplexed display module.
 * Registers are connected in series. Bits 3-0 of MSB drive digit enable,
 * LSB drives segments (a at MSb, h at LSb)
 *
 * Temperature sampling and display is handled via Timer 1 interrupt (generated every 5 ms).
 * Display is refreshed on every interrupt, temperature is sampled every 50 interrupts
 * (every 1 second).
 * 
 * Timer 1 prescaler setting: 1024
 * Timer 1 compare match register: 311.
 * 
 * Display procedure split into separate, asynchronous write to display memory
 * and regularly called display function
 * (C) 2014 Mariusz Musiał
 ******************************************************************************/

#define TEMPCORRECTION 3.6

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

#include <math.h>

float temperature;
long readDelay = TREADDELAY;
byte displayBuf[4], digit = 0;
byte clock[5] = {
  0,0,0,0,0};        // array holding curent time for display, tens of Hours, units of Hours, tens of minutes, units of minutes, seconds (in binary)
byte tmpClock[4] = {
  0,0,0,0};
boolean tick = false;    // changes state every second. If 1 then DP is displayed
byte setupMode = SMIDLE;
boolean displaySwitch = DSTEMP;
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
  pinMode(LATCH,OUTPUT);
  digitalWrite(LATCH,HIGH);
  pinMode(DATA,OUTPUT);
  digitalWrite(DATA,HIGH);
  pinMode(CLOCK,OUTPUT);
  digitalWrite(CLOCK,HIGH);
  Serial.begin(9600);
  analogReference(DEFAULT);

  // Configure pins for setting the clock
  pinMode(SETPIN,INPUT);  // button for entering and cycling through setup modes
  pinMode(UPPIN,INPUT);   // button for scrolling up values of clock digits

  cli();//stop interrupts

  //set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 15624;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  //set timer2 interrupt at 200Hz
  TCCR2A = 0;// set entire TCCR2A register to 0
  TCCR2B = 0;// same for TCCR2B
  TCNT2  = 0;//initialize counter value to 0
  // set compare match register for 8khz increments
  OCR2A = 77;// = (16*10^6) / (1024*200) - 1 (must be <256)
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);
  // Set CS22, CS21 and CS20 bits for 1024 prescaler
  TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);   
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);

  sei();//allow interrupts
}

void loop() {
  byte button = BNONE;

  button = readClockButtons(SETPIN, UPPIN);
  if(button == BSETUP)
  {
    setupMode += 1;
    if(setupMode > SMUMINUTE)
    {
      setupMode = SMIDLE;
      for(int i = 0; i < 4; i++)
        clock[i] = tmpClock[i];
      clock[4] = 0;                    // reset seconds counter
      if(!displaySwitch)                 // if we entered into setup during temperature display timeslot
        displaySwitch = !displaySwitch;  // we force time display timeslot in order to improve feeling of responsiveness
    }
    if(setupMode != SMIDLE)
    {
      clockSetup(setupMode);
    }
  }
  if(button == BUP)
  {
    if(setupMode != SMIDLE)
    {
      clockAdjust(setupMode);
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

  if(setupmode == SMTHOUR)          // we are beginning adjustment, so copy actual time to temp variable
    for(int i = 0; i < 4; i++)
      tmpClock[i] = clock[i];
  timeString[5] = NULL;
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

  if(!setupMode)          // if clock setting is not in progress
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
    // LC4LEDDisplayMemWrite("123g", displayBuf);  // display mem is only written when the temperature is read
    else
    {    
      if(readDelay == TREADDELAY)
        LC4LEDDisplayMemWrite(convTemp(readTemp()), displayBuf);  // display mem is only written when the temperature is read
    }
    readDelay--;
    if(readDelay == 0)
    {
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
 * on middle positions.
 * Positions are counted from left (1) to right (4).
 ************************************************************************/

void LC4LEDDisplayMemWrite(char *digits, byte *displayBuf)
{
  int bufIndex = 0, len = 0, dots = 0, dotsPos[4] = {
    0,0,0,0};
  boolean error = false;

  while(digits[len++] != 0x0)  // calculate the length of the string to display
    ;
  len--;
  if(len > 8 || len < 4)
  {
    error = truetrue;
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

float readTemp()
{
  float sensor = 0, voltage = 0, celcius = 0;

  for(int i = 0; i < 10; i++)        // ten readouts average
    sensor += (float) analogRead(1);
  sensor /= 10;
  voltage = (sensor * 5000) / 1024 - 500;
  celcius = voltage / 10 - TEMPCORRECTION;
  return celcius;
}

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
      tempString[i] = tens + 48;
      temperature = temperature - tens * 10;
    }
    else
      tempString[i] = ' ';
    i++;
    if(temperature >= 1.0)
    {
      units = temperature;
      tempString[i] = units + 48;
      temperature = temperature - units;
    }
    else
      tempString[i] = '0';
    i++;
    tempString[i] = 'g';
  }
  return tempString;
}

