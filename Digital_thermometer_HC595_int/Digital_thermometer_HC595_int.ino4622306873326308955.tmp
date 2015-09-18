/******************************************************************************
 * Digital thermometer using HC595 shift registers.
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


#define NEG         1
#define POS         0
#define THERMO      0
#define FALSE       0
#define TRUE        1

#define TEMPCORRECTION 3.6

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

#define LATCH      13
#define DATA       12
#define CLOCK      11

#define TREADDELAY 400

#include <math.h>

float temperature;
char *temperatureStr;
int readDelay = TREADDELAY;
byte displayBuf[4], digit = 0;

void setup() {
  pinMode(LATCH,OUTPUT);
  digitalWrite(LATCH,HIGH);
  pinMode(DATA,OUTPUT);
  digitalWrite(DATA,HIGH);
  pinMode(CLOCK,OUTPUT);
  digitalWrite(CLOCK,HIGH);
  // Serial.begin(9600);
  analogReference(DEFAULT);

  cli();//stop interrupts

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

}

ISR(TIMER2_COMPA_vect){

  if(readDelay == TREADDELAY)
  {
    temperature = readTemp();
    temperatureStr = convTemp(temperature);
    LC4LEDDisplayMemWrite(temperatureStr, displayBuf);  // display mem is only written when the temperature is read
    //    LC4LEDDisplayMemWrite("123g", displayBuf);  // display mem is only written when the temperature is read
  }
  readDelay--;
  if(readDelay == 0)
    readDelay = TREADDELAY;
  LC4LEDDisplay(displayBuf, digit);  // content of the display memory is put on display in every loop iteration
  digit += 1;
  digit &= 0x3;

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
    0,0,0,0                                                          }
  , j = 0;
  boolean error = FALSE;

  while(digits[len++] != 0x0)  // calculate the length of the string to display
    ;
  len--;
  if(len > 8 || len < 4)
  {
    error = TRUE;
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
    error = TRUE;
    goto error;
  }
  if(dots > 0)
    if(dotsPos[0] == 0)                      // string cannot start with a dot
    {
      error = TRUE;
      goto error;
    }
  for(int i = 1; i < dots; i++)
    if(dotsPos[i] == (dotsPos[i - 1] + 1))    // dots cannot be on consecutive positions
    {
      error = TRUE;
      goto error;
    }
  // encode input string in display buffer
  for(int i = 0; i < len; i++)
  {
    if(bufIndex > 3 && digits[i] != '.')
    {
      error = TRUE;
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
      error = TRUE;
    }
  }
error:
  if(error)
  {
    displayBuf[0] = DP;  // DP
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
  byte currentChar;

  digitalWrite(LATCH, LOW);
  shiftOut(DATA, CLOCK, MSBFIRST, (0xF7 >> digit) & 0x0F);          // set low the digit enable bit
  shiftOut(DATA, CLOCK, MSBFIRST, LCdisplayBuf[digit]);
  digitalWrite(LATCH, HIGH);
  //delay(5);
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



























































