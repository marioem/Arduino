/******************************************************************************
 * Digital thermometer v2
 * Display on 4-digit 7-segment multiplexed display module.
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

#include <math.h>

int digitPin[4] = {
  13, 12, 11, 10}; // Pins selecting digits
int segmentPin[8] = {
  9, 8, 7, 6, 5, 4, 3, 2}; // Pins selecting segments, from a through h
float temperature;
char *temperatureStr;
int readDelay = 1000;
byte displayBuf[4];

void setup() {
  for(int i = 0; i < 4; i++)
  {
    pinMode(digitPin[i],OUTPUT);
    digitalWrite(digitPin[i],HIGH);
  }
  for(int i = 0; i < 8; i++)
  {
    pinMode(segmentPin[i],OUTPUT);
    digitalWrite(segmentPin[i],HIGH);
  }
  Serial.begin(9600);
  analogReference(DEFAULT);
}

void loop() {

  if(readDelay == 1000)
  {
    temperature = readTemp();
    temperatureStr = convTemp(temperature);
    LC4LEDDisplayMemWrite(temperatureStr, displayBuf);  // display mem is only written when the temperature is read
  }
  readDelay--;
  if(readDelay == 0)
    readDelay = 1000;
  LC4LEDDisplay(displayBuf);  // content of the display memory is put on display in every loop iteration
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
    0,0,0,0                                          }
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
 * Globals:
 * - digitPin - 4-element int array holding pin numbers driving digit-enable
 * - segmentPin - 8-element int array holding 
 * 
 * Function puts content of the display memory pointed to by the LCDisplayBuf
 * parameter to display module (multiplexed common anode).
 * Digit enabling pins are defined in digitPin global array. Segment steering
 * pins are defined in segmentPin global array.
 ************************************************************************/

void LC4LEDDisplay(byte *LCdisplayBuf) {
  byte currentChar;

  for(int digit = 0; digit < 4; digit++)
  {
    currentChar = LCdisplayBuf[digit];
    for(int segment = 7; segment >= 0; segment--)
    {
      digitalWrite(segmentPin[segment],currentChar & 0x01);
      currentChar = currentChar >> 1;
    }
    digitalWrite(digitPin[digit],LOW);
    delay(1);
    digitalWrite(digitPin[digit],HIGH);
  }
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



















































