#define MINUS 10
#define BLANK 11
#define NEG 1
#define POS 0
#define THERMO 0
#define DP 7  // segment for Decimal Point

int digitDecode[12][8] = {
  {
    LOW,LOW,LOW,LOW,LOW,LOW,HIGH,HIGH                                  }
  ,          // 0 - a, b, c, d, e, f
  {
    HIGH,LOW,LOW,HIGH,HIGH,HIGH,HIGH,HIGH                                  }
  ,      // 1 - b, c
  {
    LOW,LOW,HIGH,LOW,LOW,HIGH,LOW,HIGH                                  }
  ,         // 2 - a, b, d, e, g
  {
    LOW,LOW,LOW,LOW,HIGH,HIGH,LOW,HIGH                                  }
  ,         // 3 - a, b, c, d, g
  {
    HIGH,LOW,LOW,HIGH,HIGH,LOW,LOW,HIGH                                  }
  ,        // 4 - b, c, f, g
  {
    LOW,HIGH,LOW,LOW,HIGH,LOW,LOW,HIGH                                  }
  ,         // 5 - a, c, d, f, g
  {
    LOW,HIGH,LOW,LOW,LOW,LOW,LOW,HIGH                                  }
  ,          // 6 - a, c, d, e, f, g
  {
    LOW,LOW,LOW,HIGH,HIGH,HIGH,HIGH,HIGH                                  }
  ,       // 7 - a, b, c
  {
    LOW,LOW,LOW,LOW,LOW,LOW,LOW,HIGH                                  }
  ,           // 8 - a, b, c, d, e, f, g
  {
    LOW,LOW,LOW,LOW,HIGH,LOW,LOW,HIGH                                  }
  ,          // 9 - a, b, c, d, f, g
  {
    HIGH,HIGH,HIGH,HIGH,HIGH,HIGH,LOW,HIGH                                  }
  ,     // "minus" - g
  {
    HIGH,HIGH,HIGH,HIGH,HIGH,HIGH,HIGH,HIGH                                  }
};   // blank

int digitPin[4] = {
  13, 12, 11, 10};
int segmentPin[8] = {
  9, 8, 7, 6, 5, 4, 3, 2};
int (*displayBuf)[8];
float temperature;
char *temperatureStr;
int readDelay = 1000;

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
  }
  readDelay--;
  if(readDelay == 0)
    readDelay = 1000;
  displayValue(temperatureStr);
}

void displayValue(char *digits)
{
  int decodeIndex;

  if(digits[0] == '-')
    decodeIndex = MINUS;
  else
    decodeIndex = BLANK;

  for(int segment = 0; segment < 8; segment++)
  {
    digitalWrite(segmentPin[segment],digitDecode[decodeIndex][segment]);
  }
  digitalWrite(digitPin[0],LOW);
  delay(1);
  digitalWrite(digitPin[0],HIGH);

  for(int digit = 1; digit < 4; digit++)
  {
    decodeIndex = digits[digit] - 48;
    for(int segment = 0; segment < 8; segment++)
    {
      digitalWrite(segmentPin[segment],digitDecode[decodeIndex][segment]);
    }
    if(digit == 2)
      digitalWrite(segmentPin[DP],LOW);
    digitalWrite(digitPin[digit],LOW);
    delay(1);
    digitalWrite(digitPin[digit],HIGH);
  }
}

float readTemp()
{
  float sensor = 0, voltage = 0, celcius = 0;

  for(int i = 0; i < 10; i++)
    sensor += (float) analogRead(1);
  sensor /= 10;
  // casting sensor to float is crucial here
  voltage = (sensor * 5080) / 1024 - 500;
  celcius = voltage / 10;
  //Serial.print("Celcius = ");
  //Serial.println(celcius);
  return celcius;
}

char *convTemp(float temperature)
{
  char *tempString = "  00";
  int tens = 0, units = 0, tenths = 0;
  int i = 0;

  if(temperature < 100)
  {
    if(temperature < 0)
      tempString[i] = '-';
    else
      tempString[i] = ' ';
    i++;
    if(temperature >= 10.0)
    {
      tens = temperature / 10;
      tempString[i] = tens + 48;
      temperature = temperature - tens * 10;
    }
    i++;
    if(temperature >= 1.0)
    {
      units = temperature;
      tempString[i] = units + 48;
      temperature = temperature - units;
    }
    i++;
    tenths = temperature * 10;
    tempString[i] = tenths + 48;
  }
  return tempString;
}

