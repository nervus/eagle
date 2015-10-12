 
#include <LiquidCrystal.h>
//#include <PID_v1.h>

#define USE_PWM_OUTPUT 1
#define DEBUG 0
#define kMaxSupportedFans 4

const int FanCount = 4;

// globals
const unsigned int UpdateIntervalMS = 1000; // update every 1s
const float PWMStartup = 64;
const float PWMMax = 255;
const float TempStartup = 22; // C
const float TempMax = 31; // C
const float TempThreshold=3; // C


enum EFanState 
{
  kOff = 0,
  kOn,
};

EFanState FanState = kOff;

// pins
const unsigned int tachPins[kMaxSupportedFans] = { 0, 1, 2, 3 };
const unsigned int PWMPins[kMaxSupportedFans] = { 10, 9, 6, 5 }; //PWM4,3,2,1
const unsigned int TempPins[kMaxSupportedFans] = { 21, 20, 19, 18 }; //A3, A2, A1, A0
const unsigned int LCDPins[6] = { 4, 7, 15, 14, 16, 8 }; // RS, EN, D4..D7
const unsigned int LCDPinsCount = sizeof(LCDPins) / sizeof(LCDPins[0]);
// devices
LiquidCrystal lcd(LCDPins[0], LCDPins[1], LCDPins[2], LCDPins[3], LCDPins[4], LCDPins[5]);


// frame data
unsigned int fanWatchdog[kMaxSupportedFans];
unsigned int outRPMCounter[kMaxSupportedFans];
unsigned int outRPM[kMaxSupportedFans];
float outTemp[kMaxSupportedFans];

unsigned long prevTime = 0;
const int fanWatchdogTimeout = 5; // wait 5 frames until triggering alarm.

void tachHandler(int tachIndex) { outRPMCounter[tachIndex]++; }
void tachHandler0() { tachHandler(0); }
void tachHandler1() { tachHandler(1); }
void tachHandler2() { tachHandler(2); }
void tachHandler3() { tachHandler(3); }

void InitializeFrameData()
{
  for (int i=0;i<kMaxSupportedFans;i++)
  {
    outRPM[i] = 0;
    outRPMCounter[i] = 0;
    outTemp[i] = 0;
  }

  prevTime = 0;
}

void SetupPins()
{
  //setup tach interrupt pins
  
  attachInterrupt(digitalPinToInterrupt(tachPins[0]), tachHandler0, RISING);
  attachInterrupt(digitalPinToInterrupt(tachPins[1]), tachHandler1, RISING);
  attachInterrupt(digitalPinToInterrupt(tachPins[2]), tachHandler2, RISING);
  attachInterrupt(digitalPinToInterrupt(tachPins[3]), tachHandler3, RISING);

  for (int i=0;i<kMaxSupportedFans;i++)
  {
    // setup analog pins
    pinMode(TempPins[i], INPUT);
  
    // setup output PWM pins  
    pinMode(PWMPins[i], OUTPUT);
  }

  // setup output LCD pins  
  for (int i=0;i<LCDPinsCount;i++)
    pinMode(LCDPins[i], OUTPUT);

  Serial.print("pins: "); Serial.println(LCDPinsCount);
}

void SetupDevices()
{
  // kill the fans
  for (int i=0;i<kMaxSupportedFans;i++)
    UpdateFan(PWMPins[i], 0);

  lcd.begin(16, 2);
 
  #if DEBUG
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }

  Serial.println("\nSerial Ready\n");
  #endif
}

void UpdateFan(int pin, float value)
{
  #if USE_PWM_OUTPUT
      analogWrite(pin, value);
  #else
    if(value > PWMStartup)
      digitalWrite(pin, HIGH);
    else
      digitalWrite(pin, LOW);
  #endif
}

void UpdateSensors()
{
  long currentTime = millis();
  long dt = currentTime - prevTime;
  if (dt > 0) //it can wrap around every 50 days or so
  {
    if (dt > UpdateIntervalMS) // update every second
    {
      for (int tachIndex=0;tachIndex<FanCount;tachIndex++)
      {
        outRPM[tachIndex] = outRPMCounter[tachIndex] * 60;
        outRPMCounter[tachIndex] = 0;
        prevTime = currentTime;
      }
    }
  }
  else
  {
      prevTime = currentTime;
  }

  for (int i=0;i<FanCount;i++)
  {
    outTemp[i] += analogRead(TempPins[i]) * 0.48828125; //  10mv / C and 5V / 1024 units
    outTemp[i] *= 0.5;
    delayMicroseconds(200);
  }
}

int activeFan = 0;
int timeout = UpdateIntervalMS;
long startttt = 0;

void setup()
{
  SetupDevices();
  SetupPins();
  InitializeFrameData();
  startttt = millis();
}

// todo: use the temp threshold to stop cooling.

int speeds[4] = {0, 0, 0, 0};
int current = 0;


void loop()
{
  /*
  for (int i=0;i<LCDPinsCount;i++)
  {
    digitalWrite(LCDPins[i], HIGH);
  }

  delayMicroseconds(1000);
  
  for (int i=0;i<LCDPinsCount;i++)
  {
    digitalWrite(LCDPins[i], LOW);
  }

  delayMicroseconds(1000);
  */
  


  
  UpdateSensors();


/*
  for (int i=0;i<FanCount;i++)
  {
    if ((FanState == kOff) && (outTemp[i] > TempStartup + TempThreshold))
          FanState = kOn;

    if (FanState == kOn)
    {
      if (outTemp`[i] > TempStartup)
      {
          float tempDiff = outTemp[i] - TempStartup;
          tempDiff /= TempMax - TempStartup;
          if (tempDiff > 1)
            tempDiff = 1;
          tempDiff *= PWMMax - PWMStartup;
          tempDiff += PWMStartup;
          UpdateFan(PWMPins[i], tempDiff);
          //Serial.print("fan ");
          //Serial.print(i);
          //Serial.print(": ");
          //Serial.println(tempDiff); 
      }
      else
        FanState = kOff;
    }
    else
      UpdateFan(PWMPins[i], 0);
  }
*/
  /*
  cli();
  long ct = millis();
  long dt = ct - startttt;
  if (dt > UpdateIntervalMS)
  {
    startttt = ct;
    analogWrite(PWMPins[activeFan], 0);
    activeFan++;
    if (activeFan == FanCount)
      activeFan =0 ;
    analogWrite(PWMPins[activeFan],64);
  }
  sei();
  */


  //speeds[current]++;
  delay(100);
  if (speeds[current] > 512)
  {
    speeds[current] = 0;
   // current++;
    current &= 3;
  }
  
  for (int i=0;i<4;i++)
    analogWrite(PWMPins[i], speeds[i] < 255 ? speeds[i] : 255);
  
  lcd.clear();
  for (int i=0;i<4;i++)
  {
    lcd.setCursor((i&1)<<3, (i>>1));
    lcd.print(speeds[i]);
  }

/*
  for (int i=0;i<FanCount;i++)
    analogWrite(PWMPins[i],0);
    
  lcd.clear();  
  delayMicroseconds(100);
  for (int i=0;i<FanCount;i++)
  {
    lcd.setCursor((i&1)<<3, (i>>1));
    lcd.print(outRPM[i]);
  }
  delayMicroseconds(100);
*/
}



