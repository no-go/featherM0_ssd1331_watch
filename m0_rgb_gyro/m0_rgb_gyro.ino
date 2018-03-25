#include <sam.h>
#include <power.h>
#include <wdt.h>

// -------------------------------------------Config Start
const int MPU=0x69;  // I2C address of the MPU-6050 (AD0 to 3.3V)

#define SET_INIT_RTC 0

#define OLED_DC      5
#define OLED_CS     A4
#define OLED_RESET   6

#define VBATPIN   A7  // A7 = D9 !!

#define VCCMAX 4370
#define VCCMIN 3550

#define BUTTON    A1 //(to - on press)
#define BUTTON2   A3 //(to -on press)
#define BUTTON3   A5 //(to -on press)
#define POTI      A2
#define SPEAKER   10 // beep on click

#define LED_WHITE  A0
#define VIBRATE    11

#define ONACCEL   32500l

long treshold = 6; //initial

byte batLength = 34;
int  DIMSEC    = 5;
int  OFFSEC    = 15;
int  alarmsec  = -1;

// -------------------------------------------Config End

#include <Adafruit_GFX.h>
#include "Adafruit_SSD1331.h"

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "BluefruitConfig.h"

#include <SPI.h>
#include <Wire.h>
#include "DS3231M.h"

int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

long stoss;
long firstStoss =-1;
int vccVal;

long barrier = 0;
unsigned long steps = 0;

int deltax,deltay;

//RTCdata data = {40,53,21, 3, 21,03,18}; // (3 == )Mittwoch, 21:53:40 Uhr 21.03.2018 //7=sonntag
#if SET_INIT_RTC > 0
  RTCdata data = {00,50,14, 7, 25,03,18};
#else
  RTCdata data;
#endif

byte tick        = 0;
int displayOnSec = 0;
int messageOnSec = -1;
byte pressTicks  = 0;

byte lhours   = 1;
byte lminutes = 1;
byte lseconds = 1;

int showDetails = 0;

char serialCache[33] = "                               ";
int cpos = 0;

uint32_t potival = 0;

// 6x16 zeichen:
#define BUFLEN 64
bool contin;

bool wled = false;
bool vibr = false;

// Color definitions
#define BLACK           0x0000
#define GREY            0b0001000010000010
#define GREYBLUE        0b0010000100010000
#define LIGHTBLUE       0b0110001000011111
#define CYAN            0x07FF
#define BLUE            0x001F
#define MAGENTA         0xF81F
#define RED             0xF800
#define RED2            0b0101100000000000
#define AMBER           0b1111101111100111
#define YELLOW          0xFFE0  
#define GREENYELLOW     0b0111111111100000
#define GREEN           0x07E0
#define WHITE           0xFFFF

#define BACKGROUND      0x0000

short colors[15] = {
  RED2, BLACK, GREY, GREYBLUE, LIGHTBLUE, CYAN, BLUE, MAGENTA,
  RED, AMBER, YELLOW, GREENYELLOW, GREEN, WHITE, WHITE
};

#define NO_TIME_THERE   42 // a flag

Adafruit_SSD1331 oled = Adafruit_SSD1331(OLED_CS, OLED_DC, OLED_RESET);
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

short clockColor  = WHITE;
short clockColor2 = RED2;


#define LOGO16_GLCD_HEIGHT 16 
#define LOGO16_GLCD_WIDTH  16 
static const unsigned char PROGMEM step_bmp[] =
{ B00000000, B00000000,
  B00000000, B00000000,
  B00000000, B00000000,
  B00111100, B00000000,
  B01111100, B00000000,
  B01111100, B01111000,
  B01111100, B01111100,
  B01111100, B01111100,
  B01111000, B01111100,
  B01111000, B01111100,
  B00000000, B00111100,
  B01111100, B00111100,
  B01111100, B00000000,
  B00111000, B01111100,
  B00000000, B01111100,
  B00000000, B00111000 };

static const uint8_t PROGMEM zero[] = {
B01110000,
B10001000,
B10001000,
B10001000,
B10001000,
B10001000,
B01110000
};

static const uint8_t PROGMEM one[] = {
B00001000,
B00001000,
B00001000,
B00001000,
B00001000,
B00001000,
B00001000
};

static const uint8_t PROGMEM two[] = {
B01110000,
B10001000,
B00001000,
B01110000,
B10000000,
B10000000,
B11111000
};

static const uint8_t PROGMEM three[] = {
B01110000,
B10001000,
B00001000,
B00110000,
B00001000,
B10001000,
B01110000
};

static const uint8_t PROGMEM four[] = {
B00010000,
B10010000,
B10010000,
B11111000,
B00010000,
B00010000,
B00010000
};

static const uint8_t PROGMEM five[] = {
B11111000,
B10000000,
B10000000,
B11110000,
B00001000,
B10001000,
B01110000
};

static const uint8_t PROGMEM six[] = {
B01110000,
B10001000,
B10000000,
B11110000,
B10001000,
B10001000,
B01110000
};

static const uint8_t PROGMEM seven[] = {
B11111000,
B00001000,
B00001000,
B00010000,
B00010000,
B00010000,
B00010000
};

static const uint8_t PROGMEM eight[] = {
B01110000,
B10001000,
B10001000,
B01110000,
B10001000,
B10001000,
B01110000
};

static const uint8_t PROGMEM nine[] = {
B01110000,
B10001000,
B10001000,
B01111000,
B00001000,
B00001000,
B01110000
};

void setBytes(const uint8_t * b, int x, int y) {
  oled.fillRect(x, y, 16, 15, BACKGROUND);
  for (int li=0; li < 7; ++li) {
    int tmp;
    for (byte bitNr=0; bitNr<8; ++bitNr) {
      if (((b[li] >> bitNr) & 0x01)) {
        tmp = 7-bitNr;
        oled.drawPixel(x + 2*tmp  , y + 2*li   , clockColor);
        oled.drawPixel(x + 2*tmp  , y + 2*li +1, clockColor2);
        oled.drawPixel(x + 2*tmp+1, y + 2*li   , clockColor2);
        oled.drawPixel(x + 2*tmp+1, y + 2*li +1, clockColor2);
      }
    }
  }
}

int myFont(int x, int y, byte b) {
    if (b>9) {
      x = myFont(x,y,b/10);
      b = b%10;
    }
    if (b == 0) {
      setBytes(zero, x, y);
    } else if (b == 1) {
      setBytes(one, x, y);
    } else if (b == 2) {
      setBytes(two, x, y);
    } else if (b == 3) {
      setBytes(three, x, y);
    } else if (b == 4) {
      setBytes(four, x, y);
    } else if (b == 5) {
      setBytes(five, x, y);
    } else if (b == 6) {
      setBytes(six, x, y);
    } else if (b == 7) {
      setBytes(seven, x, y);
    } else if (b == 8) {
      setBytes(eight, x, y);
    } else if (b == 9) {
      setBytes(nine, x, y);
    }
    return x+16;
}

void readVcc() {
  float mv = analogRead(VBATPIN);
  mv *= 2;
  mv *= 3.3;
  vccVal = mv;
  if (vccVal > VCCMAX) vccVal = VCCMAX;
  if (vccVal < VCCMIN) vccVal = VCCMIN;
}

char umlReplace(char inChar) {
  if (inChar == 159) {
    inChar = 224; // ß
  } else if (inChar == 164) {
    inChar = 132; // ä
  } else if (inChar == 182) {
    inChar = 148; // ö
  } else if (inChar == 188) {
    inChar = 129; // ü
  } else if (inChar == 132) {
    inChar = 142; // Ä
  } else if (inChar == 150) {
    inChar = 153; // Ö
  } else if (inChar == 156) {
    inChar = 154; // Ü
  } else if (inChar == 171) {
    inChar = 0xAE; // <<
  } else if (inChar == 187) {
    inChar = 0xAF; // >>
  }  
  return inChar;  
}

short green2red(int val, int maxi) {
  if (displayOnSec >= DIMSEC) return WHITE;
  // 16 bit = 5+6+5
  short result = 0x0000;
  int redPart   = 0;
  int greenPart = 0;
  if (val > (maxi/2)) {
    greenPart = 63;
    redPart = 31 - 62 * ((float) val)/((float) maxi); // 31 = 0b11111
  } else {
    redPart = 31;
    greenPart = 127 * ((float) val)/((float) maxi); // 63 = 0b111111
  }
  result += redPart  <<11;
  result += greenPart<<5;
  return result;
}

void batteryBar() {
  batteryFrame();
  readVcc();
  int val = map(vccVal, VCCMIN, VCCMAX, 0, batLength);
  int short col = clockColor;
  oled.fillRect(oled.width()-5, 2, 4, batLength-val, BACKGROUND);
  for (int v=val; v>0; --v) {
    oled.drawLine(
      oled.width()-5, batLength-v+2,
      oled.width()-2, batLength-v+2,
      green2red(v, batLength)
    );
  }
}

inline void batteryFrame() {
  oled.drawPixel(oled.width()-4, 0, clockColor);
  oled.drawPixel(oled.width()-3, 0, clockColor);
  oled.drawRect(oled.width()-6, 1, 6, batLength+2, clockColor);  
}

inline void printClock() {
  int xx = 24;
  short oldCol;
  if (lhours != data.hour) {
    if (data.hour > 9) {
      oldCol = clockColor2;
      xx = myFont( 8, 3, data.hour/10);
      clockColor2 = oldCol;
    } else {
      oled.fillRect( 8, 3, 16, 14, BACKGROUND);
    }
    oldCol = clockColor2;
    myFont(xx, 3, data.hour - 10*(data.hour/10));
    clockColor2 = oldCol;
  }
  
  if (lseconds != data.second) {
    if (data.second%2 == 0) {
      oled.fillRect(41,  5, 2, 2, clockColor);
      oled.fillRect(41, 13, 2, 2, clockColor);
    } else {
      oled.fillRect(41,  5, 2, 2, BACKGROUND);
      oled.fillRect(41, 13, 2, 2, BACKGROUND);    
    }
  }
    
  if (lminutes != data.minute) {
    oldCol = clockColor2;
    xx = myFont(50, 3, data.minute/10);
    clockColor2 = oldCol;

    oldCol = clockColor2;
    myFont(xx, 3, data.minute - 10*(data.minute/10));
    clockColor2 = oldCol;
  }

  if (showDetails>0 && lseconds != data.second) {
    xx = myFont(55, 23, data.second/10);
    myFont(xx, 23, data.second - 10*(data.second/10));
  }

  lhours = data.hour;
  lminutes = data.minute;
  lseconds = data.second;
}

inline void ticking() {
  WDT->CTRL.reg = 0;
  while(WDT->STATUS.bit.SYNCBUSY);
  WDT->INTENSET.bit.EW   = 1;
  WDT->CONFIG.bit.PER    = 0xB;
  WDT->CONFIG.bit.WINDOW = 0x5; // 256ms
  WDT->CTRL.bit.WEN      = 1;
  while(WDT->STATUS.bit.SYNCBUSY);

  WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY;
  while(WDT->STATUS.bit.SYNCBUSY);
  
  WDT->CTRL.bit.ENABLE = 1;
  while(WDT->STATUS.bit.SYNCBUSY);

  system_set_sleepmode(SYSTEM_SLEEPMODE_STANDBY);
  system_sleep();
  
  tick++;
  if (tick>3) {
    if (displayOnSec >=0) displayOnSec++;
    if (messageOnSec >=0) messageOnSec++;
    if (messageOnSec > OFFSEC) messageOnSec = -1;
    tick=0;
    if (alarmsec>0) alarmsec--;
  }
  DS3231M_get(data);
}

inline int16_t absi(int16_t val) {
  if (val<0) return -1*val;
  else val;
}

inline byte tob(char c) { return c - '0';}

void getTime() {
  byte i = NO_TIME_THERE;
  if (serialCache[2] == ':' && serialCache[5] == ':') i=0;
  if (serialCache[3] == ':' && serialCache[6] == ':') i=1;
  if (i == NO_TIME_THERE) return;
  data.hour = tob(serialCache[i])*10 + tob(serialCache[1+i]);
  data.minute = tob(serialCache[3+i])*10 + tob(serialCache[4+i]);
  data.second = tob(serialCache[6+i])*10 + tob(serialCache[7+i]);
  DS3231M_set(data);
}

void setup() {
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);
  pinMode(BUTTON3, INPUT_PULLUP);

  pinMode(POTI, INPUT);
  pinMode(SPEAKER, OUTPUT);
  pinMode(LED_WHITE, OUTPUT);
  pinMode(VIBRATE, OUTPUT);
    
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  ble.begin(false);
  ble.echo(false);

  ble.sendCommandCheckOK("AT+HWModeLED=BLEUART");
  ble.sendCommandCheckOK("AT+GAPDEVNAME=Pager");
  ble.sendCommandCheckOK("ATE=0");
  ble.sendCommandCheckOK("AT+BAUDRATE=115200");
  ble.sendCommandCheckOK("ATZ");
  ble.setMode(BLUEFRUIT_MODE_DATA);

  ble.verbose(false);
  delay(7);
  
#if SET_INIT_RTC > 0
  DS3231M_set(data);
#endif
  oled.begin();
  
  oled.fillScreen(BACKGROUND);
  oled.setTextSize(1);

  GCLK->GENDIV.reg = GCLK_GENDIV_ID(2) | GCLK_GENDIV_DIV(4);
  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(2) |
                      GCLK_GENCTRL_GENEN |
                      GCLK_GENCTRL_SRC_OSCULP32K |
                      GCLK_GENCTRL_DIVSEL;
  while(GCLK->STATUS.bit.SYNCBUSY);
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_WDT |
                      GCLK_CLKCTRL_CLKEN |
                      GCLK_CLKCTRL_GEN_GCLK2;
}





void loop() {
  ticking();
  
  if (ble.isConnected()) {
    while ( ble.available() ) {
      contin = false;
      messageOnSec = 0;
      displayOnSec = 0;
      showDetails = 1;
      clockColor2 = RED2;
      oled.writeCommand(SSD1331_CMD_DISPLAYON);
      char inChar = (char) ble.read();
      if (inChar == 194) continue; // symbol before utf-8
      if (inChar == 195) continue; // symbol before utf-8

      if (inChar == '\n') {
        getTime();
        serialCache[cpos] = '\0';
        cpos = 0;
        contin = true; 
      }
      inChar = umlReplace(inChar);

      if (contin == false) {
        serialCache[cpos] = inChar;
        cpos++;        
      }
    }
  }

  if (digitalRead(BUTTON3) == LOW) {
    alarmsec = -1; // off
    vibr = false;
    wled = false;
    digitalWrite(VIBRATE, vibr);
    digitalWrite(LED_WHITE, wled);
    snprintf(serialCache,32,"       ");
  }
  
  if (digitalRead(BUTTON2) == LOW) {
    displayOnSec=0;
    oled.writeCommand(SSD1331_CMD_DISPLAYON);
    alarmsec = (alarmsec+30)%600;
  }

  if (alarmsec == 0) {
    vibr = !vibr;
    digitalWrite(VIBRATE, vibr);
    wled = !wled;
    digitalWrite(LED_WHITE, wled);
    analogWrite(SPEAKER, 50);   
  }
      
  if (tick == 0  && displayOnSec >= 0) {
    if (displayOnSec < DIMSEC) {
      potival = analogRead(POTI);
      clockColor2 = colors[map(potival, 0, 1024, 0, 14)];
    }

    if (showDetails==0) {
      printClock();
      oled.setTextColor(clockColor2, BACKGROUND);
      oled.setCursor(19, 28);
      oled.drawBitmap(0, 21, step_bmp, 16, 16, clockColor2);
      oled.print(steps);     
      
    } else if (showDetails==2) {
      oled.setTextColor(YELLOW, GREYBLUE);
      oled.setCursor(5, 2);
      oled.print(" ");
      oled.print(data.hour);
      oled.print(":");
      if (data.minute<10) oled.print("0");
      oled.print(data.minute);
      oled.print(":");
      if (data.second<10) oled.print("0");
      oled.print(data.second);
      oled.print(" ");
      
      oled.setTextColor(WHITE, RED2);
      oled.setCursor(5, 10);
      oled.print(" ");
      oled.print(data.day);
      oled.print(".");
      oled.print(data.month);
      oled.print(".20");      
      oled.print(data.year);
      oled.print("  ");

      oled.setTextColor(LIGHTBLUE, BACKGROUND);
      oled.setCursor(5, 18);
      switch(data.dofweek) {
        case 1:
          oled.print(" Montag    ");
          break;
        case 2:
          oled.print(" Dienstag  ");
          break;
        case 3:
          oled.print(" Mittwoch  ");
          break;
        case 4:
          oled.print(" Donnerstag");
          break;
        case 5:
          oled.print(" Freitag   ");
          break;
        case 6:
          oled.print(" Samstag   ");
          break;
        case 7:
          oled.print(" Sonntag   ");
          break;
        default:
          ;
      }
      
    } else {
      printClock();
      oled.setTextColor(clockColor, BACKGROUND);
      oled.setCursor(0, 40);
      oled.print("         ");
      oled.setCursor(0, 40);
      if (alarmsec>0) {
        snprintf(serialCache,32,"%3d sec  ",alarmsec);
        oled.print(serialCache);
      } else {
        if (deltax < 120) {
          oled.print(" -> sky  ");
        } else if (deltax > 240) {
          oled.print(" -> floor");
        } else {
          oled.print(serialCache);
        }
      }
      oled.setCursor(19, 28);
      oled.drawBitmap(0, 21, step_bmp, 16, 16, clockColor2);
      oled.print(steps);
    }
  }
  
  analogWrite(SPEAKER, 0);
  
  if (digitalRead(BUTTON) == LOW) {
    pressTicks++;
    displayOnSec=0;
    if (showDetails>0) batteryBar();
    clockColor2 = RED2;
    lhours++; // force refresh
    lminutes++;
    oled.writeCommand(SSD1331_CMD_DISPLAYON);
    
    if (pressTicks>1) {
      pressTicks=0;
      showDetails++;
      oled.fillRect( 0, 0, 86, 37, BACKGROUND);
      analogWrite(SPEAKER, 50);
    }
  } else {
    pressTicks = -1;
  }

  if (showDetails>0 && data.second%3 == 0 && tick==0 && displayOnSec>=0 && messageOnSec < 0) {
    batteryBar();
  }

  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,14,true);  // request a total of 14 registers
  
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  
  stoss = absi(AcX) + absi(AcY) + absi(AcZ);
  
  barrier = treshold*5000L + 8000L;
  if (stoss > barrier) {
    steps++;
  }
  if (AcX > ONACCEL) {    
    displayOnSec=0;
    if (showDetails>0) batteryBar();
    clockColor2 = RED2;
    lhours++; // force refresh
    lminutes++;
    oled.writeCommand(SSD1331_CMD_DISPLAYON);
  }
    
  if (GyX > 17200) GyX = 17200;
  if (GyY > 17200) GyY = 17200;
  if (GyX < -17200) GyX = -17200;
  if (GyY < -17200) GyY = -17200;

  deltax = map(GyX, -17200, 17200, 0, 360);
  deltay = map(GyY, -17200, 17200, 0, 360); 

  if (displayOnSec >= DIMSEC) {
    clockColor2 = RED;
    lhours++; // force refresh
    lminutes++;
    oled.writeCommand(SSD1331_CMD_DISPLAYDIM);
  }
  
  if (displayOnSec > OFFSEC) {
    clockColor2 = RED2;
    oled.fillScreen(BLACK);
    lhours++; // force refresh
    lminutes++;
    oled.writeCommand(SSD1331_CMD_DISPLAYOFF);
    displayOnSec = -1;
    messageOnSec = -1;
    showDetails = 0;
  }
}

ISR(WDT_vect) {
  WDT->CTRL.bit.ENABLE = 0;        // Disable watchdog
  while(WDT->STATUS.bit.SYNCBUSY); // Sync CTRL write
  WDT->INTFLAG.bit.EW  = 1;        // Clear interrupt flag  
}
