# feather M0 ssd1331 smartwatch

using a adafruit feather M0 with SPI oled SSD1331 (16bit) and MPU-6050 as watch and step counter.

# bugs

sometimes I got a restart and time accurency is bad :-S

# Hardware and Features

 -  feather M0 with BLE module nRF52 (set clock and send small messages via UART)
 -  OLED 16bit color SPI display (clock, powerbar, with dim mode and off mode)
 -  MPU6050 i2c gyroscope chip (shake to switch display on and steps counting)
 -  small ear speaker (beep on button press)
 -  3 buttons: set hours, minutes, display on and show details (long press)
 -  25k poti to change the clock color
 -  3,7V 180mAh lipo (>10h)
 -  save power and high time accurency: Real Time Clock DS3231M is added to i2c

The 2 buttons to set hours and minutes is an option (set `#define SET_BUTTONS  0` from 0 to 1).

I use the watchdog timer https://github.com/adafruit/Adafruit_SleepyDog to sleep
and NOT `delay()` to save power!

# Circuit hints

 -  set DS3231M-VCC-pin to the 3V pin on your feather M0
 -  set DS3231M-SCL,SDA pins to the same pins on your feather M0
 -  set DS3231M-BAT-pin to the lipo (holding time and let clock working)
 -  set DS3231M-GND-pin to GND
 -  set AD0 of your MPU-6050 gyroscope to 3V high, because it needs the 0x69 address on I2C

# screenshots

## wackup

shake or press button

![wackup](wackup.jpg)

## dim display

![rgb dim mode](dim.jpg)

## display off

it is a black display and save power :-D

## longpress details

![details](details.jpg)

## bluetooth UART message

![message via uart](message.jpg)

## poti for different color

![color 1](color1.jpg)
![color 2](color2.jpg)
![color 3](color3.jpg)
![color 4](color4.jpg)
![color 5](color5.jpg)
