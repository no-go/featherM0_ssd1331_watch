#define ADDR_RTCDS3231M 0x68

struct RTCdata {
  byte second; 
  byte minute;
  byte hour;
  byte dofweek;
  byte day;
  byte month;
  byte year;  
};

byte decToBcd(byte val) {  return( (val/10*16) + (val%10) );  }
byte bcdToDec(byte val)  {  return( (val/16*10) + (val%16) ); }

void DS3231M_set(const RTCdata & d) {
  // sets time and date data to DS3231
  Wire.beginTransmission(ADDR_RTCDS3231M);
  Wire.write(0); // set next input to start at the seconds register
  Wire.write(decToBcd(d.second)); // set seconds
  Wire.write(decToBcd(d.minute)); // set minutes
  Wire.write(decToBcd(d.hour)); // set hours
  Wire.write(decToBcd(d.dofweek)); // set day of week (1=Sunday, 7=Saturday)
  Wire.write(decToBcd(d.day)); // set date (1 to 31)
  Wire.write(decToBcd(d.month)); // set dmonth
  Wire.write(decToBcd(d.year)); // set dyear (0 to 99)
  Wire.endTransmission();
}

void DS3231M_get(RTCdata & d) {
  Wire.beginTransmission(ADDR_RTCDS3231M);
  Wire.write(0); // set DS3231 register pointer to 00h
  Wire.endTransmission();
  Wire.requestFrom(ADDR_RTCDS3231M, 7);
  // request seven bytes of data from DS3231 starting from register 00h
  d.second = bcdToDec(Wire.read() & 0x7f);
  d.minute = bcdToDec(Wire.read());
  d.hour = bcdToDec(Wire.read() & 0x3f);
  d.dofweek = bcdToDec(Wire.read());
  d.day = bcdToDec(Wire.read());
  d.month = bcdToDec(Wire.read());
  d.year = bcdToDec(Wire.read());
}
