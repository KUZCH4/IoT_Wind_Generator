#include <Arduino.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <RTClib.h>
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <TimeLib.h>
#include <HTU21D.h>
#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <PZEM004Tv30.h>
//#include <Adafruit_GFX.h>
//#include <Adafruit_SSD1306.h>
#include <icons_energo.h>

#if defined(ESP8266)
  #define BUTTON_A  0
  #define BUTTON_B 16
  #define BUTTON_C  2
  #define WIRE Wire
#elif defined(ESP32)
  #define BUTTON_A 15
  #define BUTTON_B 32
  #define BUTTON_C 14
  #define WIRE Wire
#elif defined(ARDUINO_STM32_FEATHER)
  #define BUTTON_A PA15
  #define BUTTON_B PC7
  #define BUTTON_C PC5
  #define WIRE Wire
#elif defined(TEENSYDUINO)
  #define BUTTON_A  4
  #define BUTTON_B  3
  #define BUTTON_C  8
  #define WIRE Wire
#elif defined(ARDUINO_FEATHER52832)
  #define BUTTON_A 31
  #define BUTTON_B 30
  #define BUTTON_C 27
  #define WIRE Wire
#elif defined(ARDUINO_ADAFRUIT_FEATHER_RP2040)
  #define BUTTON_A  9
  #define BUTTON_B  8
  #define BUTTON_C  7
  #define WIRE Wire1
#else // 32u4, M0, M4, nrf52840 and 328p
  #define BUTTON_A  9
  #define BUTTON_B  6
  #define BUTTON_C  5
  #define WIRE Wire
#endif

/* display pins:
  | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 | 13 | 14 | 15 | 16 | 17 | 18 | 19 | 20 |
  |GND|Vcc|Vo |RS |R/W| E |DB0|DB1|DB2|DB3 |DB4 |DB5 |DB6 |DB7 |PSB |NC  |RST |VOUT| A  | K  |
 
  U8G2_KS0108_128X64_F u8g2(ROTATION, D0, D1, D2, D3, D2, D3, D4, D5, D6, D7,
                            EN, D/I, CS0, CS1, CS2, RESET);
  ROTATION:   
    U8G2_R0:  0 
    U8G2_R1:  90 
    U8G2_R2:  180 
    U8G2_R3:  270  
*/

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3D ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
//U8G2_ST7920_128X64_F_SW_SPI u8g2(U8G2_R0, /* clock=*/ D5, /* data=*/ D6, /* CS=*/ D7 /* reset=*/ ); // Feather HUZZAH ESP8266, E=clock=14, RW=data=13, RS=C
//Adafruit_SSD1306 u8g2 = Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &WIRE, OLED_RESET);

const char *ssid     = "test";
const char *password = "password";

static const char ntpServerName[] = "ua.pool.ntp.org";
const int timeZone = 2;
WiFiUDP Udp;
unsigned int localPort = 8888;

//const int interruptPin = 0;

const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}

time_t getNtpTime()
{
  IPAddress ntpServerIP; // NTP server's ip address

  while (Udp.parsePacket() > 0) ; // discard any previously received packets
  Serial.println("Transmit NTP Request");
  // get a random server from the pool
  WiFi.hostByName(ntpServerName, ntpServerIP);
  Serial.print(ntpServerName);
  Serial.print(": ");
  Serial.println(ntpServerIP);
  sendNTPpacket(ntpServerIP);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      Serial.println("Receive NTP Response");
      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
    }
  }
  //Serial.println("No NTP Response :-(");
  return 0; // return 0 if unable to get the time
}


Adafruit_BME280 bme;
RTC_DS3231 rtc;
HTU21D            myHTU21D(HTU21D_RES_RH12_TEMP14);
SoftwareSerial pzemSWSerial(D3, D8);
//PZEM004Tv30 pzem(pzemSWSerial);
PZEM004Tv30 pzem(Serial);

char line_volts[3], line_amps[4], line_energy[5], line_power[4];
uint32_t timer_H, disp_EN_timer;
const uint32_t pr_interval = 600000;
float   temperature = 0, aval, vt, ap, pw, kw, pr_vt, pr_ap, pr_pw, pr_kw;
float   humidity    = 0;
bool bme_state = 1;
uint32_t pressure_array[6];
short frame_count = 0;
short screen_count = 0;
short wind_state = 0;
short water_amount;
volatile uint16_t anem_rpm;
unsigned long lastflash=0;
uint32_t sumX, sumY, sumX2, sumXY;
int delta;
int dispRain;
byte time_array[6];
uint32_t frame_timer, screen_timer, click_timer;
bool frame_flag, time_flag, pzem_flag, NTP_flag, SHT_flag, WiFi_conection_status = true, summer_time;

void predict_wh(){
  long averPress = 0;
    for (byte i = 0; i < 10; i++) {
      bme.takeForcedMeasurement();
      averPress += bme.readPressure();
      delay(1);
    }
    averPress /= 10;

    for (byte i = 0; i < 5; i++) {                   // счётчик от 0 до 5 (да, до 5. Так как 4 меньше 5)
      pressure_array[i] = pressure_array[i + 1];     // сдвинуть массив давлений КРОМЕ ПОСЛЕДНЕЙ ЯЧЕЙКИ на шаг назад
    }
    pressure_array[5] = averPress;                    // последний элемент массива теперь - новое давление
    sumX = 0;
    sumY = 0;
    sumX2 = 0;
    sumXY = 0;
    for (int i = 0; i < 6; i++) {                    // для всех элементов массива
      sumX += time_array[i];
      sumY += (long)pressure_array[i];
      sumX2 += time_array[i] * time_array[i];
      sumXY += (long)time_array[i] * pressure_array[i];
    }
    aval = 0;
    aval = (long)6 * sumXY;             // расчёт коэффициента наклона приямой
    aval = aval - (long)sumX * sumY;
    aval = (float)aval / (6 * sumX2 - sumX * sumX);
    delta = aval * 6;      // расчёт изменения давления
    dispRain = map(delta, -250, 250, 100, -100);
  }

void wind_spd(){
  anem_rpm = 0.31415*4/(((float)(micros()-lastflash)/1000000));
  lastflash=micros();
}

void drawICONS(){
  for(int i=0; i<30; i++){
    for(int j=0; j<18; j++){
      if (frame_bat[j][i] == 1){
        u8g2.drawPixel(i+45,j+23);
      }
    }
  }

  for(int i=0; i<15; i++){
    for(int j=0; j<27; j++){
      if (frame_balast[j][i] == 1){
        u8g2.drawPixel(i+77,j+14);
      }
    }
  }
  for(int i=0; i<11; i++){
    for(int j=0; j<25; j++){
      if (frame_lightning[j][i] == 1){
        u8g2.drawPixel(i+2,j+2);
      }
    }
  }
  for(int i=0; i<52; i++){
    if (vertical_line[i] == 1){
        u8g2.drawPixel(0,i);
        u8g2.drawPixel(42,i);
      }
  }
  for(int i=1; i<42; i++){
    if (vertical_line[i] == 1){
      u8g2.drawPixel(i,0);
      u8g2.drawPixel(i,35);
    }
  }
  if (wind_state == 0){
    if(frame_flag){
      for(int i=0; i<49; i++){
        for(int j=0; j<10; j++){
          if (line_onBAT1[j][i] == 1){
            u8g2.drawPixel(i+57,j+41);
          }
        }
      }
    } else{
      for(int i=0; i<49; i++){
        for(int j=0; j<10; j++){
          if (line_onBAT2[j][i] == 1){
            u8g2.drawPixel(i+57,j+41);
          }
        }
      }
    }
  }

  if (wind_state == 1){
    if(frame_flag){
      for(int i=0; i<49; i++){
        for(int j=0; j<10; j++){
          if (line_onBAL1[j][i] == 1){
            u8g2.drawPixel(i+57,j+41);
          }
        }
      }
    } else{
      for(int i=0; i<49; i++){
        for(int j=0; j<10; j++){
          if (line_onBAL2[j][i] == 1){
            u8g2.drawPixel(i+57,j+41);
          }
        }
      }
    }
  }

  if (wind_state == 2){
    for(int i=0; i<49; i++){
      for(int j=0; j<10; j++){
        if (line_NO[j][i] == 1){
          u8g2.drawPixel(i+57,j+41);
        }
      }
    }
  }

  u8g2.setFont(u8g2_font_b10_t_japanese1);
  if (!frame_flag){
    if (wind_state == 0)
      u8g2.drawStr(47, 10, "!RUNNING!");
    else if (wind_state == 1)
      u8g2.drawStr(43, 10, "!ON BALAST!");
    else if (wind_state == 2)
      u8g2.drawStr(47, 10, ">NO WIND<");
  }
}

void drawCLK(){
  DateTime now = rtc.now();
  u8g2.setFont(u8g2_font_t0_11b_tf);
  char t_hour[2], t_minute[2], t_day[2], t_month[2];
  dtostrf((now.hour()), 2, 0, t_hour);
  if (now.hour()<10){u8g2.drawStr(3, 62, "0");}
  u8g2.drawStr(3, 62, t_hour);

  if (!frame_flag){
    u8g2.drawStr(14, 61, ":");
    }

  dtostrf((now.minute()), 2, 0, t_minute);
  if (now.minute()<10){u8g2.drawStr(20, 62, "0");}
  u8g2.drawStr(20, 62, t_minute);

  dtostrf((now.day()), 2, 0, t_day);
  if (now.day()<10){u8g2.drawStr(37, 62, "0");}
  u8g2.drawStr(37, 62, t_day);
  u8g2.drawStr(49, 62, "/");
  if (now.month()<10){u8g2.drawStr(55, 62, "0");}
  dtostrf((now.month()), 2, 0, t_month);
  u8g2.drawStr(55, 62, t_month);

  switch (now.dayOfTheWeek())
  {
  case 0:
    u8g2.drawStr(72, 62, "SUN");
    break;
  case 1:
    u8g2.drawStr(72, 62, "MON");
    break;
  case 2:
    u8g2.drawStr(72, 62, "TUE");
    break;
  case 3:
    u8g2.drawStr(72, 62, "WED");
    break; 
  case 4:
    u8g2.drawStr(72, 62, "THU");
    break;
  case 5:
    u8g2.drawStr(72, 62, "FRI");
    break;
  case 6:
    u8g2.drawStr(72, 62, "SAT");
    break;
        
  default:
    break;
  }

  u8g2.drawLine(0, 63, 0, 52);
  u8g2.drawLine(1, 52, 92, 52);
  u8g2.drawLine(1, 63, 92, 63);
  u8g2.drawLine(92, 62, 92, 53);
}

void drawWeather(){
  for(int i=0; i<28; i++){
    for(int j=0; j<22; j++){
      if (WiFi_high_ICON[j][i] == 1){
        u8g2.drawPixel(i+96,j+2);
      }
    }
  }  

  for(int i=0; i<19; i++){
    for(int j=0; j<27; j++){
      if (windFlagICO[j][i] == 1){
        u8g2.drawPixel(i+48,j+24);
      }
    }
  }  

  /*for(int i=0; i<7; i++){
    for(int j=0; j<10; j++){
      if (waterICO[j][i] == 1){
        u8g2.drawPixel(i+37,j+40);
      }
    }
  } */ 
  for(int i=0; i<11; i++){
    if (vertical_line[i] == 1){
        u8g2.drawPixel(127,i+52); 
      }
  }
  for(int i=0; i<35; i++){
    if (vertical_line[i] == 1){
        u8g2.drawPixel(i+92, 0);
        u8g2.drawPixel(i+93, 63);
      }
  }
  for(int i=0; i<47; i++){
    if (vertical_line[i] == 1){
        u8g2.drawPixel(i,0);
        u8g2.drawPixel(i,22);
        u8g2.drawPixel(i+46,0);
        u8g2.drawPixel(i+46,22);
      }
  }
  for(int i=0; i<9; i++){
    for(int j=0; j<9; j++){
      if (wICO_IN[j][i] == 1){
        u8g2.drawPixel(i+2,j+2);
      }
    }
  }
  for(int i=0; i<9; i++){
    for(int j=0; j<9; j++){
      if (wICO_OUT[j][i] == 1){
        u8g2.drawPixel(i+48,j+2);
      }
    }
  }    
  if (frame_flag){
    for(int i=0; i<26; i++){
      for(int j=0; j<27; j++){
        if (weather_SRAIN[j][i] == 1){
          u8g2.drawPixel(i+97,j+35);
        }
      }
    }
    /*for(int i=0; i<20; i++){
      for(int j=0; j<15; j++){
        if (ventICO2[j][i] == 1){
          u8g2.drawPixel(i+3,j+25);
        }
      }
    }*/
    for(int i=0; i<16; i++){
      for(int j=0; j<7; j++){
        if (windFlowICO1[j][i] == 1){
          u8g2.drawPixel(i+70,j+26);
        }
      }
    }
  }
  if (!frame_flag){
    for(int i=0; i<26; i++){
      for(int j=0; j<27; j++){
        if (weather_SRAIN2[j][i] == 1){
          u8g2.drawPixel(i+97,j+35);
        }
      }
    }
    /*for(int i=0; i<20; i++){
      for(int j=0; j<15; j++){
        if (ventICO1[j][i] == 1){
          u8g2.drawPixel(i+3,j+25);
        }
      }
    }*/
    for(int i=0; i<16; i++){
      for(int j=0; j<7; j++){
        if (windFlowICO2[j][i] == 1){
          u8g2.drawPixel(i+70,j+26);
        }
      }
    }  
  }

  char hum1[3], temp1[5], wind_SPEED[4], hum2[3], temp2[5], press_pr[4], press[3]; 
  u8g2.setFont(u8g2_font_mozart_nbp_tf);
  humidity    = myHTU21D.readCompensatedHumidity();
  temperature = myHTU21D.readTemperature();
  if   (temperature != HTU21D_ERROR) {
    dtostrf(temperature, 5, 1, temp1);
    u8g2.drawUTF8(2, 20, temp1);
    u8g2.drawUTF8(31, 20, "°C");
  }
  else u8g2.drawUTF8(2, 20, "NULL");
  if   (humidity != HTU21D_ERROR) {
    dtostrf(humidity, 5, 0, hum1);
    u8g2.drawUTF8(5, 10, hum1);
    u8g2.drawUTF8(35, 10, "%");
  }
  else u8g2.drawUTF8(2, 20, "NULL");
  bme.takeForcedMeasurement();
  dtostrf((float)bme.readHumidity(), 3, 0, hum2);
  u8g2.drawUTF8(62, 10, hum2);
  u8g2.drawUTF8(81, 10, "%");
  
  dtostrf((float)bme.readTemperature(), 5, 1, temp2);
  u8g2.drawUTF8(48, 20, temp2);
  u8g2.drawUTF8(77, 20, "°C");
  
  dtostrf((float)bme.readPressure() / 100.0F, 3, 0, press);
  u8g2.drawUTF8(58, 50, press);
  u8g2.drawUTF8(77, 50, "mm");

  if (millis() - timer_H >= pr_interval){
    timer_H = millis();
    if(bme_state){
      predict_wh();
    }
  }

  dtostrf(dispRain, 4, 0, press_pr);
  u8g2.drawStr(95, 33, press_pr);
  u8g2.drawStr(119, 33, "%");
  dtostrf(anem_rpm, 4, 1, wind_SPEED);
  u8g2.drawStr(49, 41, wind_SPEED);
  u8g2.drawStr(74, 41, "m/s");

  /*dtostrf(water_amount*5, 3, 0, val);
  u8g2.drawStr(3, 50, val);
  u8g2.drawStr(21, 50, "%");
  */
  for(int i=0; i<52; i++){
    if (vertical_line[i] == 1){
        u8g2.drawPixel(92,i);
        u8g2.drawPixel(127,i);
        u8g2.drawPixel(0,i);
        u8g2.drawPixel(46,i);
      }
  }
  /*
  u8g2.drawLine(28,25,28,49);
  u8g2.drawLine(34,25,34,49);
  u8g2.drawLine(29,50,33,50);
  
  u8g2.drawLine(29,50,29,water_amount+25);
  u8g2.drawLine(30,50,30,water_amount+25);
  u8g2.drawLine(31,50,31,water_amount+25);
  u8g2.drawLine(32,50,32,water_amount+25);
  u8g2.drawLine(33,50,33,water_amount+25); 
*/
}

void read_energy(){
  vt = pzem.voltage();
  if (isnan(vt)){
    vt = pr_vt; 
  }
  pr_vt = vt;
  ap = pzem.current();
  if (isnan(ap)){
    ap = pr_ap;
  }
  pr_ap = ap;
  pw = pzem.power();
  if (isnan(pw)){
    pw = pr_pw;
  }
  pr_pw = pw;
  kw = pzem.energy();
  if (isnan(kw)){
    kw = pr_kw;
  }
  pr_kw = kw;
  pzem_flag = 1;
  /*dtostrf(pzem.voltage(), 3, 0, line_volts);
  dtostrf(pzem.current(), 4, 1, line_amps);
  dtostrf(pzem.power(), 4, 0, line_power);
  dtostrf(pzem.energy(), 5, 1, line_energy);*/
}

void drawEN(){
  drawICONS();
  u8g2.setFont(u8g2_font_courR08_tr);
  u8g2.drawStr(45, 21, "24.0V");
  dtostrf(vt, 3, 0, line_volts);
  u8g2.drawStr(16, 10, line_volts);
  u8g2.drawStr(35, 10, "V");
  dtostrf(ap, 4, 1, line_amps);
  u8g2.drawStr(10, 18, line_amps);
  u8g2.drawStr(35, 18, "A");
  dtostrf(pw, 4, 0, line_power);
  u8g2.drawStr(10, 26, line_power);
  u8g2.drawStr(35, 26, "W");
  u8g2.setFont(u8g2_font_tom_thumb_4x6_tr);
  dtostrf(kw, 5, 1, line_energy);
  u8g2.drawStr(4, 33, line_energy);
  u8g2.drawStr(24, 33, "kW/h");
  u8g2.drawStr(4, 43, "->0W");
  u8g2.drawStr(4, 50, "0.0kW/h");
  
}

//---setup
void setup() {
  //Serial.begin(9600);
  u8g2.begin();
  //u8g2.begin();
  u8g2.setFont(u8g2_font_mozart_nbp_tf);
  Wire.begin();
  u8g2.firstPage();
  do{
   u8g2.drawStr(2,10, "verifying...");
  } while (u8g2.nextPage());
  delay(1000);
  short dot_counter = 1, counter = 1, try_conect = 1;
  WiFi.begin(ssid, password);
      while (WiFi.status() != WL_CONNECTED) {
        WiFi_conection_status = false;
        do{
          char valu[2];
          u8g2.firstPage();
          u8g2.drawStr(2,10, "verifying...");
          dtostrf(counter, 2, 0, valu);
          u8g2.drawStr(75,10, valu);
          u8g2.drawStr(87,10, "%");
          counter+=3;
          u8g2.drawStr(2,20, "-WiFi conecting");  
          delay ( 500 );
          for (int i=0; i<dot_counter; i++){
            u8g2.drawStr(dot_counter*3+90, 20, ".");
          }
          dot_counter++;    
          if (dot_counter > 3) {dot_counter = 1;}
        } while (u8g2.nextPage());
        if (try_conect >= 24){
          break;
        }
        try_conect++;
      }

  if (WiFi.isConnected()){
    WiFi_conection_status = true;
    do{
      u8g2.firstPage();
      u8g2.drawStr(2,10, "verifying...50%");
      u8g2.drawStr(2,20, "-WiFi conected");
    } while (u8g2.nextPage());
  }  
  delay(1000); 
  Udp.begin(localPort);
  setSyncProvider(getNtpTime);
  setSyncInterval(300);
  if (! rtc.begin()) {
    do{
      u8g2.firstPage();
      u8g2.drawStr(2,10, "verifying...50%");
      if(WiFi_conection_status == 1){
        u8g2.drawStr(2,20, "-WiFi conected");
      } else{
      u8g2.drawStr(2,20, "!-Can't conect WiFi");  
      }
      u8g2.drawStr(2, 30, "error: no RTC");
    } while (u8g2.nextPage());
    delay (3000);
  }
  for (int p=0; p<9; p++){
  do{
    u8g2.firstPage();
    u8g2.drawStr(2,10, "verifying...75%");
    if(WiFi_conection_status == 1){
        u8g2.drawStr(2,20, "-WiFi conected");
      } else{
      u8g2.drawStr(2,20, "!-Can't conect WiFi");  
      }
    u8g2.drawStr(2, 30, "-Setting RTC by NTP");
    for (int i=0; i<dot_counter; i++){
          u8g2.drawStr(dot_counter*3+113, 30, ".");
        }
        dot_counter++;  
        if (dot_counter > 3) {dot_counter = 1;}
    } while (u8g2.nextPage());
    delay(300);
  }

  if (DateTime(minute()) < 1){
   do{
    u8g2.firstPage();
    u8g2.drawStr(2,10, "verifying...80%");
    if(WiFi_conection_status == 1){
        u8g2.drawStr(2,20, "-WiFi conected");
      } else{
      u8g2.drawStr(2,20, "!-Can't conect WiFi");  
      }
    u8g2.drawStr(2, 30, "!-Can't conect to NTP");
    NTP_flag = 0; 
   } while (u8g2.nextPage()); 
  }

  else{
    if ((DateTime(month())>3) & (DateTime(month())<11)){
      summer_time = 1;
    }
    else{summer_time = 0;}
    rtc.adjust(DateTime(year(), month(), day(), hour()+(uint8_t)summer_time, minute(), second()));
    NTP_flag = 1;
    do{
      u8g2.firstPage();
      u8g2.drawStr(2,10, "verifying...80%");
      if(WiFi_conection_status == 1){
        u8g2.drawStr(2,20, "-WiFi conected");
      } else{
      u8g2.drawStr(2,20, "!-Can't conect WiFi");  
      }
      u8g2.drawStr(2, 30, "-RTC is up to date");
    } while (u8g2.nextPage());
  }

  delay (1000);

  if (myHTU21D.begin() != true){
    do{
      SHT_flag=0;
      u8g2.firstPage();
      u8g2.drawStr(2,10, "verifying...80%");
      if(WiFi_conection_status == 1){
        u8g2.drawStr(2,20, "-WiFi conected");
      } else{
      u8g2.drawStr(2,20, "!-Can't conect WiFi");  
      }
      if (NTP_flag == 1){
        u8g2.drawStr(2, 30, "-RTC is up to date");
      }
      else{
        u8g2.drawStr(2, 30, "!-Can't conect to NTP");
      }
      u8g2.drawStr(2, 40, "-SHT21 error!:(");
    } while (u8g2.nextPage());
    delay(1000);
  }
  else{
  do{
      SHT_flag=1;   
      u8g2.firstPage();
      u8g2.drawStr(2,10, "verifying...90%");
      if(WiFi_conection_status == 1){
        u8g2.drawStr(2,20, "-WiFi conected");
      } else{
      u8g2.drawStr(2,20, "!-Can't conect WiFi");  
      }
      if (NTP_flag == 1){
        u8g2.drawStr(2, 30, "-RTC is up to date");
      }
      else{
        u8g2.drawStr(2, 30, "!-Can't conect to NTP");
      }
      if(SHT_flag==1){u8g2.drawStr(2, 40, "-SHT21 is OK");}
      else{u8g2.drawStr(2, 40, "-SHT21 error!:(");}
    } while (u8g2.nextPage());
  }
  delay(1000); 
  if (!bme.begin(0x76, &Wire)){
    do{
      u8g2.firstPage();
      u8g2.drawStr(2,10, "verifying...100%");
      if(WiFi_conection_status == 1){
        u8g2.drawStr(2,20, "-WiFi conected");
      } else{
      u8g2.drawStr(2,20, "!-Can't conect WiFi");  
      }
      if (NTP_flag == 1){
        u8g2.drawStr(2, 30, "-RTC is up to date");
      }
      else{
        u8g2.drawStr(2, 30, "!-Can't conect to NTP");
      }
      if(SHT_flag==1){u8g2.drawStr(2, 40, "-SHT21 is OK");}
      else{u8g2.drawStr(2, 40, "-SHT21 error!:(");}
      u8g2.drawStr(2, 50, "-BME280 error!:(");
    } while (u8g2.nextPage());
    bme_state = 0;
  }
  else{  
  do{
        u8g2.firstPage();
        u8g2.drawStr(2,10, "verifying...100%");
        if(WiFi_conection_status == 1){
        u8g2.drawStr(2,20, "-WiFi conected");
      } else{
      u8g2.drawStr(2,20, "!-Can't conect WiFi");  
      }
        if (NTP_flag == 1){
        u8g2.drawStr(2, 30, "-RTC is up to date");
      }
      else{
        u8g2.drawStr(2, 30, "!-Can't conect to NTP");
      }
      if(SHT_flag==1){u8g2.drawStr(2, 40, "-SHT21 is OK");}
      else{u8g2.drawStr(2, 40, "-SHT21 error!:(");}
        u8g2.drawStr(2, 50, "-BME280 is OK");
      } while (u8g2.nextPage());
      bme.setSampling(Adafruit_BME280::MODE_FORCED,
                  Adafruit_BME280::SAMPLING_X1, // temperature
                  Adafruit_BME280::SAMPLING_X1, // pressure
                  Adafruit_BME280::SAMPLING_X1, // humidity
                  Adafruit_BME280::FILTER_OFF   );
  }  
  delay(2000);
  do{
      u8g2.firstPage();
      u8g2.setFont(u8g2_font_HelvetiPixel_tr);
      u8g2.drawStr(42,15, "Welcome");
      u8g2.drawStr(58,29, "to");
      u8g2.setFont(u8g2_font_crox5tb_tr);
      u8g2.drawStr(21,52, "KUZCH");
    } while (u8g2.nextPage());
    delay (3000);
    //pinMode(interruptPin, INPUT_PULLUP);
    //attachInterrupt(digitalPinToInterrupt(interruptPin), wind_spd, FALLING);
    
  //if (rtc.lostPower()) {
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  //}  

}

//---main cylce
void loop() {
  if ((micros()-lastflash)>3000000){
    anem_rpm = 0;
    }
  if (millis() - click_timer > 500){
    click_timer = millis();
    if(frame_flag == 0)
      frame_flag = 1;
    else 
      frame_flag = 0;
    water_amount++;
    if(water_amount > 25) {water_amount = 0;} 
  }
  if ((millis()-disp_EN_timer)>=1000 && !pzem_flag && screen_count == 1){
    read_energy();       
    disp_EN_timer = millis(); 
  }
  else pzem_flag = 0;
  if (millis() - screen_timer > 10000){
    screen_timer = millis();
    screen_count++;
    wind_state++;
    if (wind_state > 2) {wind_state = 0;}
    if (screen_count > 1){screen_count = 0;}
  }
  u8g2.firstPage();
  if(screen_count == 0){
    if(millis() - frame_timer > 400){
      frame_timer = millis();
      frame_count++;
      if (frame_count > 2){frame_count = 0;}
    }
    do{
      drawCLK();
      drawWeather();
    } while (u8g2.nextPage());
  }
  

  else if(screen_count == 1){
    if(millis() - frame_timer > 400){
      frame_timer = millis();
      frame_count++;
      if (frame_count > 3){frame_count = 0;}
      if (wind_state == 2){frame_count = 0;}
    }
    if (frame_count == 0){
      do{
          for(int i=0; i<32; i++){
            for(int j=0; j<63; j++){
              if (frame1[j][i] == 1){
                u8g2.drawPixel(i+95,j+1);
              }
            }
          }
          drawCLK();
          drawEN();
           
          
      }while (u8g2.nextPage());  
    }
    else if (frame_count == 1){
      do{   
          for(int i=0; i<32; i++){
            for(int j=0; j<63; j++){
              if (frame2[j][i] == 1){
                u8g2.drawPixel(i+95,j+1);
              }
            }
          }
          drawCLK();
          drawEN();
            
         
      }while (u8g2.nextPage());
    }
    else if (frame_count == 2){
      do{  
          for(int i=0; i<32; i++){
            for(int j=0; j<63; j++){
              if (frame3[j][i] == 1){
                u8g2.drawPixel(i+95,j+1);
              }
            }
          }
          drawCLK();
          drawEN();
  
      }while (u8g2.nextPage());
    }
    else if (frame_count == 3){
      do{
          for(int i=0; i<32; i++){
            for(int j=0; j<63; j++){
              if (frame4[j][i] == 1){
                u8g2.drawPixel(i+95,j+1);
              }
            }
          }
          drawCLK();
          drawEN();
      }while (u8g2.nextPage());
    }
  }   
}
    
 