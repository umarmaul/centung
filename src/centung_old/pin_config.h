/*
 * @Description: Centung_PKM KI
 * @version: V2.0
 * @Author: Umar Maulana
 * @Date: 31/08/2023
 * @LastEditors: Umar Maulana
 * @LastEditTime: 03/09/2023
 */
#pragma once

#define WIFI_CONNECT_WAIT_MAX        (30 * 1000)

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 25200;
const int   daylightOffset_sec = 0;

const IPAddress remote_ip(8, 8, 8, 8);

char timeHour[3];
char timeMin[3];
char timeSec[3];
char day[3];
char month[10];
char year[5];
char timeWeekDay[10];
int dayInWeek;
int volt;
int fromLeft=10;

String Day[7]={"SU","MO","TU","WE","TH","FR","SA"};
String SDay[7]={"Sunday","Monday","Tuesday","Wednesday","Thursday","Friday","Saturday"};
String Months[12]={"January","February","March","April","May","June","July","August","September","October","November","December"};
int mm[12]={31,28,31,30,31,30,31,31,30,31,30,31};
int dayCounter=0;
int dayInMonth=0;
int daysInMonth=0;
int firstDay=0;

int brightness=255;
int caw=22;
int cay=20;
int cax=150;
int cah=22;
int seg=0;  
long t=0;

/*firebase*/
#define API_KEY "AIzaSyAVdowkzs26fSi22a6vzQAI5zAkU5GUPtY"
#define FIREBASE_PROJECT_ID "centung-aade6"
#define USER_EMAIL "admin@admin.com"
#define USER_PASSWORD "admin123"

/*TFT CONFIG*/
#define LOADING_BACKGROUND           0xE73B
#define PROFILE_BACKGROUND           0xE73C
#define gray      0xB5B6
#define orange    0xC260
#define color2    0x22CE

/*ESP32S3*/
#define PIN_LCD_BL                   38

#define PIN_LCD_D0                   39
#define PIN_LCD_D1                   40
#define PIN_LCD_D2                   41
#define PIN_LCD_D3                   42
#define PIN_LCD_D4                   45
#define PIN_LCD_D5                   46
#define PIN_LCD_D6                   47
#define PIN_LCD_D7                   48

#define PIN_POWER_ON                 15

#define PIN_LCD_RES                  5
#define PIN_LCD_CS                   6
#define PIN_LCD_DC                   7
#define PIN_LCD_WR                   8
#define PIN_LCD_RD                   9

#define PIN_BUTTON_1                 0
#define PIN_BUTTON_2                 14
#define PIN_BAT_VOLT                 4

#define PIN_IIC_SCL                  44
#define PIN_IIC_SDA                  43

#define PIN_TOUCH_INT                16
#define PIN_TOUCH_RES                21

/* External expansion */
//Button
#define PUSH_BUTTON_1                21
#define PUSH_BUTTON_2                16
#define PUSH_BUTTON_3                2
#define PUSH_BUTTON_4                1

//HX711
#define PIN_HX711_DATA               18
#define PIN_HX711_CLOCK              17

//Buzzer
#define PIN_BUZZER                   3

//SD
#define PIN_SD_CS                    10
#define PIN_SD_MOSI                  11
#define PIN_SD_CLK                   12
#define PIN_SD_MISO                  13
