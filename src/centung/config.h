#pragma once

// =========================================================================
// Device Identity Config
// =========================================================================
/*Server and Device Information*/
#define SOCKET_ADDRESS "http://168.231.119.233:5000/"
#define DEVICE_NAME "prototype"
#define DEVICE_PASSWORD "prototype"

/*Profile Code*/
#define PROFILE_CODE_1 "prototype"
#define PROFILE_CODE_2 "A2"
#define PROFILE_CODE_3 "A3"
#define PROFILE_CODE_4 "A4"

/*firebase*/
#define API_KEY "AIzaSyAVdowkzs26fSi22a6vzQAI5zAkU5GUPtY"
#define FIREBASE_PROJECT_ID "centung-aade6"
#define USER_EMAIL "admin@admin.com"
#define USER_PASSWORD "admin123"

// =========================================================================
// Enums and Structs
// =========================================================================
enum AppState {
    STATE_PROFILE_SELECTION,
    STATE_MEASURING,
    STATE_GYRO_TEST,
    STATE_CALIBRATION,
    STATE_HISTORY,
    STATE_PAIRING,
    STATE_ALARM,
};

struct SensorData {
    float weight;
    float roll;
    float pitch;
};

struct ButtonEvent {
    int buttonId;
    int eventType;  // 0: click, 1: long press, 2: double click
};

struct HttpCommand {
    int type;  // 0: create log, 1: delete latest, 2: delete all today, 3: research only for dataset
    float weight;
    float roll;
    float pitch;
    String profile_code;
};

// =========================================================================
// Global Variables
// =========================================================================

// NTP Variables
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 25200;
const int daylightOffset_sec = 0;

// Test PING variable to Google
const IPAddress REMOTE_IP(8, 8, 8, 8);

// Time Variables
char timeHour[3];
char timeMin[3];
char timeSec[3];
char day[3];
char month[10];
char year[5];
char timeWeekDay[10];
int dayInWeek;

// Date Variables
String Day[7] = {"SU", "MO", "TU", "WE", "TH", "FR", "SA"};
String SDay[7] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
String Months[12] = {"January", "February", "March", "April", "May", "June", "July", "August", "September", "October", "November", "December"};
int mm[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
int dayCounter = 0;
int dayInMonth = 0;
int daysInMonth = 0;
int firstDay = 0;

// Display Variables
int brightness = 240;
int caw = 22;
int cay = 20;
int cax = 150;
int cah = 22;
int seg = 0;
long t = 0;

// Load Cell Variables
const int calVal_eepromAdress = 0;
const int tareOffsetVal_eepromAdress = 20;
const int stabilizingtime = 2000;
float calibrationValue = 1435.05;
float calibrationOffset = 8434937.00;
boolean _tare = true;

// MPU6050 Settings
const uint8_t MPU6050_ADDR = 0x68;
const float MPU_ACCEL_SCALE = 4096.0;  // Sesuai konfigurasi FS_SEL=1 (±8g)
const float MPU_GYRO_SCALE = 65.5;     // Sesuai konfigurasi FS_SEL=1 (±500°/s)
const int MPU_CALIBRATION_COUNT = 2000;
float GyroBiasRoll = 0, GyroBiasPitch = 0, GyroBiasYaw = 0;

// Kalman Filter Settings
float KalmanAngleRoll = 0, KalmanAnglePitch = 0;
float KalmanUncertaintyAngleRoll = 2 * 2;
float KalmanUncertaintyAnglePitch = 2 * 2;

// Smoothing Settings
const int SMOOTHING_WINDOW_SIZE = 10;
float rollWindow[SMOOTHING_WINDOW_SIZE];
float pitchWindow[SMOOTHING_WINDOW_SIZE];
int smoothingIndex = 0;

// State Variables (EDITED SOON)
bool _isAware = false;
int tanggal = 0, bulan = 0;

// =========================================================================
// Pin Config
// =========================================================================
/*ESP32S3*/
#define PIN_LCD_BL 38

#define PIN_LCD_D0 39
#define PIN_LCD_D1 40
#define PIN_LCD_D2 41
#define PIN_LCD_D3 42
#define PIN_LCD_D4 45
#define PIN_LCD_D5 46
#define PIN_LCD_D6 47
#define PIN_LCD_D7 48

#define PIN_POWER_ON 15

#define PIN_LCD_RES 5
#define PIN_LCD_CS 6
#define PIN_LCD_DC 7
#define PIN_LCD_WR 8
#define PIN_LCD_RD 9

#define PIN_BUTTON_1 0
#define PIN_BUTTON_2 14
#define PIN_BAT_VOLT 4

#define PIN_IIC_SCL 44
#define PIN_IIC_SDA 43

#define PIN_TOUCH_INT 16
#define PIN_TOUCH_RES 21

/* External expansion */
// Button
#define PUSH_BUTTON_1 21
#define PUSH_BUTTON_2 16
#define PUSH_BUTTON_3 2
#define PUSH_BUTTON_4 1

// HX711
#define PIN_HX711_DATA 18
#define PIN_HX711_CLOCK 17

// Buzzer
#define PIN_BUZZER 3

// SD
#define PIN_SD_CS 10
#define PIN_SD_MOSI 11
#define PIN_SD_CLK 12
#define PIN_SD_MISO 13

// =========================================================================
// Other Configuration
// =========================================================================
/*wifi*/
#define WIFI_CONNECT_WAIT_MAX (30 * 1000)
#define CONNECT_TIMEOUT 10
#define PORTAL_TIMEOUT 180

/*TFT CONFIG*/
#define BACKGROUND 0xE73C
#define GRAY 0xB5B6
#define ORANGE 0xC260
#define COLOR_2 0x22CE