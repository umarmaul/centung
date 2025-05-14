#pragma once

#include <ArduinoJson.h>
#include <EEPROM.h>
#include <ESP32Ping.h>
#include <HTTPClient.h>
#include <HX711_ADC.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <WiFiManager.h>
#include <Wire.h>
// edit if app is ready
#include <Firebase_ESP_Client.h>
#include <addons/TokenHelper.h>

#include "FS.h"
#include "OneButton.h"
#include "driver/rtc_io.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "screens/alarm.h"
#include "screens/bright.h"
#include "screens/datar.h"
#include "screens/kotak.h"
#include "screens/loading.h"
#include "screens/polos.h"
#include "screens/profile.h"
#include "screens/timbangan.h"
#include "time.h"