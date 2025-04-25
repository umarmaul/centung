#include "include/Utilities.h"
#include <Arduino.h>
#include "include/Config.h"

void buzzerFeedback(bool success) {
    if (success) {
        digitalWrite(PIN_BUZZER, HIGH);
        vTaskDelay(pdMS_TO_TICKS(100));
        digitalWrite(PIN_BUZZER, LOW);
    } else {
        for (int i = 0; i < 3; i++) {
            digitalWrite(PIN_BUZZER, HIGH);
            vTaskDelay(pdMS_TO_TICKS(150));
            digitalWrite(PIN_BUZZER, LOW);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}