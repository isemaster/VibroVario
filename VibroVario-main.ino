/*
 * PROGRAM DESCRIPTION:
 * DIY Variometer firmware for ESP32, based on SQFMI Watchy.
 * Features:
 * - Sensor Fusion (Kalman Filter): Combines BMP3XX barometer and BMA423 accelerometer data 
 *   for lag-free, low-noise vertical speed estimation.
 * - E-Ink Display (GxEPD2): Displays altitude, vertical speed, flight time, temperature, and battery.
 * - Haptic Feedback: Vibration patterns based on climb rates.
 * - Power Management: Deep sleep support and dedicated clock mode when idle.
 * - Auto-calibration: Removes gravity vector at startup to detect linear acceleration regardless of orientation.
 *
 * PROJECT REPOSITORY / GITHUB:
 * https://github.com/isemaster/VibroVario
*/
#include <esp_sleep.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <GxEPD2_BW.h>
#include <Fonts/FreeSansBold9pt7b.h>
#include <Fonts/FreeSansBold18pt7b.h>
#include <Fonts/FreeSansBold24pt7b.h>
#include "FreeMonoBold36pt7b.h"
#include <cmath>

// --- TUNING (SENSITIVITY) ---
// Уменьшили шум барометра (быстрее реакция на давление)
float SENS_BARO_NOISE = 0.3f;  // <--- CHANGE (было 0.5)
// Увеличили доверие к акселерометру (резче реакция на пинки воздуха)
float SENS_ACCEL_TRUST = 0.3f; // <--- CHANGE (было 0.1)

// --- CONFIG ---
#define SEALEVELPRESSURE_HPA 1015.95
#define REFRESH_MS 1000
#define LOOP_HZ 50            
#define SINK_TRH -5.0f 
#define GRAVITY_G 9.80665f

// ACCELEROMETER DEADBAND
// Если ускорение меньше этого значения (м/с^2), считаем его нулем (шум стола).
// 0.3 м/с^2 - это примерно 0.03G. Достаточно чтобы отсечь шум, но поймать поток.
#define ACCEL_DEADBAND 0.3f  // <--- NEW PARAMETER

// Lift thresholds
const float LIFT_TH[] = {0.2f, 0.4f, 0.8f, 1.4f, 2.0f}; 
const int   LIFT_PULSES[] = {1, 2, 3, 4, 0}; 

// Vibro settings
#define V_PULSE 50 
#define V_GAP   125  
#define V_PAUSE 1000  
#define VIBRO_COOLDOWN_MS 250 

// Pins
#define BTN_RIGHT 4      
#define PIN_VARIO_EN 26  
#define BTN_SELECT 35    
#define BTN_BACK 25      
#define PIN_VIBRO 13     
#define PIN_BATT 34      
#define EPD_CS 5
#define EPD_RES 9
#define EPD_DC 10
#define EPD_BUSY 19
#define ADDR_BMA 0x18
#define ADDR_RTC 0x51

// --- GLOBALS ---
enum State { SLEEP, IDLE, RUNNING, STOPPED, CALIB };
RTC_DATA_ATTR State state = SLEEP;
RTC_DATA_ATTR unsigned long stopwatchElapsed = 0;

struct SysData {
  float startAlt, alt, vel, maxV, minV, temp;
  float ax, ay, az;
  float gMagRef;   
  bool track, sensInit, accInit;
  unsigned long tStart, tScreen;
  bool lastSt[3]; 
} data;

int rtc_h, rtc_m, rtc_d, rtc_mon;
Adafruit_BMP3XX bmp;
GxEPD2_BW<GxEPD2_154_D67, 200> display(GxEPD2_154_D67(EPD_CS, EPD_DC, EPD_RES, EPD_BUSY));
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
TaskHandle_t vTaskH = NULL;

// --- KALMAN CLASS ---
class VarioKalman {
private:
    float z_ = 0.0f;        
    float vz_ = 0.0f;       
    float bias_z_ = 0.0f;   

    float P00_ = 10.0f, P10_ = 0.0f, P11_ = 1.0f;
    float P20_ = 0.0f, P21_ = 0.0f, P22_ = 0.01f;

    float Q_z_     = 0.1f;      
    float Q_vz_;              
    float Q_bias_  = 1e-6f;     
    float R_baro_;            

public:
    void init(float initialHeight = 0.0f) {
        z_ = initialHeight;
        vz_ = 0.0f;
        bias_z_ = 0.0f;
        R_baro_ = SENS_BARO_NOISE;
        Q_vz_ = SENS_ACCEL_TRUST;
        
        // Начальные ковариации сброшены для быстрой сходимости
        P00_ = 0.01f; P11_ = 0.5f; P22_ = 0.001f; 
        P10_ = P20_ = P21_ = 0.0f;
    }

    float update(float accel_linear, float baroHeight, float dt) {
        if (dt <= 0.0001f) dt = 0.001f; 

        // Prediction
        const float az_corrected = accel_linear - bias_z_;
        const float z_pred     = z_     + vz_ * dt + 0.5f * az_corrected * dt * dt;
        const float vz_pred    = vz_    + az_corrected * dt;
        const float bias_pred  = bias_z_; 

        P00_ += 2 * P10_ * dt + P11_ * dt * dt + 2 * P20_ * (-0.5f * dt * dt) + 2 * P21_ * (-dt) * (-0.5f * dt * dt) + P22_ * (0.25f * dt * dt * dt * dt) + Q_z_;
        P10_ += P11_ * dt + P21_ * (-dt) + P22_ * (-0.5f * dt * dt);
        P11_ += P22_ * dt * dt + Q_vz_;
        P20_ += P21_ * dt + P22_ * (-0.5f * dt * dt);
        P21_ += P22_ * dt;
        P22_ += Q_bias_;

        // Correction
        const float dz = baroHeight - z_pred; 
        const float S = P00_ + R_baro_; 
        const float K0 = P00_ / S;
        const float K1 = P10_ / S;
        const float K2 = P20_ / S;

        z_     = z_pred     + K0 * dz;
        vz_    = vz_pred    + K1 * dz;
        bias_z_ = bias_pred + K2 * dz;

        const float I_K0 = 1.0f - K0;
        P00_ *= I_K0; P10_ = I_K0 * P10_; P20_ = I_K0 * P20_;
        P11_ -= K1 * P10_; P21_ -= K1 * P20_;
        P22_ -= K2 * P20_;

        if (vz_ >  25.0f) vz_ =  25.0f; 
        if (vz_ < -25.0f) vz_ = -25.0f;

        return vz_;
    }

    float getAltitude() const { return z_; }
    float getVario()    const { return vz_; }
};

VarioKalman kalman;

// --- HELPERS ---
void i2cWrite(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr); Wire.write(reg); Wire.write(val); Wire.endTransmission();
}
uint8_t bcd2dec(uint8_t v) { return ((v/16*10) + (v%16)); }

void readRTC() {
  Wire.beginTransmission(ADDR_RTC); Wire.write(0x02); Wire.endTransmission();
  Wire.requestFrom(ADDR_RTC, 6);
  if(Wire.available()) {
     Wire.read(); 
     rtc_m = bcd2dec(Wire.read()&0x7F); rtc_h = bcd2dec(Wire.read()&0x3F);
     rtc_d = bcd2dec(Wire.read()&0x3F); Wire.read(); rtc_mon = bcd2dec(Wire.read()&0x1F);
  }
}

void initSensors() {
    if (bmp.begin_I2C(0x77) || bmp.begin_I2C(0x76)) {
        bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_4X);
        bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X);
        bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
        bmp.setOutputDataRate(BMP3_ODR_50_HZ);
        data.sensInit = true;
    }
    i2cWrite(ADDR_BMA, 0x7E, 0xB6); delay(20); 
    i2cWrite(ADDR_BMA, 0x7C, 0x00); delay(10); 
    i2cWrite(ADDR_BMA, 0x40, 0x28); 
    i2cWrite(ADDR_BMA, 0x41, 0x01); 
    i2cWrite(ADDR_BMA, 0x7D, 0x04); delay(50); 
    data.accInit = true;
}

void drawItem(int x, int y, const GFXfont* f, String txt) {
    display.setFont(f); 
    if (x < 0) {
        int16_t x1, y1; uint16_t w, h;
        display.getTextBounds(txt, 0, 0, &x1, &y1, &w, &h);
        x = (display.width() - w) / 2;
    }
    display.setCursor(x, y); display.print(txt);
}

// --- LOGIC CORE ---
void varioTask(void *p) {
    int pulses = 0; bool pulseOn = false; unsigned long nextT = 0; bool pause = false;
    bool vibroActive = false;
    unsigned long vibroStopT = 0;
    unsigned long lastUpdateMicros = micros();
    
    float rawBaroVel = 0;
    float lastBaroAlt = 0;
    bool firstRun = true;

    for(;;) {
        if (state == RUNNING && data.sensInit) {
            float acc_raw_mag = 1.0f; 
            float ax=0, ay=0, az=0;

            unsigned long nowMicros = micros();
            float dt = (nowMicros - lastUpdateMicros) / 1000000.0f;
            if(dt < 0.001f) dt = 0.001f; 
            lastUpdateMicros = nowMicros;

            bool accSafe = !vibroActive && (millis() - vibroStopT > VIBRO_COOLDOWN_MS);

            // 1. READ ACCEL
            if(data.accInit && accSafe) {
                Wire.beginTransmission(ADDR_BMA); Wire.write(0x12); Wire.endTransmission();
                Wire.requestFrom(ADDR_BMA, 6);
                if(Wire.available() >= 6) {
                    int16_t rx = Wire.read()|(Wire.read()<<8);
                    int16_t ry = Wire.read()|(Wire.read()<<8);
                    int16_t rz = Wire.read()|(Wire.read()<<8);
                    ax = rx / 8192.0f; 
                    ay = ry / 8192.0f; 
                    az = rz / 8192.0f;
                    acc_raw_mag = sqrt(ax*ax + ay*ay + az*az);
                }
            }

            // 2. READ BARO
            if (bmp.performReading()) {
                float baro_alt = bmp.readAltitude(SEALEVELPRESSURE_HPA);
                if (firstRun) { lastBaroAlt = baro_alt; firstRun = false; }

                // --- LOGIC MODIFICATION START ---
                
                // 1. Рассчитываем грубую скорость барометра для защиты от глюков
                float instVel = (baro_alt - lastBaroAlt) / dt;
                rawBaroVel = 0.8f * rawBaroVel + 0.2f * instVel;
                lastBaroAlt = baro_alt;

                // 2. Чистое линейное ускорение
                float acc_linear_g = acc_raw_mag - data.gMagRef; 
                float potential_acc_ms2 = acc_linear_g * GRAVITY_G;
                
                float final_acc_input = 0.0f;
                float absVel = fabs(rawBaroVel);

                // 3. ЛОГИКА ОТСЕЧЕНИЯ ШУМА (DEADBAND)
                // Если ускорение очень маленькое (лежит на столе или равномерный полет),
                // принудительно ставим 0. Это обеспечивает "железобетонную" тишину на столе.
                if (fabs(potential_acc_ms2) < ACCEL_DEADBAND) {
                    potential_acc_ms2 = 0.0f;
                    
                    // Заодно используем этот момент покоя для подстройки гравитации (убираем дрейф ориентации)
                    if (absVel < 0.2f) {
                        data.gMagRef = (data.gMagRef * 0.995f) + (acc_raw_mag * 0.005f);
                    }
                }

                // 4. ЛОГИКА СЛИЯНИЯ
                // Правило 1: Если мы уже летим быстро (вверх или вниз), верим барометру больше, 
                // аксель отключаем, чтобы вибрация строп не сбивала.
                if (absVel > 1.0f) {
                     final_acc_input = 0.0f;
                }
                // Правило 2: В начале движения (скорость около 0).
                // Если мы прошли Deadband (ускорение > 0.3 м/с2), значит это реальный рывок.
                // Передаем его СРАЗУ, не проверяя знак барометра.
                else {
                     final_acc_input = potential_acc_ms2;
                }

                // Обновляем фильтр с "умным" ускорением
                kalman.update(final_acc_input, baro_alt, dt);

                // --- LOGIC MODIFICATION END ---

                portENTER_CRITICAL(&mux);
                data.alt = kalman.getAltitude(); 
                data.vel = kalman.getVario(); 
                data.temp = bmp.temperature;
                if(accSafe) { data.ax = ax; data.ay = ay; data.az = az; }
                portEXIT_CRITICAL(&mux);

                if (data.track) {
                    if (data.vel > data.maxV) data.maxV = data.vel;
                    if (data.vel < data.minV) data.minV = data.vel;
                }

                // --- VIBRO ---
                int reqP = 0;
                float v = data.vel;
                if (v <= SINK_TRH) reqP = -1;
                else {
                    for(int i=0; i<4; i++) if(v >= LIFT_TH[i] && v < LIFT_TH[i+1]) reqP = LIFT_PULSES[i];
                }
                unsigned long now = millis();
                bool setVibro = false; 
                if (reqP == -1) { setVibro = true; pulses = 0; pulseOn = 1; pause = 0; }
                else if (reqP == 0) { setVibro = false; pulses = 0; pulseOn = 0; pause = 0; }
                else {
                    if (pulses == 0 && !pulseOn && !pause) {
                        pulses = reqP; setVibro = true; pulseOn = 1; nextT = now + V_PULSE;
                    }
                    if (now >= nextT) {
                        if (pulseOn) {
                            setVibro = false; pulseOn = 0; pulses--;
                            nextT = now + (pulses > 0 ? V_GAP : V_PAUSE);
                            if(pulses <= 0) pause = true;
                        } else {
                            if (pause) pause = false;
                            else { setVibro = true; pulseOn = 1; nextT = now + V_PULSE; }
                        }
                    } else { setVibro = pulseOn; }
                }

                if (setVibro) { digitalWrite(PIN_VIBRO, 1); vibroActive = true; } 
                else {
                    digitalWrite(PIN_VIBRO, 0);
                    if (vibroActive) vibroStopT = millis();
                    vibroActive = false;
                }
            }
        } else digitalWrite(PIN_VIBRO, 0);
        
        vTaskDelay(pdMS_TO_TICKS(1000/LOOP_HZ)); 
    }
}

void drawMain() {
    float dAlt, dVel, dMax, dMin, dTemp;
    portENTER_CRITICAL(&mux);
    dAlt = data.alt; dVel = data.vel; dMax = data.maxV; dMin = data.minV; dTemp = data.temp;
    portEXIT_CRITICAL(&mux);

    char buf[40];
    display.setPartialWindow(0, 0, 200, 200);
    display.firstPage();
    do {
        display.fillScreen(GxEPD_WHITE); display.setTextColor(GxEPD_BLACK);

        unsigned long t = stopwatchElapsed + (state==RUNNING ? (millis()-data.tStart)/1000 : 0);
        sprintf(buf, "%02lu:%02lu:%02lu", t/3600, (t%3600)/60, t%60);
        drawItem(10, 20, &FreeSansBold9pt7b, buf);
        
        sprintf(buf, "%.1fc", dTemp); drawItem(80, 20, &FreeSansBold9pt7b, buf);
        
        float v = analogReadMilliVolts(PIN_BATT)/1000.0*2.0;
        sprintf(buf, "%d%%", v>=4.2?100 : (v<=3.3?0 : (int)((v-3.3)*111.1)));
        drawItem(140, 20, &FreeSansBold9pt7b, buf);

        if(state==RUNNING || state==STOPPED) {
            drawItem(25, 50, &FreeSansBold9pt7b, "Start, m   Sea, m");
            sprintf(buf, "%+d", (int)(dAlt - data.startAlt)); drawItem(30, 90, &FreeSansBold18pt7b, buf);
            sprintf(buf, "%d", (int)dAlt); drawItem(130, 90, &FreeSansBold18pt7b, buf);
        }

        drawItem(10, 135, &FreeSansBold9pt7b, "Vario");
        if(data.track) {
            sprintf(buf, "max%+.1f min%+.1f", dMax, dMin);
            drawItem(60, 135, &FreeSansBold9pt7b, buf);
        }
        sprintf(buf, "%+.1f", dVel); drawItem(10, 190, &FreeMonoBold36pt7b, buf);
    } while (display.nextPage());
}

void drawClock(bool deep) {
    if(deep) { display.init(115200, true, 2, false); display.setFullWindow(); }
    else display.setPartialWindow(0,0,200,200);

    display.firstPage();
    do {
        display.fillScreen(GxEPD_WHITE); display.setTextColor(GxEPD_BLACK);
        float v = analogReadMilliVolts(PIN_BATT)/1000.0*2.0;
        char buf[20]; sprintf(buf, "%.2fV %d%%", v, (int)((v-3.3)*111.1));
        drawItem(10, 20, &FreeSansBold9pt7b, buf);

        sprintf(buf, "%02d:%02d", rtc_h, rtc_m); drawItem(-1, 110, &FreeSansBold24pt7b, buf);
        sprintf(buf, "%02d.%02d", rtc_d, rtc_mon); drawItem(-1, 160, &FreeSansBold18pt7b, buf);
    } while (display.nextPage());
}

void goDeepSleep() {
    if(vTaskH) vTaskDelete(vTaskH);
    digitalWrite(PIN_VARIO_EN, 0); digitalWrite(PIN_VIBRO, 0);
    display.hibernate(); Wire.end(); WiFi.mode(WIFI_OFF);
    esp_sleep_enable_ext0_wakeup((gpio_num_t)BTN_BACK, 1);
    esp_deep_sleep_start();
}

void setup() {
    pinMode(BTN_SELECT, INPUT); pinMode(BTN_RIGHT, INPUT); pinMode(BTN_BACK, INPUT);
    pinMode(PIN_VARIO_EN, OUTPUT); pinMode(PIN_VIBRO, OUTPUT); pinMode(PIN_BATT, INPUT);
    digitalWrite(PIN_VARIO_EN, 0); digitalWrite(PIN_VIBRO, 0);
    
    Wire.begin(); display.init(115200, true, 2, false); display.setRotation(1);
    setCpuFrequencyMhz(80); WiFi.mode(WIFI_OFF);
    
    readRTC();
    if (esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_EXT0) {
        drawClock(true); goDeepSleep();
    }
    drawClock(true); state = IDLE;
}

void loop() {
    if (state == SLEEP) goDeepSleep();
    if (state == IDLE && millis() > 60000) goDeepSleep();

    bool bSel = digitalRead(BTN_SELECT), bR = digitalRead(BTN_RIGHT), bBack = digitalRead(BTN_BACK);

    if (bBack && !data.lastSt[2]) { delay(50); if(digitalRead(BTN_BACK)) { readRTC(); drawClock(true); state = SLEEP; }}
    if (bSel && !data.lastSt[0] && (state==IDLE || state==STOPPED)) {
        delay(50); if(digitalRead(BTN_SELECT)) {
            digitalWrite(PIN_VARIO_EN, 1); delay(50); initSensors();
            
            display.setPartialWindow(0,0,200,200); display.firstPage();
            do { display.fillScreen(GxEPD_WHITE); drawItem(20, 90, &FreeSansBold18pt7b, "Calibrating..."); } while(display.nextPage());
            
            if(data.sensInit) {
                kalman.init(bmp.readAltitude(SEALEVELPRESSURE_HPA));
                
                // Calibration
                float sumMag = 0;
                int samples = 100;
                for(int i=0; i<samples; i++) {
                    Wire.beginTransmission(ADDR_BMA); Wire.write(0x12); Wire.endTransmission();
                    Wire.requestFrom(ADDR_BMA, 6);
                    if(Wire.available()>=6) {
                        int16_t rx = Wire.read()|(Wire.read()<<8);
                        int16_t ry = Wire.read()|(Wire.read()<<8);
                        int16_t rz = Wire.read()|(Wire.read()<<8);
                        float _ax = rx/8192.0f; 
                        float _ay = ry/8192.0f; 
                        float _az = rz/8192.0f;
                        sumMag += sqrt(_ax*_ax + _ay*_ay + _az*_az);
                    }
                    if(bmp.performReading()) kalman.update(0.0f, bmp.readAltitude(SEALEVELPRESSURE_HPA), 0.02f);
                    delay(10);
                }

                data.gMagRef = sumMag / samples;
                if(data.gMagRef < 0.5f || data.gMagRef > 1.5f) data.gMagRef = 1.0f;

                portENTER_CRITICAL(&mux);
                data.startAlt = data.alt = kalman.getAltitude(); data.maxV = data.minV = 0; 
                portEXIT_CRITICAL(&mux);
                
                data.track = false; stopwatchElapsed = 0; data.tStart = millis();
                if(!vTaskH) xTaskCreatePinnedToCore(varioTask, "V", 4096, NULL, 10, &vTaskH, 0);
                state = RUNNING;
            }
        }
    }
    if (bR && !data.lastSt[1]) {
        delay(50); if(digitalRead(BTN_RIGHT)) {
            if (state==RUNNING) {
                stopwatchElapsed += (millis()-data.tStart)/1000; state=STOPPED; digitalWrite(PIN_VIBRO, 0);
            } else if (state==IDLE) {
                i2cWrite(ADDR_RTC, 0x02, 0); i2cWrite(ADDR_RTC, 0x03, 0); i2cWrite(ADDR_RTC, 0x04, 0);
                rtc_h=0; rtc_m=0; drawClock(false);
            }
        }
    }
    
    data.lastSt[0]=bSel; data.lastSt[1]=bR; data.lastSt[2]=bBack;

    if(state == RUNNING && !data.track && (millis() - data.tStart > 5000)) data.track = true;
    
    if ((state == RUNNING || state == STOPPED) && (millis() - data.tScreen >= REFRESH_MS)) {
        data.tScreen = millis(); drawMain();
    }
    delay(10);
}
