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

// --- TUNING (Sensitivity) ---
// Lower value = higher barometer sensitivity (but more noise)
// Standard: 0.25f. For a very smooth vario: 0.5f. For a fast one: 0.1f
float SENS_BARO_NOISE = 0.5f; 

// Higher value = more trust in accelerometer (faster reaction to gusts/movements)
// Standard: 0.05f.
float SENS_ACCEL_TRUST = 0.1f;

// --- CONFIG ---
#define SEALEVELPRESSURE_HPA 1012.5
#define REFRESH_MS 1000
#define LOOP_HZ 50            // Target loop frequency
#define SINK_TRH -5.0f 

// Accelerometer settings
#define GRAVITY_G 9.80665f
// ALPHA is no longer used for projection, but kept for smoothing raw data if needed
#define ACCEL_ALPHA 0.90f     

// Lift thresholds (m/s) and vibration pulse counts
const float LIFT_TH[] = {0.2f, 0.4f, 0.8f, 1.4f, 2.0f}; 
const int   LIFT_PULSES[] = {1, 2, 3, 4, 0}; 

// Vibration (ms)
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
// Display & I2C
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
  float gMagRef;   // Reference gravity magnitude (calibrated at startup)
  bool track, sensInit, accInit;
  unsigned long tStart, tScreen;
  bool lastSt[3]; 
} data;

int rtc_h, rtc_m, rtc_d, rtc_mon;
Adafruit_BMP3XX bmp;
GxEPD2_BW<GxEPD2_154_D67, 200> display(GxEPD2_154_D67(EPD_CS, EPD_DC, EPD_RES, EPD_BUSY));
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
TaskHandle_t vTaskH = NULL;

// --- UPDATED KALMAN CLASS ---
class VarioKalman {
private:
    float z_ = 0.0f;        // estimated altitude, m
    float vz_ = 0.0f;       // estimated vertical speed, m/s
    float bias_z_ = 0.0f;   // accelerometer bias

    float P00_ = 10.0f, P10_ = 0.0f, P11_ = 1.0f;
    float P20_ = 0.0f, P21_ = 0.0f, P22_ = 0.01f;

    // Noise parameters (now configured via variables)
    float Q_z_     = 0.1f;      
    float Q_vz_;              // Taken from settings
    float Q_bias_  = 1e-6f;     
    float R_baro_;            // Taken from settings

    static constexpr float g_ = 9.80665f;        

public:
    void init(float initialHeight = 0.0f) {
        z_ = initialHeight;
        vz_ = 0.0f;
        bias_z_ = 0.0f;

        // Apply sensitivity settings
        R_baro_ = SENS_BARO_NOISE;
        Q_vz_ = SENS_ACCEL_TRUST;

        P00_ = 0.01f;  
        P11_ = 0.5f;   
        P22_ = 0.001f; 
        P10_ = P20_ = P21_ = 0.0f;
    }

    // Input: 
    // accel_linear - linear acceleration (magnitude - 1G). At rest = 0.
    // baroHeight - altitude from barometer
    float update(float accel_linear, float baroHeight, float dt) {
        if (dt <= 0.0001f) dt = 0.001f; 

        // 1. Prediction
        // We feed pure linear acceleration (gravity removed), so +g_ is not needed,
        // but the filter structure expects full acceleration to subtract bias.
        // Adaptation: assume input is already "az_corrected" (ignoring bias for now).
        // In standard model: az_true = az_sens + g. 
        // We feed (Mag - 1G). This is our estimated vertical acceleration.
        
        const float az_input = accel_linear; 
        const float az_corrected = az_input - bias_z_;

        const float z_pred     = z_     + vz_ * dt + 0.5f * az_corrected * dt * dt;
        const float vz_pred    = vz_    + az_corrected * dt;
        const float bias_pred  = bias_z_; 

        // Covariance matrix update
        P00_ += 2 * P10_ * dt + P11_ * dt * dt
                + 2 * P20_ * (-0.5f * dt * dt) + 2 * P21_ * (-dt) * (-0.5f * dt * dt)
                + P22_ * (0.25f * dt * dt * dt * dt) + Q_z_;

        P10_ += P11_ * dt + P21_ * (-dt) + P22_ * (-0.5f * dt * dt) + 0.0f;
        P11_ += P22_ * dt * dt + Q_vz_;

        P20_ += P21_ * dt + P22_ * (-0.5f * dt * dt);
        P21_ += P22_ * dt;
        P22_ += Q_bias_;

        // 2. Correction (Magic happens here - barometer corrects drift and sign)
        const float dz = baroHeight - z_pred; 

        const float S = P00_ + R_baro_; 
        const float K0 = P00_ / S;
        const float K1 = P10_ / S;
        const float K2 = P20_ / S;

        z_     = z_pred     + K0 * dz;
        vz_    = vz_pred    + K1 * dz;
        bias_z_ = bias_pred + K2 * dz;

        const float I_K0 = 1.0f - K0;
        P00_ *= I_K0;
        P10_ = I_K0 * P10_;
        P20_ = I_K0 * P20_;

        P11_ -= K1 * P10_;
        P21_ -= K1 * P20_;

        P22_ -= K2 * P20_;

        // Limiters
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
    // BMP initialization
    if (bmp.begin_I2C(0x77) || bmp.begin_I2C(0x76)) {
        bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_4X);
        bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X);
        bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
        bmp.setOutputDataRate(BMP3_ODR_50_HZ);
        data.sensInit = true;
    }
    // BMA423 initialization
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

// --- LOGIC ---
void varioTask(void *p) {
    int pulses = 0; bool pulseOn = false; unsigned long nextT = 0; bool pause = false;
    bool vibroActive = false;
    unsigned long vibroStopT = 0;
    unsigned long lastUpdateMicros = micros();

    for(;;) {
        if (state == RUNNING && data.sensInit) {
            float acc_vert_input = 0; 
            float ax=0, ay=0, az=0;

            unsigned long nowMicros = micros();
            float dt = (nowMicros - lastUpdateMicros) / 1000000.0f;
            if(dt < 0.001f) dt = 0.001f; 
            lastUpdateMicros = nowMicros;

            bool accSafe = !vibroActive && (millis() - vibroStopT > VIBRO_COOLDOWN_MS);

            if(data.accInit && accSafe) {
                Wire.beginTransmission(ADDR_BMA); Wire.write(0x12); Wire.endTransmission();
                Wire.requestFrom(ADDR_BMA, 6);
                if(Wire.available() >= 6) {
                    int16_t rx = Wire.read()|(Wire.read()<<8);
                    int16_t ry = Wire.read()|(Wire.read()<<8);
                    int16_t rz = Wire.read()|(Wire.read()<<8);
                    // Convert to G (range +/- 4G = 8192 LSB/g)
                    ax = rx / 8192.0f; 
                    ay = ry / 8192.0f; 
                    az = rz / 8192.0f;

                    // --- ORIENTATION ELIMINATION ALGORITHM ---
                    // 1. Calculate magnitude of full acceleration vector (square and root)
                    // This number is always positive and independent of device rotation.
                    float acc_mag = sqrt(ax*ax + ay*ay + az*az);
                    
                    // 2. Subtract reference gravity (measured at rest during calibration)
                    // If flying level -> acc_mag ~ 1.0 -> result 0.
                    // If sharp climb -> G-force > 1G -> result positive.
                    // If drop/sink -> weightlessness < 1G -> result negative.
                    // This gives us linear acceleration magnitude, "cleaned" of tilt.
                    float acc_linear_g = acc_mag - data.gMagRef;

                    // Convert to m/s^2
                    acc_vert_input = acc_linear_g * GRAVITY_G;
                }
            }

            if (bmp.performReading()) {
                float baro_alt = bmp.readAltitude(SEALEVELPRESSURE_HPA);
                
                // Update filter. 
                // acc_vert_input provides reaction sharpness.
                // baro_alt provides correct sign and eliminates drift.
                kalman.update(acc_vert_input, baro_alt, dt);

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

                // --- Vibro Logic ---
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
                
                // --- CALIBRATION ---
                // Determine "unit" gravity for this specific sensor
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
                // Protection against division by zero or bad calibration
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
