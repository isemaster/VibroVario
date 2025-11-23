/* Версия 1.1 с фильтром эксп. средней 2025.11.24
 * Прошивка ESP32 Вариометра
 * Основной функционал:
 * - Считывание данных с барометра BMP3XX и акселерометра (BMA).
 * - Fusion-фильтрация (EMA) данных для быстрого отклика вариометра.
 * - Вывод информации на E-Ink дисплей.
 * - Звуковая/Вибро индикация подъема и спуска.
 * - Управление питанием (Deep Sleep).
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

// --- НАСТРОЙКИ ФИЛЬТРОВ (Fusion) ---
float CFG_TAU_BARO_ALT        = 0.1f;   // Постоянная времени сглаживания высоты (сек)
float CFG_TAU_BARO_VARIO_BASE = 0.3f;   // Базовая постоянная времени вариометра (в спокойном воздухе)
float CFG_TAU_BARO_VARIO_TURB = 0.3f;   // Постоянная времени при турбулентности (динамическая настройка)
float CFG_TAU_ACCEL           = 0.1f;   // Постоянная времени фильтра акселерометра
float CFG_ACCEL_TURB_REF      = 2.0f;   // Эталонное ускорение (м/с²) для определения уровня турбулентности
float CFG_VARIO_SENS          = 1.0f;   // Масштабный коэффициент чувствительности вариометра

// --- СИСТЕМНЫЕ НАСТРОЙКИ ---
#define SEALEVELPRESSURE_HPA 1013.25     // Давление на уровне моря для расчета высоты
#define REFRESH_MS 1000                 // Период обновления экрана (мс)
#define LOOP_HZ 50                      // Частота основного цикла обработки (Гц)
#define SINK_TRH -5.0f                  // Порог срабатывания сигнала снижения (м/с)

// --- НАСТРОЙКИ АКСЕЛЕРОМЕТРА ---
#define GRAVITY_G 9.80665f              // Ускорение свободного падения
#define ACCEL_ALPHA 0.90f               // Коэффициент фильтрации (legacy, не используется в EMA)

// --- ПОРОГИ ЗВУКА И ИМПУЛЬСЫ ---
// Пороги вертикальной скорости (м/с) и соответствующее количество импульсов вибро
const float LIFT_TH[]    = {0.15f, 0.4f, 1.0f, 2.0f, 1000.0f}; 
const int   LIFT_PULSES[] = {1, 2, 3,  0};

// --- НАСТРОЙКИ ВИБРАЦИИ ---
#define V_PULSE 30                      // Длительность импульса вибрации (мс)
#define V_GAP   100                     // Пауза между импульсами в пачке (мс)
#define V_PAUSE 500                     // Длинная пауза между пачками (мс)
#define VIBRO_COOLDOWN_MS 200           // Время "остывания" после вибрации перед замером акселерометра (защита от шума)

// --- ПИНЫ (Hardware) ---
#define BTN_RIGHT 4                     
#define PIN_VARIO_EN 26                 // Пин управления питанием сенсоров
#define BTN_SELECT 35                   
#define BTN_BACK 25                     
#define PIN_VIBRO 13                    
#define PIN_BATT 34                     // АЦП батареи

// --- ДИСПЛЕЙ И I2C ---
#define EPD_CS 5                        
#define EPD_RES 9                       
#define EPD_DC 10                       
#define EPD_BUSY 19                     
#define ADDR_BMA 0x18                   
#define ADDR_RTC 0x51                   

// --- ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ ---
enum State { SLEEP, IDLE, RUNNING, STOPPED, CALIB };
RTC_DATA_ATTR State state = SLEEP;                    // Состояние сохраняется в RTC памяти при сне
RTC_DATA_ATTR unsigned long stopwatchElapsed = 0;     // Накопленное время полета

struct SysData {
  float startAlt, alt, vel, maxV, minV, temp;
  float ax, ay, az;
  float gMagRef;                                      // Откалиброванное значение 1G
  bool track, sensInit, accInit;
  unsigned long tStart, tScreen;
  bool lastSt[3];                                     // Предыдущее состояние кнопок
} data;

int rtc_h, rtc_m, rtc_d, rtc_mon;
Adafruit_BMP3XX bmp;
GxEPD2_BW<GxEPD2_154_D67, 200> display(GxEPD2_154_D67(EPD_CS, EPD_DC, EPD_RES, EPD_BUSY));
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
TaskHandle_t vTaskH = NULL;

// --- КЛАСС ФИЛЬТРАЦИИ ВАРИОМЕТРА (EMA) ---
// Использует экспоненциальное скользящее среднее с адаптивным коэффициентом
// на основе уровня "турбулентности" (данных акселерометра).
class VarioEMA {
  float altFilt_      = 0.0f;
  float altPrev_      = 0.0f;
  float varioFilt_    = 0.0f;
  float accelLinFilt_ = 0.0f;
  bool  inited_       = false;

  // Расчет коэффициента alpha на основе временной постоянной tau и dt
  float alphaFromTau(float dt, float tau) {
      if (tau <= 0.0f) return 1.0f;
      float a = dt / (tau + dt);
      if (a < 0.0f) a = 0.0f;
      if (a > 1.0f) a = 1.0f;
      return a;
  }

public:
  void init(float initialAlt) {
      altFilt_      = initialAlt;
      altPrev_      = initialAlt;
      varioFilt_    = 0.0f;
      accelLinFilt_ = 0.0f;
      inited_       = true;
  }

  // Основной метод обновления фильтра.
  // accelLinMs2 - линейное вертикальное ускорение
  // baroAlt - сырая высота с барометра
  float update(float accelLinMs2, float baroAlt, float dt) {
      if (dt < 0.001f) dt = 0.001f;
      if (dt > 0.1f)   dt = 0.1f;

      if (!inited_) {
          init(baroAlt);
      }

      // 1. Фильтрация ускорения
      float aAcc = alphaFromTau(dt, CFG_TAU_ACCEL);
      accelLinFilt_ += aAcc * (accelLinMs2 - accelLinFilt_);

      // 2. Фильтрация высоты
      float aAlt = alphaFromTau(dt, CFG_TAU_BARO_ALT);
      altFilt_ += aAlt * (baroAlt - altFilt_);

      // 3. Расчет мгновенного варио (производная)
      float varioRaw = (altFilt_ - altPrev_) / dt;
      altPrev_ = altFilt_;

      // 4. Адаптация постоянной времени вариометра по уровню турбулентности
      float turb = fabsf(accelLinFilt_);
      float turbNorm = 0.0f;
      if (CFG_ACCEL_TURB_REF > 0.0f) {
          turbNorm = turb / CFG_ACCEL_TURB_REF;
          if (turbNorm > 1.0f) turbNorm = 1.0f;
      }

      float tauVario = CFG_TAU_BARO_VARIO_BASE + turbNorm * CFG_TAU_BARO_VARIO_TURB;
      float aVario = alphaFromTau(dt, tauVario);
      
      // 5. Итоговая фильтрация варио
      varioFilt_ += aVario * (varioRaw - varioFilt_);

      if (varioFilt_ >  25.0f) varioFilt_ =  25.0f;
      if (varioFilt_ < -25.0f) varioFilt_ = -25.0f;

      return varioFilt_ * CFG_VARIO_SENS;
  }

  float getAltitude() const { return altFilt_; }
  float getVario()    const { return varioFilt_ * CFG_VARIO_SENS; }
  float getAccelLin() const { return accelLinFilt_; }
};

VarioEMA varioEMA;

// --- ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ ---
void i2cWrite(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t bcd2dec(uint8_t v) { return ((v/16*10) + (v%16)); }

void readRTC() {
  Wire.beginTransmission(ADDR_RTC); Wire.write(0x02); Wire.endTransmission();
  Wire.requestFrom(ADDR_RTC, 6);
  if(Wire.available()) {
     Wire.read(); // skip seconds
     rtc_m = bcd2dec(Wire.read()&0x7F);
     rtc_h = bcd2dec(Wire.read()&0x3F);
     rtc_d = bcd2dec(Wire.read()&0x3F);
     Wire.read(); // skip
     rtc_mon = bcd2dec(Wire.read()&0x1F);
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
    // Настройка акселерометра BMA (Reset, Config, Enable)
    i2cWrite(ADDR_BMA, 0x7E, 0xB6); delay(20);
    i2cWrite(ADDR_BMA, 0x7C, 0x00); delay(10);
    i2cWrite(ADDR_BMA, 0x40, 0x28);
    i2cWrite(ADDR_BMA, 0x41, 0x01);
    i2cWrite(ADDR_BMA, 0x7D, 0x04); delay(50);
    data.accInit = true;
}

void drawItem(int x, int y, const GFXfont* f, String txt) {
    display.setFont(f);
    if (x < 0) { // Центрирование по горизонтали
        int16_t x1, y1; uint16_t w, h;
        display.getTextBounds(txt, 0, 0, &x1, &y1, &w, &h);
        x = (display.width() - w) / 2;
    }
    display.setCursor(x, y); display.print(txt);
}

// --- ЗАДАЧА ВАРИОМЕТРА (FreeRTOS Task) ---
// Обрабатывает сенсоры с высокой частотой и управляет вибросигналом
void varioTask(void *p) {
    int pulses = 0;
    bool pulseOn = false;
    unsigned long nextT = 0;
    bool pause = false;

    bool vibroActive = false;
    unsigned long vibroStopT = 0;
    unsigned long lastUpdateMicros = micros();

    float v_smooth = 0.0f;

    for(;;) {
        if (state == RUNNING && data.sensInit) {
            float ax = 0.0f, ay = 0.0f, az = 0.0f;
            float acc_lin_ms2 = 0.0f;

            unsigned long nowMicros = micros();
            float dt = (nowMicros - lastUpdateMicros) / 1000000.0f;
            if (dt < 0.001f) dt = 0.001f;
            if (dt > 0.1f)   dt = 0.1f;
            lastUpdateMicros = nowMicros;

            // Защита от чтения акселерометра во время работы вибромотора (шум)
            bool accSafe = !vibroActive && (millis() - vibroStopT > VIBRO_COOLDOWN_MS);

            if (data.accInit && accSafe) {
                Wire.beginTransmission(ADDR_BMA); Wire.write(0x12); Wire.endTransmission();
                Wire.requestFrom(ADDR_BMA, 6);
                if (Wire.available() >= 6) {
                    int16_t rx = Wire.read() | (Wire.read() << 8);
                    int16_t ry = Wire.read() | (Wire.read() << 8);
                    int16_t rz = Wire.read() | (Wire.read() << 8);
                    ax = rx / 8192.0f;
                    ay = ry / 8192.0f;
                    az = rz / 8192.0f;

                    float acc_mag = sqrtf(ax*ax + ay*ay + az*az);
                    float acc_linear_g = acc_mag - data.gMagRef; // Вычитаем гравитацию

                    if (fabsf(acc_linear_g) < 0.02f) acc_linear_g = 0.0f; // Deadzone

                    acc_lin_ms2 = acc_linear_g * GRAVITY_G;
                }
            }

            if (bmp.performReading()) {
                float baro_alt = bmp.readAltitude(SEALEVELPRESSURE_HPA);

                // Обновление фильтра
                float v = varioEMA.update(acc_lin_ms2, baro_alt, dt);

                // Обновление глобальных данных (в критической секции)
                portENTER_CRITICAL(&mux);
                data.alt  = varioEMA.getAltitude();
                data.vel  = v;
                data.temp = bmp.temperature;
                if (accSafe) { data.ax = ax; data.ay = ay; data.az = az; }
                portEXIT_CRITICAL(&mux);

                if (data.track) {
                    if (data.vel > data.maxV) data.maxV = data.vel;
                    if (data.vel < data.minV) data.minV = data.vel;
                }

                v_smooth += (data.vel - v_smooth) * 0.2f; // Дополнительное сглаживание для логики звука

                // --- ЛОГИКА ГЕНЕРАЦИИ ИМПУЛЬСОВ ---
                int reqP = 0;
                float v_check = v_smooth;

                if (v_check <= SINK_TRH) reqP = -1; // Сильное снижение
                else {
                    // Определение количества импульсов по порогам подъема
                    for (int i = 0; i < 4; i++)
                        if (v_check >= LIFT_TH[i] && v_check < LIFT_TH[i+1])
                            reqP = LIFT_PULSES[i];
                }

                unsigned long now = millis();
                bool setVibro = false;

                if (reqP == -1) {
                    // Непрерывный сигнал при снижении
                    setVibro = true; pulses = 0; pulseOn = 1; pause = 0;
                } else if (reqP == 0) {
                    // Тишина
                    setVibro = false; pulses = 0; pulseOn = 0; pause = 0;
                } else {
                    // Генерация пачек импульсов
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

                // Управление физическим пином
                if (setVibro) {
                    digitalWrite(PIN_VIBRO, 1); vibroActive = true;
                } else {
                    digitalWrite(PIN_VIBRO, 0);
                    if (vibroActive) vibroStopT = millis(); // Запомнить момент остановки для защиты акселерометра
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
        display.fillScreen(GxEPD_WHITE);
        display.setTextColor(GxEPD_BLACK);

        // Время полета
        unsigned long t = stopwatchElapsed + (state==RUNNING ? (millis()-data.tStart)/1000 : 0);
        sprintf(buf, "%02lu:%02lu:%02lu", t/3600, (t%3600)/60, t%60);
        drawItem(10, 20, &FreeSansBold9pt7b, buf);

        sprintf(buf, "%.1fc", dTemp); drawItem(80, 20, &FreeSansBold9pt7b, buf);

        // Заряд батареи
        float v = analogReadMilliVolts(PIN_BATT)/1000.0*2.0;
        sprintf(buf, "%d%%", v>=4.2?100 : (v<=3.3?0 : (int)((v-3.3)*111.1)));
        drawItem(140, 20, &FreeSansBold9pt7b, buf);

        if(state==RUNNING || state==STOPPED) {
            drawItem(25, 50, &FreeSansBold9pt7b, "Start, m   Sea, m");
            sprintf(buf, "%+d", (int)(dAlt - data.startAlt));
            drawItem(30, 90, &FreeSansBold18pt7b, buf);
            sprintf(buf, "%d", (int)dAlt);
            drawItem(130, 90, &FreeSansBold18pt7b, buf);
        }

        drawItem(10, 135, &FreeSansBold9pt7b, "Vario");
        if(data.track) {
            sprintf(buf, "max%+.1f min%+.1f", dMax, dMin);
            drawItem(60, 135, &FreeSansBold9pt7b, buf);
        }
        sprintf(buf, "%+.1f", dVel);
        drawItem(10, 190, &FreeMonoBold36pt7b, buf);
    } while (display.nextPage());
}

void drawClock(bool deep) {
    if(deep) { display.init(115200, true, 2, false); display.setFullWindow(); }
    else display.setPartialWindow(0,0,200,200);

    display.firstPage();
    do {
        display.fillScreen(GxEPD_WHITE);
        display.setTextColor(GxEPD_BLACK);
        float v = analogReadMilliVolts(PIN_BATT)/1000.0*2.0;
        char buf[20]; sprintf(buf, "%.2fV %d%%", v, (int)((v-3.3)*111.1));
        drawItem(10, 20, &FreeSansBold9pt7b, buf);

        sprintf(buf, "%02d:%02d", rtc_h, rtc_m);
        drawItem(-1, 110, &FreeSansBold24pt7b, buf);
        sprintf(buf, "%02d.%02d", rtc_d, rtc_mon);
        drawItem(-1, 160, &FreeSansBold18pt7b, buf);
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
    pinMode(BTN_SELECT, INPUT);
    pinMode(BTN_RIGHT, INPUT);
    pinMode(BTN_BACK, INPUT);
    pinMode(PIN_VARIO_EN, OUTPUT);
    pinMode(PIN_VIBRO, OUTPUT);
    pinMode(PIN_BATT, INPUT);
    digitalWrite(PIN_VARIO_EN, 0);
    digitalWrite(PIN_VIBRO, 0);

    Wire.begin();
    display.init(115200, true, 2, false);
    display.setRotation(1);
    setCpuFrequencyMhz(80); // Экономия энергии
    WiFi.mode(WIFI_OFF);

    readRTC();
    if (esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_EXT0) {
        // Просыпание не от кнопки -> показать часы и спать
        drawClock(true); goDeepSleep();
    }
    drawClock(true);
    state = IDLE;
}

void loop() {
    if (state == SLEEP) goDeepSleep();
    if (state == IDLE && millis() > 60000) goDeepSleep(); // Автовыключение

    bool bSel = digitalRead(BTN_SELECT);
    bool bR   = digitalRead(BTN_RIGHT);
    bool bBack= digitalRead(BTN_BACK);

    if (bBack && !data.lastSt[2]) {
        delay(50);
        if(digitalRead(BTN_BACK)) {
            readRTC(); drawClock(true); state = SLEEP;
        }
    }

    // Старт полета (Select)
    if (bSel && !data.lastSt[0] && (state==IDLE || state==STOPPED)) {
        delay(50);
        if(digitalRead(BTN_SELECT)) {
            digitalWrite(PIN_VARIO_EN, 1); delay(50); initSensors();
            display.setPartialWindow(0,0,200,200); display.firstPage();
            do {
                display.fillScreen(GxEPD_WHITE);
                drawItem(20, 90, &FreeSansBold18pt7b, "Calibrating...");
            } while(display.nextPage());

            if(data.sensInit) {
                // --- КАЛИБРОВКА И ПРОГРЕВ ФИЛЬТРА ---
                
                // 1. Инициализируем фильтр начальным значением перед циклом
                if (bmp.performReading()) {
                    varioEMA.init(bmp.readAltitude(SEALEVELPRESSURE_HPA));
                }

                float sumMag = 0.0f;
                const int samples = 100;

                for(int i=0; i<samples; i++) {
                    // Чтение Акселерометра (для калибровки G)
                    Wire.beginTransmission(ADDR_BMA); Wire.write(0x12); Wire.endTransmission();
                    Wire.requestFrom(ADDR_BMA, 6);
                    if(Wire.available()>=6) {
                        int16_t rx = Wire.read()|(Wire.read()<<8);
                        int16_t ry = Wire.read()|(Wire.read()<<8);
                        int16_t rz = Wire.read()|(Wire.read()<<8);
                        float _ax = rx/8192.0f;
                        float _ay = ry/8192.0f;
                        float _az = rz/8192.0f;
                        sumMag += sqrtf(_ax*_ax + _ay*_ay + _az*_az);
                    }

                    // Чтение Барометра и "прогрев" фильтра
                    if(bmp.performReading()) {
                        float rawAlt = bmp.readAltitude(SEALEVELPRESSURE_HPA);
                        // Передаем сырую высоту в фильтр.
                        // Линейное ускорение считаем 0 (мы стоим на месте).
                        // dt ставим ~0.02 сек (delay 10 + время I2C).
                        varioEMA.update(0.0f, rawAlt, 0.02f);
                    }
                    delay(10);
                }

                // Завершаем калибровку акселерометра
                data.gMagRef = sumMag / samples;
                if(data.gMagRef < 0.5f || data.gMagRef > 1.5f) data.gMagRef = 1.0f; // Fallback

                // 2. Берем высоту ИЗ ФИЛЬТРА, а не среднее арифметическое
                // Это убирает "ступеньку" при старте
                float finalAlt = varioEMA.getAltitude();

                portENTER_CRITICAL(&mux);
                data.startAlt = finalAlt; 
                data.alt      = finalAlt;
                data.vel      = 0.0f;
                data.maxV     = 0.0f;
                data.minV     = 0.0f;
                portEXIT_CRITICAL(&mux);

                data.track = false;
                stopwatchElapsed = 0;
                data.tStart = millis();
                
                // Запуск задачи обработки (если еще не запущена)
                if(!vTaskH) xTaskCreatePinnedToCore(varioTask, "V", 4096, NULL, 10, &vTaskH, 0);
                state = RUNNING;
            }
        }
    }

    // Сброс времени или стоп полета (Right)
    if (bR && !data.lastSt[1]) {
        delay(50);
        if(digitalRead(BTN_RIGHT)) {
            if (state==RUNNING) {
                stopwatchElapsed += (millis()-data.tStart)/1000;
                state=STOPPED;
                digitalWrite(PIN_VIBRO, 0);
            } else if (state==IDLE) {
                // Сброс RTC в 00:00
                i2cWrite(ADDR_RTC, 0x02, 0);
                i2cWrite(ADDR_RTC, 0x03, 0);
                i2cWrite(ADDR_RTC, 0x04, 0);
                rtc_h=0; rtc_m=0; drawClock(false);
            }
        }
    }

    data.lastSt[0]=bSel; data.lastSt[1]=bR; data.lastSt[2]=bBack;

    if(state == RUNNING && !data.track && (millis() - data.tStart > 5000)) data.track = true;

    // Обновление экрана
    if ((state == RUNNING || state == STOPPED) && (millis() - data.tScreen >= REFRESH_MS)) {
        data.tScreen = millis(); drawMain();
    }
    delay(10);
}
