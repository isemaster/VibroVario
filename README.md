# VibroVario  
*The variometer that vibrates — no sound, no distraction. Works on ESP32 E-Ink watches.*

[![VibroVario in lift] [https://github.com/isemaster/VibroVario/main/media/varioinlift.mp4](https://github.com/isemaster/VibroVario/blob/main/media/varioinlift.mp4)
*(Click to watch: reaction in lift)*

- ✅ **Silent** — no beeping in your ears, no distraction  
- ✅ **Tactile coding** — your hand feels better than ears:  
  - `·` (100 ms) = +0.5 m/s  
  - `· –` (100 + 300 ms) = +1.0 m/s  
  - `– – – –` = sinking (caution!)  
- ✅ **E-Ink display** — visible in direct sunlight, 0 mW in idle  
- ✅ **One file** — just `VibroVario.ino`, open in Arduino IDE → upload → fly  
- ✅ **Lightweight** — 38 g (with battery)  
- ✅ **Long runtime** — 14+ hours on 200 mAh LiPo  

---

### 🧰 Hardware

| Part | Where to buy | Price |
|------|--------------|-------|
| ESP32 Open Source Watchy V2.0 | [AliExpress](https://aliexpress.com/item/xxxx) | $30 |
| BMP390 Sensor | [AliExpress](https://aliexpress.ru/item/1005007988507429.html) | $3 |

> 🔋 Note: Uses stock watch battery (200 mAh) — no extra weight.

---

### 🔌 Wiring

1. Open the watch (4 screws on the back)  
2. Solder BMP390:  
   - `VCC` → `3.3V`  
   - `GND` → `GND`  
   - `SCL` → `GPIO 22`  
   - `SDA` → `GPIO 21`  

![Wiring guide](https://github.com/isemaster/VibroVario/raw/main/docs/wiring.jpg)  
*(Photo: red = VCC, black = GND, yellow = SCL, green = SDA)*

---

### ⚙️ Setup

1. Install **Arduino IDE**  
2. Add ESP32 support:  
   - `File → Preferences → Additional Boards Manager URLs`:  
     ```
     https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
     ```
3. Install boards:  
   - `Tools → Board → Boards Manager → search "esp32" → install "ESP32 by Espressif"`  
4. Install libraries (**Sketch → Include Library → Manage Libraries**):  
   - `BMP390` by Bosch  
   - `GxEPD2` by Jean-Marc Zingg  
   - `BMA423` by Bosch *(optional, for motion compensation)*  
5. Open `VibroVario.ino`  
6. Select:  
   - Board: **ESP32 Dev Module**  
   - Port: your COM port  
7. Click **Upload** → done! ✅  

> 📌 First run: keep still for 20 seconds — auto-calibration.

---

### 📊 Tech Specs

| Metric | Value |
|--------|-------|
| Weight | 38 g (with battery) |
| Dimensions | 46 × 38 × 14 mm |
| Vz resolution | ±5 cm/s (BMP390 + 2nd-order filter) |
| Latency | 280 ms |
| Avg current | 7.1 mA (5 vib/min) |
| Runtime | 28.2 h (theoretical), 14+ h (real flight) |

- `Vz` — vertical speed (updates 2×/sec)  
- Battery % and temperature from BMP390  

---

### 🖼️ Gallery

| ![Mod](https://github.com/isemaster/VibroVario/raw/main/media/watch-mod.jpg) | ![Screen](https://github.com/isemaster/VibroVario/raw/main/media/eink-screen.jpg) |
|:---:|:---:|
| BMP390 soldered inside | E-Ink display in sunlight |

---

### 📜 License

MIT — use, modify, sell. Just keep the attribution.

---

### ❓ FAQ

**Q: What if I have older watch (ESP32-PICO-D4)?**  
A: Works! Just slower E-Ink refresh (~1.5 sec vs 0.8 sec).  

**Q: Can I disable E-Ink to save power?**  
A: Yes! Uncomment `#define DISABLE_DISPLAY` at the top of `.ino`.  

**Q: Where’s the calibration code?**  
A: Built-in: first 20 sec = auto-offset calibration. For advanced: see `extras/calibration.ino`.  

**Q: Does it compensate for G-forces?**  
A: Yes — if your watch has BMA423 (most do), it subtracts vertical acceleration.

---

### 🙏 Thanks to

- Arduino IDE  
- BMP390 + BMA423 (Bosch)  
- ESP32-S3 / ESP32-PICO-D4  
- And a lot of test flights ✈️
