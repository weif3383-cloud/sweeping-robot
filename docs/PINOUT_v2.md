# 掃地機器人 v2 腳位與接線總表

> 板子：NodeMCU-32S（ESP32）　韌體：Arduino ESP32 core 3.x
> 最後更新：2026-11-07（換到 v2 新底盤前的整理）
> 改了任何接線，就同時改這份檔案和韌體裡的腳位常數。

---

## 1. ESP32 腳位總表

| GPIO | 板上標示 | 接到 | 方向 | 備註 |
|---|---|---|---|---|
| 32 | P32 | L298N **IN3** | 輸出 PWM | 左輪 L_IN1 |
| 33 | P33 | L298N **IN4** | 輸出 PWM | 左輪 L_IN2 |
| 26 | P26 | L298N **IN1** | 輸出 PWM | 右輪 R_IN1 |
| 27 | P27 | L298N **IN2** | 輸出 PWM | 右輪 R_IN2 |
| 34 | P34 | 左 HC-020K **OUT** | 輸入 | 只能當輸入，沒有內建上拉 |
| 35 | P35 | 右 HC-020K **OUT** | 輸入 | 只能當輸入，沒有內建上拉 |
| 21 | P21 | MPU6050 **SDA** | I2C | |
| 22 | P22 | MPU6050 **SCL** | I2C | |
| 4 | P4 | HC-SR04 **Trig** | 輸出 | |
| 39 | SVN / VN | HC-SR04 **Echo**（經過分壓） | 輸入 | **不可以直接接 Echo**，見第 4 節 |
| 13 | P13 | 側刷 MOSFET 模組 **TRIG/PWM** | 輸出 PWM | 5 kHz，duty 190 |
| 25 | P25 | 風扇 MOSFET 模組 **TRIG/PWM** | 輸出 PWM | 5 kHz，0 / 128 / 255 |
| 5V | 5V | MP1584 **OUT+**（5.0 V） | 電源輸入 | 用電池時 |
| 3V3 | 3V3 | 麵包板 3.3V 軌 | 電源輸出 | MPU6050、HC-020K ×2 |
| GND | GND | 麵包板 GND 軌 | — | **所有模組共地** |

### 還沒用的腳（之後擴充可以用）
- 一般 IO：GPIO 14、16、17、18、19、23
- 只能輸入：GPIO 36（SVP / VP）
- **盡量不要用**：GPIO 0、2、5、12、15（開機時有特殊用途）、GPIO 6–11（接 Flash，不能用）、GPIO 1、3（USB 序列埠）

---

## 2. 馬達：L298N

| L298N | 接到 |
|---|---|
| IN1 | GPIO 26（右） |
| IN2 | GPIO 27（右） |
| IN3 | GPIO 32（左） |
| IN4 | GPIO 33（左） |
| ENA / ENB | **跳帽保持插著**（不接 ESP32） |
| OUT1 / OUT2 | 右 TT 馬達 |
| OUT3 / OUT4 | 左 TT 馬達 |
| 12V | 電池 P+（經過開關） |
| GND | 電池 P− ＋ 共地 |
| 5V | **不接**（ESP32 改由 MP1584 供電） |

- 韌體設定：`L_INVERT = true`、`R_INVERT = true`（v2.3 測試確定的方向）
- 如果之後換線導致方向錯了，先改 INVERT，不用重接線。

---

## 3. 編碼器：HC-020K ×2（三支腳：VCC / GND / OUT）

| HC-020K | 接到 |
|---|---|
| VCC | **3.3V**（先試；規格是 5V，見下方說明） |
| GND | GND |
| OUT | 左 → GPIO 34，右 → GPIO 35 |

- ⚠ 模組規格寫 5 V（4.5–5.5 V）。目前**先試 3.3 V**：OUT 輸出也是 3.3 V，可以直接接 GPIO，不用加零件。
  - 驗證：燈會跟著轉動一閃一閃，`n` / `j` 轉一圈約 20 格 → 就維持 3.3 V
  - 如果燈不閃、數字亂跳或漏格 → 改接 5V，但 **OUT 一定要經過分壓才能接 GPIO 34/35**，用 10kΩ（上）＋ 20kΩ（下，或 10kΩ×2 串聯）。不要用 1kΩ 分壓，模組輸出可能有內建上拉電阻，1kΩ 會把高電位拉得太低。
- 碼盤 20 格，轉一圈 = 20 個脈波（`SLOTS = 20`）
- 碼盤的缺口那一圈要經過 U 型槽的小窗；轉動時模組上的燈要一閃一閃

---

## 4. 超音波：HC-SR04（Echo 要分壓）

| HC-SR04 | 接到 |
|---|---|
| VCC | **5V** |
| GND | GND |
| Trig | GPIO 4 |
| Echo | 分壓電路 → GPIO 39 |

Echo 分壓（5 V → 約 3.3 V）：

```
Echo ──[1kΩ]──┬── GPIO 39 (SVN/VN)
              │
           [1kΩ]
              │
           [1kΩ]     ← 兩顆 1kΩ 串聯 = 2kΩ
              │
             GND
```

---

## 5. 陀螺儀：MPU6050

| MPU6050 | 接到 |
|---|---|
| VCC | **3.3V** |
| GND | GND |
| SCL | GPIO 22 |
| SDA | GPIO 21 |
| XDA / XCL / AD0 / INT | 不接 |

- I2C 位址 0x68，速度 400 kHz
- 已測試：順時針轉約 90° → yaw 讀到 −92°，正常

---

## 6. 側刷與風扇：MOSFET 模組 ×2

| MOSFET 模組 | 側刷 | 風扇 |
|---|---|---|
| TRIG / PWM | GPIO 13 | GPIO 25 |
| GND（訊號那側） | GND | GND |
| VIN+ / VIN− | 電池 P+ / P− | 電池 P+ / P− |
| OUT+ / OUT− | N20 馬達 | 吸塵風扇 |

- N20 兩端要並聯 **1N5819**：有白色條紋那端接 **OUT+**
- N20 是 6 V 馬達，用 7.4 V 電池時 PWM 只開約 75%（duty 190）

---

## 7. 電源

```
18650 ×2（2S，Samsung 35E）
   │  B− / BM（中間抽頭）/ B+
2S 保護板（BMS）
   │  P+ / P−
開關（電池盒上）
   ├── L298N 12V / GND
   ├── MOSFET 模組 VIN ×2
   ├── Type-C 2S 充電模組（充電時開關要打開）
   └── MP1584（先調到 5.0 V）──→ ESP32 5V / GND
```

**重要規則**
1. **USB 和電池不要同時接**：插 USB 燒錄時，電池開關關掉（或拔掉 MP1584 到 ESP32 5V 的線）。
2. MP1584 是可調式的，**接 ESP32 前一定要先用電表量到 5.0 V**。
3. L298N 的 5V 腳**不要**接到 ESP32。
4. 所有 GND 都要接在一起（電池 P−、L298N、MP1584、ESP32、各模組）。

---

## 8. 麵包板電源軌

| 軌 | 電壓 | 來源 |
|---|---|---|
| 上排紅 | 3.3V | ESP32 3V3 |
| 下排紅 | 5V | ESP32 5V 腳（USB 或 MP1584） |
| 上下藍 | GND | 上下兩排要互相接通 |

接 3.3V 的：MPU6050、HC-020K ×2
接 5V 的：HC-SR04

---

## 9. 韌體裡的腳位常數（v2_bench_test.ino）

```cpp
const int L_IN1 = 32, L_IN2 = 33;      // 左輪：L298N IN3 / IN4
const int R_IN1 = 26, R_IN2 = 27;      // 右輪：L298N IN1 / IN2
const bool L_INVERT = true, R_INVERT = true;
const int ENC_L = 34, ENC_R = 35;
const int SLOTS = 20;
const int TRIG = 4, ECHO = 39;
const int BRUSH_PIN = 13, FAN_PIN = 25;
const int SDA_PIN = 21, SCL_PIN = 22;
const uint8_t MPU = 0x68;
```

（remote_car.ino 和 encoder_test.ino 的腳位跟這裡一樣。）

---

## 10. 接完後的檢查順序

1. 先不接 ESP32，打開電池，量 MP1584 輸出 = 5.0 V
2. 關電池，插 USB，燒 v2_bench_test
3. `i` → MPU6050 有回應
4. `u` → 超音波距離合理
5. `o` 監看，用手轉輪子 → 左右編碼器數字會變
6. `w / s / a / d` → 馬達方向正確
7. `n` / `j` → 輪子各轉一圈後停下，約 20 格
8. `b` → 側刷轉；`f` → 風扇三段
9. 拔 USB，改用電池供電，再跑一次 `w`
