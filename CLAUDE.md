# 專案說明（給 Claude 看的工作筆記）

使用者：台灣，用繁體中文溝通。ESP32 NodeMCU-32S 掃地機器人 v2（24 cm 圓形底盤）。

## 同步流程
- 使用者電腦資料夾：`C:\Users\user\Desktop\掃地機器人`（透過 remote-devices 連線）
- GitHub：`weif3383-cloud/sweeping-robot`（private，branch main）
- **使用者資料夾是主要來源**（他會在 Arduino IDE 裡改）。改程式前先把那邊的檔案拉回來比對，
  改完同時寫回他的資料夾並 commit / push。
- `secrets.h`（家裡 WiFi 名稱密碼）由使用者自己填，不讀、不上傳（.gitignore）。

## 硬體 / 腳位
完整見 `docs/PINOUT_v2.md`。重點：
- 馬達：左 IN3/IN4 → GPIO 32/33，右 IN1/IN2 → GPIO 26/27，L_INVERT = R_INVERT = true
- 編碼器 HC-020K（VCC/GND/OUT）：左 34、右 35，3.3V 供電，碼盤 20 缺口
- MPU6050：SDA 21、SCL 22，3.3V，位址 0x68
- HC-SR04：Trig 4、Echo 經 1k/2k 分壓 → 39
- MOSFET：側刷 13、風扇 25
- 輪徑 65 mm，有效輪距 199 mm（陀螺儀實測）

## Arduino 注意事項（踩過的雷）
- ESP32 core 3.3.x：`ledcAttach(pin, freq, bits)` / `ledcWrite(pin, duty)`；`timerBegin(hz)` / `timerAttachInterrupt` / `timerAlarm`
- Arduino IDE 會在第一個函式前自動產生函式宣告：自訂 struct 要放在所有函式之前
- `inline` + `IRAM_ATTR` 一起用會連結失敗（dangerous relocation），ISR 函式不要加 inline
- 不要用預設參數（自動宣告會出錯）
- 編碼器用計時器中斷每 100 µs 讀 `GPIO_IN1_REG`，連續 5 次才承認，避免 WiFi 忙時漏格

## 資料夾結構
程式依迭代分類（新程式放到對應或新的迭代資料夾，並更新該資料夾的 `說明.md`）：
- `迭代一_小車馬達/`：car_test、car_test02
- `迭代二_編碼器/`：encoder_test、encoder_read、encoder_wifi
- `迭代三_陀螺儀/`：robot_wifi（目前主程式）

## 代辦
見 `代辦.md`（使用者要的檔名），每次做完一項就更新。

## 目前進度
- encoder_wifi：走圈數 / 原地轉、左右同步 PI、柔和停車、提前量自動學習、20/40 格、里程計路徑地圖
- robot_wifi（主程式）：MPU6050 階段 ①② 完成（讀取、自動零點、正負號自動判斷、地圖方向用陀螺儀）、
  家裡 WiFi（secrets.h）+ RobotCar 熱點、mDNS robot.local、`/data` JSON（參數、狀態、最近 30 趟紀錄）
- 讀數據：用內建瀏覽器開 http://robot.local/data（或 IP），電腦要跟機器人在同一個 WiFi
- 2026-11-07 實測：陀螺儀正負號 −1（已自動存）、零點約 −0.23 °/s；40 格模式轉彎 6 次 → 有效輪距 199 mm
- 階段 ③ 轉彎用陀螺儀停：已寫好（提前量左右各自學、存 Preferences），待實車測
- 下一步：④ 直走鎖方向（陀螺儀 + 編碼器同步一起用，使用者選 A）、
  陀螺儀比例校正（原地轉 5 圈）、D/W 校正（正方形測試）
