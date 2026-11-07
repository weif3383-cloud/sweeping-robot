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
- **Arduino IDE 開著檔案時，按編譯/上傳會把 IDE 裡的舊內容存回去，蓋掉 Claude 寫入的新版**（2026-11-07 OTA 版被蓋掉過）。
  改完檔案要請使用者關掉該分頁 / 重新開啟再燒錄
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
- 階段 ③ 實測（固定提前度數版）：22 次轉彎後 90° 收斂到 ±0.5°，但 45° 少轉約 3°、180° 多轉約 2°
  → v3 改成「提前度數 = K × 放開時轉速」，學 K（秒），模擬 45/90/180 都在 ±0.6°
- 階段 ④ 直走鎖方向（v3）：陀螺儀 PID（KH 6、KHI 8、KHD 0.4）+ 編碼器同步只留 40% 比例項（積分會跟陀螺儀打架）；
  鎖方向時兩輪平均走到才一起停。模擬：一邊打滑時偏 3.5° → 0.9°
- 自動測試方法：內建瀏覽器開 robot.local，用 javascript_tool 在頁面裡 fetch /cmd、輪詢 /status 等 IDLE、最後讀 /data
- v3 實車校準結果（已寫進程式預設）：轉彎 K 右 0.094 左 0.079 秒 → 45°/90°/180° 都在 ±1°；
  直走鎖方向 KH 14、KHI 25、KHD 0.7 → 前進/後退 2 圈停下偏 ±0.9° 內（關掉時偏 −4.7～−7.1°）
- 車子 vs 地板測試（鎖方向關，來回 3 趟）：6 次都往車子左邊偏 → 主要是車子（左輪吃虧）；地板讓偏差忽大忽小（−3～−20°）
- v4：直走微調自動學習（trim 前進/後退各一，存 Preferences）；直走一律平均走到一起停；直走提前量用平均誤差學
- v4 實測：鎖方向開 10 趟偏 ±1.2° 內，學到微調前進 +8.1、後退 +4.2 PWM；但鎖方向關掉時仍偏 −3.5～+4.7°（沒改善）
  → 原因：編碼器同步堅持左右格數一樣，但走直其實要左輪多轉約 2.3%（10 趟平均 dl/dr = 1.023）
- v5：學「輪子比例」wheelRatio（左格數 ÷ 右格數，預設 1.023，存 Preferences），用在同步、平均距離、地圖。模擬鎖方向關：−3.7° → ±1.5°
- 使用者流程：改程式前先請使用者關掉 Arduino IDE，寫入後再請他打開燒錄（IDE 會把舊內容存回去）
- v5 實測：輪子比例學到 1.034；鎖方向關掉時直走 2 圈偏 +0.3、+0.9、+0.6、−2.1、+0.7、+0.7°
  （v4 時 −3.5～+4.7°、最早 −4.7～−7.1°）。repo 預設已改 1.034（使用者資料夾等下次燒錄時一起寫入）
- OTA：IDE 顯示 No response from device → 多半是 Windows 防火牆 / 公用網路；已給使用者設定步驟（Claude 不能改系統安全設定）
- 下一步：陀螺儀比例校正（需使用者目視 360° 是否轉回原方向）；D 校正（需捲尺）
