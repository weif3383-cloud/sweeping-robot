# 掃地機器人（ESP32 NodeMCU-32S）

自己動手做的掃地機器人：2 顆 TT 馬達 + L298N、HC-020K 編碼器、MPU6050、HC-SR04，2S 18650 供電。

## 資料夾（從舊到新）

| 資料夾 | 內容 |
|---|---|
| `迭代一_小車馬達/` | car_test、car_test02：第一版小車測馬達 |
| `迭代二_編碼器/` | encoder_test → encoder_read → encoder_wifi：編碼器、走圈數、無線控制、路徑地圖 |
| `迭代三_陀螺儀/` | **robot_wifi（目前主程式）**：＋ MPU6050、家裡 WiFi、`/data` |
| `docs/` | 腳位總表 `PINOUT_v2.md` |
| `cad/` | v2 底盤 STL 與產生腳本（Python） |
| `代辦.md` | 待辦清單 |

每個迭代資料夾裡都有 `說明.md`，寫了裡面程式的先後順序。

## 使用 robot_wifi

1. 把 `迭代三_陀螺儀/robot_wifi/secrets_example.h` 複製成同資料夾的 `secrets.h`，填入家裡 WiFi（2.4 GHz）
2. 用 Arduino IDE（ESP32 core 3.x）開 `迭代三_陀螺儀/robot_wifi/robot_wifi.ino`，選 NodeMCU-32S 燒錄
3. 開機後車子靜止 1 秒（陀螺儀量零點）
4. 家裡 WiFi：http://robot.local（或網頁上顯示的 IP）；熱點：連 `RobotCar`（密碼 `12345678`）開 http://192.168.4.1

## 注意

- USB 和電池（MP1584 → ESP32 5V）不要同時供電給 ESP32
- `secrets.h`（家裡 WiFi 密碼）不會上傳到 GitHub
