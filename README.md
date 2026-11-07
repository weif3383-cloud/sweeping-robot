# 掃地機器人（ESP32 NodeMCU-32S）

自己動手做的掃地機器人：2 顆 TT 馬達 + L298N、HC-020K 編碼器、MPU6050、HC-SR04，2S 18650 供電。

## 資料夾

| 資料夾 | 內容 |
|---|---|
| `car_test/` | 第一版：序列埠指令測馬達 |
| `car_test02/` | 第一版修改 |
| `encoder_test/` | 編碼器測試（含馬達、抗抖動） |
| `encoder_read/` | 編碼器讀取 + 走指定圈數（序列埠版） |
| `encoder_wifi/` | 無線網頁控制、走圈數、左右同步、柔和停車、路徑地圖（沒有 MPU 的穩定版） |
| `robot_wifi/` | **目前主程式**：encoder_wifi ＋ MPU6050 陀螺儀 ＋ 連家裡 WiFi ＋ `/data` 數據 |
| `docs/` | 腳位總表 `PINOUT_v2.md` |
| `cad/` | v2 底盤 STL 與產生腳本（Python） |

## 使用 robot_wifi

1. 把 `robot_wifi/secrets_example.h` 複製成 `robot_wifi/secrets.h`，填入家裡 WiFi（2.4 GHz）
2. 用 Arduino IDE（ESP32 core 3.x）開 `robot_wifi/robot_wifi.ino`，選 NodeMCU-32S 燒錄
3. 開機後車子靜止 1 秒（陀螺儀量零點）
4. 家裡 WiFi：http://robot.local（或網頁上顯示的 IP）；熱點：連 `RobotCar`（密碼 `12345678`）開 http://192.168.4.1

## 注意

- USB 和電池（MP1584 → ESP32 5V）不要同時供電給 ESP32
- `secrets.h`（家裡 WiFi 密碼）不會上傳到 GitHub
