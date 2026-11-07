// 把這個檔案複製一份，改名成 secrets.h（跟 robot_wifi.ino 放同一個資料夾），填入家裡 WiFi。
// secrets.h 不會上傳到 GitHub。只支援 2.4 GHz 的 WiFi（ESP32 不支援 5 GHz）。
#pragma once
#define HOME_WIFI_SSID "你家WiFi名稱"
#define HOME_WIFI_PASS "你家WiFi密碼"
// （選用）無線燒錄密碼：取消下一行的註解並改成你要的密碼，Arduino IDE 上傳時會問
// #define OTA_PASS "自己設一組"
