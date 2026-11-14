/*
  紅外線測試 ir_test.ino（紅外線模組 ×2，4 腳 VCC / GND / A0 / D0，只用 D0）
  ------------------------------------------------------------
  接線（見 docs/PINOUT_v2.md 第 6.5 節）
    紅外線 VCC → 3.3V   GND → GND
    左 D0 → GPIO 16       右 D0 → GPIO 17       A0 不接

  序列埠監控視窗：115200
  用手或白紙慢慢靠近、遠離模組，狀態有變就會印出來。
  多數模組（LM393）偵測到東西時 D0 = 0（LOW），沒東西 = 1；
  實際是哪一種，看印出來的結果就知道。
  模組上有電位器：用螺絲起子慢慢轉，調整多遠會偵測到。
*/

const int IR_L = 16, IR_R = 17;

int lastL = -1, lastR = -1;
unsigned long lastPrint = 0;

void setup() {
  Serial.begin(115200);
  delay(300);
  pinMode(IR_L, INPUT);
  pinMode(IR_R, INPUT);
  Serial.println("紅外線測試：用手或白紙靠近模組，看左 / 右的數字");
}

void loop() {
  int l = digitalRead(IR_L), r = digitalRead(IR_R);
  unsigned long now = millis();
  if ((l != lastL || r != lastR) && now - lastPrint >= 50) {
    lastPrint = now; lastL = l; lastR = r;
    Serial.printf("左 %d %s    右 %d %s\n", l, l ? "□" : "■", r, r ? "□" : "■");
  }
}
