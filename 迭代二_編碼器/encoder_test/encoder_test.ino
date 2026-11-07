/*
  編碼器測試 encoder_test.ino（HC-020K ×2，三支腳 VCC / GND / OUT）
  ------------------------------------------------------------
  接線（見 PINOUT_v2.md）
    HC-020K VCC → 3.3V   GND → GND
    左 OUT → GPIO 34      右 OUT → GPIO 35
  馬達（只有打 m / n / j 才會動）
    左輪 L298N IN3/IN4 → GPIO 32/33，右輪 IN1/IN2 → GPIO 26/27

  序列埠監控視窗：115200，結尾選「Newline」

  指令
    （開機後直接用手轉輪子，數字有變就會印出來）
    v  即時看 OUT 電位（調整碼盤插入深度用）—— 再按一次關掉
    z  計數歸零
    m  兩輪一起往前轉 3 秒，算轉速
    n  左輪轉一圈（數到 20 格停）
    j  右輪轉一圈（數到 20 格停）
    + / -  馬達速度（PWM）加減
    x  停止
    h  說明
*/

// ---------- 腳位 ----------
const int ENC_L = 34, ENC_R = 35;
const int L_IN1 = 32, L_IN2 = 33;          // 左輪：L298N IN3 / IN4
const int R_IN1 = 26, R_IN2 = 27;          // 右輪：L298N IN1 / IN2
const bool L_INVERT = true, R_INVERT = true;

// ---------- 參數 ----------
const int SLOTS = 20;                       // 碼盤格數 = 轉一圈的脈波數
const int FILTER_SAMPLES = 20;              // 取樣 10 kHz：新電位要連續 20 次（2 ms）才算數
const unsigned long REV_TIMEOUT_MS = 5000;  // 轉一圈測試最多跑 5 秒
const unsigned long RUN_MS = 3000;          // m 測試時間
int speed = 160;

// ---------- 計數 ----------
// 不用「一有邊緣就中斷」：碼盤停在缺口邊緣時，感測器會在 0/1 之間快速抖動，
// 每抖一下都會被當成一格。改成計時器每 0.1 ms 取樣一次，電位穩定 2 ms 才承認改變。
volatile long cntL = 0, cntR = 0;
volatile long noiseL = 0, noiseR = 0;       // 原始訊號跳動、但沒被承認的次數
struct Filt { int integ; int state; int raw; };
volatile Filt fL = {0, 0, 0}, fR = {0, 0, 0};
hw_timer_t *timer = nullptr;

inline void IRAM_ATTR step(volatile Filt &f, int level, volatile long &cnt, volatile long &noise) {
  if (level != f.raw) { f.raw = level; if (level == f.state) noise++; }   // 跳回原狀態 = 抖動
  if (level == f.state) { f.integ = 0; return; }                         // 只要中間跳回來一次就重算
  if (++f.integ >= FILTER_SAMPLES) {                                     // 連續 2 ms 都是新電位才承認
    f.state = level; f.integ = 0;
    if (level) cnt++;                                                    // 穩定變成 1 → 算一格
  }
}
void IRAM_ATTR onTimer() {
  step(fL, digitalRead(ENC_L), cntL, noiseL);
  step(fR, digitalRead(ENC_R), cntR, noiseR);
}

void readCounts(long &l, long &r, long &nl, long &nr) {
  noInterrupts(); l = cntL; r = cntR; nl = noiseL; nr = noiseR; interrupts();
}

// ---------- 馬達 ----------
void motor(int in1, int in2, int v) {
  v = constrain(v, -255, 255);
  if (v >= 0) { ledcWrite(in1, v); ledcWrite(in2, 0); }
  else        { ledcWrite(in1, 0); ledcWrite(in2, -v); }
}
void drive(int left, int right) {
  motor(L_IN1, L_IN2, L_INVERT ? -left : left);
  motor(R_IN1, R_IN2, R_INVERT ? -right : right);
}

// ---------- 狀態 ----------
enum Mode { IDLE, RUN, REV };
Mode mode = IDLE;
int revSide = 0;                 // -1 左, +1 右
long revStart = 0, lastShownL = -1, lastShownR = -1, lastShownN = 0;
long runStartL = 0, runStartR = 0;
unsigned long modeStart = 0, lastPrint = 0;
bool watchLevel = false;

void zero() {
  if (mode != IDLE) { drive(0, 0); mode = IDLE; Serial.println("測試中按了 z → 先停止測試"); }
  noInterrupts(); cntL = cntR = noiseL = noiseR = 0; interrupts();
  revStart = runStartL = runStartR = 0;
  lastShownL = lastShownR = -1; lastShownN = 0;
  lastPrint = 0;
  Serial.println("計數歸零，繼續轉輪子就會往上數");
}

void stopAll(const char *why) {
  drive(0, 0); mode = IDLE;
  if (why) Serial.println(why);
}

void help() {
  Serial.println();
  Serial.println("=== 編碼器測試 ===");
  Serial.println("用手轉輪子：數字有變就會印出來，轉一圈應該約 +20");
  Serial.println("v 看 OUT 電位(再按關)  z 歸零  m 兩輪轉3秒  n 左轉一圈  j 右轉一圈");
  Serial.println("+/- 速度  x 停止  h 說明");
  Serial.printf("目前速度 PWM = %d\n\n", speed);
}

void printCounts() {
  long l, r, nl, nr; readCounts(l, r, nl, nr);
  Serial.printf("左 %5ld 格 (%.2f 圈)   右 %5ld 格 (%.2f 圈)", l, (float)l / SLOTS, r, (float)r / SLOTS);
  if (nl || nr) Serial.printf("   抖動(已濾掉) 左%ld 右%ld", nl, nr);
  Serial.println();
}

void handle(char c) {
  switch (c) {
    case 'v': watchLevel = !watchLevel;
              Serial.println(watchLevel ? "即時電位：慢慢轉輪子，0 和 1 應該一格一格交替；一直 0 或一直 1 就調整碼盤插入深度" : "關閉即時電位");
              break;
    case 'z': zero(); break;
    case 'm': {
      long nl, nr; readCounts(runStartL, runStartR, nl, nr);
      drive(speed, speed); mode = RUN; modeStart = millis();
      Serial.printf("兩輪往前轉 %lu 秒（PWM %d）...\n", RUN_MS / 1000, speed);
    } break;
    case 'n': case 'j': {
      long l, r, nl, nr; readCounts(l, r, nl, nr);
      revSide = (c == 'n') ? -1 : 1;
      revStart = (revSide < 0) ? l : r;
      if (revSide < 0) drive(speed, 0); else drive(0, speed);
      mode = REV; modeStart = millis();
      Serial.printf("%s輪轉一圈（數到 %d 格停）...\n", revSide < 0 ? "左" : "右", SLOTS);
    } break;
    case '+': speed = min(255, speed + 10); Serial.printf("速度 PWM = %d\n", speed); break;
    case '-': speed = max(60, speed - 10);  Serial.printf("速度 PWM = %d\n", speed); break;
    case 'x': stopAll("停止"); break;
    case 'h': help(); break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  pinMode(ENC_L, INPUT);           // GPIO 34/35 沒有內建上拉，靠模組自己的輸出
  pinMode(ENC_R, INPUT);
  int l0 = digitalRead(ENC_L), r0 = digitalRead(ENC_R);
  fL.raw = fL.state = l0; fL.integ = 0;
  fR.raw = fR.state = r0; fR.integ = 0;
  timer = timerBegin(1000000);              // 1 MHz 計時器
  timerAttachInterrupt(timer, &onTimer);
  timerAlarm(timer, 100, true, 0);          // 每 100 µs 取樣一次
  const int ins[] = {L_IN1, L_IN2, R_IN1, R_IN2};
  for (int p : ins) { ledcAttach(p, 1000, 8); ledcWrite(p, 0); }
  help();
  Serial.printf("開機時 OUT 電位：左 %d  右 %d\n", digitalRead(ENC_L), digitalRead(ENC_R));
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c != '\n' && c != '\r' && c != ' ') handle(c);
  }

  long l, r, nl, nr; readCounts(l, r, nl, nr);
  unsigned long now = millis();

  if (mode == REV) {
    long done = ((revSide < 0) ? l : r) - revStart;
    if (done >= SLOTS) {
      stopAll(nullptr);
      Serial.printf("完成：%s輪數到 %ld 格。看輪子是不是剛好轉一圈（停下後慣性可能多 1~2 格）\n",
                    revSide < 0 ? "左" : "右", done);
    } else if (now - modeStart > REV_TIMEOUT_MS) {
      stopAll(nullptr);
      Serial.printf("逾時：5 秒只數到 %ld 格 → 輪子沒轉，或碼盤沒經過感測器小窗\n", done);
    }
  }

  if (mode == RUN && now - modeStart >= RUN_MS) {
    stopAll(nullptr);
    long dl = l - runStartL, dr = r - runStartR;
    float sec = RUN_MS / 1000.0f;
    Serial.printf("3 秒結果：左 %ld 格 = %.0f RPM   右 %ld 格 = %.0f RPM\n",
                  dl, dl * 60.0f / SLOTS / sec, dr, dr * 60.0f / SLOTS / sec);
    if (dl == 0 || dr == 0) Serial.println("  有一邊是 0 → 檢查那邊的接線或碼盤位置");
    else {
      float diff = 100.0f * fabs((float)(dl - dr)) / max(dl, dr);
      Serial.printf("  左右差 %.0f%%（10%% 以內正常，之後可用程式補償）\n", diff);
    }
  }

  if (watchLevel && now - lastPrint >= 50) {
    lastPrint = now;
    int a = digitalRead(ENC_L), b = digitalRead(ENC_R);
    Serial.printf("左 %d %s   右 %d %s   | 左 %ld 右 %ld\n", a, a ? "■" : "□", b, b ? "■" : "□", l, r);
  } else if (!watchLevel && mode == IDLE &&
             ((now - lastPrint >= 200 && (l != lastShownL || r != lastShownR)) ||
              (now - lastPrint >= 1000 && nl + nr != lastShownN))) {   // 格數沒變但一直在抖，每秒也報一次
    lastPrint = now; lastShownL = l; lastShownR = r; lastShownN = nl + nr;
    printCounts();
  }
}
