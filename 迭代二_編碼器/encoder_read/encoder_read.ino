/*
  編碼器讀取 + 走指定圈數 encoder_read.ino（v5：提前煞車＋自動學習）
  ------------------------------------------------------------
  接線（見 PINOUT_v2.md）
    HC-020K  VCC→3.3V  GND→GND  左 OUT→GPIO 34  右 OUT→GPIO 35
    L298N    IN1/IN2→GPIO 26/27（右輪）  IN3/IN4→GPIO 32/33（左輪）  ENA/ENB 跳帽插著
  序列埠：115200，結尾一定要選「Newline」（指令要整行送出）

  指令（打完按 Enter）
    g3     兩輪前進 3 圈就停（可以打小數，例如 g0.5、g2.25）
    g-2    兩輪後退 2 圈就停
    w / s  前進 / 後退 2 秒（自動停）
    x      馬上停止
    + / -  速度加減（PWM 60~255）
    z      格數歸零
    e      看目前的提前量；e2 = 兩輪提前量都設成 2 格；e0 = 關掉提前
    h      說明

  走圈數的做法
    - 每個輪子各自數格數，自己到了就自己煞車（左右都剛好走到指定圈數）
    - 剩最後幾格先減速，到了用 L298N 煞車（兩個 IN 都給高電位），減少衝過頭
    - 停穩後回報：目標幾格、實際幾格、多走或少走幾格
    - 1 格 = 1/20 圈，所以圈數的精準度是 0.05 圈

  提前煞車（v5）
    煞車後輪子還會滑一點（實測約 2 格），所以「還差 N 格」就先煞車，N 叫提前量。
    每次走完會看多走或少走幾格，自動微調提前量（每次修正一半，慢慢收斂）。
    左右輪各自學各自的。重開機會回到預設值 LEAD_INIT。
*/

// ---------- 腳位 ----------
const int ENC_L = 34, ENC_R = 35;
const int L_IN1 = 32, L_IN2 = 33;          // 左輪：L298N IN3 / IN4
const int R_IN1 = 26, R_IN2 = 27;          // 右輪：L298N IN1 / IN2
const bool L_INVERT = true, R_INVERT = true;

// ---------- 參數（調精準度主要改這裡） ----------
const int SLOTS = 20;                      // 碼盤格數
const unsigned long STABLE_US = 500;       // 新電位穩定 0.5 ms 才承認
const unsigned long MOVE_MS = 2000;        // w / s 跑多久
int speed = 160;                           // 一般速度
int slowPWM = 110;                         // 快到終點時的慢速（太低會轉不動）
int slowZone = 8;                          // 剩幾格開始減速（v4 是 4，太短來不及慢下來）
const float LEAD_INIT = 2.0;               // 提前量預設（v4 實測兩輪都衝過頭 2 格）
const float LEAD_MAX = 6.0;
const float LEARN = 0.5;                   // 每次修正誤差的一半
const long LEARN_MIN_TARGET = 10;          // 半圈以上才拿來學習（太短的距離還沒跑到穩定速度）
float leadL = LEAD_INIT, leadR = LEAD_INIT;
const unsigned long BRAKE_MS = 150;        // 煞車時間
const unsigned long STALL_MS = 1500;       // 這麼久都沒有新的一格 → 判定卡住

// ---------- 編碼器 ----------
struct Enc {
  int pin, state, last, dir;               // dir：+1 前進，-1 後退
  unsigned long since;
  long count;
};
Enc L = {ENC_L, 0, 0, 1, 0, 0}, R = {ENC_R, 0, 0, 1, 0, 0};

void update(Enc &e, unsigned long now) {
  int v = digitalRead(e.pin);
  if (v != e.last) { e.last = v; e.since = now; }
  else if (v != e.state && now - e.since >= STABLE_US) {
    e.state = v;
    if (v == 1) e.count += e.dir;          // 0→1 算一格
  }
}
void readBoth() { unsigned long n = micros(); update(L, n); update(R, n); }

// ---------- 馬達 ----------
void motor(int in1, int in2, int v) {
  v = constrain(v, -255, 255);
  if (v >= 0) { ledcWrite(in1, v); ledcWrite(in2, 0); }
  else        { ledcWrite(in1, 0); ledcWrite(in2, -v); }
}
void setLeft(int v)  { if (v) L.dir = v > 0 ? 1 : -1; motor(L_IN1, L_IN2, L_INVERT ? -v : v); }
void setRight(int v) { if (v) R.dir = v > 0 ? 1 : -1; motor(R_IN1, R_IN2, R_INVERT ? -v : v); }
void brakeLeft()  { ledcWrite(L_IN1, 255); ledcWrite(L_IN2, 255); }   // 兩腳都高 = 煞車
void brakeRight() { ledcWrite(R_IN1, 255); ledcWrite(R_IN2, 255); }
void allOff() { setLeft(0); setRight(0); }

// 等 ms 毫秒，期間持續讀編碼器（不能用 delay，不然會漏格）
void waitReading(unsigned long ms) { unsigned long t0 = millis(); while (millis() - t0 < ms) readBoth(); }

// ---------- 狀態 ----------
enum Mode { IDLE, TIMED, REVS };
Mode mode = IDLE;
unsigned long moveStart = 0;
long startL = 0, startR = 0;

// 走圈數用
long target = 0;                           // 目標格數（正數）
int goDir = 1;
bool doneL = false, doneR = false;
unsigned long lastTickL = 0, lastTickR = 0;
bool stalled = false;
long prevL = 0, prevR = 0;

void learn(float &lead, long err, const char *name) {
  float old = lead;
  lead = constrain(lead + LEARN * err, 0.0f, LEAD_MAX);
  if (fabs(lead - old) > 0.01f) Serial.printf("  %s輪提前量 %.1f → %.1f 格\n", name, old, lead);
}

void report(const char *why) {
  waitReading(400);                        // 等輪子停穩，慣性滑行也算進去
  allOff();
  long dl = L.count - startL, dr = R.count - startR;
  Serial.printf("%s：左 %ld 格 (%.2f 圈)，右 %ld 格 (%.2f 圈)\n",
                why, dl, (float)dl / SLOTS, dr, (float)dr / SLOTS);
  if (mode == REVS) {
    long want = goDir * target;
    long errL = goDir * (dl - want), errR = goDir * (dr - want);
    Serial.printf("  目標 %ld 格 → 左差 %+ld 格，右差 %+ld 格（+ 是衝過頭）\n", want, errL, errR);
    if (target >= LEARN_MIN_TARGET && !stalled) {
      learn(leadL, errL, "左");
      learn(leadR, errR, "右");
    }
  } else {
    long al = labs(dl), ar = labs(dr);
    if (al == 0 || ar == 0) Serial.println("  有一邊是 0 → 那邊的輪子沒轉，或編碼器沒讀到");
    else Serial.printf("  左右差 %.0f%%\n", 100.0f * labs(al - ar) / max(al, ar));
  }
  mode = IDLE;
}

void startTimed(int dir) {
  startL = L.count; startR = R.count;
  setLeft(dir * speed); setRight(dir * speed);
  mode = TIMED; moveStart = millis();
  Serial.printf("%s %lu 秒（PWM %d）\n", dir > 0 ? "前進" : "後退", MOVE_MS / 1000, speed);
}

void startRevs(float revs) {
  if (revs == 0) { Serial.println("圈數不能是 0"); return; }
  goDir = revs > 0 ? 1 : -1;
  target = lround(fabs(revs) * SLOTS);
  if (target < 1) target = 1;
  startL = prevL = L.count; startR = prevR = R.count;
  doneL = doneR = false; stalled = false;
  lastTickL = lastTickR = millis();
  mode = REVS; moveStart = millis();
  setLeft(goDir * speed); setRight(goDir * speed);
  Serial.printf("%s %.2f 圈 = %ld 格（PWM %d，最後 %d 格降到 %d，提前量 左%.1f 右%.1f）\n",
                goDir > 0 ? "前進" : "後退", fabs(revs), target, speed, slowZone, slowPWM, leadL, leadR);
}

// 每個輪子各自檢查：到了就煞車，快到了就減速
void runWheel(Enc &e, long start, bool &done, unsigned long &lastTick, long &prev, float lead,
              void (*setW)(int), void (*brakeW)()) {
  if (done) return;
  long moved = (e.count - start) * goDir;
  if (e.count != prev) { prev = e.count; lastTick = millis(); }
  long brakeAt = max(1L, target - lround(lead));          // 還差 lead 格就先煞車
  if (moved >= brakeAt) { brakeW(); done = true; return; }
  setW(goDir * ((target - moved <= slowZone) ? min(slowPWM, speed) : speed));
}

void stopNow(const char *why) {
  if (mode == IDLE) { allOff(); Serial.println("停止"); return; }
  brakeLeft(); brakeRight();
  waitReading(BRAKE_MS);
  report(why);
}

void help() {
  Serial.println();
  Serial.println("=== 編碼器 + 走指定圈數 ===");
  Serial.println("g3 前進3圈  g-2 後退2圈  g0.5 半圈");
  Serial.println("w 前進2秒  s 後退2秒  x 停止  +/- 速度  z 歸零  e 提前量  h 說明");
  Serial.println("建議先把車子架高，讓輪子懸空測試");
  Serial.printf("速度 PWM %d，慢速 PWM %d，最後 %d 格減速，提前量 左%.1f 右%.1f\n\n",
                speed, slowPWM, slowZone, leadL, leadR);
}

void handle(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;
  char c = cmd.charAt(0);
  switch (c) {
    case 'g': startRevs(cmd.substring(1).toFloat()); break;
    case 'w': startTimed(1); break;
    case 's': startTimed(-1); break;
    case 'x': stopNow("手動停止"); break;
    case '+': speed = min(255, speed + 10); Serial.printf("速度 PWM = %d\n", speed); break;
    case '-': speed = max(60, speed - 10);  Serial.printf("速度 PWM = %d\n", speed); break;
    case 'z': L.count = R.count = 0; Serial.println("格數歸零"); break;
    case 'e':
      if (cmd.length() > 1) { leadL = leadR = constrain(cmd.substring(1).toFloat(), 0.0f, LEAD_MAX); }
      Serial.printf("提前量：左 %.1f 格，右 %.1f 格\n", leadL, leadR);
      break;
    case 'h': help(); break;
    default: Serial.println("看不懂這個指令，打 h 看說明");
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  pinMode(ENC_L, INPUT);
  pinMode(ENC_R, INPUT);
  L.state = L.last = digitalRead(ENC_L); L.since = micros();
  R.state = R.last = digitalRead(ENC_R); R.since = micros();
  const int ins[] = {L_IN1, L_IN2, R_IN1, R_IN2};
  for (int p : ins) { ledcAttach(p, 1000, 8); ledcWrite(p, 0); }
  help();
}

void loop() {
  readBoth();

  // 讀一整行指令
  static String line;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') { handle(line); line = ""; }
    else if (line.length() < 20) line += c;
  }

  if (mode == TIMED && millis() - moveStart >= MOVE_MS) stopNow("時間到停止");

  if (mode == REVS) {
    runWheel(L, startL, doneL, lastTickL, prevL, leadL, setLeft, brakeLeft);
    runWheel(R, startR, doneR, lastTickR, prevR, leadR, setRight, brakeRight);
    if (doneL && doneR) { waitReading(BRAKE_MS); report("到達"); }
    else if ((!doneL && millis() - lastTickL > STALL_MS) || (!doneR && millis() - lastTickR > STALL_MS)) {
      brakeLeft(); brakeRight(); waitReading(BRAKE_MS);
      stalled = true;
      report(!doneL && millis() - lastTickL > STALL_MS ? "左輪卡住或沒讀到，停止" : "右輪卡住或沒讀到，停止");
    }
  }

  // 平常每 0.2 秒印格數（有變才印）
  static unsigned long lastPrint = 0;
  static long shownL = 0, shownR = 0;
  if (millis() - lastPrint >= 200 && (L.count != shownL || R.count != shownR)) {
    lastPrint = millis(); shownL = L.count; shownR = R.count;
    Serial.printf("左 %5ld 格 (%.2f 圈)   |   右 %5ld 格 (%.2f 圈)\n",
                  L.count, (float)L.count / SLOTS, R.count, (float)R.count / SLOTS);
  }
}
