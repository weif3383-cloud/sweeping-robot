/*
  編碼器測試（無線版）encoder_wifi.ino
  ------------------------------------------------------------
  功能跟 encoder_read v5 一樣（走指定圈數、提前煞車、自動學習），
  另外多了 WiFi：手機連上就能在網頁上按按鈕控制、看格數和回報。

  使用方式
    1. 燒錄（燒錄時插 USB，電池開關關掉）
    2. 拔 USB，開電池（ESP32 由 MP1584 供 5V）
    3. 手機 WiFi 連「RobotCar」，密碼 12345678
    4. 瀏覽器打開 http://192.168.4.1
    （插著 USB 時序列埠指令也還能用，115200，結尾選 Newline）

  接線（見 PINOUT_v2.md）
    HC-020K  VCC→3.3V  GND→GND  左 OUT→GPIO 34  右 OUT→GPIO 35
    L298N    IN1/IN2→GPIO 26/27（右輪）  IN3/IN4→GPIO 32/33（左輪）  ENA/ENB 跳帽插著

  左右同步（v2）
    走的時候每 20 ms 比一次左右格數：哪邊跑在前面就讓它慢一點、另一邊快一點
    （PI 控制：差幾格 × KP，再加上累積誤差 × KI，會自己學到「右輪本來就比較快」）。
    回報會多一行「最大左右差」，可以按「同步 開/關」比較效果。

  柔和停車（v3，預設開）
    起步：0.4 秒內從慢速漸漸加到設定速度（地上比較不會打滑）
    快到終點：最後 12 格速度跟著剩下的格數一路降，到了就放開（滑行），不急煞
    提前量一樣會自動學，學的是「放開後還會滑幾格」
    按「停車方式」可以切回急停比較；手動按「停止」永遠是急煞（安全）

  每圈格數（v4）
    20 格：只在「進缺口」那一下算一格（1 格 ≈ 1 cm）
    40 格：進缺口、出缺口都算（1 格 ≈ 0.5 cm，精度加倍）
    切換時格數會歸零，提前量自動換算。40 格需要碼盤缺口和實心部分差不多寬才會平均。

  路徑地圖（v5）
    用左右輪格數推算機器人位置（里程計）：兩輪走一樣多 = 直走，走不一樣多 = 轉彎。
    網頁上黑色地圖，白色是走過的地方（寬度 = 機身 24 cm），可以切成細線。
    這是「機器人以為自己在哪」，拿去跟地上實際位置比，就知道誤差多少。
    轉彎：t90 原地右轉 90°，t-90 左轉；q3 = 正方形測試（每邊 3 圈，走 4 邊回原點）
    校正：D65 = 輪子直徑 65 mm，W204 = 左右輪距 204 mm（量兩個輪胎中心的距離）
    m = 清除地圖、位置歸零

  跟 encoder_read 的差別
    編碼器改成用計時器中斷每 0.1 ms 讀一次（直接讀暫存器），
    這樣 WiFi 在忙的時候也不會漏格。
*/
#include <WiFi.h>
#include <WebServer.h>
#include <stdarg.h>
#include "soc/soc.h"
#include "soc/gpio_reg.h"

// ---------- WiFi ----------
const char *AP_SSID = "RobotCar";
const char *AP_PASS = "12345678";          // 至少 8 碼
WebServer server(80);

// ---------- 腳位 ----------
const int ENC_L = 34, ENC_R = 35;
const int L_IN1 = 32, L_IN2 = 33;          // 左輪：L298N IN3 / IN4
const int R_IN1 = 26, R_IN2 = 27;          // 右輪：L298N IN1 / IN2
const bool L_INVERT = true, R_INVERT = true;

// ---------- 編碼器資料結構 ----------
// （一定要放在所有函式前面：Arduino IDE 會在第一個函式前自動產生函式宣告，
//   如果那時候 Enc 還沒定義就會編譯失敗）
struct Enc {
  volatile long count;
  volatile int dir;                        // +1 前進，-1 後退
  volatile uint8_t state, run;             // 承認的電位、新電位連續出現幾次
};
Enc L = {0, 1, 0, 0}, R = {0, 1, 0, 0};

// ---------- 參數（調精準度主要改這裡） ----------
const int DISC_SLOTS = 20;                 // 碼盤缺口數
volatile bool bothEdges = false;           // false：一圈 20 格；true：一圈 40 格
int cpr() { return bothEdges ? 2 * DISC_SLOTS : DISC_SLOTS; }   // 一圈幾格
int sc(int n) { return n * cpr() / DISC_SLOTS; }                // 以「20 格」寫的參數換算成目前的格數
const int STABLE_SAMPLES = 5;              // 取樣 10 kHz：新電位連續 5 次（0.5 ms）才承認
const unsigned long MOVE_MS = 2000;        // w / s 跑多久
const float MAX_REVS = 20;                 // 一次最多走幾圈（安全）
int speed = 160;                           // 一般速度
int slowPWM = 110;                         // 快到終點時的慢速（太低會轉不動）
int slowZone = 8;                          // 急停模式：剩幾格開始減速
bool softStop = true;                      // 柔和停車（false = 舊的急停）
const int RAMP_ZONE = 12;                  // 柔和模式：最後幾格一路減速
const unsigned long START_RAMP_MS = 400;   // 起步加速時間
const unsigned long COAST_MS = 700;        // 放開後等多久再回報（滑行）
const float LEAD_INIT = 2.0;               // 提前量預設（實測兩輪都衝過頭 2 格）
const float LEAD_MAX = 6.0;
const float LEARN = 0.5;                   // 每次修正誤差的一半
const long LEARN_MIN_TARGET = 10;          // 半圈以上才拿來學習
// 提前量：柔和、急停各記一組（兩種停法滑行距離不同），單位是「目前模式的格」
float leadTab[2][2] = {{LEAD_INIT, LEAD_INIT}, {LEAD_INIT, LEAD_INIT}};   // [急停/柔和][左/右]
float &leadL() { return leadTab[softStop ? 1 : 0][0]; }
float &leadR() { return leadTab[softStop ? 1 : 0][1]; }
const unsigned long BRAKE_MS = 150;        // 煞車時間
const unsigned long STALL_MS = 1500;       // 這麼久都沒有新的一格 → 判定卡住

// ---------- 左右同步（走直線） ----------
bool syncOn = true;
float KP = 20;                             // 差 1 格 → 快的那邊少 20、慢的那邊多 20 PWM
float KI = 30;                             // 一直差 1 格持續 1 秒 → 再多修正 30 PWM
const int CORR_MAX = 70;                   // 修正量上限
const unsigned long CTRL_MS = 20;          // 每 20 ms 修正一次
float integ = 0;                           // 累積誤差（格·秒）
long maxErr = 0;                           // 這趟最大左右差（格）
unsigned long lastCtrl = 0;

// ---------- 路徑地圖（里程計） ----------
float WHEEL_MM = 65.0;                     // 輪子直徑（D 指令可改）
float TRACK_MM = 204.0;                    // 左右輪胎中心距離（W 指令可改）
const float ROBOT_MM = 240;                // 機身直徑（地圖上白色路徑的寬度）
float odoX = 0, odoY = 0, odoTh = 0;       // mm、mm、弧度（0 = 一開始的車頭方向，順時針為正）
long odoL = 0, odoR = 0;                   // 上次算到的格數
unsigned long lastOdo = 0;
const int PATHN = 1500;                    // 最多記幾個點（超過會丟掉最舊的）
int16_t pathX[PATHN], pathY[PATHN];
long pathSeq = 0;                          // 總共記過幾個點
int mapId = 0;                             // 每次清除地圖 +1，網頁看到變了就清畫面

// ---------- 編碼器（計時器中斷裡讀） ----------
hw_timer_t *timer = nullptr;

void IRAM_ATTR sample(Enc &e, uint8_t v) {             // 不加 inline：inline + IRAM 會連結失敗
  if (v == e.state) { e.run = 0; return; }
  if (++e.run >= STABLE_SAMPLES) {
    e.state = v; e.run = 0;
    if (v || bothEdges) e.count += e.dir;  // 20 格：只算 0→1；40 格：0→1、1→0 都算
  }
}
void IRAM_ATTR onTimer() {
  uint32_t in = REG_READ(GPIO_IN1_REG);    // GPIO 32~39 的電位（第 0 位 = GPIO32）
  sample(L, (in >> (ENC_L - 32)) & 1);
  sample(R, (in >> (ENC_R - 32)) & 1);
}

// ---------- 輸出：同時印到序列埠和網頁的紀錄 ----------
const int LOGN = 40;
String logs[LOGN];
long logSeq = 0;
void out(const char *fmt, ...) {
  char buf[200];
  va_list a; va_start(a, fmt); vsnprintf(buf, sizeof buf, fmt, a); va_end(a);
  Serial.println(buf);
  logs[logSeq % LOGN] = buf;
  logSeq++;
}

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

// ---------- 狀態 ----------
enum Mode { IDLE, TIMED, REVS };
Mode mode = IDLE;
unsigned long moveStart = 0;
long startL = 0, startR = 0;
long target = 0;                           // 目標格數（正數）
int goDir = 1;                             // 直走時的方向
int gdL = 1, gdR = 1;                      // 這趟左右輪各自的方向（轉彎時一正一負）
bool turning = false;
String cmdQ[12];                          // 連續動作（例如正方形測試）
int qHead = 0, qTail = 0;
unsigned long nextQ = 0;
bool doneL = false, doneR = false, stalled = false;
unsigned long lastTickL = 0, lastTickR = 0;
long prevL = 0, prevR = 0;
// 停車後要等輪子停穩才回報：不用 delay 卡住，WiFi 照常回應
unsigned long settleUntil = 0;
const char *settleWhy = nullptr;

void learn(float &lead, long err, const char *name) {
  float old = lead;
  lead = constrain(lead + LEARN * err, 0.0f, LEAD_MAX * cpr() / DISC_SLOTS);
  if (fabs(lead - old) > 0.01f) out("  %s輪提前量 %.1f → %.1f 格", name, old, lead);
}

void report(const char *why) {
  allOff();
  long dl = L.count - startL, dr = R.count - startR;
  out("%s：左 %ld 格 (%.2f 圈)，右 %ld 格 (%.2f 圈)", why, dl, (float)dl / cpr(), dr, (float)dr / cpr());
  out("  最大左右差 %ld 格（同步%s）", maxErr, syncOn ? "開" : "關");
  if (mode == REVS) {
    long errL = gdL * dl - target, errR = gdR * dr - target;
    out("  目標 %ld 格 → 左差 %+ld 格，右差 %+ld 格（+ 是衝過頭）", target, errL, errR);
    if (!turning && target >= sc(LEARN_MIN_TARGET) && !stalled) {   // 轉彎不拿來學
      learn(leadL(), errL, "左");
      learn(leadR(), errR, "右");
    }
  } else {
    long al = labs(dl), ar = labs(dr);
    if (al == 0 || ar == 0) out("  有一邊是 0 → 那邊的輪子沒轉，或編碼器沒讀到");
    else out("  左右差 %.0f%%", 100.0f * labs(al - ar) / max(al, ar));
  }
  out("  地圖位置：x %.1f cm，y %.1f cm，方向 %.0f°", odoX / 10, odoY / 10, odoTh * 180 / PI);
  mode = IDLE;
  turning = false;
}

// 停車 → 等輪子停穩 → 回報
//   brake = true ：急煞，等 0.55 秒
//   brake = false：放開滑行，等 0.7 秒
void beginSettle(const char *why, bool brake) {
  if (brake) { brakeLeft(); brakeRight(); } else allOff();
  settleWhy = why;
  settleUntil = millis() + (brake ? BRAKE_MS + 400 : COAST_MS);
}

bool busy() { return mode != IDLE || settleWhy != nullptr; }

void startTimed(int dir) {
  if (busy()) { out("還在跑，先按停止"); return; }
  startL = L.count; startR = R.count;
  goDir = gdL = gdR = dir; turning = false; integ = 0; maxErr = 0; lastCtrl = millis();
  int s0 = softStop ? min(slowPWM, speed) : speed;
  setLeft(dir * s0); setRight(dir * s0);
  mode = TIMED; moveStart = millis();
  out("%s %lu 秒（PWM %d）", dir > 0 ? "前進" : "後退", MOVE_MS / 1000, speed);
}

// 兩輪各走 tgt 格，方向 dl、dr（+1 / -1）
void startTarget(long tgt, int dl, int dr) {
  target = max(1L, tgt);
  gdL = dl; gdR = dr; goDir = dl;
  turning = (dl != dr);
  startL = prevL = L.count; startR = prevR = R.count;
  doneL = doneR = false; stalled = false;
  integ = 0; maxErr = 0; lastCtrl = millis();
  lastTickL = lastTickR = millis();
  mode = REVS; moveStart = millis();
  int s0 = softStop ? min(slowPWM, speed) : speed;
  setLeft(gdL * s0); setRight(gdR * s0);
}

void startRevs(float revs) {
  if (busy()) { out("還在跑，先按停止"); return; }
  if (revs == 0) { out("圈數不能是 0"); return; }
  revs = constrain(revs, -MAX_REVS, MAX_REVS);
  int d = revs > 0 ? 1 : -1;
  startTarget(lround(fabs(revs) * cpr()), d, d);
  out("%s %.2f 圈 = %ld 格（PWM %d，%s，提前量 左%.1f 右%.1f）",
      d > 0 ? "前進" : "後退", fabs(revs), target, speed, softStop ? "柔和停車" : "急停", leadL(), leadR());
}

// 原地轉 deg 度（正 = 右轉／順時針）：每個輪子要走的弧長 = 角度 × 輪距 / 2
void startTurn(float deg) {
  if (busy()) { out("還在跑，先按停止"); return; }
  if (deg == 0) { out("角度不能是 0"); return; }
  deg = constrain(deg, -720.0f, 720.0f);
  float arc = fabs(deg) * PI / 180 * TRACK_MM / 2;
  float mmPer = PI * WHEEL_MM / cpr();
  int d = deg > 0 ? 1 : -1;
  startTarget(lround(arc / mmPer), d, -d);   // 右轉：左輪前進、右輪後退
  out("原地%s轉 %.0f° = 每輪 %ld 格", d > 0 ? "右" : "左", fabs(deg), target);
}

// 每個輪子各自檢查：到了就停（急煞或放開）；回傳這個輪子的基本速度
int checkWheel(Enc &e, long start, int wd, bool &done, unsigned long &lastTick, long &prev, float lead,
               void (*brakeW)(), void (*setW)(int)) {
  if (done) return 0;
  long c = e.count;
  long moved = (c - start) * wd;
  if (c != prev) { prev = c; lastTick = millis(); }
  long brakeAt = max(1L, target - lround(lead));          // 還差 lead 格就先停
  if (moved >= brakeAt) {
    if (softStop) setW(0); else brakeW();                 // 柔和：放開滑行；急停：煞車
    done = true; return 0;
  }
  long remain = target - moved;
  int lo = min(slowPWM, speed);
  if (softStop) {
    long zone = sc(RAMP_ZONE);
    if (remain >= zone) return speed;
    return lo + (speed - lo) * remain / zone;             // 剩越少越慢，一路降到慢速
  }
  return (remain <= sc(slowZone)) ? lo : speed;
}

// 起步加速：前 0.4 秒從慢速漸漸加到目標速度
int softStart(int base) {
  if (!softStop || base <= 0) return base;
  unsigned long t = millis() - moveStart;
  if (t >= START_RAMP_MS) return base;
  int lo = min(slowPWM, base);
  return lo + (base - lo) * (long)t / START_RAMP_MS;
}

// 左右同步：算出修正量，快的那邊減、慢的那邊加
void driveSynced(int baseL, int baseR) {
  unsigned long now = millis();
  if (now - lastCtrl < CTRL_MS) return;
  float dt = (now - lastCtrl) / 1000.0f;
  lastCtrl = now;
  long err = (R.count - startR) * gdR - (L.count - startL) * gdL;     // 正 = 右輪跑在前面（走得比較多）
  if (labs(err) > maxErr) maxErr = labs(err);
  int corr = 0;
  bool both = baseL > 0 && baseR > 0;                     // 兩輪都還在跑才同步
  if (syncOn && both) {
    float k = (float)DISC_SLOTS / cpr();                  // 40 格模式一格只有一半長，增益減半
    integ = constrain(integ + err * k * dt, -3.0f, 3.0f);
    corr = constrain((int)lroundf(KP * err * k + KI * integ), -CORR_MAX, CORR_MAX);
  }
  baseL = softStart(baseL); baseR = softStart(baseR);
  if (baseL > 0) setLeft(gdL * constrain(baseL + corr, 0, 255));
  if (baseR > 0) setRight(gdR * constrain(baseR - corr, 0, 255));
}

void stopNow() {
  if (qHead != qTail) { qHead = qTail = 0; out("連續動作已取消"); }
  if (!busy()) { allOff(); out("停止"); return; }
  if (settleWhy) return;                   // 已經在煞車了
  beginSettle("手動停止", true);
}

// ---------- 里程計 ----------
void addPoint() {
  int i = pathSeq % PATHN;
  pathX[i] = (int16_t)constrain(lroundf(odoX), -32000L, 32000L);
  pathY[i] = (int16_t)constrain(lroundf(odoY), -32000L, 32000L);
  pathSeq++;
}

void resetMap() {
  odoX = odoY = odoTh = 0;
  odoL = L.count; odoR = R.count;
  pathSeq = 0; mapId++;
  addPoint();
}

// 每 20 ms：用這段時間左右輪各走了幾格，推算位置
void updateOdo() {
  unsigned long now = millis();
  if (now - lastOdo < 20) return;
  lastOdo = now;
  long l = L.count, r = R.count;
  long dlc = l - odoL, drc = r - odoR;
  if (dlc == 0 && drc == 0) return;
  odoL = l; odoR = r;
  float mmPer = PI * WHEEL_MM / cpr();
  float dL = dlc * mmPer, dR = drc * mmPer;
  float ds = (dL + dR) / 2;                // 中心走了多遠
  float dth = (dL - dR) / TRACK_MM;        // 轉了多少（左輪走比較多 = 右轉 = 順時針）
  float mid = odoTh + dth / 2;
  odoX += ds * sinf(mid);                  // x 往右
  odoY += ds * cosf(mid);                  // y 往車頭一開始的方向
  odoTh += dth;
  if (odoTh > PI) odoTh -= 2 * PI;
  if (odoTh < -PI) odoTh += 2 * PI;
  int last = (pathSeq - 1) % PATHN;
  float mx = odoX - pathX[last], my = odoY - pathY[last];
  if (mx * mx + my * my >= 15 * 15) addPoint();   // 每移動 1.5 cm 記一個點
}

// ---------- 連續動作 ----------
void enqueue(const String &c) {
  int n = (qTail + 1) % 12;
  if (n == qHead) return;
  cmdQ[qTail] = c; qTail = n;
}

void help() {
  out("=== 編碼器 + 走指定圈數（無線版）===");
  out("g3 前進3圈  g-2 後退2圈  w/s 前進/後退2秒  x 停止  +/- 速度  z 歸零  e2 提前量  k 同步  b 停車方式  c20/c40 每圈格數");
  out("t90 右轉90°  t-90 左轉  q3 正方形測試  m 清地圖  D65 輪徑  W204 輪距");
  out("一圈 %d 格，%s，同步%s，速度 PWM %d，慢速 PWM %d，提前量 左%.1f 右%.1f",
      cpr(), softStop ? "柔和停車" : "急停", syncOn ? "開" : "關", speed, slowPWM, leadL(), leadR());
}

void handle(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;
  char c = cmd.charAt(0);
  switch (c) {
    case 'g': startRevs(cmd.substring(1).toFloat()); break;
    case 'w': startTimed(1); break;
    case 's': startTimed(-1); break;
    case 'x': stopNow(); break;
    case '+': speed = min(255, speed + 10); out("速度 PWM = %d", speed); break;
    case '-': speed = max(60, speed - 10);  out("速度 PWM = %d", speed); break;
    case 'z':
      if (busy()) { out("跑的時候不能歸零"); break; }
      L.count = R.count = 0; odoL = odoR = 0; out("格數歸零（地圖位置不變）"); break;
    case 'e':
      if (cmd.length() > 1) leadL() = leadR() = constrain(cmd.substring(1).toFloat(), 0.0f, LEAD_MAX * cpr() / DISC_SLOTS);
      out("提前量：左 %.1f 格，右 %.1f 格", leadL(), leadR());
      break;
    case 'c': {
      if (busy()) { out("跑的時候不能切換"); break; }
      bool want = (cmd.length() > 1) ? (cmd.substring(1).toInt() == 40) : !bothEdges;
      if (want != bothEdges) {
        float f = want ? 2.0f : 0.5f;                     // 提前量跟著換算
        for (int m = 0; m < 2; m++) for (int w = 0; w < 2; w++) leadTab[m][w] = constrain(leadTab[m][w] * f, 0.0f, LEAD_MAX * 2);
        bothEdges = want;
      }
      L.count = R.count = 0; odoL = odoR = 0;
      out("每圈 %d 格（1 格約 %.1f cm），格數歸零，提前量 左%.1f 右%.1f", cpr(), 20.4f / cpr(), leadL(), leadR());
    } break;
    case 'b':
      if (busy()) { out("跑的時候不能切換"); break; }
      if (cmd.length() > 1) softStop = cmd.charAt(1) != '0'; else softStop = !softStop;
      out("停車方式：%s，提前量 左%.1f 右%.1f（兩種停法各記各的）", softStop ? "柔和（減速後滑行）" : "急停（煞車）", leadL(), leadR());
      break;
    case 'k':
      if (cmd.length() > 1) syncOn = cmd.charAt(1) != '0'; else syncOn = !syncOn;
      out("左右同步：%s（KP %.0f，KI %.0f）", syncOn ? "開" : "關", KP, KI);
      break;
    case 't': startTurn(cmd.substring(1).toFloat()); break;
    case 'm':
      if (busy()) { out("跑的時候不能清地圖"); break; }
      resetMap(); out("地圖清除，位置歸零（車頭方向 = 地圖上方）"); break;
    case 'q': {
      if (busy()) { out("還在跑，先按停止"); break; }
      float side = cmd.length() > 1 ? cmd.substring(1).toFloat() : 3;
      if (side <= 0) side = 3;
      for (int i = 0; i < 4; i++) { enqueue("g" + String(side, 2)); enqueue("t90"); }
      out("正方形測試：每邊 %.2f 圈（約 %.0f cm），右轉 4 次，應該回到起點", side, side * PI * WHEEL_MM / 10);
    } break;
    case 'D':
      if (cmd.length() > 1) WHEEL_MM = constrain(cmd.substring(1).toFloat(), 30.0f, 120.0f);
      out("輪子直徑 %.1f mm（一圈 %.1f cm）", WHEEL_MM, PI * WHEEL_MM / 10); break;
    case 'W':
      if (cmd.length() > 1) TRACK_MM = constrain(cmd.substring(1).toFloat(), 80.0f, 400.0f);
      out("輪距 %.1f mm", TRACK_MM); break;
    case 'h': help(); break;
    default: out("看不懂這個指令：%s", cmd.c_str());
  }
}

// ---------- 網頁 ----------
const char PAGE[] PROGMEM = R"HTML(<!doctype html>
<html lang="zh-Hant"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1">
<title>編碼器測試</title>
<style>
:root{--bg:#f4f5f2;--card:#fff;--ink:#1d2321;--mute:#6b736f;--acc:#1f6f78;--warn:#c0392b;--line:#dfe3df}
*{box-sizing:border-box}body{margin:0;font-family:system-ui,-apple-system,"Noto Sans TC",sans-serif;background:var(--bg);color:var(--ink)}
main{max-width:520px;margin:0 auto;padding:12px 14px 30px}
h1{font-size:18px;margin:4px 0 10px;display:flex;align-items:center;gap:8px}
#dot{width:10px;height:10px;border-radius:50%;background:#bbb}#dot.on{background:#2e9d5b}
.card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:12px;margin-bottom:10px}
.cnt{display:grid;grid-template-columns:1fr 1fr;gap:10px;text-align:center}
.cnt b{display:block;font-size:34px;font-variant-numeric:tabular-nums}.cnt small{color:var(--mute)}
#st{font-size:13px;color:var(--mute);margin-top:8px;text-align:center}
.lbl{font-size:13px;color:var(--mute);margin:0 0 6px}
.row{display:grid;grid-template-columns:repeat(5,1fr);gap:6px;margin-bottom:6px}
button{font:inherit;font-size:15px;padding:11px 0;border-radius:9px;border:1px solid var(--line);background:#fff;color:var(--ink);touch-action:manipulation}
button:active{transform:scale(.97)}
.fw{background:#e7f1f2;border-color:#bcd7da}
.opt{display:flex;align-items:center;gap:10px;margin-top:8px}.opt span{flex:1;font-size:15px}
.seg{display:grid;grid-template-columns:1fr 1fr;width:170px;border:1px solid var(--line);border-radius:9px;overflow:hidden}
.seg button{border:0;border-radius:0;padding:10px 0;background:#fff;color:var(--mute)}.seg button.on{background:var(--acc);color:#fff;font-weight:600}.bw{background:#f6ede4;border-color:#e3cdb6}
#stop{width:100%;background:var(--warn);color:#fff;border:0;font-size:20px;font-weight:700;padding:16px 0;margin-bottom:10px}
.pair{display:grid;grid-template-columns:1fr 1fr;gap:6px}
.inl{display:flex;gap:6px;align-items:center}.inl input{flex:1;min-width:0;font:inherit;font-size:16px;padding:10px;border:1px solid var(--line);border-radius:9px}
.inl button{padding:11px 14px}
#sp{min-width:56px;text-align:center;font-size:18px;font-weight:600}
#map{display:block;width:100%;height:360px;background:#000;border-radius:9px;touch-action:none}
.pose{display:flex;justify-content:space-between;font-size:13px;color:var(--mute);margin:6px 2px 8px;font-variant-numeric:tabular-nums}
.row4{display:grid;grid-template-columns:repeat(4,1fr);gap:6px;margin-bottom:6px}
#log{height:220px;overflow:auto;font:12.5px/1.45 ui-monospace,Menlo,Consolas,monospace;white-space:pre-wrap;background:#1d2321;color:#d9e4df;border-radius:9px;padding:8px}
</style></head><body><main>
<h1><span id="dot"></span>編碼器測試</h1>
<div class="card"><div class="cnt">
 <div><small>左輪</small><b id="l">0</b><small id="lr">0.00 圈</small></div>
 <div><small>右輪</small><b id="r">0</b><small id="rr">0.00 圈</small></div>
</div><div id="st">連線中…</div></div>

<button id="stop" onclick="send('x')">停止</button>

<div class="card">
 <p class="lbl">前進幾圈</p>
 <div class="row"><button class="fw" onclick="send('g0.5')">0.5</button><button class="fw" onclick="send('g1')">1</button><button class="fw" onclick="send('g2')">2</button><button class="fw" onclick="send('g3')">3</button><button class="fw" onclick="send('g5')">5</button></div>
 <p class="lbl">後退幾圈</p>
 <div class="row"><button class="bw" onclick="send('g-0.5')">0.5</button><button class="bw" onclick="send('g-1')">1</button><button class="bw" onclick="send('g-2')">2</button><button class="bw" onclick="send('g-3')">3</button><button class="bw" onclick="send('g-5')">5</button></div>
 <div class="inl" style="margin-top:4px"><input id="rev" type="number" step="0.05" placeholder="自訂圈數，負的是後退"><button onclick="go()">走</button></div>
</div>

<div class="card">
 <p class="lbl">路徑地圖（白色 = 走過的地方，綠圈 = 機器人，上方 = 一開始的車頭方向）</p>
 <canvas id="map"></canvas>
 <div class="pose"><span id="pxy">x 0 cm，y 0 cm</span><span id="pth">方向 0°</span><span id="psc"></span></div>
 <div class="row4"><button class="bw" onclick="send('t-90')">左轉 90°</button><button class="bw" onclick="send('t-45')">左轉 45°</button><button class="fw" onclick="send('t45')">右轉 45°</button><button class="fw" onclick="send('t90')">右轉 90°</button></div>
 <div class="pair"><button onclick="sq()">正方形測試</button><button onclick="send('m')">清除地圖</button></div>
 <div class="opt"><span>路徑顯示</span><div class="seg"><button id="vw" onclick="view(1)">機身寬</button><button id="vl" onclick="view(0)">細線</button></div></div>
</div>

<div class="card">
 <p class="lbl">計時（2 秒自動停）</p>
 <div class="pair"><button class="fw" onclick="send('w')">前進 2 秒</button><button class="bw" onclick="send('s')">後退 2 秒</button></div>
</div>

<div class="card">
 <p class="lbl">速度（PWM）</p>
 <div class="inl"><button onclick="send('-')">−</button><span id="sp">160</span><button onclick="send('+')">＋</button><span style="flex:1"></span><button onclick="send('z')">格數歸零</button></div>
 <p class="lbl" style="margin-top:10px">提前量（煞車提早幾格）：<span id="ld">-</span></p>
 <div class="inl"><input id="lead" type="number" step="0.5" min="0" max="6" placeholder="例如 2"><button onclick="setLead()">設定</button></div>
</div>

<div class="card">
 <p class="lbl">模式設定（跑的時候不能切換）</p>
 <div class="opt"><span>每圈格數</span><div class="seg"><button id="c20" onclick="send('c20')">20 格</button><button id="c40" onclick="send('c40')">40 格</button></div></div>
 <div class="opt"><span>停車方式</span><div class="seg"><button id="b1" onclick="send('b1')">柔和</button><button id="b0" onclick="send('b0')">急停</button></div></div>
 <div class="opt"><span>左右同步</span><div class="seg"><button id="k1" onclick="send('k1')">開</button><button id="k0" onclick="send('k0')">關</button></div></div>
</div>

<div class="card">
 <p class="lbl">自己打指令（例如 t180、q2、D64.5、W200）</p>
 <div class="inl"><input id="cmd" type="text" autocapitalize="off" autocomplete="off" placeholder="指令"><button onclick="const v=$('cmd').value.trim();if(v){send(v);$('cmd').value='';}">送出</button></div>
</div>

<div class="card"><p class="lbl">回報</p><div id="log"></div></div>
</main>
<script>
let seq=0, fails=0, mid=-1, ps=0, pts=[], pose={x:0,y:0,th:0}, wide=1, robotW=240;
function sq(){const v=prompt('正方形每邊幾圈？（3 圈約 61 cm）','3');if(v)send('q'+parseFloat(v));}
function view(v){wide=v;$('vw').classList.toggle('on',!!v);$('vl').classList.toggle('on',!v);draw();}
function draw(){
 const c=$('map'),r=c.getBoundingClientRect(),dpr=window.devicePixelRatio||1;
 c.width=r.width*dpr;c.height=r.height*dpr;const g=c.getContext('2d');g.setTransform(dpr,0,0,dpr,0,0);
 const W=r.width,H=r.height;g.fillStyle='#000';g.fillRect(0,0,W,H);
 // 找範圍：包含所有路徑點和機器人，至少 1.2 m × 1.2 m
 let x0=pose.x,x1=pose.x,y0=pose.y,y1=pose.y;
 for(const p of pts){x0=Math.min(x0,p[0]);x1=Math.max(x1,p[0]);y0=Math.min(y0,p[1]);y1=Math.max(y1,p[1]);}
 const pad=robotW,cx=(x0+x1)/2,cy=(y0+y1)/2;
 const spanX=Math.max(x1-x0+2*pad,1200),spanY=Math.max(y1-y0+2*pad,1200);
 const s=Math.min(W/spanX,H/spanY);                       // 1 mm = s px
 const X=x=>W/2+(x-cx)*s, Y=y=>H/2-(y-cy)*s;
 // 很淡的格線：每 50 cm
 g.strokeStyle='#161616';g.lineWidth=1;
 const gs=500,gx0=Math.floor((cx-W/2/s)/gs)*gs,gy0=Math.floor((cy-H/2/s)/gs)*gs;
 for(let x=gx0;x<cx+W/2/s;x+=gs){g.beginPath();g.moveTo(X(x),0);g.lineTo(X(x),H);g.stroke();}
 for(let y=gy0;y<cy+H/2/s;y+=gs){g.beginPath();g.moveTo(0,Y(y));g.lineTo(W,Y(y));g.stroke();}
 // 走過的路徑
 if(pts.length){
  g.strokeStyle='#fff';g.fillStyle='#fff';g.lineCap='round';g.lineJoin='round';
  g.lineWidth=wide?Math.max(2,robotW*s):2;
  g.beginPath();g.moveTo(X(pts[0][0]),Y(pts[0][1]));for(const p of pts)g.lineTo(X(p[0]),Y(p[1]));g.lineTo(X(pose.x),Y(pose.y));g.stroke();
  if(wide){g.beginPath();g.arc(X(pts[0][0]),Y(pts[0][1]),robotW*s/2,0,7);g.fill();}
  // 起點標記
  g.fillStyle='#e0a030';g.beginPath();g.arc(X(pts[0][0]),Y(pts[0][1]),4,0,7);g.fill();
 }
 // 機器人：綠色圓圈 + 車頭方向
 const a=pose.th*Math.PI/180,rx=X(pose.x),ry=Y(pose.y),rr=Math.max(6,robotW*s/2);
 g.strokeStyle='#3ddc84';g.lineWidth=2;g.beginPath();g.arc(rx,ry,rr,0,7);g.stroke();
 g.beginPath();g.moveTo(rx,ry);g.lineTo(rx+Math.sin(a)*rr*1.3,ry-Math.cos(a)*rr*1.3);g.stroke();
 $('psc').textContent='格線 50 cm';
}
window.addEventListener('resize',draw);
const $=id=>document.getElementById(id);
function send(c){fetch('/cmd?c='+encodeURIComponent(c)).then(()=>poll()).catch(()=>{});}
function go(){const v=parseFloat($('rev').value);if(!isNaN(v)&&v!==0)send('g'+v);}
function setLead(){const v=parseFloat($('lead').value);if(!isNaN(v))send('e'+v);}
function poll(){
 fetch('/status?since='+seq+'&ps='+ps+'&mid='+mid).then(r=>r.json()).then(d=>{
  fails=0;$('dot').className='on';
  $('l').textContent=d.l;$('r').textContent=d.r;
  $('lr').textContent=(d.l/d.c).toFixed(2)+' 圈';$('rr').textContent=(d.r/d.c).toFixed(2)+' 圈';
  $('sp').textContent=d.sp;$('ld').textContent='左 '+d.la.toFixed(1)+' ／ 右 '+d.lb.toFixed(1)+' 格';
  const on=(id,v)=>$(id).classList.toggle('on',!!v);
  on('c20',d.c==20);on('c40',d.c==40);on('b1',d.b);on('b0',!d.b);on('k1',d.k);on('k0',!d.k);
  $('st').textContent={IDLE:'待命',TIMED:'計時行駛中',REVS:'走圈數中',SETTLE:'煞車中'}[d.m]||d.m;
  if(d.log.length){const L=$('log');L.textContent+=d.log.join('\n')+'\n';L.scrollTop=L.scrollHeight;}
  seq=d.seq;
  if(d.mid!==mid){mid=d.mid;pts=[];}
  for(let i=0;i<d.p.length;i+=2)pts.push([d.p[i],d.p[i+1]]);
  if(pts.length>3000)pts=pts.slice(-3000);
  ps=d.pe;robotW=d.wd;pose={x:d.x,y:d.y,th:d.th};
  $('pxy').textContent='x '+(d.x/10).toFixed(1)+' cm，y '+(d.y/10).toFixed(1)+' cm';
  $('pth').textContent='方向 '+Math.round(d.th)+'°';
  draw();
 }).catch(()=>{if(++fails>2){$('dot').className='';$('st').textContent='連線中斷，確認手機還連著 RobotCar';}});
}
view(1);setInterval(poll,300);poll();
</script></body></html>)HTML";

String jsonEsc(const String &s) {
  String o; o.reserve(s.length() + 8);
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '"' || c == '\\') { o += '\\'; o += c; }
    else if (c == '\n') o += "\\n";
    else o += c;
  }
  return o;
}

void handleStatus() {
  long since = server.hasArg("since") ? server.arg("since").toInt() : 0;
  if (since < logSeq - LOGN) since = logSeq - LOGN;
  if (since < 0) since = 0;
  const char *m = settleWhy ? "SETTLE" : (mode == TIMED ? "TIMED" : mode == REVS ? "REVS" : "IDLE");
  String j = "{\"l\":" + String(L.count) + ",\"r\":" + String(R.count) +
             ",\"m\":\"" + m + "\",\"sp\":" + String(speed) +
             ",\"la\":" + String(leadL(), 1) + ",\"lb\":" + String(leadR(), 1) + ",\"k\":" + String(syncOn ? 1 : 0) + ",\"b\":" + String(softStop ? 1 : 0) + ",\"c\":" + String(cpr()) +
             ",\"seq\":" + String(logSeq) + ",\"log\":[";
  for (long i = since; i < logSeq; i++) {
    if (i > since) j += ',';
    j += '"'; j += jsonEsc(logs[i % LOGN]); j += '"';
  }
  j += "]";
  // 地圖：位置 + 新的路徑點
  long ps = server.hasArg("ps") ? server.arg("ps").toInt() : 0;
  int mid = server.hasArg("mid") ? server.arg("mid").toInt() : -1;
  if (mid != mapId) ps = 0;
  if (ps < pathSeq - PATHN) ps = pathSeq - PATHN;
  if (ps < 0) ps = 0;
  long pe = min(pathSeq, ps + 300);        // 一次最多送 300 點
  j += ",\"mid\":" + String(mapId) + ",\"pe\":" + String(pe) +
       ",\"x\":" + String(odoX, 0) + ",\"y\":" + String(odoY, 0) + ",\"th\":" + String(odoTh * 180 / PI, 1) +
       ",\"wd\":" + String(ROBOT_MM, 0) + ",\"p\":[";
  for (long i = ps; i < pe; i++) {
    if (i > ps) j += ',';
    j += String(pathX[i % PATHN]); j += ','; j += String(pathY[i % PATHN]);
  }
  j += "]}";
  server.send(200, "application/json", j);
}

void setup() {
  Serial.begin(115200);
  delay(300);
  pinMode(ENC_L, INPUT);
  pinMode(ENC_R, INPUT);
  L.state = digitalRead(ENC_L);
  R.state = digitalRead(ENC_R);
  const int ins[] = {L_IN1, L_IN2, R_IN1, R_IN2};
  for (int p : ins) { ledcAttach(p, 1000, 8); ledcWrite(p, 0); }

  timer = timerBegin(1000000);              // 1 MHz
  timerAttachInterrupt(timer, &onTimer);
  timerAlarm(timer, 100, true, 0);          // 每 100 µs 讀一次編碼器

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  server.on("/", []() { server.send(200, "text/html; charset=utf-8", PAGE); });
  server.on("/cmd", []() { handle(server.arg("c")); server.send(200, "text/plain", "ok"); });
  server.on("/status", handleStatus);
  server.begin();

  resetMap();
  help();
  out("WiFi：%s / %s，網址 http://%s", AP_SSID, AP_PASS, WiFi.softAPIP().toString().c_str());
}

void loop() {
  server.handleClient();

  // 序列埠指令（插 USB 時也能用）
  static String line;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') { handle(line); line = ""; }
    else if (line.length() < 20) line += c;
  }

  if (mode == TIMED && !settleWhy) {
    if (millis() - moveStart >= MOVE_MS) beginSettle("時間到停止", !softStop);
    else driveSynced(speed, speed);
  }

  if (mode == REVS && !settleWhy) {
    int bL = checkWheel(L, startL, gdL, doneL, lastTickL, prevL, leadL(), brakeLeft, setLeft);
    int bR = checkWheel(R, startR, gdR, doneR, lastTickR, prevR, leadR(), brakeRight, setRight);
    driveSynced(bL, bR);
    if (doneL && doneR) beginSettle("到達", !softStop);
    else if ((!doneL && millis() - lastTickL > STALL_MS) || (!doneR && millis() - lastTickR > STALL_MS)) {
      stalled = true;
      beginSettle(!doneL && millis() - lastTickL > STALL_MS ? "左輪卡住或沒讀到，停止" : "右輪卡住或沒讀到，停止", true);
    }
  }

  updateOdo();

  if (settleWhy && (long)(millis() - settleUntil) >= 0) {
    const char *why = settleWhy;
    settleWhy = nullptr;
    updateOdo();
    report(why);
    nextQ = millis() + 300;                // 連續動作之間停 0.3 秒
  }

  // 連續動作：上一個做完就做下一個
  if (!busy() && qHead != qTail && (long)(millis() - nextQ) >= 0) {
    String c = cmdQ[qHead]; qHead = (qHead + 1) % 12;
    handle(c);
  }

  // 序列埠每 0.2 秒印格數（有變才印；不放進網頁紀錄，網頁上方會即時顯示）
  static unsigned long lastPrint = 0;
  static long shownL = 0, shownR = 0;
  long l = L.count, r = R.count;
  if (millis() - lastPrint >= 200 && (l != shownL || r != shownR)) {
    lastPrint = millis(); shownL = l; shownR = r;
    Serial.printf("左 %5ld 格 (%.2f 圈)   |   右 %5ld 格 (%.2f 圈)\n", l, (float)l / cpr(), r, (float)r / cpr());
  }
}
