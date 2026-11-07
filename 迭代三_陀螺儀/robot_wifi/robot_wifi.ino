/*
  掃地機器人主程式 robot_wifi.ino
  ------------------------------------------------------------
  從 encoder_wifi v5 延伸：走圈數、原地轉、左右同步、柔和停車、路徑地圖，全部保留，另外加上：

  ① MPU6050 陀螺儀（Z 軸）
    - 開機靜止 1 秒量零點；之後只要車子停著（編碼器 1 秒沒動）就自動重新量零點，角度不會一直漂
    - 停著而且幾乎沒在轉時，不累加角度（再擋一層漂移）
    - 正負號：第一次原地轉彎會自動判斷，方向反了就自動翻過來並存起來（也可以按「反轉方向」）
    - 讀不到 MPU 時自動改回只用編碼器，網頁會顯示

  ② 地圖方向改用陀螺儀
    - 走多遠：編碼器；車頭方向：陀螺儀（打滑也不影響）
    - 網頁同時顯示「陀螺儀方向」和「編碼器推算方向」，可以比較；H0 / H1 切換地圖用哪個

  ③ 轉彎用陀螺儀決定停在哪（v2）
    t90：一直轉到陀螺儀說轉了 90° 才停；最後 35° 漸漸減速
    提前量（度）會自動學：放開後還會多轉幾度，左轉右轉各記一組，存在 ESP32 裡
    還沒確認過陀螺儀正負號時，第一次轉彎先用編碼器（確認方向後才換陀螺儀）
    轉的方向跟陀螺儀相反時會立刻翻正負號，不會一直轉

  ③-2 提前量跟著轉速走（v3）
    放開後多轉的角度 ≈ 放開那一刻的轉速 × 時間常數 K。學的是 K（秒），
    所以轉 45°、90°、180° 都準（v2 學固定度數，45° 少轉、180° 多轉）

  ④ 直走鎖方向（v3）
    出發時記下陀螺儀方向，途中歪了就修：歪 1° → 兩輪差 KH PWM，再加上轉速的阻尼 KHD
    編碼器同步還在（比重降到 40%），兩個一起用；J1 / J0 開關

  ④-2 直走微調自動學習（v4）
    車子本身會往一邊偏（實測：左輪比較吃虧，前進往左、後退往右）。
    每趟直走結束，看鎖方向平均修正了多少，把其中一半記成「微調」（前進、後退各一個），
    下一趟一開始就先補上。學好以後鎖方向幾乎不用出力，車子不靠陀螺儀也比較直。T 查看、T0 歸零

  ⑥ 無線燒錄（OTA）
    連上家裡 WiFi 後，Arduino IDE「工具 → 連接埠」會多一個網路連接埠「robot at 192.168.x.x」，
    選它就能不插 USB 直接上傳。燒錄開始時馬達會自動停。

  ⑤ 連家裡 WiFi（同時保留 RobotCar 熱點）
    - 把 secrets_example.h 複製成 secrets.h，填入家裡 WiFi 名稱和密碼（secrets.h 不會上傳 GitHub）
    - 沒有 secrets.h 或連不上時，只開 RobotCar 熱點（跟以前一樣）
    - 連上後網址：http://robot.local 或序列埠 / 網頁上顯示的 IP
    - /data 提供完整數據（JSON），給電腦讀取分析用

  使用方式
    1. 燒錄（燒錄時插 USB，電池開關關掉）
    2. 拔 USB，開電池，車子放平「不要動」1 秒（陀螺儀量零點）
    3. 手機連「RobotCar」（密碼 12345678）開 http://192.168.4.1，或在家裡 WiFi 開 http://robot.local

  接線（見 docs/PINOUT_v2.md）
    HC-020K  VCC→3.3V  GND→GND  左 OUT→GPIO 34  右 OUT→GPIO 35
    L298N    IN1/IN2→GPIO 26/27（右輪）  IN3/IN4→GPIO 32/33（左輪）  ENA/ENB 跳帽插著
    MPU6050  VCC→3.3V  GND→GND  SDA→GPIO 21  SCL→GPIO 22（平放、靠近兩輪中間、貼牢）

  指令（網頁「自己打指令」或序列埠）
    g3 / g-2   前進 3 圈 / 後退 2 圈        t90 / t-90  原地右轉 / 左轉
    w / s      前進 / 後退 2 秒               x           停止
    q3         正方形測試                     m           清地圖
    + / -      速度                           z           格數歸零
    e2         提前量                         k / b / c20 / c40  同步、停車方式、每圈格數
    D65 / W204 輪徑 / 輪距（mm）              H1 / H0     地圖方向用陀螺儀 / 編碼器
    G          陀螺儀正負號反轉               Y           陀螺儀重新量零點（車子要靜止）
    S1.00      陀螺儀比例                     h           說明
*/
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <Preferences.h>
#if __has_include("secrets.h")
#include "secrets.h"                       // 家裡 WiFi（自己建，不上傳）
#endif
#ifndef HOME_WIFI_SSID
#define HOME_WIFI_SSID ""
#define HOME_WIFI_PASS ""
#endif
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
float maxHead = 0;                         // 這趟直走時最大方向偏差（°）
unsigned long lastCtrl = 0;

// ---------- 路徑地圖（里程計） ----------
float WHEEL_MM = 65.0;                     // 輪子直徑（D 指令可改）
float TRACK_MM = 199.0;                    // 有效輪距（2026-11-07 用陀螺儀實測 6 次轉彎算出，W 指令可改，會存起來）
const float ROBOT_MM = 240;                // 機身直徑（地圖上白色路徑的寬度）
float odoX = 0, odoY = 0, odoTh = 0;       // mm、mm、弧度（0 = 一開始的車頭方向，順時針為正）
long odoL = 0, odoR = 0;                   // 上次算到的格數
unsigned long lastOdo = 0;
float lastYawOdo = 0;                      // 上次算地圖時的陀螺儀角度
const int PATHN = 1500;                    // 最多記幾個點（超過會丟掉最舊的）
int16_t pathX[PATHN], pathY[PATHN];
long pathSeq = 0;                          // 總共記過幾個點
int mapId = 0;                             // 每次清除地圖 +1，網頁看到變了就清畫面
float odoThEnc = 0;                        // 只用編碼器推算的方向（比較用）

// ---------- MPU6050 陀螺儀 ----------
const int SDA_PIN = 21, SCL_PIN = 22;
const uint8_t MPU = 0x68;
bool mpuOk = false;
bool useGyro = true;                       // 地圖方向用陀螺儀（H1）或編碼器（H0）
float gyroSign = 1;                        // 正負號（存在 ESP32 裡）
float gyroScale = 1.0;                     // 比例（存在 ESP32 裡）
float gyroBias = 0;                        // 零點（°/s）
float gyroRate = 0;                        // 目前轉速（°/s，順時針為正，已扣零點）
float yawDeg = 0;                          // 陀螺儀累積角度（°，順時針為正，不限 ±180）
unsigned long lastGyroUs = 0;
int gyroFails = 0;
// 自動量零點：車子停著時收集讀數
double zuptSum = 0; long zuptN = 0;
unsigned long stillSince = 0;
long stillL = 0, stillR = 0;
bool zeroed = false;                       // 至少量過一次零點
int signChecked = 0;                       // 0 = 還沒用原地轉確認過正負號
float turnYaw0 = 0;                        // 這趟開始時的陀螺儀角度
float turnEnc0 = 0;                        // 這趟開始時的編碼器方向
Preferences prefs;

// 陀螺儀轉彎
bool gyroTurn = false;                     // 這趟轉彎用陀螺儀決定停
float turnDeg = 0;                         // 要轉幾度（正數）
int turnDir = 1;                           // +1 右轉、-1 左轉
float turnK[2] = {0.094f, 0.079f};         // 提前量時間常數（秒）[右轉, 左轉]：提前度數 = K × 放開時轉速（自動學習，存起來）
                                           // 2026-11-07 實車 26 次轉彎學出來：45°/90°/180° 都在 ±1° 內
float relRate = 0, relLead = 0;            // 這趟放開時的轉速（°/s）和提前度數
bool headHold = true;                      // 直走鎖方向（J1 / J0）
float KH = 14;                             // 歪 1° → 修正 14 PWM（2026-11-07 實車調：6 → 14）
float KHD = 0.7f;                          // 每 1 °/s 轉速 → 再修正 0.7 PWM（阻尼，避免左右扭）
float KHI = 25;                            // 一直歪 1° 持續 1 秒 → 再多修正 25 PWM（消除固定偏差，例如一邊打滑）
float hInteg = 0;
float trim[2] = {0, 0};                    // 直走微調（PWM，正 = 左輪多給、右輪少給）[前進, 後退]（自動學習，存起來）
double corrSum = 0; long corrN = 0;        // 這趟直走的修正量加總（算平均用）
const float ENC_WEIGHT_WITH_GYRO = 0.4f;   // 有陀螺儀時，編碼器同步的比重
const float TURN_SLOW_DEG = 35;            // 最後幾度開始減速
const unsigned long TURN_TIMEOUT_MS = 8000;
unsigned long lastYawMoveMs = 0;
float lastYawSeen = 0;

// ---------- 每趟紀錄（給 /data 讀） ----------
struct Run {
  char kind;                               // 'g' 走圈數、't' 轉彎、'w' 計時
  long target, dl, dr, errL, errR, maxErr;
  float gyroDeg, encDeg, x, y;
  unsigned long t;
};
const int RUNN = 30;
Run runs[RUNN];
long runSeq = 0;

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
  if (mode == REVS && gyroTurn) {
    // 陀螺儀轉彎的結果在下面方向變化那裡算
  } else if (mode == REVS) {
    long errL = gdL * dl - target, errR = gdR * dr - target;
    out("  目標 %ld 格 → 左差 %+ld 格，右差 %+ld 格（+ 是衝過頭）", target, errL, errR);
    if (!turning && target >= sc(LEARN_MIN_TARGET) && !stalled) {   // 轉彎不拿來學
      // 直走是兩輪平均走到一起停，所以用平均誤差學、兩輪提前量保持一樣（各自學會越差越多）
      long errAvg = lround((errL + errR) / 2.0);
      float avgLead = (leadL() + leadR()) / 2;
      leadL() = leadR() = avgLead;
      learn(leadL(), errAvg, "左");
      learn(leadR(), errAvg, "右");
    }
  } else {
    long al = labs(dl), ar = labs(dr);
    if (al == 0 || ar == 0) out("  有一邊是 0 → 那邊的輪子沒轉，或編碼器沒讀到");
    else out("  左右差 %.0f%%", 100.0f * labs(al - ar) / max(al, ar));
  }
  float gd = yawDeg - turnYaw0;
  float ed = odoThEnc * 180 / PI - turnEnc0;
  if (ed > 180) ed -= 360; if (ed < -180) ed += 360;
  if (mpuOk) out("  方向變化：陀螺儀 %+.1f°，編碼器推算 %+.1f°", gd, ed);
  if (mpuOk && !turning) out("  直走鎖方向%s：途中最大偏 %.1f°，停下時偏 %+.1f°", headHold ? "開" : "關", maxHead, gd);
  bool held = headHold && mpuOk && useGyro && signChecked;
  if (!turning && held && !stalled && corrN > 25) {          // 鎖方向開著、跑超過 0.5 秒才學
    int k = goDir > 0 ? 0 : 1;
    float meanC = corrSum / corrN, old = trim[k];
    trim[k] = constrain(trim[k] + 0.5f * (meanC - trim[k]), -40.0f, 40.0f);
    prefs.putFloat(k ? "trB" : "trF", trim[k]);
    out("  直走微調（%s）：平均修正 %+.1f → 微調 %+.1f → %+.1f PWM", k ? "後退" : "前進", meanC, old, trim[k]);
  }
  // 正負號自動判斷：原地轉彎時，陀螺儀轉的方向應該跟下的指令一樣
  if (mpuOk && turning && fabs(gd) > 20) {
    int cmdDir = gdL > 0 ? 1 : -1;         // 左輪前進 = 右轉 = 順時針 = 正
    if ((gd > 0 ? 1 : -1) != cmdDir) {
      gyroSign = -gyroSign; yawDeg = -yawDeg; gd = -gd;
      prefs.putFloat("gsign", gyroSign);
      out("  ⚠ 陀螺儀正負號相反，已自動翻過來並存起來");
    } else if (!signChecked) out("  陀螺儀正負號正確");
    if (!signChecked) prefs.putInt("sok", 1);
    signChecked = 1;
  }
  if (gyroTurn && !stalled) {
    float err = gd * turnDir - turnDeg;    // + 是轉過頭
    int k = turnDir > 0 ? 0 : 1;
    float old = turnK[k];
    if (relRate > 20) turnK[k] = constrain(turnK[k] + 0.5f * err / relRate, 0.0f, 0.6f);
    prefs.putFloat(k ? "tkL" : "tkR", turnK[k]);
    out("  目標 %.0f° → 實際 %.1f°（%+.1f°）；放開時 %.0f °/s、提前 %.1f°；%s轉 K %.3f → %.3f 秒",
        turnDeg, gd * turnDir, err, relRate, relLead, k ? "左" : "右", old, turnK[k]);
  }
  out("  地圖位置：x %.1f cm，y %.1f cm，方向 %.0f°", odoX / 10, odoY / 10, odoTh * 180 / PI);
  Run &rr = runs[runSeq % RUNN];
  rr.kind = (mode == TIMED) ? 'w' : (turning ? 't' : 'g');
  rr.target = target; rr.dl = dl; rr.dr = dr; rr.maxErr = maxErr;
  rr.errL = (mode == REVS) ? gdL * dl - target : 0; rr.errR = (mode == REVS) ? gdR * dr - target : 0;
  rr.gyroDeg = gd; rr.encDeg = ed; rr.x = odoX; rr.y = odoY; rr.t = millis();
  runSeq++;
  mode = IDLE;
  turning = false;
  gyroTurn = false;
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
  goDir = gdL = gdR = dir; turning = false; integ = 0; hInteg = 0; maxErr = 0; maxHead = 0; lastCtrl = millis();
  corrSum = 0; corrN = 0;
  int s0 = softStop ? min(slowPWM, speed) : speed;
  setLeft(dir * s0); setRight(dir * s0);
  mode = TIMED; moveStart = millis();
  turnYaw0 = yawDeg; turnEnc0 = odoThEnc * 180 / PI;
  out("%s %lu 秒（PWM %d）", dir > 0 ? "前進" : "後退", MOVE_MS / 1000, speed);
}

// 兩輪各走 tgt 格，方向 dl、dr（+1 / -1）
void startTarget(long tgt, int dl, int dr) {
  target = max(1L, tgt);
  gdL = dl; gdR = dr; goDir = dl;
  turning = (dl != dr);
  startL = prevL = L.count; startR = prevR = R.count;
  doneL = doneR = false; stalled = false;
  integ = 0; hInteg = 0; maxErr = 0; maxHead = 0; lastCtrl = millis();
  corrSum = 0; corrN = 0;
  lastTickL = lastTickR = millis();
  mode = REVS; moveStart = millis();
  turnYaw0 = yawDeg; turnEnc0 = odoThEnc * 180 / PI;
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
  gyroTurn = mpuOk && useGyro && signChecked;
  turnDeg = fabs(deg); turnDir = d;
  lastYawMoveMs = millis(); lastYawSeen = yawDeg;
  if (gyroTurn)
    out("原地%s轉 %.0f°（看陀螺儀，提前量 K = %.3f 秒）", d > 0 ? "右" : "左", fabs(deg), turnK[d > 0 ? 0 : 1]);
  else
    out("原地%s轉 %.0f° = 每輪 %ld 格（用編碼器%s）", d > 0 ? "右" : "左", fabs(deg), target,
        mpuOk ? "，這次順便確認陀螺儀方向" : "");
}

// 陀螺儀轉彎：每圈 loop 檢查轉了幾度
void runGyroTurn() {
  float doneDeg = (yawDeg - turnYaw0) * turnDir;
  if (doneDeg < -15) {                     // 陀螺儀說在往反方向轉 → 正負號錯了，立刻翻
    gyroSign = -gyroSign; yawDeg = turnYaw0 - (yawDeg - turnYaw0); lastYawOdo = yawDeg;
    prefs.putFloat("gsign", gyroSign);
    out("  ⚠ 陀螺儀方向相反，已翻正負號");
    doneDeg = -doneDeg;
  }
  if (fabs(yawDeg - lastYawSeen) > 1) { lastYawSeen = yawDeg; lastYawMoveMs = millis(); }
  float remain = turnDeg - doneDeg;
  float rateNow = gyroRate * turnDir;      // 目前往目標方向轉的速度（°/s）
  float lead = max(1.0f, turnK[turnDir > 0 ? 0 : 1] * max(0.0f, rateNow));
  if (remain <= lead) { relRate = rateNow; relLead = lead; beginSettle("到達", !softStop); return; }
  if (millis() - lastYawMoveMs > STALL_MS) { stalled = true; beginSettle("轉不動（陀螺儀角度沒變），停止", true); return; }
  if (millis() - moveStart > TURN_TIMEOUT_MS) { stalled = true; beginSettle("轉彎逾時，停止", true); return; }
  int lo = min(slowPWM, speed);
  int base = remain >= TURN_SLOW_DEG ? speed : lo + (int)((speed - lo) * remain / TURN_SLOW_DEG);
  driveSynced(base, base);                 // 左右輪照樣保持同步
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
  bool gyroHold = headHold && mpuOk && useGyro && signChecked && !turning && both;
  float c = 0;
  if (syncOn && both) {
    float k = (float)DISC_SLOTS / cpr();                  // 40 格模式一格只有一半長，增益減半
    integ = constrain(integ + err * k * dt, -3.0f, 3.0f);
    // 有陀螺儀時：編碼器同步只留比例項、比重 40%（當阻尼用）。
    // 不留積分項，不然它會堅持「左右格數一樣」，跟陀螺儀「方向不歪」互相拉扯（一邊打滑時兩者要的不一樣）
    c = gyroHold ? ENC_WEIGHT_WITH_GYRO * KP * err * k : KP * err * k + KI * integ;
  }
  if (gyroHold) {
    // 歪向右（順時針，hErr > 0）→ 前進時左輪要慢、右輪要快（corr 往負）；後退時相反
    float hErr = yawDeg - turnYaw0;
    if (fabs(hErr) > maxHead) maxHead = fabs(hErr);
    hInteg = constrain(hInteg + hErr * dt, -5.0f, 5.0f);
    c -= goDir * (KH * hErr + KHI * hInteg + KHD * gyroRate);
  }
  if (!turning && both) {
    c += trim[goDir > 0 ? 0 : 1];          // 先補上學到的微調
    if (millis() - moveStart > START_RAMP_MS) { corrSum += c; corrN++; }   // 起步加速後才算平均
  }
  corr = constrain((int)lroundf(c), -CORR_MAX, CORR_MAX);
  baseL = softStart(baseL); baseR = softStart(baseR);
  if (baseL > 0) setLeft(gdL * constrain(baseL + corr, 0, 255));
  if (baseR > 0) setRight(gdR * constrain(baseR - corr, 0, 255));
}

// 直走：兩輪「平均」走到就一起停（各自停會在最後把車頭扭歪）
void runStraightHold() {
  long ml = (L.count - startL) * gdL, mr = (R.count - startR) * gdR;
  long c = L.count + R.count;
  if (c != prevL) { prevL = c; lastTickL = lastTickR = millis(); }
  float moved = (ml + mr) / 2.0f;
  float lead = (leadL() + leadR()) / 2;
  if (moved >= max(1.0f, target - lead)) {
    if (softStop) allOff(); else { brakeLeft(); brakeRight(); }
    beginSettle("到達", !softStop);
    return;
  }
  if (millis() - lastTickL > STALL_MS) { stalled = true; beginSettle("輪子卡住或沒讀到，停止", true); return; }
  float remain = target - moved;
  int lo = min(slowPWM, speed), base;
  if (softStop) { long zone = sc(RAMP_ZONE); base = remain >= zone ? speed : lo + (int)((speed - lo) * remain / zone); }
  else base = remain <= sc(slowZone) ? lo : speed;
  driveSynced(base, base);
}

void stopNow() {
  if (qHead != qTail) { qHead = qTail = 0; out("連續動作已取消"); }
  if (!busy()) { allOff(); out("停止"); return; }
  if (settleWhy) return;                   // 已經在煞車了
  beginSettle("手動停止", true);
}

// ---------- MPU6050 ----------
bool mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU); Wire.write(reg); Wire.write(val);
  return Wire.endTransmission() == 0;
}
int mpuRead(uint8_t reg, uint8_t *buf, int n) {
  Wire.beginTransmission(MPU); Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0;
  int got = Wire.requestFrom((int)MPU, n);
  for (int i = 0; i < got; i++) buf[i] = Wire.read();
  return got;
}
bool readGyroZ(float &dps) {
  uint8_t b[2];
  if (mpuRead(0x47, b, 2) != 2) return false;
  dps = (int16_t)((b[0] << 8) | b[1]) / 131.0f;   // ±250°/s → 131 LSB per °/s
  return true;
}

// 開機時量零點：車子要靜止
void calibrateGyro(int ms) {
  double sum = 0; int n = 0; float v;
  unsigned long t0 = millis();
  while (millis() - t0 < (unsigned long)ms) {
    if (readGyroZ(v)) { sum += v; n++; }
    delay(4);
  }
  if (n > 20) { gyroBias = sum / n; zeroed = true; }
}

bool initMPU() {
  Wire.begin(SDA_PIN, SCL_PIN, 400000);
  uint8_t who = 0;
  if (mpuRead(0x75, &who, 1) != 1) return false;
  if (who != 0x68 && who != 0x70 && who != 0x72 && who != 0x98) return false;
  mpuWrite(0x6B, 0x00);                    // 喚醒
  delay(50);
  mpuWrite(0x1A, 0x03);                    // 低通濾波 44 Hz
  mpuWrite(0x1B, 0x00);                    // 陀螺儀 ±250°/s
  delay(50);
  return true;
}

// 每 5 ms 讀一次，累加角度；停著時自動更新零點
void updateGyro() {
  if (!mpuOk) return;
  unsigned long now = micros();
  if (now - lastGyroUs < 5000) return;
  float dt = (now - lastGyroUs) / 1e6f;
  lastGyroUs = now;
  float raw;
  if (!readGyroZ(raw)) {
    if (++gyroFails > 40) { mpuOk = false; out("⚠ 讀不到 MPU6050，改用編碼器推算方向"); }
    return;
  }
  gyroFails = 0;
  float rate = gyroSign * gyroScale * (raw - gyroBias);

  // 判斷是不是靜止：沒在跑、編碼器沒動
  long l = L.count, r = R.count;
  bool idle = !busy();
  if (!idle || l != stillL || r != stillR) { stillSince = millis(); stillL = l; stillR = r; zuptSum = 0; zuptN = 0; }
  bool still = idle && millis() - stillSince > 1000;

  if (still && fabs(raw - gyroBias) < 3.0f) {
    zuptSum += raw; zuptN++;
    if (zuptN >= 200) {                    // 收集 1 秒 → 更新零點（新舊各半，避免跳動）
      gyroBias = 0.5f * gyroBias + 0.5f * (float)(zuptSum / zuptN);
      zuptSum = 0; zuptN = 0; zeroed = true;
    }
    if (fabs(rate) < 1.0f) rate = 0;       // 停著而且幾乎沒轉：不累加角度
  }
  gyroRate = rate;
  if (dt < 0.1f) yawDeg += rate * dt;
}

// ---------- 里程計 ----------
void addPoint() {
  int i = pathSeq % PATHN;
  pathX[i] = (int16_t)constrain(lroundf(odoX), -32000L, 32000L);
  pathY[i] = (int16_t)constrain(lroundf(odoY), -32000L, 32000L);
  pathSeq++;
}

void resetMap() {
  odoX = odoY = odoTh = 0; odoThEnc = 0; yawDeg = 0; lastYawOdo = 0;
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
  if (dlc == 0 && drc == 0) {
    if (mpuOk && useGyro) {                // 輪子沒動但車子被轉了（例如用手轉）：方向還是要更新
      odoTh += (yawDeg - lastYawOdo) * PI / 180;
      if (odoTh > PI) odoTh -= 2 * PI;
      if (odoTh < -PI) odoTh += 2 * PI;
    }
    lastYawOdo = yawDeg;
    return;
  }
  odoL = l; odoR = r;
  float mmPer = PI * WHEEL_MM / cpr();
  float dL = dlc * mmPer, dR = drc * mmPer;
  float ds = (dL + dR) / 2;                // 中心走了多遠
  float dthEnc = (dL - dR) / TRACK_MM;     // 編碼器推算：左輪走比較多 = 右轉 = 順時針
  odoThEnc += dthEnc;
  if (odoThEnc > PI) odoThEnc -= 2 * PI;
  if (odoThEnc < -PI) odoThEnc += 2 * PI;
  float dth = dthEnc;
  if (mpuOk && useGyro) dth = (yawDeg - lastYawOdo) * PI / 180;   // 改用陀螺儀這段時間轉的角度
  lastYawOdo = yawDeg;
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
  out("H1/H0 地圖方向用陀螺儀/編碼器  G 陀螺儀正負號反轉  Y 重新量零點  S1.00 陀螺儀比例");
  out("轉彎：%s（K 右 %.3f 左 %.3f 秒），直走鎖方向：%s，輪徑 %.1f mm，輪距 %.1f mm",
      (mpuOk && useGyro && signChecked) ? "看陀螺儀" : "看編碼器", turnK[0], turnK[1], headHold ? "開" : "關", WHEEL_MM, TRACK_MM);
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
      if (cmd.length() > 1) { WHEEL_MM = constrain(cmd.substring(1).toFloat(), 30.0f, 120.0f); prefs.putFloat("wheel", WHEEL_MM); }
      out("輪子直徑 %.1f mm（一圈 %.1f cm）", WHEEL_MM, PI * WHEEL_MM / 10); break;
    case 'W':
      if (cmd.length() > 1) { TRACK_MM = constrain(cmd.substring(1).toFloat(), 80.0f, 400.0f); prefs.putFloat("track", TRACK_MM); }
      out("輪距 %.1f mm", TRACK_MM); break;
    case 'H':
      useGyro = cmd.length() > 1 ? cmd.charAt(1) != '0' : !useGyro;
      out("地圖方向來源：%s", (useGyro && mpuOk) ? "陀螺儀" : (useGyro ? "陀螺儀（但 MPU 沒偵測到，暫用編碼器）" : "編碼器"));
      break;
    case 'K':                              // K6 / K6,8,0.4 = KH, KHI, KHD
      if (cmd.length() > 1) {
        String a = cmd.substring(1); int p1 = a.indexOf(','), p2 = p1 >= 0 ? a.indexOf(',', p1 + 1) : -1;
        KH = a.toFloat();
        if (p1 >= 0) KHI = a.substring(p1 + 1).toFloat();
        if (p2 >= 0) KHD = a.substring(p2 + 1).toFloat();
        prefs.putFloat("KH", KH); prefs.putFloat("KHI", KHI); prefs.putFloat("KHD", KHD);
      }
      out("鎖方向參數：KH %.1f，KHI %.1f，KHD %.2f", KH, KHI, KHD); break;
    case 'T':
      if (cmd.length() > 1) { trim[0] = trim[1] = cmd.substring(1).toFloat(); prefs.putFloat("trF", trim[0]); prefs.putFloat("trB", trim[1]); }
      out("直走微調：前進 %+.1f、後退 %+.1f PWM（正 = 左輪多給）", trim[0], trim[1]); break;
    case 'J':
      headHold = cmd.length() > 1 ? cmd.charAt(1) != '0' : !headHold;
      out("直走鎖方向：%s（KH %.1f，KHD %.2f）", headHold ? "開" : "關", KH, KHD); break;
    case 'G':
      gyroSign = -gyroSign; yawDeg = -yawDeg; lastYawOdo = yawDeg; prefs.putFloat("gsign", gyroSign);
      signChecked = 1; prefs.putInt("sok", 1);
      out("陀螺儀正負號已反轉（已存起來）"); break;
    case 'S':
      if (cmd.length() > 1) { gyroScale = constrain(cmd.substring(1).toFloat(), 0.8f, 1.2f); prefs.putFloat("gscale", gyroScale); }
      out("陀螺儀比例 %.4f", gyroScale); break;
    case 'Y':
      if (busy()) { out("跑的時候不能量零點"); break; }
      if (!mpuOk) { mpuOk = initMPU(); if (!mpuOk) { out("讀不到 MPU6050，檢查 SDA 21 / SCL 22 / 3.3V / GND"); break; } }
      out("陀螺儀量零點中，車子不要動…");
      calibrateGyro(1000); lastGyroUs = micros();
      out("陀螺儀零點 %.3f °/s", gyroBias); break;
    case 'h': help(); break;
    default: out("看不懂這個指令：%s", cmd.c_str());
  }
}

// ---------- 網頁 ----------
const char PAGE[] PROGMEM = R"HTML(<!doctype html>
<html lang="zh-Hant"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1">
<title>掃地機器人</title>
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
<h1><span id="dot"></span>掃地機器人</h1>
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
 <p class="lbl">陀螺儀（MPU6050）<span id="gst"></span></p>
 <div class="cnt">
  <div><small>陀螺儀方向</small><b id="gy">-</b><small id="gr">0 °/s</small></div>
  <div><small>編碼器推算方向</small><b id="ge">-</b><small>比較用</small></div>
 </div>
 <p class="lbl" style="margin-top:8px">檢查：用手把車子<b>順時針</b>轉，陀螺儀方向應該變大。變小就按「反轉方向」。</p>
 <div class="pair"><button onclick="send('G')">反轉方向</button><button onclick="send('Y')">重新量零點（車子別動）</button></div>
 <div class="opt"><span>直走鎖方向</span><div class="seg"><button id="j1" onclick="send('J1')">開</button><button id="j0" onclick="send('J0')">關</button></div></div>
 <div class="opt"><span>地圖方向來源</span><div class="seg"><button id="h1" onclick="send('H1')">陀螺儀</button><button id="h0" onclick="send('H0')">編碼器</button></div></div>
 <p class="lbl" id="ipl" style="margin-top:8px"></p>
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
  $('gst').textContent=d.mpu?(d.zero?'　正常':'　量零點中…'):'　⚠ 沒偵測到，改用編碼器';
  $('gy').textContent=d.mpu?d.yaw.toFixed(1)+'°':'-';$('gr').textContent=d.rate.toFixed(1)+' °/s';
  $('ge').textContent=d.the.toFixed(1)+'°';
  on('h1',d.hg);on('h0',!d.hg);on('j1',d.hh);on('j0',!d.hh);
  $('ipl').textContent=d.ip?('家裡 WiFi 網址：http://'+d.ip+'　或 http://robot.local'):'目前只有 RobotCar 熱點';
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
       ",\"wd\":" + String(ROBOT_MM, 0) +
       ",\"mpu\":" + String(mpuOk ? 1 : 0) + ",\"hg\":" + String(useGyro ? 1 : 0) + ",\"hh\":" + String(headHold ? 1 : 0) +
       ",\"yaw\":" + String(yawDeg, 1) + ",\"rate\":" + String(gyroRate, 1) +
       ",\"the\":" + String(odoThEnc * 180 / PI, 1) + ",\"zero\":" + String(zeroed ? 1 : 0) +
       ",\"sok\":" + String(signChecked) +
       ",\"ip\":\"" + (WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : String("")) + "\"" +
       ",\"p\":[";
  for (long i = ps; i < pe; i++) {
    if (i > ps) j += ',';
    j += String(pathX[i % PATHN]); j += ','; j += String(pathY[i % PATHN]);
  }
  j += "]}";
  server.send(200, "application/json", j);
}

// 完整數據（給電腦讀取分析）：參數 + 目前狀態 + 最近 30 趟紀錄
void handleData() {
  String j = "{\"params\":{";
  j += "\"cpr\":" + String(cpr()) + ",\"speed\":" + String(speed) + ",\"slowPWM\":" + String(slowPWM);
  j += ",\"softStop\":" + String(softStop ? 1 : 0) + ",\"sync\":" + String(syncOn ? 1 : 0);
  j += ",\"KP\":" + String(KP, 1) + ",\"KI\":" + String(KI, 1);
  j += ",\"leadL\":" + String(leadL(), 2) + ",\"leadR\":" + String(leadR(), 2);
  j += ",\"wheelMM\":" + String(WHEEL_MM, 2) + ",\"trackMM\":" + String(TRACK_MM, 2);
  j += ",\"gyroSign\":" + String(gyroSign, 0) + ",\"gyroScale\":" + String(gyroScale, 4) + ",\"gyroBias\":" + String(gyroBias, 4);
  j += ",\"useGyro\":" + String(useGyro ? 1 : 0) + ",\"signOk\":" + String(signChecked);
  j += ",\"turnKR\":" + String(turnK[0], 3) + ",\"turnKL\":" + String(turnK[1], 3);
  j += ",\"headHold\":" + String(headHold ? 1 : 0) + ",\"KH\":" + String(KH, 2) + ",\"KHI\":" + String(KHI, 2) + ",\"KHD\":" + String(KHD, 2);
  j += ",\"trimF\":" + String(trim[0], 2) + ",\"trimB\":" + String(trim[1], 2) + "}";
  j += ",\"state\":{\"mpu\":" + String(mpuOk ? 1 : 0) + ",\"yaw\":" + String(yawDeg, 2) + ",\"encHeading\":" + String(odoThEnc * 180 / PI, 2);
  j += ",\"x\":" + String(odoX, 1) + ",\"y\":" + String(odoY, 1) + ",\"heading\":" + String(odoTh * 180 / PI, 2);
  j += ",\"L\":" + String(L.count) + ",\"R\":" + String(R.count) + ",\"busy\":" + String(busy() ? 1 : 0) + ",\"ms\":" + String(millis()) + "}";
  j += ",\"runs\":[";
  long from = max(0L, runSeq - RUNN);
  for (long i = from; i < runSeq; i++) {
    const Run &r = runs[i % RUNN];
    if (i > from) j += ',';
    j += "{\"n\":" + String(i) + ",\"kind\":\"" + String(r.kind) + "\",\"target\":" + String(r.target);
    j += ",\"dl\":" + String(r.dl) + ",\"dr\":" + String(r.dr) + ",\"errL\":" + String(r.errL) + ",\"errR\":" + String(r.errR);
    j += ",\"maxErr\":" + String(r.maxErr) + ",\"gyroDeg\":" + String(r.gyroDeg, 1) + ",\"encDeg\":" + String(r.encDeg, 1);
    j += ",\"x\":" + String(r.x, 0) + ",\"y\":" + String(r.y, 0) + ",\"t\":" + String(r.t) + "}";
  }
  j += "],\"log\":[";
  long lf = max(0L, logSeq - LOGN);
  for (long i = lf; i < logSeq; i++) { if (i > lf) j += ','; j += '"'; j += jsonEsc(logs[i % LOGN]); j += '"'; }
  j += "]}";
  server.sendHeader("Access-Control-Allow-Origin", "*");
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

  // MPU6050：開機量零點（車子要靜止）
  prefs.begin("robot", false);
  gyroSign = prefs.getFloat("gsign", 1.0f);
  gyroScale = prefs.getFloat("gscale", 1.0f);
  signChecked = prefs.getInt("sok", 0);
  turnK[0] = prefs.getFloat("tkR", turnK[0]);
  turnK[1] = prefs.getFloat("tkL", turnK[1]);
  KH = prefs.getFloat("KH", KH); KHI = prefs.getFloat("KHI", KHI); KHD = prefs.getFloat("KHD", KHD);
  trim[0] = prefs.getFloat("trF", 0); trim[1] = prefs.getFloat("trB", 0);
  WHEEL_MM = prefs.getFloat("wheel", WHEEL_MM);
  TRACK_MM = prefs.getFloat("track", TRACK_MM);
  mpuOk = initMPU();
  if (mpuOk) { calibrateGyro(1000); lastGyroUs = micros(); }

  // WiFi：有 secrets.h 就同時連家裡 WiFi，熱點 RobotCar 一直保留
  bool home = strlen(HOME_WIFI_SSID) > 0;
  WiFi.mode(home ? WIFI_AP_STA : WIFI_AP);
  WiFi.setSleep(false);
  if (home) {
    WiFi.setAutoReconnect(true);
    WiFi.begin(HOME_WIFI_SSID, HOME_WIFI_PASS);
    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) delay(200);
  }
  WiFi.softAP(AP_SSID, AP_PASS);
  if (MDNS.begin("robot")) MDNS.addService("http", "tcp", 80);
  if (home && WiFi.status() == WL_CONNECTED) {
    ArduinoOTA.setHostname("robot");
#ifdef OTA_PASS
    ArduinoOTA.setPassword(OTA_PASS);
#endif
    ArduinoOTA.onStart([]() { allOff(); mode = IDLE; settleWhy = nullptr; qHead = qTail = 0; });
    ArduinoOTA.begin();
  }
  server.on("/", []() { server.send(200, "text/html; charset=utf-8", PAGE); });
  server.on("/cmd", []() { handle(server.arg("c")); server.send(200, "text/plain", "ok"); });
  server.on("/status", handleStatus);
  server.on("/data", handleData);
  server.begin();

  resetMap();
  help();
  out("熱點：%s / %s，網址 http://%s", AP_SSID, AP_PASS, WiFi.softAPIP().toString().c_str());
  if (home) {
    if (WiFi.status() == WL_CONNECTED) out("家裡 WiFi 已連上：http://%s 或 http://robot.local", WiFi.localIP().toString().c_str());
    else out("⚠ 家裡 WiFi 連不上（10 秒），只用熱點；之後會自動重試");
  } else out("沒有 secrets.h：只開熱點");
  if (mpuOk) out("MPU6050 正常，零點 %.3f °/s，正負號 %+.0f，比例 %.4f", gyroBias, gyroSign, gyroScale);
  else out("⚠ 沒偵測到 MPU6050：地圖方向用編碼器推算");
}

void loop() {
  ArduinoOTA.handle();
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

  if (mode == REVS && !settleWhy && gyroTurn) {
    runGyroTurn();
  } else if (mode == REVS && !settleWhy && !turning) {
    runStraightHold();                     // 直走一律兩輪平均走到才一起停（各自停會在最後扭一下）
  } else if (mode == REVS && !settleWhy) {
    int bL = checkWheel(L, startL, gdL, doneL, lastTickL, prevL, leadL(), brakeLeft, setLeft);
    int bR = checkWheel(R, startR, gdR, doneR, lastTickR, prevR, leadR(), brakeRight, setRight);
    driveSynced(bL, bR);
    if (doneL && doneR) beginSettle("到達", !softStop);
    else if ((!doneL && millis() - lastTickL > STALL_MS) || (!doneR && millis() - lastTickR > STALL_MS)) {
      stalled = true;
      beginSettle(!doneL && millis() - lastTickL > STALL_MS ? "左輪卡住或沒讀到，停止" : "右輪卡住或沒讀到，停止", true);
    }
  }

  updateGyro();
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
