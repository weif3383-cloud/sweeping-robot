// 小車測試程式 v1 — 階段 3、4：單輪測試、雙輪行走、直線校正
// 板子：NodeMCU-32S（Arduino-ESP32 核心 3.x）
// 用法：序列監視器 115200 baud，輸入指令字母後按 Enter。輸入 h 看說明。
// 安全設計：每個動作會自動停止，不會一直衝出去。
#include <Arduino.h>

// ---- 腳位（與開發流程 PDF 相同）----
const int L_EN = 25, L_IN1 = 26, L_IN2 = 27;   // 左輪：L298N #1 ENA / IN1 / IN2
const int R_EN = 14, R_IN1 = 32, R_IN2 = 33;   // 右輪：L298N #1 ENB / IN3 / IN4
const int PWM_FREQ = 1000, PWM_RES = 8;        // 8 bit：0~255

// ---- 可調參數 ----
int speed = 180;   // 行走速度（+ / - 調整）
int bal = 0;       // 直線校正：左輪 = speed + bal，右輪 = speed - bal（q / e 調整）

const unsigned long MOVE_MS = 1500;      // w / s 的動作時間
const unsigned long TURN_MS = 700;       // a / d 的動作時間
const unsigned long STRAIGHT_MS = 5000;  // g 直線測試時間
const unsigned long SPIN_MS = 5000;      // t 原地旋轉測試時間
const unsigned long SWEEP_STEP_MS = 600; // l / r 掃描時每一階停留時間

enum Mode { IDLE, MOVING, SWEEP };
Mode mode = IDLE;
unsigned long modeStart = 0, moveDuration = 0, lastStep = 0;
int sweepSide = 0;  // 0 = 左輪, 1 = 右輪
int sweepDuty = 0;
unsigned long sweepStarted = 0;

// s：-255 ~ 255，負值為反轉
void motor(int en, int in1, int in2, int s) {
  s = constrain(s, -255, 255);
  digitalWrite(in1, s > 0 ? HIGH : LOW);
  digitalWrite(in2, s < 0 ? HIGH : LOW);
  ledcWrite(en, abs(s));
}

void drive(int left, int right) {
  motor(L_EN, L_IN1, L_IN2, left);
  motor(R_EN, R_IN1, R_IN2, right);
}

void stopAll() {
  drive(0, 0);
  mode = IDLE;
}

void startMove(int left, int right, unsigned long ms) {
  drive(left, right);
  mode = MOVING;
  modeStart = millis();
  moveDuration = ms;
}

void printStatus() {
  Serial.printf("speed = %d   bal = %d   （左輪 PWM = %d，右輪 PWM = %d）\n",
                speed, bal, constrain(speed + bal, 0, 255), constrain(speed - bal, 0, 255));
}

void help() {
  Serial.println();
  Serial.println("===== 小車測試指令（輸入字母後按 Enter）=====");
  Serial.println(" l / r : 左輪 / 右輪 單獨從低速慢慢加速（看到開始轉就按 Enter 停止）");
  Serial.println(" w / s : 前進 / 後退 1.5 秒");
  Serial.println(" a / d : 原地左轉 / 右轉 0.7 秒");
  Serial.println(" g     : 直線測試，前進 5 秒（放在地上測）");
  Serial.println(" t     : 原地右轉 5 秒（數轉幾圈，算出轉一圈要幾秒）");
  Serial.println(" + / - : 速度加減 10");
  Serial.println(" q     : 車子往「左」偏時按（左輪加快、右輪減慢）");
  Serial.println(" e     : 車子往「右」偏時按（右輪加快、左輪減慢）");
  Serial.println(" x     : 立即停止");
  Serial.println(" p     : 顯示目前參數    h : 顯示這份說明");
  Serial.println("============================================");
  printStatus();
}

void startSweep(int side) {
  drive(0, 0);
  sweepSide = side;
  sweepDuty = 20;  // 第一階會先 +10，從 30 開始
  mode = SWEEP;
  lastStep = millis();
  sweepStarted = lastStep;
  Serial.printf("%s輪掃描開始：PWM 從 30 慢慢往上加，輪子一開始轉就按 Enter\n",
                side == 0 ? "左" : "右");
}

void handle(char c) {
  switch (c) {
    case 'l': startSweep(0); break;
    case 'r': startSweep(1); break;
    case 'w': Serial.println("前進"); startMove(speed + bal, speed - bal, MOVE_MS); break;
    case 's': Serial.println("後退"); startMove(-(speed + bal), -(speed - bal), MOVE_MS); break;
    case 'a': Serial.println("原地左轉"); startMove(-speed, speed, TURN_MS); break;
    case 'd': Serial.println("原地右轉"); startMove(speed, -speed, TURN_MS); break;
    case 'g': Serial.println("直線測試 5 秒"); startMove(speed + bal, speed - bal, STRAIGHT_MS); break;
    case 't': Serial.println("原地右轉 5 秒，請數圈數"); startMove(speed, -speed, SPIN_MS); break;
    case 'x': stopAll(); Serial.println("停止"); break;
    case '+': speed = min(255, speed + 10); printStatus(); break;
    case '-': speed = max(0, speed - 10); printStatus(); break;
    case 'q': bal += 3; printStatus(); break;
    case 'e': bal -= 3; printStatus(); break;
    case 'p': printStatus(); break;
    case 'h': help(); break;
    default: break;  // 忽略其他字元
  }
}

void setup() {
  Serial.begin(115200);
  const int outs[] = {L_IN1, L_IN2, R_IN1, R_IN2};
  for (int p : outs) pinMode(p, OUTPUT);
  ledcAttach(L_EN, PWM_FREQ, PWM_RES);
  ledcAttach(R_EN, PWM_FREQ, PWM_RES);
  drive(0, 0);
  delay(500);
  help();
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (mode == SWEEP) {  // 掃描中按任何鍵（包含 Enter）就停
      if (millis() - sweepStarted < 300) continue;  // 忽略啟動指令同一行的 Enter
      stopAll();
      Serial.printf(">> 停在 PWM = %d，這附近就是這顆馬達的最低啟動值，請記下來\n", sweepDuty);
      while (Serial.available()) Serial.read();  // 清掉同一行剩下的字元
      break;
    }
    if (c == '\r' || c == '\n' || c == ' ') continue;
    handle(c);
  }

  unsigned long now = millis();

  if (mode == MOVING && now - modeStart >= moveDuration) {
    drive(0, 0);
    mode = IDLE;
    Serial.println("（自動停止）");
  }

  if (mode == SWEEP && now - lastStep >= SWEEP_STEP_MS) {
    lastStep = now;
    sweepDuty += 10;
    if (sweepDuty > 250) {
      stopAll();
      Serial.println("掃描結束：到 250 都沒轉，請檢查接線（EN 跳帽、共地、馬達線）");
    } else {
      if (sweepSide == 0) motor(L_EN, L_IN1, L_IN2, sweepDuty);
      else                motor(R_EN, R_IN1, R_IN2, sweepDuty);
      Serial.printf("PWM = %d\n", sweepDuty);
    }
  }
}
