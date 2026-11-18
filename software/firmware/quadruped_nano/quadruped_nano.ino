/*
* ミニ四脚歩行ロボット ファームウェア (Arduino Nano 互換機 + PCA9685)
*
* コンパイル確認済み (2026-11-11): arduino-cli 1.6.0、
* arduino:avr 1.8.8、Adafruit PWM Servo Driver Library 3.0.3 で
* arduino:avr:nano 向けにコンパイル成功 (Flash 8186 bytes / RAM 304 bytes)。
* フラッシュ書き込みと実機動作の確認は未実施 (docs/BRINGUP.md の Stage A)。
*
* シリアルコマンド (9600 bps, 改行終端): 起動時は歩行せず中立姿勢で保持する。
*   n = 中立姿勢, c = 全サーボ raw 90°, s <ch> <deg> = 個別サーボ指定,
*   g = 歩行の開始/停止トグル, ? = ヘルプ表示。校正手順は docs/BRINGUP.md。
*
* 依存ライブラリ: Adafruit PWM Servo Driver Library
* (ライブラリマネージャからインストール。Wire は標準ライブラリ)
*
* 構成: Nano --(I2C)--> PCA9685 --(PWM)--> MG90S x8
* 歩容パラメータと計算式は PC 側 gait.cpp / leg_ik.cpp と同一。
* パラメータを変更したら C++ 側 (GaitConfig) と揃えること。
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ===================== パラメータ (ここに集約) =====================
// 脚寸法 [mm] (leg_ik.hpp の LegGeometry と同じ)
const float L1 = 50.0; // 上リンク
const float L2 = 60.0; // 下リンク

// 歩容 (gait.hpp の GaitConfig と同じ)
const float HIP_HEIGHT = 95.0; // 股関節高さ = 足先 z [mm]
const float STRIDE = 36.0; // 足先の前後移動量 [mm]
const float STEP_HEIGHT = 14.0; // 足上げ高さ [mm]
const float DUTY = 0.5; // 支持期の割合 (トロット)
const float DIRECTION = 1.0; // +1: 前進, -1: 後退
const float TURN = 0.0; // -1..+1: 左右のストライド倍率 (gait.hpp の turn と同じ)。左脚 x(1-TURN), 右脚 x(1+TURN)
const unsigned long CYCLE_MS = 1600; // 1周期の時間 [ms] (ゆっくり)
const unsigned long UPDATE_MS = 20; // サーボ更新間隔 [ms]

// PWM (PCA9685, 50Hz)
const int PWM_FREQ = 50;
const int PULSE_MIN_US = 500; // 0 deg
const int PULSE_MAX_US = 2500; // 180 deg

// サーボ変換の校正 (実機で詰める。gait.hpp の *_offset / *_dir に対応)
const float SERVO_CENTER = 90.0;
const float KNEE_OFFSET = -25.0; // 膝サーボの取付オフセット

// 脚の定義: 0=FL(前左), 1=FR(前右), 2=RL(後左), 3=RR(後右)
const float PHASE_OFFSET[4] = {0.0, 0.5, 0.5, 0.0}; // トロット (対角ペア)
const int HIP_DIR[4] = {1, 1, 1, 1};
const int KNEE_DIR[4] = {1, 1, 1, 1};
const float HIP_OFFSET[4] = {0.0, 0.0, 0.0, 0.0};
const int CH_HIP[4] = {0, 2, 4, 6}; // PCA9685 チャンネル
const int CH_KNEE[4] = {1, 3, 5, 7};

// ===================== 実装 =====================
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

void setServoAngle(int ch, float deg) {
if (deg < 0.0) deg = 0.0;
if (deg > 180.0) deg = 180.0;
float us = PULSE_MIN_US + (PULSE_MAX_US - PULSE_MIN_US) * deg / 180.0;
int counts = (int)(us * 4096.0 * PWM_FREQ / 1000000.0);
pwm.setPWM(ch, 0, counts);
}

// 足先軌跡: gait.cpp の foot_position と同じ (支持期=直線, 遊脚期=楕円の上半分)
// phase は [0,1)、戻り値は股関節基準の (x, z) [mm]、z は下方 +
// stride は脚ごとの実効ストライド (旋回時は左右で異なる)
void footPosition(float phase, float stride, float &x, float &z) {
float u = phase - floor(phase);
float half = stride / 2.0;
float lift;
if (u < DUTY) {
x = half - stride * (u / DUTY);
lift = 0.0;
} else {
float v = (u - DUTY) / (1.0 - DUTY);
x = -half * cos(PI * v);
lift = STEP_HEIGHT * sin(PI * v);
}
x *= DIRECTION;
z = HIP_HEIGHT - lift;
}

// 逆運動学: leg_ik.cpp の inverse_kinematics と同じ (elbow_sign=+1, 膝後ろ配置)
// 到達不能の場合はリーチ内に丸める (実機保護のため例外ではなくクランプ)
void inverseKinematics(float x, float z, float &a1deg, float &a2deg) {
float r = sqrt(x * x + z * z);
float maxR = L1 + L2 - 0.01;
if (r > maxR) { x *= maxR / r; z *= maxR / r; r = maxR;}
float cosA2 = (x * x + z * z - L1 * L1 - L2 * L2) / (2.0 * L1 * L2);
if (cosA2 > 1.0) cosA2 = 1.0;
if (cosA2 < -1.0) cosA2 = -1.0;
float a2 = acos(cosA2);
float a1 = atan2(x, z) - atan2(L2 * sin(a2), L1 + L2 * cos(a2));
a1deg = a1 * 180.0 / PI;
a2deg = a2 * 180.0 / PI;
}

void updateLeg(int leg, float basePhase) {
float x, z, a1, a2;
bool right = (leg == 1 || leg == 3); // FR, RR
float stride = STRIDE * (right ? (1.0 + TURN) : (1.0 - TURN));
footPosition(basePhase + PHASE_OFFSET[leg], stride, x, z);
inverseKinematics(x, z, a1, a2);
float hip = SERVO_CENTER + HIP_DIR[leg] * a1 + HIP_OFFSET[leg];
float knee = SERVO_CENTER + KNEE_DIR[leg] * a2 + KNEE_OFFSET;
setServoAngle(CH_HIP[leg], hip);
setServoAngle(CH_KNEE[leg], knee);
}

// ===================== シリアル校正モード =====================
bool gaitRunning = false; // 起動時は歩行しない (安全のため)

void setNeutralPose() {
for (int leg = 0; leg < 4; leg++) {
float a1, a2;
inverseKinematics(0.0, HIP_HEIGHT, a1, a2);
float hip = SERVO_CENTER + HIP_DIR[leg] * a1 + HIP_OFFSET[leg];
float knee = SERVO_CENTER + KNEE_DIR[leg] * a2 + KNEE_OFFSET;
setServoAngle(CH_HIP[leg], hip);
setServoAngle(CH_KNEE[leg], knee);
}
}

void setAllCenter() {
for (int ch = 0; ch < 8; ch++) setServoAngle(ch, SERVO_CENTER);
}

void printHelp() {
Serial.println("cmds: n=neutral, c=center90, s <ch> <deg>, g=gait on/off, ?=help");
}

char cmdBuf[32];
int cmdLen = 0;

void runCommand(char *line) {
if (line[0] == 'n') { gaitRunning = false; setNeutralPose(); Serial.println("neutral"); }
else if (line[0] == 'c') { gaitRunning = false; setAllCenter(); Serial.println("center"); }
else if (line[0] == 'g') { gaitRunning = !gaitRunning; Serial.println(gaitRunning ? "gait on" : "gait off"); }
else if (line[0] == 's') {
int ch; float deg;
if (sscanf(line + 1, "%d %f", &ch, &deg) == 2 && 0 <= ch && ch < 8) {
gaitRunning = false;
setServoAngle(ch, deg);
Serial.println("set");
} else { Serial.println("usage: s <ch 0-7> <deg>"); }
}
else if (line[0] == '?') { printHelp(); }
}

void pollSerial() {
while (Serial.available() > 0) {
char c = (char)Serial.read();
if (c == '\n' || c == '\r') {
if (cmdLen > 0) { cmdBuf[cmdLen] = '\0'; runCommand(cmdBuf); cmdLen = 0; }
} else if (cmdLen < (int)sizeof(cmdBuf) - 1) {
cmdBuf[cmdLen++] = c;
}
}
}

void setup() {
Serial.begin(9600);
Wire.begin();
pwm.begin();
pwm.setPWMFreq(PWM_FREQ);
setNeutralPose();
printHelp();
}

void loop() {
pollSerial();
if (!gaitRunning) return;
static unsigned long last = 0;
unsigned long now = millis();
if (now - last < UPDATE_MS) return;
last = now;

float basePhase = (now % CYCLE_MS) / (float)CYCLE_MS;
for (int leg = 0; leg < 4; leg++) {
updateLeg(leg, basePhase);
}
}
