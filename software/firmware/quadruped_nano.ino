/*
* ミニ四脚歩行ロボット ファームウェア骨子 (Arduino Nano 互換機 + PCA9685)
*
* このスケッチは Arduino IDE / arduino-cli の無い環境で書かれた
* 骨子であり、コンパイル確認は行われていない。実機投入前に必ず
* Arduino IDE でコンパイルと単体動作確認を行うこと。
*
* 依存ライブラリ: Adafruit PWM Servo Driver Library
* (ライブラリマネージャからインストール。Wire は標準ライブラリ)
*
* 構成: Nano --(I2C)--> PCA9685 --(PWM)--> MG90S x8
* 歩容パラメータと計算式は PC 側 gait.py / leg_ik.py と同一。
* パラメータを変更したら Python 側 (GaitConfig) と揃えること。
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ===================== パラメータ (ここに集約) =====================
// 脚寸法 [mm] (leg_ik.LegGeometry と同じ)
const float L1 = 50.0; // 上リンク
const float L2 = 60.0; // 下リンク

// 歩容 (gait.GaitConfig と同じ)
const float HIP_HEIGHT = 95.0; // 股関節高さ = 足先 z [mm]
const float STRIDE = 36.0; // 足先の前後移動量 [mm]
const float STEP_HEIGHT = 14.0; // 足上げ高さ [mm]
const float DUTY = 0.5; // 支持期の割合 (トロット)
const float DIRECTION = 1.0; // +1: 前進, -1: 後退
const unsigned long CYCLE_MS = 1600; // 1周期の時間 [ms] (ゆっくり)
const unsigned long UPDATE_MS = 20; // サーボ更新間隔 [ms]

// PWM (PCA9685, 50Hz)
const int PWM_FREQ = 50;
const int PULSE_MIN_US = 500; // 0 deg
const int PULSE_MAX_US = 2500; // 180 deg

// サーボ変換の校正 (実機で詰める。gait.py の *_offset / *_dir に対応)
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

// 足先軌跡: gait.foot_position と同じ (支持期=直線, 遊脚期=楕円の上半分)
// phase は [0,1)、戻り値は股関節基準の (x, z) [mm]、z は下方 +
void footPosition(float phase, float &x, float &z) {
float u = phase - floor(phase);
float half = STRIDE / 2.0;
float lift;
if (u < DUTY) {
x = half - STRIDE * (u / DUTY);
lift = 0.0;
} else {
float v = (u - DUTY) / (1.0 - DUTY);
x = -half * cos(PI * v);
lift = STEP_HEIGHT * sin(PI * v);
}
x *= DIRECTION;
z = HIP_HEIGHT - lift;
}

// 逆運動学: leg_ik.inverse_kinematics と同じ (elbow_sign=+1, 膝後ろ配置)
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
footPosition(basePhase + PHASE_OFFSET[leg], x, z);
inverseKinematics(x, z, a1, a2);
float hip = SERVO_CENTER + HIP_DIR[leg] * a1 + HIP_OFFSET[leg];
float knee = SERVO_CENTER + KNEE_DIR[leg] * a2 + KNEE_OFFSET;
setServoAngle(CH_HIP[leg], hip);
setServoAngle(CH_KNEE[leg], knee);
}

void setup() {
Wire.begin();
pwm.begin();
pwm.setPWMFreq(PWM_FREQ);
}

void loop() {
static unsigned long last = 0;
unsigned long now = millis();
if (now - last < UPDATE_MS) return;
last = now;

float basePhase = (now % CYCLE_MS) / (float)CYCLE_MS;
for (int leg = 0; leg < 4; leg++) {
updateLeg(leg, basePhase);
}
}
