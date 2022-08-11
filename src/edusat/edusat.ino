#include "MPU9250.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <obniz.h>
#include <TinyGPSPlus.h>
#include <math.h> /* atan/sqrt/pow */

// 円周率
#define PI (3.14159265358979323846264338327950288)
// 地球の半径 [m]
#define RADIUS_OF_THE_EARTH (6378137)

// ゴールの緯度(北緯) [°]
const float goal_lat = 35.676960;
// ゴールの経度(東経) [°]
const float goal_lng = 139.475372;
// ゴールの緯度(北緯) [rad]
const float goal_lat_rad = 0.622680;
// ゴールの経度(東経) [rad]
const float goal_lng_rad = 2.434304;
// ゴールの半径 [m]
const float L = 1.0;

const float mC = 261.626; // ド
const float mD = 293.665; // レ
const float mE = 329.628; // ミ
const float mF = 349.228; // ファ
const float mG = 391.995; // ソ
const float mA = 440.000; // ラ 
const float mB = 493.883; // シ
const float nn = 0.0;

const float beep_wakeup[7] = {mE * 4, mA * 4, mB * 4, mA * 4, mE * 2, mE * 4, mB * 4}; // 起動音
const float beep_start[3] = {mC * 2, mD * 2, mE * 2};
const float beep_end[3] = {mE * 2, mD * 2, mC * 2};
const float beep_error[5] = {mE * 4, mE * 4, mE * 4, mE * 4, mE * 4};

MPU9250 mpu;
TinyGPSPlus gps;
HardwareSerial hs(2);

const uint8_t pin_sda        = 21;
const uint8_t pin_scl        = 22;
const uint8_t pin_button     = 35;
const uint8_t pin_led        = 2;
const uint8_t pin_heat       = 15;
const uint8_t pin_speaker    = 12;
const uint8_t pin_sd_miso    = 19;
const uint8_t pin_sd_mosi    = 23;
const uint8_t pin_sd_sclk    = 18;
const uint8_t pin_sd_cs      = 5;
const uint8_t pin_motor_A[3] = {4, 13, 25};  // AIN1, AIN2, PWMA
const uint8_t pin_motor_B[3] = {14, 27, 26}; // BIN1, BIN2, PWMB
const uint8_t pin_gps_tx     = 16;
const uint8_t pin_gps_rx     = 17;

const int CHANNEL_A = 0; // PWMA
const int CHANNEL_B = 1; // PWMB
const int CHANNEL_C = 2; // Speaker

const int LEDC_TIMER_8_BIT    = 8;
const int LEDC_TIMER_13_BIT   = 13;
const int LEDC_BASE_FREQ_490  = 490;
const int LEDC_BASE_FREQ_5000 = 490;

volatile byte led_state = LOW;
volatile long interrupt_prev_ms = millis();

struct SensorVal {
  float roll;
  float pitch;
  float yaw;
  float lat;
  float lng;
  float lat_rad; // 現在地の緯度 [rad]
  float lng_rad; // 現在地の経度 [rad]
} sensorVal;

struct MoveLog {
  float last_theta; // 移動前の角度 [rad]
  float last_r;     // 移動前のゴールまでの距離 [m]
} moveLog;

/** CanSatの状態遷移用の列挙型 */
enum {
  ST_STAND_BY = 0, // 待機
  ST_DRIVE,        // 目標地点へ走行
  ST_GOAL,         // 目標地点に到着
};

/** CanSatの状態遷移ステータス */
volatile int state = ST_STAND_BY;

/**
 * setup関数
 * 最初に1回だけ実行される
 */
void setup() {
  Serial.begin(115200);
  Serial.println("Hello 100kinsat!");
  delay(2000);

  // 初期化処理
  obniz_init();
  pin_init();
  sd_init();
  mpu_init();
  gps_init();

  delay(2000);
  startUpdateMPUValTask();
  startUpdateGPSValTask();
  startSendObnizTask();

  beep(beep_wakeup, sizeof(beep_wakeup) / sizeof(float), 150);
}

/**
 * loop関数
 * 繰り返し実行される
 */
void loop() {
  switch (state)
  {
  case ST_STAND_BY:
    Serial.println("*** ST_STAND_BY ***");
    printCurrentPosition();
    stand_by();
    break;

  case ST_DRIVE:
    Serial.println("*** ST_DRIVE ***");
    drive();
    break;

  case ST_GOAL:
    Serial.println("*** ST_GOAL ***");
    goal();
    break;

  default:
    break;
  }
  delay(200);
}

/**
 * toRadian()は度数degreeをラジアンに変換します
 */
float toRadian(float degree) {
  return degree * (PI / 180);
}

/**
 * toDegree()はラジアンradianを度数に変換します
 */
float toDegree(float radian) {
  return radian * (180 / PI);
}

/**
 * calcD()はそのまま直進してゴールに辿り着けるかの判定式を計算します。
 * D > 0 => 到達可能
 * D <= 0 => 到達不可能
 */
float calcD(float r, float d_r, float d_theta) {
  // 一旦D()は使わずにθの変化が小さくなったら直進とする
  if (fabsf(toDegree(d_theta)) < 5) {
    return 1;
  } else {
    return 0;
  }

  // return pow(d_r, 2.0) - (pow(L, 2.0) - 1) * pow(r, 2.0) * pow(d_theta, 2.0)
}

/** ボタンの割り込み関数 */
void IRAM_ATTR onButton() {
  if (millis() > interrupt_prev_ms + 500) { // チャタリング防止
    led_state = !led_state;
    state = (state + 1) % 3;
    interrupt_prev_ms = millis();
  }
}

/**
 * マルチタスクで実行する関数
 * 9軸センサの値の更新
 */
TaskHandle_t updateMPUValTaskHandle;
void updateMPUValTask(void *pvParameters) {
  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  while (1) {
    if (mpu.update()) {
      static uint32_t prev_ms = millis();
      if (millis() > prev_ms + 25) {
        prev_ms = millis();
      }
      sensorVal.roll = mpu.getRoll();
      sensorVal.pitch = mpu.getPitch();
      sensorVal.yaw = mpu.getYaw();
    }
    delay(10);
  }
}

/**
 * マルチタスクで実行する関数（updateMPUValTask）の開始
 */
void startUpdateMPUValTask() {
  xTaskCreatePinnedToCore(
    updateMPUValTask,
    "updateMPUValTask",
    8192,
    NULL,
    1,
    &updateMPUValTaskHandle,
    APP_CPU_NUM
  );
}

/**
 * マルチタスクで実行する関数
 * GPSセンサの値の更新
 */
TaskHandle_t updateGPSValTaskHandle;
void updateGPSValTask(void *pvParameters) {
  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  while (1) {
    while (hs.available() > 0) {
      char c = hs.read();
      gps.encode(c);
      if (gps.location.isUpdated()) {
        sensorVal.lat = gps.location.lat();
        sensorVal.lng = gps.location.lng();
        sensorVal.lat_rad = toRadian(gps.location.lat());
        sensorVal.lng_rad = toRadian(gps.location.lng());
      }
    }
    delay(100);
  }
}

/**
 * マルチタスクで実行する関数（updateGPSValTask）の開始
 */
void startUpdateGPSValTask() {
  xTaskCreatePinnedToCore(
    updateGPSValTask,
    "updateGPSValTask",
    8192,
    NULL,
    1,
    &updateGPSValTaskHandle,
    APP_CPU_NUM
  );
}

/**
 * マルチタスクで実行する関数
 * CanSat -> JavaScript へのデータ送信
 */
TaskHandle_t sendObnizTaskHandle;
void sendObnizTask(void *pvParameters) {
  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  while (1) {
    char message[512]; // JavaScriptへ送信するメッセージ用
    char roll_buf[16];
    char pitch_buf[16];
    char yaw_buf[16];
    char lat_buf[16];
    char lng_buf[16];

    if (obniz.isOnline()) {
      dtostrf(sensorVal.roll, -1, 2, (char*)roll_buf);
      dtostrf(sensorVal.pitch, -1, 2, (char*)pitch_buf);
      dtostrf(sensorVal.yaw, -1, 2, (char*)yaw_buf);
      dtostrf(sensorVal.lat, -1, 6, (char*)lat_buf);
      dtostrf(sensorVal.lng, -1, 6, (char*)lng_buf);
      sprintf(message, "%d,%s,%s,%s,%s,%s",
        state,
        roll_buf, pitch_buf, yaw_buf,
        lat_buf, lng_buf);
      obniz.commandSend((uint8_t*)message, strlen(message));
    }
    delay(500);
  }
}

/**
 * マルチタスクで実行する関数（sendObnizTask）の開始
 */
void startSendObnizTask() {
  xTaskCreatePinnedToCore(
    sendObnizTask,
    "sendObnizTask",
    8192,
    NULL,
    1,
    &sendObnizTaskHandle,
    APP_CPU_NUM
  );
}

/** 待機状態 */
void stand_by() {
  digitalWrite(pin_led, led_state);
}

/** 目標地点へ走行 */
void drive() {
  float now_theta = getAngleOfCurrentPosition();
  float now_r = getDistanceToGoal();
  float d_r = now_r - moveLog.last_r;
  float d_theta = now_theta - moveLog.last_theta;
  float result = calcD(moveLog.last_r, d_r, d_theta);

  if (now_r < 2*L) {
    state = ST_GOAL;
    return;
  }

  if (result > 0) {
    if (d_r < 0) {
      // そのまま直進(つまりここでは何もしない)
    } else {
      // 右か左に大きく回転
      right(255);
      delay(400);
    }
  } else {
    if (d_theta > 0) {
      // 左回転
      left(255);
      delay(200);
    } else {
      // 右回転
      right(255);
      delay(200);
    }
  }

  moveLog.last_theta = getAngleOfCurrentPosition();
  moveLog.last_r = getDistanceToGoal();
  forward(255);
  delay(200);
}

/** 目標地点に到着 */
void goal() {
  stop();
}

/** ObnizOSの初期化処理 */
void obniz_init() {
  obniz.start(NULL); // 引数にNULLを渡すとObnizOSのログがシリアルモニタに表示されなくなる
  // IOの管理をobnizOSから外す
  obniz.pinReserve(pin_sda);
  obniz.pinReserve(pin_scl);
  obniz.pinReserve(pin_button);
  obniz.pinReserve(pin_led);
  obniz.pinReserve(pin_heat);
  obniz.pinReserve(pin_speaker);
  obniz.pinReserve(pin_sd_miso);
  obniz.pinReserve(pin_sd_mosi);
  obniz.pinReserve(pin_sd_sclk);
  obniz.pinReserve(pin_sd_cs);
  for (int i = 0; i < 3; i++) {
    obniz.pinReserve(pin_motor_A[i]);
    obniz.pinReserve(pin_motor_B[i]);
  }
  obniz.pinReserve(pin_gps_tx);
  obniz.pinReserve(pin_gps_rx);
}

/** GPIOの初期化処理 */
void pin_init() {
  pinMode(pin_button, INPUT);
  attachInterrupt(pin_button, onButton, FALLING);
  pinMode(pin_led, OUTPUT);
  pinMode(pin_heat, OUTPUT);
  digitalWrite(pin_heat, LOW); // 電熱線のピンはLOWにしておく
  pinMode(pin_speaker, OUTPUT);
  for (int i = 0; i < 3; i++) {
    pinMode(pin_motor_A[i], OUTPUT);
    pinMode(pin_motor_B[i], OUTPUT);
  }
  ledcSetup(CHANNEL_A, LEDC_BASE_FREQ_490, LEDC_TIMER_8_BIT);
  ledcSetup(CHANNEL_B, LEDC_BASE_FREQ_490, LEDC_TIMER_8_BIT);
  ledcAttachPin(pin_motor_A[2], CHANNEL_A);
  ledcAttachPin(pin_motor_B[2], CHANNEL_B);
}

/** SDカードの初期化処理 */
void sd_init() {
  if (!SD.begin()) {
    Serial.println("Card mount failed.");
    beep(beep_error, sizeof(beep_error) / sizeof(float), 100);
    return;
  }
  uint8_t cardType = SD.cardType();

  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached.");
    // 初期化に失敗したらエラー音を鳴らす
    beep(beep_error, sizeof(beep_error) / sizeof(float), 100);
    return;
  }

  Serial.print("SD Card Type: ");
  if (cardType == CARD_MMC) {
    Serial.println("MMC");
  } else if (cardType == CARD_SD) {
    Serial.println("SDSC");
  } else if (cardType == CARD_SDHC) {
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);

  writeFile(SD, "/100kinsat.txt", "Hello 100kinSAT!!!");
}

/** 9軸センサの初期化処理 */
void mpu_init() {
  Wire.begin();
  delay(2000);

  // if (!mpu.setup(0x69)) { // サイはI2Cアドレスが違う
  if (!mpu.setup(0x68)) {
    Serial.println("MPU connection failed.");
    // 初期化に失敗したらエラー音を鳴らす
    beep(beep_error, sizeof(beep_error) / sizeof(float), 100);
    return;
  }
  mpu.setMagneticDeclination(-7.49); // 磁気偏角の設定（府中駅: -7.49）
  mpu.selectFilter(QuatFilterSel::MADGWICK); // フィルターの設定
  mpu.setFilterIterations(10);

  mpu.verbose(true);

  // 加速度/ジャイロセンサのキャリブレーション
  // キャリブレーション中はCanSatを平らな地面で静止させておく
  beep(beep_start, sizeof(beep_start) / sizeof(float), 150);
  delay(500);
  mpu.calibrateAccelGyro();
  beep(beep_end, sizeof(beep_end) / sizeof(float), 150);

  delay(1000);

  // 地磁気センサのキャリブレーション
  // キャリブレーション中はCanSatをぐるぐる回転させる
  beep(beep_start, sizeof(beep_start) / sizeof(float), 150);
  delay(500);
  mpu.calibrateMag();
  beep(beep_end, sizeof(beep_end) / sizeof(float), 150);

  mpu.verbose(false);
}

/** GPSセンサの初期化処理 */
void gps_init() {
  hs.begin(9600);
}

/** 前進 */
void forward(int pwm) {
  if (pwm < 0) pwm = 0;
  if (pwm > 255) pwm = 255;

  // 左モータ（CCW，反時計回り）
  digitalWrite(pin_motor_A[0], LOW);
  digitalWrite(pin_motor_A[1], HIGH);
  ledcWrite(CHANNEL_A, pwm);

  // 右モータ（CW，時計回り）
  digitalWrite(pin_motor_B[1], LOW);
  digitalWrite(pin_motor_B[0], HIGH);
  ledcWrite(CHANNEL_B, pwm);
}

/** 後退 */
void back(int pwm) {
  if (pwm < 0) pwm = 0;
  if (pwm > 255) pwm = 255;

  // 左モータ（CW，時計回り）
  digitalWrite(pin_motor_A[1], LOW);
  digitalWrite(pin_motor_A[0], HIGH);
  ledcWrite(CHANNEL_A, pwm);

  // 右モータ（CCW，反時計回り）
  digitalWrite(pin_motor_B[0], LOW);
  digitalWrite(pin_motor_B[1], HIGH);
  ledcWrite(CHANNEL_B, pwm);
}

/** 停止 */
void stop() {
  // 左モータ停止
  digitalWrite(pin_motor_A[0], LOW);
  digitalWrite(pin_motor_A[1], LOW);
  ledcWrite(CHANNEL_A, HIGH);

  // 右モータ停止
  digitalWrite(pin_motor_B[0], LOW);
  digitalWrite(pin_motor_B[1], LOW);
  ledcWrite(CHANNEL_B, HIGH);
}

/** 右回転 */
void right(int pwm) {
  if (pwm < 0) pwm = 0;
  if (pwm > 255) pwm = 255;

  // 左モータ（CCW 反時計回り）
  digitalWrite(pin_motor_A[0], LOW);
  digitalWrite(pin_motor_A[1], HIGH);
  ledcWrite(CHANNEL_A, pwm);
}

/** 左回転 */
void left(int pwm) {
  if (pwm < 0) pwm = 0;
  if (pwm > 255) pwm = 255;

  // 右モータ（CW 時計回り）
  digitalWrite(pin_motor_B[1], LOW);
  digitalWrite(pin_motor_B[0], HIGH);
  ledcWrite(CHANNEL_B, pwm);
}

/** SDカードに新規書き込みする */
void writeFile(fs::FS &fs, const char *path, const char *message) {
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

/** SDカードに追記する */
void appendFile(fs::FS &fs, const char *path, const char *message) {
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if (file.print(message)) {
    Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}

/** スピーカーから音を鳴らす */
void beep(const float *mm, int m_size, int t_ms) {
  for (int i = 0; i < m_size; i++) {
    tone(pin_speaker, mm[i], t_ms);
  }
  noTone(pin_speaker);
}

void tone(int pin, int freq, int t_ms) {
  ledcSetup(CHANNEL_C, LEDC_BASE_FREQ_5000, LEDC_TIMER_13_BIT);
  ledcAttachPin(pin, CHANNEL_C);
  ledcWriteTone(CHANNEL_C, freq);

  delay(t_ms);
}

void noTone(int pin) {
  ledcWriteTone(CHANNEL_C, 0.0);
}

/**
 * calcAngleOfCurrentPosition()は原点をゴールとし東をx軸方向、北をy軸方向とした際、x軸と原点から見た(lat_rad, lng_rad)の方向のなす角をラジアン[rad]で返します
 */
float calcAngleOfCurrentPosition(float lat_rad, float lng_rad) {
  float a = (lng_rad - goal_lng_rad) * cos(goal_lat_rad);
  float b = lat_rad - goal_lat_rad;
  float theta = atan(b/a);
  if (a < 0) {
    theta -= PI;
  }

  return theta;
}

/**
 * getAngleOfCurrentPosition()は原点をゴールとし東をx軸方向、北をy軸方向とした際、x軸と原点から見たcansatの現在地の方向のなす角をラジアン[rad]で返します
 */
float getAngleOfCurrentPosition() {
  return calcAngleOfCurrentPosition(sensorVal.lat_rad, sensorVal.lng_rad);
}

/**
 * calcDistanceToGoal()は(lat_rad, lng_rad)とゴールの間の距離[m]を返します
 */
float calcDistanceToGoal(float lat_rad, float lng_rad) {
  float a = (lng_rad - goal_lng_rad) * cos(goal_lat_rad);
  float b = lat_rad - goal_lat_rad;
  return RADIUS_OF_THE_EARTH * sqrt(pow(a, 2.0) + pow(b, 2.0));
}

/**
 * getDistanceToGoal()は現在地とゴールの間の距離[m]を返します
 */
float getDistanceToGoal() {
  return calcDistanceToGoal(sensorVal.lat_rad, sensorVal.lng_rad);
}

// デバッグ用関数

/**
 * printCurrentPosition()は現在地情報(北緯, 東経, ゴールまでの距離)を以下のフォーマットでシリアルポートに出力します
 * ----- 現在地情報 -----
 * 北緯: 35.676960 [°]
 * 東経: 139.475372 [°]
 * 角度: 40.000001 [°]
 * ゴールまでの距離: 10.000001 [m]
 * -----------------
 */
float printCurrentPosition() {
  Serial.println("----- 現在地情報 -----");
  Serial.print("北緯: ");
  Serial.print(gps.location.lat());
  Serial.println(" [°]");
  Serial.print("東経: ");
  Serial.print(gps.location.lng());
  Serial.println(" [°]");
  Serial.print("角度: ");
  Serial.print(toDegree(getAngleOfCurrentPosition()));
  Serial.println(" [°]");
  Serial.print("ゴールまでの距離: ");
  Serial.print(getDistanceToGoal());
  Serial.println(" [m]");
  Serial.println("-----------------");
}
