# 事後課題

※ SDカードに制御ログを落とすところまで行かなかったので制御ログに関する説明はありません。

## 理論

### 緯度と経度からゴールまでの距離と角度を求める

北方向を$y$軸、東方向を$x$軸とし、

| 変数名 | 意味 |
| -- | -- |
| $lng_G$ | ゴールの経度(東経) |
| $lng$ | cansatの経度(東経) |
| $lat_G$ | ゴールの緯度(北緯) |
| $lat$ | cansatの緯度(北緯) |
| $\theta$ | 曲座標表示したときのcansatの位置の$\theta$成分 |
| $r$ | 曲座標表示したときのcansatの位置の$r$成分 |
| $R$ | 地球の半径 |

このときcansatの位置を$xy$成分で$(a, b)$と表現したとき

$$
a = R \cdot rad(lng - lng_G) \cdot cos(lat_G) \\
b = R \cdot rad(lat - lat_G)
$$

(※ $rad()$は度数表記をラジアン表記にする関数)

となる。

このとき

$$
r = \sqrt{a^2 + b^2} \\
\theta = a > 0\ ?\ atan(b/a) : -atan(b/a)
$$

(※ ただし $-\frac{\pi}{2} < atan(x) < \frac{\pi}{2}$)

と求められる。

これによって緯度・経度からゴールに対してcansatがどこにいるかという$(r, \theta)$の情報が得られた。

![cansat 01][image01]

### 誘導アルゴリズムのための準備

上記からcansatのゴールに対する位置$(r, \theta)$はわかったが、cansatがどちらを向いているかはわからない。

したがってcansatを少し前進させ、その位置の変化から方向を得る。

またその方向に直進したとき、原点にある半径$L$の円を横切るかどうかを判定する判別式$D(r, \Delta{r}, \Delta{\theta})$も求めておく。
(原点にある半径$L$の円を横切るということは、ゴールがそこにあった場合到達するということ。)

![cansat 02][image02]

![cansat 03][image03]

### 誘導アルゴリズム

![cansat 04][image04]

## プログラムの説明

### 度数法と弧度法の変換

```ino
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
```

### 判別式D

```ino
/**
 * calcD()はそのまま直進してゴールに辿り着けるかの判定式を計算します。
 * D > 0 => 到達可能
 * D <= 0 => 到達不可能
 */
float calcD(float r, float d_r, float d_theta) {
  return pow(d_r, 2.0) - (pow(L, 2.0) - 1) * pow(r, 2.0) * pow(d_theta, 2.0)
}
```

### 誘導アルゴリズム

```ino
/** 目標地点へ走行 */
void drive() {
  float now_theta = getAngleOfCurrentPosition();
  float now_r = getDistanceToGoal();
  float d_r = now_r - moveLog.last_r;
  float d_theta = now_theta - moveLog.last_theta;
  float result = calcD(moveLog.last_r, d_r, d_theta);

  // r < L になった => ゴールに到達した
  if (now_r < L) {
    state = ST_GOAL;
    return;
  }

  // 判別式D()の結果とΔθを用いた誘導アルゴリズムの実装
  if (result > 0) {
    if (d_r < 0) {
      // そのまま直進(つまりここでは何もしない)
      Serial.println("直進: 1");
    } else {
      Serial.println("大きく回転: 2");
      // 右か左に大きく回転
      right(255);
      delay(400);
    }
  } else {
    if (d_theta > 0) {
      Serial.println("左回転: 3");
      // 左回転
      left(255);
      delay(200);
    } else {
      Serial.println("右回転: 4");
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
```

### 右・左に回転

```ino
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
```

### 緯度・経度からθを計算

```ino
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
```

### 緯度・経度からrを計算

```ino
/**
 * calcDistanceToGoal()は(lat_rad, lng_rad)とゴールの間の距離[m]を返します
 */
float calcDistanceToGoal(float lat_rad, float lng_rad) {
  float a = (lng_rad - goal_lng_rad) * cos(goal_lat_rad);
  float b = lat_rad - goal_lat_rad;
  return RADIUS_OF_THE_EARTH * sqrt(pow(a, 2.0) + pow(b, 2.0));
}
```

# seccamp2022-sample
セキュリティ・ネクストキャンプ2022講義課題用リポジトリ

本講義は3〜4人でチームを組んで行います。
チームメンバーのうち1人がこのリポジトリをフォークしてそれぞれ開発を行ってください。

受講生ははじめに[講義に関するドキュメント](./doc/)を読んでください。

## ディレクトリ構成

```txt
.
|- doc/     # 講義に関するドキュメント
|- log/     # 制御ログの提出場所
|- src/     # 制御プログラムの提出場所
```

[image01]: images/cansat-01.png
[image02]: images/cansat-02.png
[image03]: images/cansat-03.png
[image04]: images/cansat-04.png
