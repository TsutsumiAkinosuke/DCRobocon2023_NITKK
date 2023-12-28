#include <Arduino.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <micro_ros_arduino.h>

#include <LwIP.h>
#include <STM32Ethernet.h>

#include <Wire.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#include <MD.h>
#include <MDC.h>  // Wire -> Wire1に変更

// micro-ROS
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// micro-ROSのチェック用のマクロ
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn;}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// LimitSwitchメッセージ送信用のpublisher
rcl_publisher_t ls_publisher;

// RobotInfoメッセージ送信用のpublisher
rcl_publisher_t info_publisher;

// RobotCommandメッセージ受信用のsubscriber
rcl_subscription_t cmd_subscriber;

// タイマー
rcl_timer_t timer;

// メッセージ型をインクルードして宣言
#include <custom_interfaces/msg/robot_command.h>
custom_interfaces__msg__RobotCommand cmd_msg;

#include <custom_interfaces/msg/robot_info.h>
custom_interfaces__msg__RobotInfo info_msg;

#include <custom_interfaces/msg/limit_switch.h>
custom_interfaces__msg__LimitSwitch ls_msg;

// cmdメッセージ用添字マクロ
#define CMD_ELECAS_FORWARD  0
#define CMD_ELECAS_BACK     1
#define CMD_RF_MODE         2
#define CMD_RF_RISE         3
#define CMD_RF_FALL         4
#define CMD_DECON_AUTO      5
#define CMD_DECON_LEFT      6
#define CMD_DECON_RIGHT     7
#define CMD_CABLE_WIND      8
#define CMD_CABLE_RELEASE   9

// Wire0はLAN用SPI通信のポートとかぶっているためWire1を使用
TwoWire Wire1;

// MDCの添字用マクロ
#define NUM_MDC           6
#define MDC_JACK_FORWARD  0
#define MDC_JACK_BACK     1
#define MDC_RF_MAIN       2
#define MDC_RF_SUB        3
#define MDC_DECON         4
#define MDC_CABLE         5

// MDCのインスタンス
MDC mdc(PF0, PF1);

// MDCのアドレス
int mdc_addr[NUM_MDC] = {0x10, 0x11, 0x12, 0x13, 0x14, 0x15};

/* ----------------------------------------------------------- */

// リミットスイッチの数
#define NUM_LS 10

// リミットスイッチ添字用マクロ
#define LS_ELECAS_FORWARD_TOP      0
#define LS_ELECAS_FORWARD_BOTTOM   1
#define LS_ELECAS_BACK_TOP         2
#define LS_ELECAS_BACK_BOTTOM      3
#define LS_RF_MAIN_TOP             4
#define LS_RF_MAIN_BOTTOM          5
#define LS_RF_SUB_TOP              6
#define LS_RF_SUB_BOTTOM           7  //
#define LS_DECON_LEFT              8
#define LS_DECON_RIGHT             9  //

int ls_pin[NUM_LS] = {PD12,PD11,PA0,PE0,PB10,PE15,PE14,PE12,PE10,PE7};
bool ls_state[NUM_LS] = {true};
bool pre_ls_state[NUM_LS] = {true};

// リミットスイッチの状態のチェック・更新
void ls_check(void)
{
  // 前回チェック時と状態が変わったリミットスイッチの数
  int ls_cnt = 0;

  for(int i = 0; i < NUM_LS; i++) {
    // 前回チェック時の状態を保存
    pre_ls_state[i] = ls_state[i];

    // 現在の状態を代入
    ls_state[i] = digitalRead(ls_pin[i]);

    // 状態が変わっていたらカウント+1
    if(pre_ls_state[i] != ls_state[i]) {
      ls_cnt += 1;
    }
  }

  // もし状態の変わったリミットスイッチが1つ以上あれば
  if(ls_cnt > 0){

    // lsメッセージの内容を更新
    for(int i = 0; i < NUM_LS; i++) {
      ls_msg.ls[i] = ls_state[i];
    }

    // LimitSwitchメッセージをパブリッシュ
    rcl_publish(&ls_publisher, &ls_msg, NULL);
  }
}

/* ----------------------------------------------------------- */

// 左右
#define CRAWLER_LEFT  0
#define CRAWLER_RIGHT 1

// MDライブラリのインスタンス
MD mt_crawler_right(PC11,PC10,PC9);
MD mt_crawler_left(PA6,PD14,PD15);

// クローラーのduty比を格納
double crawler_left_duty = 0.0, crawler_right_duty = 0.0;

// クローラーを制御する関数
void crawler_control(void)
{
  // クローラーのDuty比を計算
  crawler_left_duty = max(-0.99, min(0.99, cmd_msg.value[CRAWLER_LEFT]/128.0 - cmd_msg.value[CRAWLER_RIGHT]/128.0));
  crawler_right_duty = max(-0.99, min(0.99, cmd_msg.value[CRAWLER_LEFT]/128.0 + cmd_msg.value[CRAWLER_RIGHT]/128.0));

  // クローラーのモーターを回転
  mt_crawler_left.move(crawler_left_duty);
  mt_crawler_right.move(crawler_right_duty);
}

/* ----------------------------------------------------------- */

#define ROLL  0
#define PITCH 1
#define YAW   2

// BNO055のインスタンス
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire1);

// BNO055を認識しているかどうか
bool bno_is_found = false;

// ロボットの初期角度
double init_degree[3] = {0.0};

// ロボットの角度
double degree[3] = {0.0};

//BNO055のセットアップ
void setup_bno055(void)
{
  // 念のため遅延
  delay(100);

  // BNO055のセットアップを実行
  bno_is_found = bno.begin();

  // BNO055が見つかったなら
  if(bno_is_found) {

    // 念のため遅延
    delay(100);

    // 初期角度を取得
    imu::Vector<3> gyro_vec = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    init_degree[ROLL] = gyro_vec.z();
    init_degree[PITCH] = gyro_vec.y();

    Serial.println("BNO055 is found");

    // BNO055と通信成功したなら青色LED点灯
    digitalWrite(PB7,HIGH);

  // BNO055が見つからないなら
  } else {
    Serial.println("BNO055 is NOT found");
  }
}

void imu_control(void)
{
  // もしBNO055が接続されているなら
  if (bno_is_found) {

    // 角度を取得
    imu::Vector<3> gyro_vec = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    degree[ROLL] = gyro_vec.z() - init_degree[ROLL];
    degree[PITCH] = gyro_vec.y() - init_degree[PITCH];
    degree[YAW] = gyro_vec.x();

    // pitch角とyaw角をrobot_infoメッセージに代入
    info_msg.pitch = max(-90, min(90, int(degree[PITCH])));
    //info_msg.yaw = max(0, min(65535, int(degree[YAW] * 182)));
    info_msg.yaw = max(0, min(65535, int(degree[YAW] * 100)));
  }
}

/* ----------------------------------------------------------- */

#define RF_SUB_RADIUS 15.5

double rf_main_duty = 0.0, rf_sub_duty = 0.0;

// 昇降機構を制御
void rf_control(void)
{
  // メイン昇降モードなら
  if (cmd_msg.mode[CMD_RF_MODE] == false) {

    // サブ昇降停止
    rf_sub_duty = 0.0;

    // 上昇コマンド & 頂のリミットスイッチが押されていないなら
    if (cmd_msg.mode[CMD_RF_RISE] && !ls_state[LS_RF_MAIN_TOP]) {

      // 昇降機構を上昇させる
      rf_main_duty = 0.90;
    
    // 下降コマンド & 底のリミットスイッチが押されていないなら
    } else if(cmd_msg.mode[CMD_RF_FALL] && !ls_state[LS_RF_MAIN_BOTTOM]) {
      // 昇降機構を下降させる
      rf_main_duty = -0.7;
    } else {
      // メイン昇降停止
      rf_main_duty = 0.0;
    }
  // サブ昇降モードなら
  } else if(cmd_msg.mode[CMD_RF_MODE] == true) {

    // メイン昇降停止
    rf_main_duty = 0.0;

    // 上昇コマンド & 頂のリミットスイッチが押されていないなら
    if (cmd_msg.mode[CMD_RF_RISE] && !ls_state[LS_RF_SUB_TOP]) {

      // 昇降機構を上昇させる
      rf_sub_duty = -0.99;
    
    // 下降コマンド & 底のリミットスイッチが押されていないなら
    } else if(cmd_msg.mode[CMD_RF_FALL] && !ls_state[LS_RF_SUB_BOTTOM]) {

      // 昇降機構を下降させる
      rf_sub_duty = 0.7;

    } else {

      // メイン昇降停止
      rf_sub_duty = 0.0;

    }
  } else {
    // 昇降停止
    rf_main_duty = 0.0;
    rf_sub_duty = 0.0;
  }

  // メイン・サブの昇降機構を駆動
  mdc.move(mdc_addr[MDC_RF_MAIN], rf_main_duty);
  mdc.move(mdc_addr[MDC_RF_SUB], rf_sub_duty);
}

/* ----------------------------------------------------------- */

// エレキャスワイヤのリールの半径
#define ELECAS_RADIUS 22.28169

// エレキャスの現在の長さ
double elecas_length_back = 0.0, elecas_length_forward = 0.0;

// エレキャスのモータのDuty比
double elecas_forward_duty = 0.0;
double elecas_back_duty = 0.0;

// 前後のエレキャスを制御
void elecas_control(void)
{
  // エレキャスの伸びている距離を計算
  //elecas_length_forward = mdc.angle(MDC_JACK_FORWARD) * ELECAS_RADIUS;
  //elecas_length_back = mdc.angle(MDC_JACK_BACK) * ELECAS_RADIUS;

  // 前方エレキャスが展開モードなら
  if (cmd_msg.mode[CMD_ELECAS_FORWARD] == true) {
    elecas_forward_duty = !ls_state[LS_ELECAS_FORWARD_BOTTOM] * 0.45;
  // 前方エレキャスが格納モードなら
  } else {
    elecas_forward_duty = !ls_state[LS_ELECAS_FORWARD_TOP] * -0.25;
  }

  // 後方エレキャスが展開モードなら
  if (cmd_msg.mode[CMD_ELECAS_BACK] == true) {
    elecas_back_duty = !ls_state[LS_ELECAS_BACK_BOTTOM] * 0.45;
  // 後方エレキャスが格納モードなら
  } else {
    elecas_back_duty = !ls_state[LS_ELECAS_BACK_TOP] * -0.25;
  }

  mdc.move(mdc_addr[MDC_JACK_FORWARD], elecas_forward_duty);
  mdc.move(mdc_addr[MDC_JACK_BACK], elecas_back_duty);
}

/* ----------------------------------------------------------- */

// 除染機構横のモーターのDuty比
double decon_x_duty = 0.0;

// 除染機構自動制御のシーケンス
int decon_auto_sequence = 0;

// 自動除染の往復回数
int decon_count = 0;

#define DECON_RADIUS 17.13

// 除染機構を手動制御
void decon_manual(void)
{
  // 除染機構自動制御の変数を初期化
  decon_count = 0;
  decon_auto_sequence = 0;

  // 昇降機構を制御
  rf_control();

  // 左コマンド & 左端のリミットスイッチが押されていないなら
  if (cmd_msg.mode[CMD_DECON_LEFT] && !ls_state[LS_DECON_LEFT]) {

    // 除染機構を左に移動
    decon_x_duty = -0.99;
  
  // 右コマンド & 右端のリミットスイッチが押されていないなら
  } else if(cmd_msg.mode[CMD_DECON_RIGHT] && !ls_state[LS_DECON_RIGHT]) {

    // 除染機構を右に移動
    decon_x_duty = 0.99;

  } else {

    // メイン昇降停止
    decon_x_duty = 0.0;

  }

  mdc.move(mdc_addr[MDC_DECON], decon_x_duty);
}

#define AUTO_DECON_FALL_MSEC 140
#define AUTO_DECON_RISE_MSEC  50

// 
unsigned long d_time = millis();

// 除染機構を自動制御
void decon_auto(void)
{
  // サブ昇降底のリミットスイッチが押されたらシーケンスを8までとばす
  if (ls_state[LS_RF_SUB_BOTTOM] == true) {
    decon_auto_sequence = 8;
  }

  switch(decon_auto_sequence) {

    case 0: // サブ昇降の頂のリミットスイッチが押されるまでサブ昇降を上昇
            if (ls_state[LS_RF_SUB_TOP] == false) {
              rf_sub_duty = -0.5;
            } else {
              rf_sub_duty = 0.0;
              decon_count = 0;
              decon_auto_sequence = 1;
            }
            mdc.move(mdc_addr[MDC_RF_SUB], rf_sub_duty);

            // 除染機構のモーターは停止
            decon_x_duty = 0.0;
            mdc.move(mdc_addr[MDC_DECON], decon_x_duty);
            break;

    case 1: // 除染機構の左のリミットスイッチが押されるまで除染機構を左に移動
            if (ls_state[LS_DECON_LEFT] == false) {
              decon_x_duty = -0.9;
            } else {
              decon_x_duty = 0.0;
              decon_auto_sequence = 2;
            }
            mdc.move(mdc_addr[MDC_DECON], decon_x_duty);
            break;

    case 2: // 右のリミットスイッチが押されるまで除染機構を右に移動
            if (ls_state[LS_DECON_RIGHT] == false) {
              decon_x_duty = 0.9;
            } else {
              decon_x_duty = 0.0;
              d_time = millis();
              decon_auto_sequence = 3;
            }
            mdc.move(mdc_addr[MDC_DECON], decon_x_duty);
            break;

    case 3: // 指定時間だけ下に下げる
            if (millis() - d_time < AUTO_DECON_FALL_MSEC) {
              rf_sub_duty = 0.3;
            } else {
              rf_sub_duty = 0.0;
              d_time = millis();
              decon_auto_sequence = 4;
            }
            mdc.move(mdc_addr[MDC_RF_SUB], rf_sub_duty);
            break;
    
    case 4: // 指定時間だけ上に上げる
            if (millis() - d_time < AUTO_DECON_RISE_MSEC) {
              rf_sub_duty = -0.2;
            } else {
              rf_sub_duty = 0.0;
              decon_auto_sequence = 5;
            }
            mdc.move(mdc_addr[MDC_RF_SUB], rf_sub_duty);
            break;
    
    case 5: // 左のリミットスイッチが押されるまで除染機構を左に移動
            if (ls_state[LS_DECON_LEFT] == false) {
              decon_x_duty = -0.9;
            } else {
              decon_x_duty = 0.0;
              d_time = millis();
              decon_auto_sequence = 6;
            }
            mdc.move(mdc_addr[MDC_DECON], decon_x_duty);
            break;
    
    case 6: // 指定時間だけ下に下げる
            if (millis() - d_time < AUTO_DECON_FALL_MSEC) {
              rf_sub_duty = 0.3;
            } else {
              rf_sub_duty = 0.0;
              d_time = millis();
              decon_auto_sequence = 7;              
            }
            mdc.move(mdc_addr[MDC_RF_SUB], rf_sub_duty);
            break;
    
    case 7: // 指定時間だけ上に上げる
            if (millis() - d_time < AUTO_DECON_RISE_MSEC) {
              rf_sub_duty = -0.2;
            } else {
              rf_sub_duty = 0.0;
              decon_count += 1;
              // 往復回数が15回未満なら
              if (decon_count < 15) {
                decon_auto_sequence = 2;
              // 往復回数が15回に達したら
              } else {
                decon_auto_sequence = 8;
              }
            }
            mdc.move(mdc_addr[MDC_RF_SUB], rf_sub_duty);
            break;

    case 8: // サブ昇降の頂のリミットスイッチが押されるまでサブ昇降を上昇
            if (ls_state[LS_RF_SUB_TOP] == false) {
              rf_sub_duty = -0.5;
              decon_x_duty = 0.0;
            } else {
              rf_sub_duty = 0.0;
              decon_count = 0;
              decon_auto_sequence = 9;
            }
            mdc.move(mdc_addr[MDC_RF_SUB], rf_sub_duty);
            mdc.move(mdc_addr[MDC_DECON], decon_x_duty);
            break;

    case 9: // モーターを停止
    default:
            rf_sub_duty = 0.0;
            mdc.move(mdc_addr[MDC_RF_SUB], rf_sub_duty);
            decon_x_duty = 0.0;
            mdc.move(mdc_addr[MDC_DECON], decon_x_duty);
  }
}

// 除染機構を制御
void decon_control(void)
{
  // 除染機構が手動モードなら
  if (cmd_msg.mode[CMD_DECON_AUTO] == false) {

    decon_manual();

  // 除染機構が自動モードなら
  } else if(cmd_msg.mode[CMD_DECON_AUTO] == true) {

    decon_auto();

  }
}

/* ----------------------------------------------------------- */

// LANケーブル用リールの仮想半径
#define REEL_RADIUS 40.5

// ケーブル巻取り・排出モーターのDuty比
double cable_duty = 0.0;

// ケーブルの排出距離(cm)
int16_t cable_length = 0;

// ケーブルの巻取り・排出を制御
void cable_control(void)
{
  // 排出しているケーブルの長さを計算
  cable_length = int(mdc.angle(mdc_addr[MDC_CABLE]) * REEL_RADIUS / -10.0 / 2);
  info_msg.cable_length = cable_length;

  // ケーブル巻取り・排出のモータのDuty比を計算
  cable_duty = cmd_msg.mode[CMD_CABLE_RELEASE] * 0.4 + cmd_msg.mode[CMD_CABLE_WIND] * -0.4;

  // ケーブル巻取り・排出のモーターを回転
  mdc.move(mdc_addr[MDC_CABLE], cable_duty);
}

/* ----------------------------------------------------------- */

unsigned long s_time = millis();

// cmdメッセージ受信時に発火するコールバック
void cmd_callback(const void * msgin)
{
    // 受信したメッセージを格納
    const custom_interfaces__msg__RobotCommand * msg = (const custom_interfaces__msg__RobotCommand *)msgin;

    // 最後にメッセージを受信した時間を記録
    s_time = millis();
}

/* ----------------------------------------------------------- */

// すべてのモータのDuty比をリセット
void duty_reset(void)
{
  crawler_left_duty = 0.0;
  crawler_right_duty = 0.0;
  cable_duty = 0.0;
  rf_main_duty = 0.0;
  rf_sub_duty = 0.0;
  decon_x_duty = 0.0;
  elecas_forward_duty = 0.0;
  elecas_back_duty = 0.0;

  for(int i = 0; i < 10; i++) {
    cmd_msg.mode[i] = false;
  }
  cmd_msg.value[0] = 0;
  cmd_msg.value[1] = 0;
}

/* ----------------------------------------------------------- */

int interrupt_count = 0;

// タイマー割り込みで実行する関数
void timer_interrupt(rcl_timer_t *timer, int64_t last_call_time)
{

  RCLC_UNUSED(last_call_time);

  // 割り込み回数を加算
  interrupt_count += 1;

  // 割り込み2回に1回
  if (interrupt_count % 2 == 0) {
    imu_control();
    crawler_control();
    decon_control();
    elecas_control();
    cable_control();
  }

  // 割り込み50回に1回
  if (interrupt_count % 50 == 0) {
    // RobotInfoメッセージをパブリッシュ
    rcl_publish(&info_publisher, &info_msg, NULL);
    interrupt_count = 0;
  }
}

/* ----------------------------------------------------------- */

void setup()
{
  Serial.begin(115200);

  pinMode(PB7, OUTPUT);
  pinMode(PB14, OUTPUT);

  // マイコンのMACアドレス(適当でOK)とIPアドレス(最後の数字だけ1~254のうちまだLAN内にない番号に変えること)
  byte mac_address[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
  IPAddress arduino_ip(192, 168, 0, 46);

  // AgentをたてたPCのIPアドレス(あらかじめ固定しておくと楽)とポート番号(8888か2000が主流だが何でもOK)
  IPAddress agent_ip(192, 168, 0, 10);
  int agent_port = 2000;

  // micro-ROSをセットアップする関数
  // Ethernet経由で通信する場合の引数はMACアドレス，マイコンのIPアドレス，AgentのIPアドレス，ポート番号
  set_microros_native_ethernet_udp_transports(mac_address, arduino_ip, agent_ip, agent_port);

  // セットアップが完了するまで少しの間待機
  delay(2000);

  // BNO055のセットアップ
  setup_bno055();

  // allocatorを宣言
  allocator = rcl_get_default_allocator();

  // 通信機能をサポートする構造体を宣言
  // 引数は サポートの構造体, argc, argv, allocator
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // nodeを作成
  // 引数は nodeの構造体, nodeの名前(分かりやすい名前でOK), 名前空間, サポートの構造体
  RCCHECK(rclc_node_init_default(&node, "robot_node", "", &support));

  RCCHECK(rclc_publisher_init_best_effort(&ls_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(custom_interfaces, msg, LimitSwitch), "ls"));
  RCCHECK(rclc_publisher_init_best_effort(&info_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(custom_interfaces, msg, RobotInfo), "info"));

  // subscriptionを作成
  // #引数は subscriptionの構造体, nodeの構造体, *メッセージサポート(型を合わせる必要あり), *topicの名前
  RCCHECK(rclc_subscription_init_default(&cmd_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(custom_interfaces, msg, RobotCommand), "cmd"));

  rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(10), timer_interrupt);

  // executorを作成
  // 引数は executorの構造体, サポートのコンテクスト, *ハンドル数(必要に応じて数を変える必要あり), allocator
  // ハンドル数：マイコンで処理するtopic, service, timerなどのコールバックの数
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));

  // executorにsubscriptionを追加(すべてのsubscriptionについて実行する必要あり)
  // 引数は executor, subscription, メッセージの変数, コールバック関数, オプション
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_subscriber, &cmd_msg, &cmd_callback, ON_NEW_DATA));  

  rclc_executor_add_timer(&executor, &timer);

  for(int i = 0; i < NUM_LS; i++)
  {
      pinMode(ls_pin[i], INPUT_PULLUP);
      delay(1);
  }

  // setup関数を抜けるときに赤色LED点灯
  digitalWrite(PB14, HIGH);
}

void loop()
{

  // リミットスイッチの状態チェック・更新
  ls_check();

  // 最後にRobotCommandメッセージを受信してから100ms以上経過したらDuty比を0にする(エレキャスは縮める)
  // スリップリング等の影響でPC-マイコン間通信が途絶えたときにロボットを停止させるための処理
  if (millis() - s_time > 100) {
    duty_reset();
  }

  // 10ms待機
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}