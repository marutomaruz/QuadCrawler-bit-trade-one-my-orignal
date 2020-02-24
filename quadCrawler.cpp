#include <stdint.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_NeoPixel.h>
#include <quadCrawler.h>

// ポート定義

#define Bz          8       //Ｂｅｅｐ音用ＰＩＮ定義

#define Echo        A0      //赤外線受信用ＰＩＮ定義
#define Trig        A1      //赤外線発信用ＰＩＮ定義

#define Moter_EN    A3      // Surbo Moter Drive Enable Pin

#define Neopix      A2      //ロボット基板上のＬＥＤコントロート用ＰＩＮ定義

//ＬＥＤコントロール宣言
Adafruit_NeoPixel strip = Adafruit_NeoPixel(8, Neopix, NEO_GRB + NEO_KHZ800);

//サーボコントロール宣言
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

//サーボ動作に関わる数値定義
#define servo_min   200     // Min pulse length out of 4096
#define servo_max   400     // Max pulse length out of 4096
#define deg_max     170     //使用可能最大角度
#define deg_min      10     //使用可能最小角度
//#define servo_min 200   // Min pulse length out of 4096
//#define servo_max 400   // Max pulse length out of 4096
//#define deg_max   130   //使用可能最大角度
//#define deg_min    20   //使用可能最小角度


//各サーボ定義

//腰
//Right Front Crach SV1-3(-7)
#define RFC       0             //サーボチャネル番号
#define RFC_AD    6             //ニュートラル時
#define RFC_N    85 + RFC_AD    //ニュートラル時
#define RFC_R    60 + RFC_AD    //後進時
#define RFC_F    90 + RFC_AD    //前進時

//Right Rear Crach SV3-4
#define RRC       4
#define RRC_AD   -5
#define RRC_N    85 + RRC_AD
#define RRC_R    80 + RRC_AD
#define RRC_F   110 + RRC_AD

//Left Front Crach SV2-4
#define LFC       2
#define LFC_AD  -10
#define LFC_N    85 + LFC_AD
#define LFC_R   110 + LFC_AD
#define LFC_F    80 + LFC_AD

//Left Rear Crach SV4-4
#define LRC       6
#define LRC_AD  -10
#define LRC_N    85 + LRC_AD
#define LRC_R    90 + LRC_AD
#define LRC_F    60 + LRC_AD



//足 Zero位置が８０として計算する
//Right Front Knee SV1-1 (-10)
#define RFK       1             //サーボチャネル番号
#define RFK_AD  -10             //サーボの個体差を埋める数値
#define RFK_N    70 + RFK_AD    //ニュートラル時
#define RFK_U    55 + RFK_AD    //通常の下げ足
#define RFK_D    80 + RFK_AD    //通常の揚げ足
#define RFK_UD  130 + RFK_AD    //最大伸び

//Right Rear Knee SV3-1(+0)
#define RRK       5
#define RRK_AD    0
#define RRK_N    70 + RRK_AD
#define RRK_U    55 + RRK_AD
#define RRK_D    80 + RRK_AD
#define RRK_UD  130 + RRK_AD

//Left Front Knee SV2-1(+9)     //右と左では回転方向が逆なので数値が逆になる
#define LFK       3
#define LFK_AD    9
#define LFK_N    90 + LFK_AD
#define LFK_U   105 + LFK_AD
#define LFK_D    80 + LFK_AD
#define LFK_UD   20 + LFK_AD

//Left Rear Knee SV4-1(-5)
#define LRK       7
#define LRK_AD   -5
#define LRK_N    90 + LRK_AD
#define LRK_U   105 + LRK_AD
#define LRK_D    80 + LRK_AD
#define LRK_UD   20 + LRK_AD




enum {
  ServoOff = 0,
  ServoNeutral,
  ServoNormal,
  ServoRepeat0,
  ServoRepeat1,
  ServoRepeat2,
  ServoRepeat3,
  ServoPose,
};

static uint8_t  servoState = 0;
static uint32_t servoTime = 0;
static uint16_t servoDelay = quadCrawler_fast;
static const uint8_t (*servoMotion)[8] = NULL;

static uint8_t cur_com = stop;

extern volatile unsigned long timer0_millis;


//-----------------------------------------------------------
//　指定されたIDのサーボをdegだけ動かす
static void set_servo_deg(uint8_t id, unsigned int deg) {

    //受け渡されたidをサーボチャネルに変換
    const uint8_t channel_table[] = { RFK, RFC, RRK, RRC, LFK, LFC, LRK, LRC };
    uint8_t channel = channel_table[id];

    //受け渡された角度をPWM値変換する式（何故map関数を使用しないんだろう．．．）
    unsigned int setdata = ( unsigned int )
                           (
                                ( deg       -   deg_min  ) 
                              * ( servo_max -   servo_min) 
                              / ( deg_max   -   deg_min  )
                           ) 
                         + servo_min;

    if (setdata <= servo_max) {     //サーボ可動最大値とのチェック

        if (setdata >= servo_min) {     //サーボ可動最小値とのチェック

            //可動範囲内だったら、サーボ駆動
            pwm.setPWM(channel, 0, setdata);
            delay(20);

        }
    }
}

static void set_servo_deg8(const uint8_t motion[8], uint8_t state) {
    uint8_t i;
    for (i = 0; i < 8; i++) {
        if (motion[i] != -1)
            set_servo_deg(i, motion[i]);
    }
    servoState = state;
    servoTime = timer0_millis;
}

static void set_servo_motion(const uint8_t(*motion)[8], uint8_t state) {
    servoMotion = motion;
    set_servo_deg8(motion[0], state);
}


static void set_servo_off8(uint8_t state) {

    uint8_t i;

    for (i = 0; i < 8; i++)
        pwm.setPWM(i, 0, 0);
    servoState = state;

    servoTime = timer0_millis;
    cur_com = stop;

}


static void sv_init() {
    const uint8_t motion[] = { RFK_N, RFC_N, RRK_N, RRC_N, LFK_N, LFC_N, LRK_N, LRC_N };
    set_servo_motion(&motion, ServoNeutral);
}

static const uint8_t angle_table[5][8] = {
    {-1,    -1,    -1,    -1,    -1,    -1,    -1,    -1},			// keep
    {RFK_N, RFC_N, RRK_N, RRC_N, LFK_N, LFC_N, LRK_N, LRC_N},		// neutral
    {RFK_U, RFC_R, RRK_U, RRC_R, LFK_U, LFC_R, LRK_U, LRC_R},		// up/rear
    {RFK_D, RFC_F, RRK_D, RRC_F, LFK_D, LFC_F, LRK_D, LRC_F},		// down/forward
    {RFK_UD,RFC_N, RRK_UD,RRC_N, LFK_UD,LFC_N, LRK_UD,LRC_N},		// downMax
};

void quadCrawler_setPose4(uint8_t rfk, uint8_t rfc, uint8_t rrk, uint8_t rrc, uint8_t lfk, uint8_t lfc, uint8_t lrk, uint8_t lrc) {
    uint8_t motion[8];
    motion[0] = angle_table[rfk][0];
    motion[1] = angle_table[rfc][1];
    motion[2] = angle_table[rrk][2];
    motion[3] = angle_table[rrc][3];
    motion[4] = angle_table[lfk][4];
    motion[5] = angle_table[lfc][5];
    motion[6] = angle_table[lrk][6];
    motion[7] = angle_table[lrc][7];
    set_servo_motion(&motion, ServoPose);
    cur_com = pose;
}

void quadCrawler_setPose1(uint8_t index, uint8_t knee, uint8_t crach){

    if (index >= 4) return;

    uint8_t motion[8] = { -1, -1, -1, -1, -1, -1, -1, -1 };
    motion[index * 2 + 0] = angle_table[knee][index * 2 + 0];
    motion[index * 2 + 1] = angle_table[crach][index * 2 + 1];
    set_servo_motion(&motion, ServoPose);
    cur_com = pose;
}

uint8_t quadCrawler_checkServoON(void){

    return (servoState != ServoOff);
}

void quadCrawler_servoLoop(void){

    uint32_t elapsed = timer0_millis - servoTime;
    // サーボOFFか、サーボHoldか、delay時間が経過してないとき
    switch (servoState) {
    case ServoOff:
        return;

    case ServoPose:
    case ServoNormal:
        if (elapsed >= 180 * 1000UL) {
            set_servo_off8(ServoOff);
            digitalWrite(13, 0);
            return;
        }
        if ((elapsed >> 8) & 1)      // 256
            digitalWrite(13, 1);
        else
            digitalWrite(13, 0);
        return;

    default:
        if (elapsed < servoDelay)
            return;
        break;
    }

    // normal-なにもしない、repeat-動作を繰り返し
    switch (servoState) {
      case ServoRepeat0:
        set_servo_deg8(servoMotion[1], ServoRepeat1);
        break;
      case ServoRepeat1:
        set_servo_deg8(servoMotion[2], ServoRepeat2);
        break;
      case ServoRepeat2:
        set_servo_deg8(servoMotion[3], ServoRepeat3);
        break;
      case ServoRepeat3:
        set_servo_deg8(servoMotion[0], ServoRepeat0);
        break;
      case ServoNeutral:
        set_servo_off8(ServoOff);
        break;
    }
}

void quadCrawler_setSpeed(uint16_t speed) {
    servoDelay = speed;
}

void quadCrawler_Walk(uint16_t speed, uint8_t com) {
    servoDelay = speed;

    if (cur_com == com) return;
    cur_com = com;

    if (com == stop) {
        const uint8_t motion[] = { RFK_N, RFC_N, RRK_N, RRC_N, LFK_N, LFC_N, LRK_N, LRC_N };

        // normal, repeatのとき : neutral姿勢
        if (servoState != ServoNeutral && servoState != ServoOff) {
            set_servo_motion(&motion, ServoNeutral);
        }
    }
    else if (com == fw) {
        //Serial.println ("-fw");
        static const uint8_t motion[4][8] = {
                               {-1,    RFC_F, -1,    RRC_R, -1,    LFC_R, -1,    LRC_F},
                               {RFK_D, -1,    RRK_U, -1,    LFK_U, -1,    LRK_D, -1,  },
                               {-1,    RFC_R, -1,    RRC_F, -1,    LFC_F, -1,    LRC_R},
                               {RFK_U, -1,    RRK_D, -1,    LFK_D, -1,    LRK_U, -1,  } };
        set_servo_motion(motion, ServoRepeat0);
    }
    else if (com == rw) {
        //Serial.println ("-rw");
        static const uint8_t motion[4][8] = {
                               {-1,    RFC_F, -1,    RRC_R, -1,    LFC_R, -1,    LRC_F},
                               {RFK_U, -1,    RRK_D, -1,    LFK_D, -1,    LRK_U, -1,  },
                               {-1,    RFC_R, -1,    RRC_F, -1,    LFC_F, -1,    LRC_R},
                               {RFK_D, -1,    RRK_U, -1,    LFK_U, -1,    LRK_D, -1,  } };
        set_servo_motion(motion, ServoRepeat0);
    }
    else if (com == cw) {
        //Serial.println ("-cw");
        static const uint8_t motion[4][8] = {
                               {-1,    RFC_F, -1,    RRC_R, -1,    LFC_F, -1,    LRC_R},
                               {RFK_U, -1,    RRK_D, -1,    LFK_D, -1,    LRK_U, -1,  },
                               {-1,    RFC_R, -1,    RRC_F, -1,    LFC_R, -1,    LRC_F},
                               {RFK_D, -1,    RRK_U, -1,    LFK_U, -1,    LRK_D, -1,  } };
        set_servo_motion(motion, ServoRepeat0);
    }
    else if (com == ccw) {
        //Serial.println ("-ccw");
        static const uint8_t motion[4][8] = {
                               {-1,    RFC_F, -1,    RRC_R, -1,    LFC_F, -1,    LRC_R},
                               {RFK_D, -1,    RRK_U, -1,    LFK_U, -1,    LRK_D, -1,  },
                               {-1,    RFC_R, -1,    RRC_F, -1,    LFC_R, -1,    LRC_F},
                               {RFK_U, -1,    RRK_D, -1,    LFK_D, -1,    LRK_U, -1,  } };
        set_servo_motion(motion, ServoRepeat0);
    }
    else if (com == Rigt) {
        //Serial.println ("-right");
        static const uint8_t motion[4][8] = {
                               {-1,    RFC_F, -1,    RRC_F, -1,    LFC_F, -1,    LRC_F},
                               {RFK_U, -1,    RRK_D, -1,    LFK_D, -1,    LRK_U, -1,  },
                               {-1,    RFC_R, -1,    RRC_R, -1,    LFC_R, -1,    LRC_R},
                               {RFK_D, -1,    RRK_U, -1,    LFK_U, -1,    LRK_D, -1,  } };
        set_servo_motion(motion, ServoRepeat0);
    }
    else if (com == Left) {
        //Serial.println ("-right");
        static const uint8_t motion[4][8] = {
                               {-1,    RFC_F, -1,    RRC_F, -1,    LFC_F, -1,    LRC_F},
                               {RFK_D, -1,    RRK_U, -1,    LFK_U, -1,    LRK_D, -1   },
                               {-1,    RFC_R, -1,    RRC_R, -1,    LFC_R, -1,    LRC_R},
                               {RFK_U, -1,    RRK_D, -1,    LFK_D, -1,    LRK_U, -1   } };
        set_servo_motion(motion, ServoRepeat0);
    }
    /*
      else if (com == nt) {
        //Serial.println ("-nutral");
        const uint8_t motion[] = {RFK_N, RFC_N, RRK_N, RRC_N, LFK_N, LFC_N, LRK_N, LRC_N};
        set_servo_motion(&motion, ServoNormal);
      }
    */
    else if (com == all_up) {
        //Serial.println ("-all up");
        const uint8_t motion[] = { RFK_U, -1,    RRK_U, -1,    LFK_U, -1,    LRK_U, -1 };
        set_servo_motion(&motion, ServoNormal);
    }
    else if (com == all_dn) {
        //Serial.println ("-all down");
        const uint8_t motion[] = { RFK_UD,-1,    RRK_UD,-1,    LFK_UD,-1,    LRK_UD,-1 };
        set_servo_motion(&motion, ServoNormal);
    }
    else if (com == t_dn) {
        //Serial.println ("-Hip up");
        const uint8_t motion[] = { RFK_U, -1,    RRK_UD,-1,    LFK_U, -1,    LRK_UD,-1 };
        set_servo_motion(&motion, ServoNormal);
    }
    else if (com == h_dn) {
        //Serial.println ("-Top up");
        const uint8_t motion[] = { RFK_UD,-1,    RRK_U, -1,    LFK_UD,-1,    LRK_U, -1 };
        set_servo_motion(&motion, ServoNormal);
    }
    else if (com == l_dn) {
        //Serial.println ("-Right up");
        const uint8_t motion[] = { RFK_U, -1,    RRK_U, -1,    LFK_UD,-1,    LRK_UD,-1 };
        set_servo_motion(&motion, ServoNormal);
    }
    else if (com == r_dn) {
        //Serial.println ("Left up");
        const uint8_t motion[] = { RFK_UD,-1,    RRK_UD,-1,    LFK_U, -1,    LRK_U, -1 };
        set_servo_motion(&motion, ServoNormal);
    }
    else if (com == t_up_dn) {
        static const uint8_t motion[4][8] = {
                               {RFK_U, -1,    RRK_U, -1,    LFK_U, -1,    LRK_U, -1   },
                               {RFK_U, -1,    RRK_UD,-1,    LFK_U, -1,    LRK_UD,-1   },
                               {RFK_U, -1,    RRK_U, -1,    LFK_U, -1,    LRK_U, -1   },
                               {RFK_U, -1,    RRK_UD,-1,    LFK_U, -1,    LRK_UD,-1   } };
        set_servo_motion(motion, ServoRepeat0);
    }
    else if (com == l_r_up) {
        static const uint8_t motion[4][8] = {
                               {RFK_UD,-1,    RRK_UD,-1,    LFK_U, -1,    LRK_U, -1   },
                               {RFK_U, -1,    RRK_U, -1,    LFK_UD,-1,    LRK_UD,-1   },
                               {RFK_UD,-1,    RRK_UD,-1,    LFK_U, -1,    LRK_U, -1   },
                               {RFK_U, -1,    RRK_U, -1,    LFK_UD,-1,    LRK_UD,-1   } };
        set_servo_motion(motion, ServoRepeat0);
    }
    else if (com == all_up_dn) {
        static const uint8_t motion[4][8] = {
                               {RFK_U, -1,    RRK_U, -1,    LFK_U, -1,    LRK_U, -1   },
                               {RFK_UD,-1,    RRK_UD,-1,    LFK_UD,-1,    LRK_UD,-1   },
                               {RFK_U, -1,    RRK_U, -1,    LFK_U, -1,    LRK_U, -1   },
                               {RFK_UD,-1,    RRK_UD,-1,    LFK_UD,-1,    LRK_UD,-1   } };
        set_servo_motion(motion, ServoRepeat0);
    }
    else {

    }
}


//-----------------------------------------------------------
//ロボット基板上のＬＥＤの発色の処理

//-----------------------------------------------------------
//　ＬＥＤの発光
static void colorWipe(uint32_t c, uint8_t wait) {

    for (uint16_t i = 0; i < strip.numPixels(); i++) {

        strip.setPixelColor(i, c);
        strip.show();
        delay(wait);

    }

}

//-----------------------------------------------------------
//　発光する色の決定
void quadCrawler_colorWipe(uint8_t color) {

    switch (color) {
      case COLOR_RED:
        colorWipe(strip.Color(255, 0, 0), 10);
        break;
      
      case COLOR_GREEN:
        colorWipe(strip.Color(0, 255, 0), 10);
        break;
      
      case COLOR_BLUE:
        colorWipe(strip.Color(0, 0, 255), 10);
        break;
      
      case COLOR_YELLOW:
        colorWipe(strip.Color(128, 128, 0), 10);
        break;
      
      case COLOR_PURPLE:
        colorWipe(strip.Color(128, 0, 128), 10);
        break;
      
      case COLOR_LIGHTBLUE:
        colorWipe(strip.Color(0, 128, 128), 10);
        break;
      
      default:
        break;
    
    }
}

//-----------------------------------------------------------
//　虹色を取得する関数
static uint32_t Wheel(byte WheelPos) {

    WheelPos = 255 - WheelPos;

    if (WheelPos < 85) {
        return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
    }

    if (WheelPos < 170) {
        WheelPos -= 85;
        return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
    }

    WheelPos -= 170;

    return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);

}

//-----------------------------------------------------------
//　ＬＥＤを未地色に光らせる
void quadCrawler_rainbow(uint8_t wait) {
    uint16_t i, j;

    for (j = 0; j < 256; j++) {

        for (i = 0; i < strip.numPixels(); i++) {
            strip.setPixelColor(i, Wheel((i + j) & 255));
        }

        strip.show();
        delay(wait);

    }
}

//-----------------------------------------------------------
//赤外線センサー値を取得する関数
double quadCrawler_getSonner() {

    double data;
    double distance;

    //赤外線、発射！
    digitalWrite(Trig, LOW);
    delayMicroseconds(2);
    digitalWrite(Trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(Trig, LOW);

    //赤外線、受信！
    data = pulseIn(Echo, HIGH);
    if (data > 0) {
        distance = data * 0.017;
        return distance;
    }
    else {
        return 0;
    }
}


//-----------------------------------------------------------
//　Ｂｅｅｐ音発信（説明するまでもないね）
void quadCrawler_beep(int time) {

    for (int i = 0; i < time; i++) {
        digitalWrite(Bz, HIGH);
        delayMicroseconds(400);
        digitalWrite(Bz, LOW);
        delayMicroseconds(400);
    }
}

//-----------------------------------------------------------
//　電源ＯＮ時の初期化処理

void quadCrawler_init(void){

    pinMode(Moter_EN, OUTPUT);
    digitalWrite(Moter_EN, HIGH);

    pinMode(Echo, INPUT);       //赤外線受信準備
    pinMode(Trig, OUTPUT);      //赤外線発射準備

    pinMode(Bz, OUTPUT);        //Ｂｅｅｐ音準備

    pinMode(Sw1, INPUT_PULLUP); //Ｓｗプルアップで準備
    pinMode(Sw2, INPUT_PULLUP);
    pinMode(Sw3, INPUT_PULLUP);
    pinMode(Sw4, INPUT_PULLUP);

    //ロボット基板上のＬＥＤ操作開始宣言
    strip.begin();
    strip.show();   //一旦全消し

    //サーボ使用宣言
    pwm.begin();
    pwm.setPWMFreq(50);  // Analog servos run at ~50 Hz updates

    yield();

    digitalWrite(Moter_EN, LOW);

    sv_init();

}

