#include <Arduino.h>
#include "driver/pcnt.h"

//////////////////////
// L298N FRONT
//////////////////////
const int M1_IN1=25, M1_IN2=26, M1_EN=32;
const int M2_IN1=27, M2_IN2=14, M2_EN=33;

//////////////////////
// L298N REAR
//////////////////////
const int M3_IN1=12, M3_IN2=13, M3_EN=21;
const int M4_IN1=22, M4_IN2=23, M4_EN=5;

//////////////////////
// PWM
//////////////////////
const int PWM_FREQ=20000;
const int PWM_RES=10;
const int CH[4]={0,1,2,3};

//////////////////////
// ENCODER
//////////////////////
const int EA[4]={34,39,18,16};
const int EB[4]={35,36,19,17};

pcnt_unit_t encUnit[4]={
  PCNT_UNIT_0,PCNT_UNIT_1,
  PCNT_UNIT_2,PCNT_UNIT_3
};

void setupEncoder(int i){
  pcnt_config_t cfg={};
  cfg.pulse_gpio_num=EA[i];
  cfg.ctrl_gpio_num=EB[i];
  cfg.unit=encUnit[i];
  cfg.channel=PCNT_CHANNEL_0;
  cfg.pos_mode=PCNT_COUNT_INC;
  cfg.neg_mode=PCNT_COUNT_DEC;
  cfg.lctrl_mode=PCNT_MODE_REVERSE;
  cfg.hctrl_mode=PCNT_MODE_KEEP;
  cfg.counter_h_lim=30000;
  cfg.counter_l_lim=-30000;
  pcnt_unit_config(&cfg);

  cfg.pulse_gpio_num=EB[i];
  cfg.ctrl_gpio_num=EA[i];
  cfg.channel=PCNT_CHANNEL_1;
  cfg.pos_mode=PCNT_COUNT_DEC;
  cfg.neg_mode=PCNT_COUNT_INC;
  pcnt_unit_config(&cfg);

  pcnt_counter_pause(encUnit[i]);
  pcnt_counter_clear(encUnit[i]);
  pcnt_counter_resume(encUnit[i]);
}

long readEncoder(int i){
  int16_t c;
  pcnt_get_counter_value(encUnit[i], &c);
  return (long)c;
}

void motorSet(int in1,int in2,int ch,int speed,bool dir){
  digitalWrite(in1,dir);
  digitalWrite(in2,!dir);
  speed=constrain(speed,0,1023);
  ledcWrite(ch,speed);
}

void setup(){
  Serial.begin(115200);

  pinMode(M1_IN1,OUTPUT); pinMode(M1_IN2,OUTPUT);
  pinMode(M2_IN1,OUTPUT); pinMode(M2_IN2,OUTPUT);
  pinMode(M3_IN1,OUTPUT); pinMode(M3_IN2,OUTPUT);
  pinMode(M4_IN1,OUTPUT); pinMode(M4_IN2,OUTPUT);

  for(int i=0;i<4;i++){
    ledcSetup(CH[i],PWM_FREQ,PWM_RES);
  }

  ledcAttachPin(M1_EN,CH[0]);
  ledcAttachPin(M2_EN,CH[1]);
  ledcAttachPin(M3_EN,CH[2]);
  ledcAttachPin(M4_EN,CH[3]);

  for(int i=0;i<4;i++){
    pinMode(EA[i],INPUT);
    pinMode(EB[i],INPUT);
    setupEncoder(i);
  }

  Serial.println("L298N Ready");
}

unsigned long lastPrint=0;

void loop(){

  // เดินหน้าตลอด
  motorSet(M1_IN1,M1_IN2,CH[0],800,true);
  motorSet(M2_IN1,M2_IN2,CH[1],800,true);
  motorSet(M3_IN1,M3_IN2,CH[2],800,true);
  motorSet(M4_IN1,M4_IN2,CH[3],800,true);

  if(millis()-lastPrint>100){
    lastPrint=millis();
    Serial.print("ENC:\t");
    for(int i=0;i<4;i++){
      Serial.print(readEncoder(i));
      Serial.print("\t");
    }
    Serial.println();
  }
}
