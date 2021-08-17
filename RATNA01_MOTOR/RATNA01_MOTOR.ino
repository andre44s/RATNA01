#include "HardwareTimer.h"

//set frekuensi
#define freq 20000
#define freqCPU (72000000/freq)
#define MAPfreq1 labs((pwmValue1/65535) * (freqCPU))
#define MAPfreq2 labs((pwmValue2/65535) * (freqCPU))
HardwareTimer pwmtimer(2);

//Variable Remote Drone
int32_t channel_1_start, channel_1;
int32_t channel_2_start, channel_2;
int32_t channel_3_start, channel_3;
int32_t channel_4_start, channel_4;
float spd = 0, backspd = 0;

//Variable Jarak Detector
const int sensorDepan = PA15;
const int sensorBelakang = PB3;
const int sensorKanan = PB4;
const int sensorKiri = PB5;

bool stateDepan = false;
bool stateBelakang = false;
bool stateKanan = false;
bool stateKiri = false;

//Variable Motor
const int enc1a = PA8; // Pin Encoder 1
const int enc1b = PA9; // Pin Encoder 2
const int mpwm1a = PA3; // Pin PWM 1
const int mpwm1b = PA2; // Pin PWM 2
const int mtr1a = PB12; // Pin Motor 1
const int mtr1b = PB13; // Pin Motor 2

const int enc2a = PB6; // Pin Encoder 1
const int enc2b = PB7; // Pin Encoder 2
const int mpwm2a = PA1; // Pin PWM 1
const int mpwm2b = PA0; // Pin PWM 2
const int mtr2a = PB14; // Pin Motor 1
const int mtr2b = PB15; // Pin Motor 2

//Encoder
#define ENC_COUNT_REV 2520 //Jumlah Gigi Encoder
long previousMillis = 0; // Waktu Sebelumnya
long currentMillis = 0; // Waktu Sekarang
long revolutions1 = 0; // Putaran dalam satuan waktu
long revolutions2 = 0; // Putaran dalam satuan waktu
unsigned long interval = 100; //Satuan Waktu untuk revolution1
float speedMotor1;
double valMotor1;
float speedMotor2;
double valMotor2;
int mtrdir1;
int mtrdir2;

//Jangan diubah-ubah
float PIDValue1 = 0, pwmValue1 = 0;
double error1, previouserror1;
double P_1 = 0, I_1 = 0, D_1 = 0;
float PIDValue2 = 0, pwmValue2 = 0;
double error2, previouserror2;
double P_2 = 0, I_2 = 0, D_2 = 0;

int a = 0;
int b = 0;
int c = 0;

void setup() {
  Serial.begin(115200);

  Timer3.attachCompare1Interrupt(handler_channel_1);
  Timer3.attachCompare2Interrupt(handler_channel_2);
  Timer3.attachCompare3Interrupt(handler_channel_3);
  Timer3.attachCompare4Interrupt(handler_channel_4);
  TIMER3_BASE->CR1 = TIMER_CR1_CEN;
  TIMER3_BASE->CR2 = 0;
  TIMER3_BASE->SMCR = 0;
  TIMER3_BASE->DIER = TIMER_DIER_CC1IE | TIMER_DIER_CC2IE | TIMER_DIER_CC3IE | TIMER_DIER_CC4IE;
  TIMER3_BASE->EGR = 0;
  TIMER3_BASE->CCMR1 = 0b100000001; //Register is set like this due to a bug in the define table (30-09-2017)
  TIMER3_BASE->CCMR2 = 0b100000001;
  TIMER3_BASE->CCER = TIMER_CCER_CC1E | TIMER_CCER_CC2E | TIMER_CCER_CC3E | TIMER_CCER_CC4E;
  TIMER3_BASE->PSC = 71;
  TIMER3_BASE->ARR = 0xFFFF;
  TIMER3_BASE->DCR = 0;

  //Motor 1
  pinMode(enc1a, INPUT);    //Pin Encoder Motor 1
  pinMode(enc1b, INPUT);    //Pin Encoder Motor 1
  pinMode(mpwm1a, PWM);      //Pin PWM Motor 1
  pinMode(mpwm1b, PWM);      //Pin PWM Motor 1
  pinMode(mtr1a, OUTPUT);      //Pin Digital Motor 1
  pinMode(mtr1b, OUTPUT);      //Pin Digital Motor 1
  //Motor 2
  pinMode(enc2a, INPUT);    //Pin Encoder Motor 2
  pinMode(enc2b, INPUT);    //Pin Encoder Motor 2
  pinMode(mpwm2a, PWM);      //Pin PWM Motor 2
  pinMode(mpwm2b, PWM);      //Pin PWM Motor 2
  pinMode(mtr2a, OUTPUT);      //Pin Digital Motor 2
  pinMode(mtr2b, OUTPUT);      //Pin Digital Motor 2

  previousMillis = millis();

  attachInterrupt(digitalPinToInterrupt(enc1a), func1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc1b), func1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc2a), func2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc2b), func2, CHANGE);

  //set frekuensi
  pwmtimer.setPrescaleFactor(1);
  pwmtimer.setOverflow(freqCPU);
}

void loop() {
  bacaRemote();
  bacaSensor();

  if (c == 1) {
    speedMotor1 = a;
    speedMotor2 = b;
    c = 0;
  }

  currentMillis = millis();
  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;

    // Calculate RPM
    valMotor1 = (float)(revolutions1 * 600 / ENC_COUNT_REV);
    valMotor2 = (float)(revolutions2 * 600 / ENC_COUNT_REV);

    VPID1();
    VPID2();

    revolutions1 = 0;
    revolutions2 = 0;

    if (valMotor1 == 0 && (speedMotor1 != 0) && (pwmValue1 > 65534 || pwmValue1 < -65534)) {
      pwmValue1 = 0;
      speedMotor1 = 0;
    }
    if (valMotor2 == 0 && (speedMotor2 != 0) && (pwmValue2 > 65534 || pwmValue2 < -65534)) {
      pwmValue2 = 0;
      speedMotor2 = 0;
    }

    speedMotor1 = 100;
    speedMotor2 = 100;

    //    Serial.print(valMotor1);
    //    Serial.print("\t");
    //    Serial.print(valMotor2);
    //    Serial.print("\t");
    //    Serial.print(MAPfreq1);
    //    Serial.print("\t");
    //    Serial.println(MAPfreq2);

    Serial.print(channel_1);
    Serial.print("  ");
    Serial.print(channel_2);
    Serial.print("  ");
    Serial.print(channel_3);
    Serial.print("  ");
    Serial.println(channel_4);
  }
}

void bacaSensor() {
  if (digitalRead(sensorDepan)) {
    stateDepan = true;
  }
  else {
    stateDepan = false;
  }

  if (digitalRead(sensorBelakang)) {
    stateBelakang = true;
  }
  else {
    stateBelakang = false;
  }

  if (digitalRead(sensorKanan)) {
    stateKanan = true;
  }
  else {
    stateKanan = false;
  }

  if (digitalRead(sensorKiri)) {
    stateKiri = true;
  }
  else {
    stateKiri = false;
  }
}

void bacaRemote() {
  if (channel_1 > 900 && channel_1 < 2100 && channel_2 > 900 && channel_2 < 2100 && channel_3 > 900 && channel_3 < 2100 && channel_4 > 900 && channel_4 < 2100) {
    //Baca Speed
    spd = map(channel_1, 990, 2014, 0, 100);
    backspd = spd * -1;
    //Putar Kanan
    if (channel_2 > 1600 && stateKanan == true) {
      a = backspd;
      b = backspd;
      c = 1;
      //Serial.println("Putar Kanan");
    }

    //Putar Kiri
    if (channel_2 < 1400 && stateKiri == true) {
      a = spd;
      b = spd;
      c = 1;
      //Serial.println("Putar Kiri");
    }

    //Maju
    if (channel_3 > 1600 && stateDepan == true) {
      speedMotor1 = backspd;
      speedMotor2 = spd;
      //Serial.println("Maju");
    }

    //Mundur
    if (channel_3 < 1400 && stateBelakang == true) {
      speedMotor1 = spd;
      speedMotor2 = backspd;
      //Serial.println("Mundur");
    }

    //Maju Kanan
    if (channel_3 > 1600 && channel_4 > 1600 && stateDepan == true && stateKiri == true) {
      speedMotor1 = backspd;
      speedMotor2 = spd / 2;
      //Serial.println("Maju Kanan");
    }

    //Maju Kiri
    if (channel_3 > 1600 && channel_4 < 1400 && stateDepan == true && stateKanan == true) {
      speedMotor1 = backspd / 2;
      speedMotor2 = spd;
      //Serial.println("Maju Kiri");
    }

    //Mundur Kanan
    if (channel_3 < 1400 && channel_4 > 1600 && stateBelakang == true && stateKanan == true) {
      speedMotor1 = spd;
      speedMotor2 = backspd / 2;
      //Serial.println("Mundur Kanan");
    }

    //Mundur Kiri
    if (channel_3 < 1400 && channel_4 < 1400 && stateBelakang == true && stateKiri == true) {
      speedMotor1 = spd / 2;
      speedMotor2 = backspd;
      //Serial.println("Mundur Kiri");
    }

    //diam
    if (channel_2 >= 1400 && channel_2 <= 1600 && channel_3 >= 1400 && channel_3 <= 1600 && channel_4 >= 1400 && channel_4 <= 1600) {
      speedMotor1 = 0;
      speedMotor2 = 0;
    }
  }

  else {
    speedMotor1 = 0;
    speedMotor2 = 0;
    //Serial.println("FailSafe");
  }
}

void func1() {
  revolutions1++;
}

void func2() {
  revolutions2++;
}

float Kp1 = 400;
float Ki1 = 0;
float Kd1 = 100;
float Kp2 = 400;
float Ki2 = 0;
float Kd2 = 100;

void VPID1() {
  error1 = abs(speedMotor1) - valMotor1;
  P_1 = error1;
  I_1 = I_1 + error1;
  D_1 = error1 - previouserror1;

  if (I_1 > 65535) {
    I_1 = 65535;
  }
  if (I_1 < 0) {
    I_1 = 0;
  }

  PIDValue1 = (Kp1 * P_1) + (Ki1 * I_1) + (Kd1 * D_1);
  previouserror1 = error1;
  pwmValue1 = pwmValue1 + PIDValue1;

  if (pwmValue1 >= 65535) {
    pwmValue1 = 65535;
  }
  if (pwmValue1 <= 0) {
    pwmValue1 = 0;
  }

  if (speedMotor1 > 0) {
    pwmWrite(mpwm1a, MAPfreq1);
    pwmWrite(mpwm1b, 0);
    digitalWrite(mtr1a, HIGH);
    digitalWrite(mtr1b, HIGH);
  }
  else if (speedMotor1 < 0) {
    pwmWrite(mpwm1a, 0);
    pwmWrite(mpwm1b, abs(MAPfreq1));
    digitalWrite(mtr1a, HIGH);
    digitalWrite(mtr1b, HIGH);
  }
  if (speedMotor1 == 0) {
    PIDValue1 = 0;
    pwmValue1 = 0;
    pwmWrite(mpwm1a, 0);
    pwmWrite(mpwm1b, 0);
    digitalWrite(mtr1a, LOW);
    digitalWrite(mtr1b, LOW);
  }
}

void VPID2() {
  error2 = abs(speedMotor2) - valMotor2;
  P_2 = error2;
  I_2 = I_2 + error2;
  D_2 = error2 - previouserror2;

  if (I_2 > 65535) {
    I_2 = 65535;
  }
  if (I_2 < 0) {
    I_2 = 0;
  }

  PIDValue2 = (Kp2 * P_2) + (Ki2 * I_2) + (Kd2 * D_2);
  previouserror2 = error2;
  pwmValue2 = pwmValue2 + PIDValue2;

  if (pwmValue2 >= 65535) {
    pwmValue2 = 65535;
  }
  if (pwmValue2 <= 0) {
    pwmValue2 = 0;
  }

  if (speedMotor2 > 0) {
    pwmWrite(mpwm2a, MAPfreq2);
    pwmWrite(mpwm2b, 0);
    digitalWrite(mtr2a, HIGH);
    digitalWrite(mtr2b, HIGH);
  }
  else if (speedMotor2 < 0) {
    pwmWrite(mpwm2a, 0);
    pwmWrite(mpwm2b, abs(MAPfreq2));
    digitalWrite(mtr2a, HIGH);
    digitalWrite(mtr2b, HIGH);
  }
  if (speedMotor2 == 0) {
    PIDValue2 = 0;
    pwmValue2 = 0;
    pwmWrite(mpwm2a, 0);
    pwmWrite(mpwm2b, 0);
    digitalWrite(mtr2a, LOW);
    digitalWrite(mtr2b, LOW);
  }
}

void handler_channel_1(void) {
  if (0b1 & GPIOA_BASE->IDR >> 6) {
    channel_1_start = TIMER3_BASE->CCR1;
    TIMER3_BASE->CCER |= TIMER_CCER_CC1P;
  }
  else {
    channel_1 = TIMER3_BASE->CCR1 - channel_1_start;
    if (channel_1 < 0)channel_1 += 0xFFFF;
    TIMER3_BASE->CCER &= ~TIMER_CCER_CC1P;
  }
}

void handler_channel_2(void) {
  if (0b1 & GPIOA_BASE->IDR >> 7) {
    channel_2_start = TIMER3_BASE->CCR2;
    TIMER3_BASE->CCER |= TIMER_CCER_CC2P;
  }
  else {
    channel_2 = TIMER3_BASE->CCR2 - channel_2_start;
    if (channel_2 < 0)channel_2 += 0xFFFF;
    TIMER3_BASE->CCER &= ~TIMER_CCER_CC2P;
  }
}

void handler_channel_3(void) {
  if (0b1 & GPIOB_BASE->IDR >> 0) {
    channel_3_start = TIMER3_BASE->CCR3;
    TIMER3_BASE->CCER |= TIMER_CCER_CC3P;
  }
  else {
    channel_3 = TIMER3_BASE->CCR3 - channel_3_start;
    if (channel_3 < 0)channel_3 += 0xFFFF;
    TIMER3_BASE->CCER &= ~TIMER_CCER_CC3P;
  }
}

void handler_channel_4(void) {
  if (0b1 & GPIOB_BASE->IDR >> 1) {
    channel_4_start = TIMER3_BASE->CCR4;
    TIMER3_BASE->CCER |= TIMER_CCER_CC4P;
  }
  else {
    channel_4 = TIMER3_BASE->CCR4 - channel_4_start;
    if (channel_4 < 0)channel_4 += 0xFFFF;
    TIMER3_BASE->CCER &= ~TIMER_CCER_CC4P;
  }
}
