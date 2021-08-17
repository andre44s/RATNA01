//Variable Remote Drone
int32_t channel_1_start, channel_1;
int32_t channel_2_start, channel_2;
int32_t channel_3_start, channel_3;
int32_t channel_4_start, channel_4;
float spd = 0, backspd = 0;

int relay[4] = {PB15, PB14, PB13, PB12};
int millisOn = 80, millisOff = 2000, interval, currentMillis, previousMillis, pumpState = 0;

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

  for (int i = 0; i <= 3; i++) {
    pinMode(relay[i], OUTPUT);
    digitalWrite(relay[i], LOW);
  }
  previousMillis = millis();

  digitalWrite(relay[0], HIGH);
  digitalWrite(relay[1], HIGH);
  digitalWrite(relay[2], HIGH);
  digitalWrite(relay[3], HIGH);
}

void loop() {
  Serial.print(" 1:"); Serial.print(channel_1);
  Serial.print(" 2:"); Serial.print(channel_2);
  Serial.print(" 3:"); Serial.print(channel_3);
  Serial.print(" 4:"); Serial.println(channel_4);

  if (channel_1 <= 1000 && channel_1 > 900) {
    if (channel_2 <= 1000 && channel_2 > 900) {
      //Serial.println("OFF ALL");
      digitalWrite(relay[0], HIGH);
      digitalWrite(relay[1], HIGH);
      digitalWrite(relay[2], HIGH);
    }
    else if (channel_2 > 1000 && channel_2 < 2000) {
      //Middle Condition
    }
    else if (channel_2 >= 2000) {
      Serial.println("ON ALL");
      pumpSpray();
      digitalWrite(relay[1], LOW); //Ozone
      digitalWrite(relay[2], LOW); //UV
    }
  }

  else if (channel_1 > 1000 && channel_1 < 2000) {
    if (channel_2 <= 1000 && channel_2 > 900) {
      Serial.println("UV");
      digitalWrite(relay[0], HIGH); //DC Pump
      digitalWrite(relay[1], HIGH); //Ozone
      digitalWrite(relay[2], LOW); //UV
    }
    else if (channel_2 > 1000 && channel_2 < 2000) {
      Serial.println("Disinfectan");
      pumpSpray();
      digitalWrite(relay[1], HIGH); //Ozone
      digitalWrite(relay[2], HIGH); //UV
    }
    else if (channel_2 >= 2000) {
      Serial.println("Ozon");
      digitalWrite(relay[0], HIGH); //DC Pump
      digitalWrite(relay[1], LOW); //Ozone
      digitalWrite(relay[2], HIGH); //UV
    }
  }

  else if (channel_1 >= 2000) {
    if (channel_2 <= 1000 && channel_2 > 900) {
      Serial.println("UV Ozon");
      digitalWrite(relay[0], HIGH); //DC Pump
      digitalWrite(relay[1], LOW); //Ozone
      digitalWrite(relay[2], LOW); //UV
    }
    else if (channel_2 > 1000 && channel_2 < 2000) {
      Serial.println("UV Disinfectan");
      pumpSpray();
      digitalWrite(relay[1], HIGH); //Ozone
      digitalWrite(relay[2], LOW); //UV
    }
    else if (channel_2 >= 2000) {
      Serial.println("Ozon Disinfectan");
      pumpSpray();
      digitalWrite(relay[1], LOW); //Ozone
      digitalWrite(relay[2], HIGH); //UV
    }
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

void pumpSpray() {
  if (pumpState == 0) {
    interval = millisOff;
    digitalWrite(relay[0], HIGH); //DC Pump
  }
  else if (pumpState == 1) {
    interval = millisOn;
    digitalWrite(relay[0], LOW); //DC Pump
  }

  currentMillis = millis();
  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;
    pumpState = !pumpState;
    Serial.println(pumpState);
  }
}
