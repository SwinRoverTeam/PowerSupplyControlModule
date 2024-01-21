#include <SPI.h>
#include "mcp_can.h"
#include "SwinCAN.h"

const int SPI_CS_PIN = 17;              // CANBed V1

#define ControlElec 4
#define MotorLeft 5
#define MotorRight 6
#define Arm1 8
#define Arm2 9

#define HBPeriod 1000

unsigned long MotorLast = 0;
unsigned long ArmLast = 0;
unsigned long cubeLast = 0;
unsigned long lastHB = 0;

MCP_CAN CAN(SPI_CS_PIN); 

bool ElecTog = false;
bool MLTog = false;
bool MRTog = false;
bool A1Tog = false;
bool A2Tog = false;
bool timerSet = false;

void setup() {
  pinMode(ControlElec, OUTPUT);
  pinMode(MotorLeft, OUTPUT); 
  pinMode(MotorRight, OUTPUT); 
  pinMode(Arm1, OUTPUT); 
  pinMode(Arm2, OUTPUT);
  //Pins initalised 
  digitalWrite(ControlElec, HIGH); // Turns on 5.3V rail

  while (CAN_OK != CAN.begin(CAN_1000KBPS))    // init can bus : baudrate = 1000k
  {
    Serial.println("CAN BUS FAIL!");
    delay(100);
  }
  Serial.println("CAN BUS OK!");

}

void heartbeat(int id) {
  if (id & 0xF00 == cube) {
    cubeLast = millis();
  }
}

void setRelay(unsigned char msg[8]) {
  //8 byte frame, bytes are assigned to
  //7th - Control Elec
  //6th - Motor Left
  //5th - Motor Right
  //4th - Arm 1
  //3rd - Arm 2
  //2nd -> 0th - unused
  //
  if(msg[7] == 0x1) {
    digitalWrite(ControlElec, LOW);
    ElecTog = true;
    if(timerSet == false) {setTimer();}
  }
  else if (msg[6] = 0x1) {
    digitalWrite(MotorLeft, LOW);
    MLTog = true;
    if(timerSet == false) {setTimer();}
  }
  else if (msg[5] = 0x1) {
    digitalWrite(MotorRight, LOW);
    MRTog = true;
    if(timerSet == false) {setTimer();}
  }
  else if (msg[4] = 0x1) {
    digitalWrite(Arm1, LOW);
    A1Tog = true;
    if(timerSet == false) {setTimer();}
  }
  else if (msg[3] = 0x1) {
    digitalWrite(Arm2, LOW);
    A2Tog = true;
    if(timerSet == false) {setTimer();}
  }
  else if (msg[2] = 0x1) {
    //Placeholder
  }
  else if (msg[1] = 0x1) {
    //Placeholder
  }
  else if (msg[0] = 0x1) {
    //Placeholder
  }
}

void setTimer() {
  //0.5Hz timer
  cli();

  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  OCR1A = 31249;

  TCCR1B |= (1 << WGM12);

  TCCR1B |= (1 << CS12) | (1 << CS10);

  TIMSK1 |= (1 << OCIE1A);

  sei();
}

ISR(TIMER1_COMPA_vect) {
  timerSet = false;
  if(ElecTog == true) {
    digitalWrite(ControlElec, HIGH);
    ElecTog = false;
  }
  if(MRTog == true) {
    digitalWrite(MotorRight, HIGH);
    MRTog = false;
  }
  if(MLTog == true) {
    digitalWrite(MotorLeft, HIGH);
    MLTog = false;
  }
  if(A1Tog == true) {
    digitalWrite(Arm1, HIGH);
    A1Tog = false;
  }
  if(A2Tog == true) {
    digitalWrite(Arm2, HIGH);
    A2Tog = false;
  }
  timerSet = false;
}

void loop() {
  if (millis() >= (cubeLast + 10 * HBPeriod)) {
    digitalWrite(ControlElec, LOW);
    ElecTog = true;
    if(timerSet == false) {
      setTimer();
    }
  }
  if (millis() >= lastHB + 1000) {
    unsigned char stmp[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    lastHB = millis();
    // send data:  id = 0x00, standrad frame, data len = 8, stmp: data buf
    stmp[7] = stmp[7]+1;
    if(stmp[7] == 100)
    {
      stmp[7] = 0;
      stmp[6] = stmp[6] + 1;
      
      if(stmp[6] == 100)
      {
        stmp[6] = 0;
        stmp[5] = stmp[6] + 1;
      }
    }
    
    CAN.sendMsgBuf(0x101, 0, 8, stmp);
  }
  unsigned char len = 0;
  unsigned char buf[8];

  if(CAN_MSGAVAIL == CAN.checkReceive())            // check if data coming
  {
    CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf

    unsigned long canId = CAN.getCanId();

    if ((canId & heart_beat) == 0x1) { //Heartbeat message
      heartbeat(canId);
    }
    if (canId == (cube + set_relay)) {
      setRelay(buf[8]);
    }
  }
}