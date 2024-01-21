#include <SPI.h>

const int SPI_CS_PIN = 17;              // CANBed V1

#define ControlElec 4
#define MotorLeft 5
#define MotorRight 6
#define Arm1 8
#define Arm2 9

MCP_CAN CAN(SPI_CS_PIN); 

bool ElecTog = false;
bool MLTog = false;
bool MRTog = false;
bool A1Tog = false;
bool A2Tog = false;

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

void loop() {

  unsigned char len = 0;
  unsigned char buf[8];

  if(CAN_MSGAVAIL == CAN.checkReceive())            // check if data coming
  {
    CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf

    unsigned long canId = CAN.getCanId();

    unsigned char canCMD = buf[0];
    
    if(canId == 0x10) { //Heartbeat
      //SwinCAN.ReplyHeart(--NodeId--)
    }
    else if(canId == 0x30) { //Status
      //SwinCan.ReplyStatus
    }
    else if (canId == 0x50)  {// Relay Control
      if(buf[7]&0b1 == 1) { //Cycle 5.3V
        digitalWrite(ControlElec, LOW);
        ElecTog = true;
      }
      if((buf[7]&0b10) >> 1 == 1) { //Cycle Motor Left
        digitalWrite(MotorLeft, LOW);
        MLTog = true;
      }
      if((buf[7]&0b100) >> 2 == 1) { //Cycle Motor Right
        digitalWrite(MotorRight, LOW);
        MRTog = true;
      }
      if((buf[7]&0b1000) >> 3 == 1) { //Cycle Arm 1
        digitalWrite(Arm1, LOW);
        A1Tog = true;
      }
      if((buf[7]&0b10000) >> 4 == 1) { //Cycle Arm 2
        digitalWrite(Arm2, LOW);
        A2Tog = true;
      }
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
      
      //SwinCan.ReplyRelay(--NodeId--)
    }
  }

}

ISR(TIMER1_COMPA_vect) {
  if(ElecTog == true) {
    digitalWrite(ControlElec, HIGH);
    ElecTog = false;
  }
  if(MLTog == true) {
    digitalWrite(MotorLeft, HIGH);
    MLTog = false;
  }
  if(MRTog == true) {
    digitalWrite(MotorRight, HIGH);
    MRTog = false;
  }
  if(A1Tog == true) {
    digitalWrite(Arm1, HIGH);
    A1Tog = false;
  }
  if(A2Tog == true) {
    digitalWrite(Arm2, HIGH);
    A2Tog = false;
  }
}
