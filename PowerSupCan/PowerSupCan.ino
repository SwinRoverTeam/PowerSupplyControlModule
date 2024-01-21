#define ControlElec 4
#define MotorLeft 5
#define MotorRight 6
#define Arm1 8
#define Arm2 9

MCP_CAN CAN(SPI_CS_PIN);  

void setup() {
  bool ElecTog = false;
  bool MLTog = false;
  bool MRTog = false;
  bool A1Tog = false;
  bool A2Tog = false;

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

    canCMD = buf[0];
    
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
      
      //SwinCan.ReplyRelay(--NodeId--)
    }
  }

}

void turnOn() {
  //
}
