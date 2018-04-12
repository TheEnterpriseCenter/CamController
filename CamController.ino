#include "math.h"
byte up_tilt[] = {0x81, 0x01, 0x06, 0x01, 0xa0, 0xa0, 0x03, 0x02, 0xff};
byte pan_tilt[] = {0x81, 0x01, 0x06, 0x01, 0x03, 0x03, 0x03, 0x03, 0xff};
byte ptMessage[] = {0x81, 0x01, 0x06, 0x01, 0x03, 0x03, 0x03, 0x03, 0xff};
byte cam_reset[] = {0x88, 0x01, 0x00, 0x01, 0xff};
byte address_set[] = {0x88, 0x30, 0x01, 0xff};
byte pan_stop[] = {0x81, 0x01, 0x06, 0x01, 0x03, 0x03, 0x03, 0x03, 0xff};
byte zoom_stop[] = {0x81, 0x01, 0x04, 0x07, 0x00, 0xff};
byte zMessage[] = {0x81, 0x01, 0x04, 0x07, 0x00, 0xff};
byte serialSpeed[] = {0x81, 0x01, 0x34, 0x01, 0xff};
byte responseOK[] = {0x90, 0x50, 0xff};
byte responseBuffer[7];
byte axisZPast = 0x00;
byte axisPPast = 0x00;
byte axisTPast = 0x00;
const int axisPHIGH = 2230;
const int axisPLOW = 1900;
const int axisTHIGH = 2150;
const int axisTLOW = 1900;
const int axisZHIGH = 2180;
const int axisZLOW = 1850;
const int myLogScale = 256;
const float myLogConst = 1.0 / 256.0;
const float myLogCurve = 1.8;
void setup() {
    Serial1.begin(9600);
    Serial.begin(9600);
    Serial1.write(serialSpeed, 5);
    delay(10);
    if (Serial1.available()) {
      for(int i; i < 7 && Serial1.available(); i++)
      {
        responseBuffer[i] = Serial1.read();
      }
      if(responseBuffer[1] == 0x50) {
        Serial1.begin(115200);
        delay(20*1000);
      }
    }
    /*pinMode(D0, INPUT_PULLUP);*/
    /*Serial1.write(cam_reset, 5);*/
    /*delay(100);*/
    /*Serial1.write(address_set, 4);*/
    /*delay(100);*/
    /*digitalWrite(D0, HIGH);*/
}

void loop() {
  /*testAxis();*/
  if(millis() % 20 == 0){
    updatePTZ();
    testAxis();
  }

  /*updatePTZ();*/
  /*if (Serial1.available()) {*/
    /*while (Serial1.available()) {*/
      /*Serial.printlnf("%x", Serial1.read());*/
      /*Serial1.read();*/
    /*}*/
  /*}*/

}
void testAxis() {
  int axisZ = analogRead(A2);
  int axisT = analogRead(A1);
  int axisP = analogRead(A0);
  Serial.printlnf("P: %u T: %u Z: %u", axisP, axisT, axisZ);
  delay(10);
}
void myLogTest() {
  for (int i = 0; i < 255; i++)
  {
    Serial.println(myLog(i));
  }
}

int myLog(int n) {
  float newN = float(n);
  return round(myLogScale * pow(newN * myLogConst, myLogCurve));
  /*return round(pow(newN, 0.8) + (newN * ((170.0 + newN) / 254.0)) - newN);*/
}

void updatePTZ() {
  byte *axisBytes;
  int axisZ = analogRead(A2);
  int axisT = analogRead(A1);
  int axisP = analogRead(A0);
  /*Serial.print("Z ");*/
  byte *axisZBytes = getAxis(axisZ, axisZHIGH, axisZLOW);
  byte zDirectionSpeed;
  byte zDirection = *(axisZBytes);
  byte zSpeed = *(axisZBytes + 1);
  if (zSpeed != axisZPast) {
    if(zDirection != 0x03) {
      axisZPast = zSpeed;
      if (zDirection == 0x01) {
        zDirection = 0x03;
      }
      if(zSpeed >> 4 != 0) {
        zDirectionSpeed = (zDirection << 4) + 0x02;
      } else {
        zDirectionSpeed = (zDirection << 4) + 0x01;
      }
      zMessage[4] = zDirectionSpeed;
      Serial1.write(zMessage, 6);
    } else {
      Serial1.write(zoom_stop, 6);
    }
  }
  /*Serial.print("T ");*/
  byte *axisTBytes = getAxis(axisT, axisTHIGH, axisTLOW);
  byte tDirection = *(axisTBytes);
  byte tSpeed = *(axisTBytes + 1);
  /*Serial.print("P ");*/
  byte *axisPBytes = getAxis(axisP, axisPHIGH, axisPLOW);
  byte pDirection = *(axisPBytes);
  byte pSpeed = *(axisPBytes + 1);
  if(pSpeed != axisPPast || tSpeed != axisTPast) {
    if(tDirection != 0x03 || pDirection != 0x03 ) {
      ptMessage[4] = pSpeed;
      ptMessage[5] = tSpeed;
      ptMessage[6] = pDirection;
      ptMessage[7] = tDirection;
      Serial1.write(ptMessage, 9);
      axisPPast = pSpeed;
      axisTPast = tSpeed;
    } else {
      Serial1.write(pan_stop, 9);
    }
  }
  /*Serial.println("");*/
}

byte * getAxis(int axis, int axisHIGH, int axisLOW) {
  static byte axisBytes[2];
  int axisspeed;
  if(axis > axisHIGH) {
    axisspeed = (axis - axisHIGH) >> 3;
    if(axisspeed > 254) {
      axisspeed = 254;
    } else {
      axisspeed = myLog(axisspeed);
    }
    axisBytes[1] = axisspeed;
    axisBytes[0] = 0x01;
    /*Serial.println(axisspeed);*/
    return axisBytes;
  } else if(axis < axisLOW) {
    int axisDecSpeed = axis - axisLOW;
    axisspeed = abs(axisDecSpeed) >> 3;
    if(axisspeed > 254) {
      axisspeed = 254;
    } else {
      axisspeed = myLog(axisspeed);
    }
    axisBytes[1] = axisspeed;
    axisBytes[0] = 0x02;
    /*Serial.println(axisspeed);*/
    return axisBytes;
  } else {
    axisBytes[1] = 0x03;
    axisBytes[0] = 0x03;
    return axisBytes;
  }

}
