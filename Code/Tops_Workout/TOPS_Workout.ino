//TOPS Quadruped Workout Program
#include <Arduino.h>
#include <ODriveTeensyCAN.h>
#include <LiquidCrystal_I2C.h>
#include <RC_Reciever.h>
#include <QIK.h>

ODriveTeensyCAN odriveCAN(250000);       //CAN bus object
LiquidCrystal_I2C lcd(0x27, 16, 2);      //16x2 LCD object
RC_Reciever RC(0, 1, 2, 3, 4, 5, 6, 7);  //pwm pins for each channel
QIK QIK(94.5, 180.0, 180.0);             //QIK object: lengths a, b, and c in mm

//CAN Bus Variables
CAN_message_t inMsg;                     //incoming CAN message
EncoderEstimatesMsg_t encoderEstimates;  //struct that holds encoder data

//QIK Variables
float normX = 110, normYF = -30, normYB = -100, normZ = 300;  //normal X, Y(front legs), Y(back legs), and Z positions of the foot
float GR = 9.0;                                               //gear ratio of the actuator
//absolute value of actuator angle offsets at the physical limits (in degrees) { abab, hip, knee}
float offset[4][3] = { { 120.0, 0, 47.449 },       //FL leg
                       { 120.0, 0, 58.851 },       //FR leg
                       { 120.0, 180, 298.149 },    //BL leg
                       { 120.0, 180, 307.388 } };  //BR leg
//direction of increasing position according to the Odrive (CW = +1, CCW = -1) { abab, hip, knee}
float dir[4][3] = { { 1, 1, 1 },     //FL leg
                    { -1, -1, -1 },  //FR leg
                    { -1, -1, -1 },  //BL leg
                    { 1, 1, 1 } };   //BR leg

int mode = 0;  //an int 0 to 3

void setup() {
  Serial.begin(115200);  //start Serial monitor
  RC.begin(-10, 10);     //set RC analog value range
  lcd.init();
  lcd.clear();
  lcd.backlight();
}
void loop() {
  //print home screen while  chn 5 is 0 (middle)
  while (RC.getChn(5) == 0) {
    lcd.setCursor(5, 0);
    lcd.print("TOPS");
    lcd.setCursor(3, 1);
    lcd.print("QUADRUPED");
    while (RC.getChn(5) == 0) {}
  }
  lcd.clear();
  //select a mode if while chn 5 is 1 (pushed forward)
  while (RC.getChn(5) == 1) {
    if (RC.getChn(6)) {              //if channel 6 is pressed
      while (RC.getChn(6)) {}        //wait until it is unpressed
      mode += (mode == 3 ? -3 : 1);  //keeps mode value in range (0-3)
      lcd.clear();
    }
    lcd.setCursor(2, 0);
    lcd.print("-SELECT-");
    printModeName(1);  //print the mode name on the LCD on row 1
  }
  lcd.clear();
  //enter the selected mode if chn 5 is -1 (pulled back)
  if (RC.getChn(5) == -1) {
    //confirm entering the selected mode
    if (confirmSelection()) {
      switch (mode) {
        case 0:  //Reboot
          REBOOT();
          break;
        case 1:  //Teleop
          TELEOP();
          break;
        case 2:  //Idle
          IDLE();
          break;
        case 3:  //Calibrate
          CALIBRATE();
          break;
      }
    }
    //if the mode not confirmed ("NO" is selected)
    while (RC.getChn(5) == -1) {
      printModeName(0);  //print the mode name on the LCD on row 0
      lcd.setCursor(3, 1);
      lcd.print("CANCELED");
    }
  }
  lcd.clear();
}
//set a parameter: velocity limit (v), position gain (p), velocity gain (d)
//actuators of the same type are set to the same param (abad, hip, knee)
void setParam(char param, float abad, float hip, float knee) {
  float vals[3] = { abad, hip, knee };  //put node param values in an array
  for (int i = 0; i < 12; i++) {        //cycle through each actuator
    switch (param) {
      //velocity limit
      case 'v':
        odriveCAN.SetLimits(i, vals[i % 3], 36);  //axisID, velocity limit, current limit (always 36A)
        break;
      //position gain
      case 'p':
        odriveCAN.SetPositionGain(i, vals[i % 3]);  //axisID, position gain
        break;
      //velocity gain
      case 'd':
        odriveCAN.SetVelocityGains(i, vals[i % 3], 0);  //axisID, velocity gain, velocity integrator gain
        break;
    }
  }
}
//move each foot to an X, Y, Z position
void moveTOPS(float X0, float Y0, float Z0,    //FL leg
              float X1, float Y1, float Z1,    //FR leg
              float X2, float Y2, float Z2,    //BL leg
              float X3, float Y3, float Z3) {  //BR leg
  float relPos;                                //odrive relative position
  //X, Y, Z positions of each foot
  float pos[4][3] = { { X0, Y0, Z0 },    //FL leg
                      { X1, Y1, Z1 },    //FR leg
                      { X2, Y2, Z2 },    //BL leg
                      { X3, Y3, Z3 } };  //BR leg
  //move each actuator
  for (int i = 0; i < 4; i++) {                                                  //cycle through each leg (FL, FR, BL, BR)
    for (int j = 0; j < 3; j++) {                                                //cycle through each legs joints (abad, hip, knee)
      relPos = QIK.getTheta(j, pos[i][0], pos[i][1], pos[i][2]) - offset[i][j];  //calculate relative joint position and make positive
      relPos = fabs(relPos / 360.0 * GR) * dir[i][j];                            //converts relative joint position from degrees to counts using gear ratio
      odriveCAN.SetPosition(i * 3 + j, relPos);                                  //moves the actuator to its joint position (axisID, relative position)
    }
  }
}
//step 'F', 'B', 'R', 'L', or 'C'
void step(char dir) {
  normYF = (dir == 'F') * -50;
  normYB = (dir == 'B' ? -50 : dir == 'F' ? -110
                                          : -100);
  int offset = (dir == 'F') * 60 + (dir == 'B') * -60 + (dir == 'R') * 50 + (dir == 'L' || dir == 'C') * -50;
  int offsetX = 0, offsetY = 0;
  int time = 60;
  moveTOPS(normX, normYF, normZ - abs(offset),   //FL
           normX, normYF, normZ,                 //FR
           normX, normYB, normZ,                 //BL
           normX, normYB, normZ - abs(offset));  //BR
  delay(time);
  offsetX = (dir == 'R' || dir == 'L' || dir == 'C') * offset;
  offsetY = (dir == 'F' || dir == 'B') * offset;
  moveTOPS(normX - offsetX, normYF + offsetY, normZ - abs(offset),                           //FL
           normX, normYF, normZ,                                                             //FR
           normX, normYB, normZ,                                                             //BL
           normX + offsetX * (dir == 'C' ? -1 : 1), normYB + offsetY, normZ - abs(offset));  //BR
  delay(time);
  moveTOPS(normX - offsetX, normYF + offsetY, normZ,                           //FL
           normX, normYF, normZ,                                               //FR
           normX, normYB, normZ,                                               //BL
           normX + offsetX * (dir == 'C' ? -1 : 1), normYB + offsetY, normZ);  //BR
  delay(time);
  moveTOPS(normX, normYF, normZ,                //FL
           normX, normYF, normZ - abs(offset),  //FR
           normX, normYB, normZ - abs(offset),  //BL
           normX, normYB, normZ);               //BR
  delay(time);
  moveTOPS(normX, normYF, normZ,                                                            //FL
           normX + offsetX * (dir == 'C' ? -1 : 1), normYF + offsetY, normZ - abs(offset),  //FR
           normX - offsetX, normYB + offsetY, normZ - abs(offset),                          //BL
           normX, normYB, normZ);                                                           //BR
  delay(time);
  moveTOPS(normX, normYF, normZ,                                              //FL
           normX + offsetX * (dir == 'C' ? -1 : 1), normYF + offsetY, normZ,  //FR
           normX - offsetX, normYB + offsetY, normZ,                          //BL
           normX, normYB, normZ);                                             //BR
  delay(time);
}
//trots in place
void inPlace() {
  normYF = -30;
  normYB = -100;
  normZ = 300;
  moveTOPS(normX, normYF, normZ - 80,   //FL
           normX, normYF, normZ,        //FR
           normX, normYB, normZ,        //BL
           normX, normYB, normZ - 80);  //BR
  delay(95);
  moveTOPS(normX, normYF, normZ,   //FL
           normX, normYF, normZ,   //FR
           normX, normYB, normZ,   //BL
           normX, normYB, normZ);  //BR
  delay(95);
  moveTOPS(normX, normYF, normZ,       //FL
           normX, normYF, normZ - 80,  //FR
           normX, normYB, normZ - 80,  //BL
           normX, normYB, normZ);      //BR
  delay(95);
  moveTOPS(normX, normYF, normZ,   //FL
           normX, normYF, normZ,   //FR
           normX, normYB, normZ,   //BL
           normX, normYB, normZ);  //BR
  delay(95);
}
//prints the name of the mode using the mode global variable
void printModeName(int row) {
  lcd.setCursor(0, row);
  switch (mode) {
    case 0:
      lcd.print("MODE: REBOOT");
      break;
    case 1:
      lcd.print("MODE: TELEOP");
      break;
    case 2:
      lcd.print("MODE: IDLE");
      break;
    case 3:
      lcd.print("MODE: CALIBRATE");
      break;
  }
}
//confirm the selection
bool confirmSelection() {
  int count = 2;  //counts the number of times that channel 6 is pressed
  int timeI;
  //Confirm entering a certain mode
  while (RC.getChn(5) == -1) {
    //if chn 6 is pressed
    if (RC.getChn(6)) {
      timeI = millis();                                   //set initial time
      while (RC.getChn(6) && millis() - timeI < 1000) {}  //wait until chn 6 is unpressed or 1 second has gone by
      lcd.clear();
      //if channel 6 was pressed for at least one second
      if (millis() - timeI >= 1000) {
        break;  //break from the while loop
      }
      //if channel 6 was pressed for less than one second
      else {
        count++;
      }
    }
    printModeName(0);  //print the mode name on the LCD on row 0
    lcd.setCursor(0, 1);
    if (count % 2) {
      lcd.print("ENTER?  NO *YES*");
    } else {
      lcd.print("ENTER? *NO* YES");
    }
  }
  return count % 2;  //return 1 or 0 for "YES" or "NO"
}
//Mode 0: REBOOT
void REBOOT() {
  //**do once**
  IDLE();                  //put the robot in idle mode
  printModeName(0);        //print the mode name on the LCD on row 0
  while (RC.getChn(6)) {}  //wait for channel 6 to be unpressed
  int leg = 0;             //leg number
  int timeI;
  String legName[4] = { "FRONT LEFT", "FRONT RIGHT", "BACK LEFT", "BACK RIGHT" };  //array of leg names
  //**loop until chn 5 is -1**
  while (RC.getChn(5) == -1) {
    //if chn 6 is pressed
    if (RC.getChn(6)) {
      timeI = millis();                                   //set initial time
      while (RC.getChn(6) && millis() - timeI < 1000) {}  //wait until chn 6 is unpressed or 1 second has gone by
      lcd.clear();
      //if channel 6 was pressed for at least one second then calibrate the selected leg
      if (millis() - timeI >= 1000) {  //if channel 6 was held for at least a second
        lcd.setCursor(0, 0);
        lcd.print("LEG: ");
        lcd.print(legName[leg]);
        while (RC.getChn(6)) {}  //wait for channel 6 to be unpressed
        lcd.setCursor(2, 1);
        lcd.print("REBOOTING...");
        //reboots the 3 actuators on that leg
        for (int i = leg * 3; i < leg * 3 + 3; i++) {
          odriveCAN.RebootOdrive(i);
        }
        delay(3000);  //wait 3 seconds for reboot
        lcd.clear();
        lcd.setCursor(4, 0);
        lcd.print("FINISHED");
        delay(1000);
        leg--;  //prevents the leg value from increasing
      }
      leg += (leg == 3 ? -3 : 1);  //keeps leg value in range (0-3)
      lcd.clear();
    }
    printModeName(0);  //print the mode name on the LCD on row 0
    lcd.setCursor(0, 1);
    lcd.print("LEG: ");
    lcd.print(legName[leg]);
  }
}
//Mode 1: Teleoperation
void TELEOP() {
  //**do once**
  for (int i = 0; i < 12; i++) {            // put the robot in closed loop control mode
    odriveCAN.RunState(i, 8);               //set state to closed loop control
    odriveCAN.SetControllerModes(i, 3, 1);  //axisID, control mode (3 = position), input mode (1 = passthrough)
  }
  setParam('v', 33, 33, 33);  //set velocity limits
  setParam('p', 60, 60, 60);  //set proportional gains
  //home TOPS
  moveTOPS(normX, normYF, normZ,   //FL
           normX, normYF, normZ,   //FR
           normX, normYB, normZ,   //BL
           normX, normYB, normZ);  //BR
  //**loop until chn 5 is -1**
  while (RC.getChn(5) == -1) {
    //Navigate (mode: 1)
    if (RC.getChn(7) == 1) {
      lcd.clear();
      printModeName(0);
      lcd.setCursor(0, 1);
      lcd.print("NAVIGATE");
      int reset = 0;
      normYF = -30;
      normYB = -100;
      normZ = 300;
      //home TOPS
      moveTOPS(normX, normYF, normZ,   //FL
               normX, normYF, normZ,   //FR
               normX, normYB, normZ,   //BL
               normX, normYB, normZ);  //BR
      while (RC.getChn(7) == 1 && RC.getChn(5) == -1) {
        while (RC.getChn(2) >= 7) {  //forward step
          reset = 1;
          step('F');
        }
        while (RC.getChn(2) <= -7) {  //backward step
          reset = 1;
          step('B');
        }
        while (RC.getChn(1) >= 7) {  //right step
          reset = 1;
          step('R');
        }
        while (RC.getChn(1) <= -7) {  //left step
          reset = 1;
          step('L');
        }
        while (RC.getChn(4) >= 7) {  //turn right
          reset = 1;
          step('C');
        }
        while (RC.getChn(6)) {  //trot in place is button is pushed
          inPlace();
        }
        if (reset) {
          moveTOPS(normX, normYF, normZ - 60,   //FL
                   normX, normYF, normZ,        //FR
                   normX, normYB, normZ,        //BL
                   normX, normYB, normZ - 50);  //BR
          delay(60);
          moveTOPS(normX, normYF, normZ,   //FL
                   normX, normYF, normZ,   //FR
                   normX, normYB, normZ,   //BL
                   normX, normYB, normZ);  //BR
          delay(60);
          reset = 0;
        }
      }
    }
    //inverse kinematics demo (mode: -1)
    else if (RC.getChn(7) == -1) {
      lcd.clear();
      printModeName(0);
      lcd.setCursor(0, 1);
      lcd.print("INVERSE KINEM");
      int chn2 = 0;
      normYF = 50;
      normYB = -50;
      int time = 50;
      moveTOPS(normX, normYF, normZ,   //FL
               normX, normYF, normZ,   //FR
               normX, normYB, normZ,   //BL
               normX, normYB, normZ);  //B
      while (RC.getChn(7) == -1 && RC.getChn(5) == -1) {
        if (RC.getChn(6)) {
          //bend forward
          for (int i = 0; i <= 50; i += 5) {
            moveTOPS(normX, normYF + i, normZ - i,   //FL
                     normX, normYF + i, normZ - i,   //FR
                     normX, normYB + i, normZ + i,   //BL
                     normX, normYB + i, normZ + i);  //BR
            delay(time);
          }
          //bend backwards
          for (int i = 50; i >= -50; i -= 5) {
            moveTOPS(normX, normYF + i, normZ - i,   //FL
                     normX, normYF + i, normZ - i,   //FR
                     normX, normYB + i, normZ + i,   //BL
                     normX, normYB + i, normZ + i);  //BR
            delay(time);
          }
          //back to home
          for (int i = -50; i <= 0; i += 5) {
            moveTOPS(normX, normYF + i, normZ - i,   //FL
                     normX, normYF + i, normZ - i,   //FR
                     normX, normYB + i, normZ + i,   //BL
                     normX, normYB + i, normZ + i);  //BR
            delay(time);
          }
          //bend right
          for (int i = 0; i <= 50; i += 5) {
            moveTOPS(normX - i * 2, normYF, normZ + i,   //FL
                     normX + i * 2, normYF, normZ - i,   //FR
                     normX - i * 2, normYB, normZ + i,   //BL
                     normX + i * 2, normYB, normZ - i);  //BR
            delay(time);
          }
          //bend left
          for (int i = 50; i >= -50; i -= 5) {
            moveTOPS(normX - i * 2, normYF, normZ + i,   //FL
                     normX + i * 2, normYF, normZ - i,   //FR
                     normX - i * 2, normYB, normZ + i,   //BL
                     normX + i * 2, normYB, normZ - i);  //BR
            delay(time);
          }
          //back to home
          for (int i = -50; i <= 0; i += 5) {
            moveTOPS(normX - i * 2, normYF, normZ + i,   //FL
                     normX + i * 2, normYF, normZ - i,   //FR
                     normX - i * 2, normYB, normZ + i,   //BL
                     normX + i * 2, normYB, normZ - i);  //BR
            delay(time);
          }
          //go up
          for (int i = 0; i <= 50; i += 5) {
            moveTOPS(normX, normYF, normZ + i,   //FL
                     normX, normYF, normZ + i,   //FR
                     normX, normYB, normZ + i,   //BL
                     normX, normYB, normZ + i);  //BR
            delay(time);
          }
          //go down
          for (int i = 50; i >= -100; i -= 5) {
            moveTOPS(normX, normYF, normZ + i,   //FL
                     normX, normYF, normZ + i,   //FR
                     normX, normYB, normZ + i,   //BL
                     normX, normYB, normZ + i);  //BR
            delay(time);
          }
          //back to home
          for (int i = -100; i <= 0; i += 5) {
            moveTOPS(normX, normYF, normZ + i,   //FL
                     normX, normYF, normZ + i,   //FR
                     normX, normYB, normZ + i,   //BL
                     normX, normYB, normZ + i);  //BR
            delay(time);
          }
        }
      }
    }
    //Dance
    else if (RC.getChn(7) == 0) {
      lcd.clear();
      printModeName(0);
      lcd.setCursor(0, 1);
      lcd.print("DANCE");
      normYF = 50;
      normYB = -100;
      int time = 15;
      while (RC.getChn(7) == 0 && RC.getChn(5) == -1) {
        if (RC.getChn(6)) {
          /*
          // Dance #1
          for (int i = 0; i < 10; i++) {
            time = 17;
            //go up
            for (int i = 0; i <= 30; i += 5) {
              moveTOPS(normX, normYF, normZ + i,   //FL
                       normX, normYF, normZ + i,   //FR
                       normX, normYB, normZ + i,   //BL
                       normX, normYB, normZ + i);  //BR
              delay(time);
            }
            //go down
            for (int i = 30; i >= -30; i -= 5) {
              moveTOPS(normX, normYF, normZ + i,   //FL
                       normX, normYF, normZ + i,   //FR
                       normX, normYB, normZ + i,   //BL
                       normX, normYB, normZ + i);  //BR
              delay(time);
            }
            //back to home
            for (int i = -30; i <= 0; i += 5) {
              moveTOPS(normX, normYF, normZ + i,   //FL
                       normX, normYF, normZ + i,   //FR
                       normX, normYB, normZ + i,   //BL
                       normX, normYB, normZ + i);  //BR
              delay(time);
            }
          }
          */
          //Dance #2
          /*
          for (int j = 0; j < 1; j++) {
            //bend right
            for (int i = 0; i <= 50; i += 5) {
              moveTOPS(normX - i * 2, normYF, normZ + i,   //FL
                       normX + i * 2, normYF, normZ - i,   //FR
                       normX - i * 2, normYB, normZ + i,   //BL
                       normX + i * 2, normYB, normZ - i);  //BR
              delay(time);
            }
            //bend left
            for (int i = 50; i >= -50; i -= 5) {
              moveTOPS(normX - i * 2, normYF, normZ + i,   //FL
                       normX + i * 2, normYF, normZ - i,   //FR
                       normX - i * 2, normYB, normZ + i,   //BL
                       normX + i * 2, normYB, normZ - i);  //BR
              delay(time);
            }
            //back to home
            for (int i = -50; i <= 0; i += 5) {
              moveTOPS(normX - i * 2, normYF, normZ + i,   //FL
                       normX + i * 2, normYF, normZ - i,   //FR
                       normX - i * 2, normYB, normZ + i,   //BL
                       normX + i * 2, normYB, normZ - i);  //BR
              delay(time);
            }
            //bend forward
            for (int i = 0; i <= 50; i += 5) {
              moveTOPS(normX, normYF + i, normZ - i,   //FL
                       normX, normYF + i, normZ - i,   //FR
                       normX, normYB + i, normZ + i,   //BL
                       normX, normYB + i, normZ + i);  //BR
              delay(time);
            }
            //bend backwards
            for (int i = 50; i >= -50; i -= 5) {
              moveTOPS(normX, normYF + i, normZ - i,   //FL
                       normX, normYF + i, normZ - i,   //FR
                       normX, normYB + i, normZ + i,   //BL
                       normX, normYB + i, normZ + i);  //BR
              delay(time);
            }
            //back to home
            for (int i = -50; i <= 0; i += 5) {
              moveTOPS(normX, normYF + i, normZ - i,   //FL
                       normX, normYF + i, normZ - i,   //FR
                       normX, normYB + i, normZ + i,   //BL
                       normX, normYB + i, normZ + i);  //BR
              delay(time);
            }
          }
          /*
          //Dance #3
          /*
          for (int j = 0; j < 10; j++) {
            //bend right
            for (int i = 0; i <= 50; i += 5) {
              moveTOPS(normX - i * 2, normYF, normZ + i,   //FL
                       normX + i * 2, normYF, normZ - i,   //FR
                       normX - i * 2, normYB, normZ + i,   //BL
                       normX + i * 2, normYB, normZ - i);  //BR
              delay(time);
            }
            //bend left
            for (int i = 50; i >= -50; i -= 5) {
              moveTOPS(normX - i * 2, normYF, normZ + i,   //FL
                       normX + i * 2, normYF, normZ - i,   //FR
                       normX - i * 2, normYB, normZ + i,   //BL
                       normX + i * 2, normYB, normZ - i);  //BR
              delay(time);
            }
            //back to home
            for (int i = -50; i <= 0; i += 5) {
              moveTOPS(normX - i * 2, normYF, normZ + i,   //FL
                       normX + i * 2, normYF, normZ - i,   //FR
                       normX - i * 2, normYB, normZ + i,   //BL
                       normX + i * 2, normYB, normZ - i);  //BR
              delay(time);
            }
          }
          for (int j = 0; j < 10; j++) {
            //bend forward
            for (int i = 0; i <= 50; i += 5) {
              moveTOPS(normX, normYF + i, normZ - i,   //FL
                       normX, normYF + i, normZ - i,   //FR
                       normX, normYB + i, normZ + i,   //BL
                       normX, normYB + i, normZ + i);  //BR
              delay(time);
            }
            //bend backwards
            for (int i = 50; i >= -50; i -= 5) {
              moveTOPS(normX, normYF + i, normZ - i,   //FL
                       normX, normYF + i, normZ - i,   //FR
                       normX, normYB + i, normZ + i,   //BL
                       normX, normYB + i, normZ + i);  //BR
              delay(time);
            }
            //back to home
            for (int i = -50; i <= 0; i += 5) {
              moveTOPS(normX, normYF + i, normZ - i,   //FL
                       normX, normYF + i, normZ - i,   //FR
                       normX, normYB + i, normZ + i,   //BL
                       normX, normYB + i, normZ + i);  //BR
              delay(time);
            }
          }
          */
        }
      }
    }
  }
}
//Mode 2: Idle
void IDLE() {
  //**do once**
  printModeName(0);               //print the mode name on the LCD on row 0
  for (int i = 0; i < 12; i++) {  //put each actuator in IDLE mode
    odriveCAN.RunState(i, 1);
  }
  //**loop until chn 5 is -1 except mode is not 3**
  while (RC.getChn(5) == -1 && mode == 2) {}
}
//Mode 3: Calibrate
void CALIBRATE() {
  //**do once**
  IDLE();                  //put the robot in idle mode
  printModeName(0);        //print the mode name on the LCD on row 0
  while (RC.getChn(6)) {}  //wait for channel 6 to be unpressed
  int leg = 0;             //leg number
  int timeI;
  String legName[4] = { "FRONT LEFT", "FRONT RIGHT", "BACK LEFT", "BACK RIGHT" };  //array of leg names
  //**loop until chn 5 is -1**
  while (RC.getChn(5) == -1) {
    //if chn 6 is pressed
    if (RC.getChn(6)) {
      timeI = millis();                                   //set initial time
      while (RC.getChn(6) && millis() - timeI < 1000) {}  //wait until chn 6 is unpressed or 1 second has gone by
      lcd.clear();
      //if channel 6 was pressed for at least one second then calibrate the selected leg
      if (millis() - timeI >= 1000) {  //if channel 6 was held for at least a second
        lcd.setCursor(0, 0);
        lcd.print("LEG: ");
        lcd.print(legName[leg]);
        while (RC.getChn(6)) {}  //wait for channel 6 to be unpressed
        lcd.setCursor(2, 1);
        lcd.print("CALIBRATING...");
        //calbrates the 3 actuators on that leg
        for (int i = leg * 3; i < leg * 3 + 3; i++) {
          odriveCAN.RunState(i, 3);  //set state to calibrate
        }
        delay(15000);  //wait 15 seconds for the calibration
        lcd.clear();
        lcd.setCursor(4, 0);
        lcd.print("FINISHED");
        delay(1000);
        leg--;  //prevents the leg value from increasing
      }
      leg += (leg == 3 ? -3 : 1);  //keeps leg value in range (0-3)
      lcd.clear();
    }
    printModeName(0);  //print the mode name on the LCD on row 0
    lcd.setCursor(0, 1);
    lcd.print("LEG: ");
    lcd.print(legName[leg]);
  }
}