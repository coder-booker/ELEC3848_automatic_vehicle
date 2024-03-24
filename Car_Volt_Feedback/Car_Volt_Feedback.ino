#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MPU6050_light.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     28 //4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
int oldV=1, newV=0;
#include <SoftwareSerial.h>
//UNO: (2, 3)
//SoftwareSerial mySerial(4, 6); // RX, TX
int pan = 90;
int tilt = 120;
int window_size = 0;
int BT_alive_cnt = 0;
int voltCount = 0;
#include <Servo.h>
Servo servo_pan;
Servo servo_tilt;
int servo_min = 20;
int servo_max = 160;

unsigned long time;

//FaBoPWM faboPWM;
int pos = 0;
int MAX_VALUE = 2000;
int MIN_VALUE = 300;

// Define motor pins
#define PWMA 12    //Motor A PWM
#define DIRA1 34
#define DIRA2 35  //Motor A Direction
#define PWMB 8    //Motor B PWM
#define DIRB1 37
#define DIRB2 36  //Motor B Direction
#define PWMC 6   //Motor C PWM
#define DIRC1 43
#define DIRC2 42  //Motor C Direction
#define PWMD 5    //Motor D PWM
#define DIRD1 A4  //26  
#define DIRD2 A5  //27  //Motor D Direction

#define MOTORA_FORWARD(pwm)    do{digitalWrite(DIRA1,LOW); digitalWrite(DIRA2,HIGH);analogWrite(PWMA,pwm);}while(0)
#define MOTORA_STOP(x)         do{digitalWrite(DIRA1,LOW); digitalWrite(DIRA2,LOW); analogWrite(PWMA,0);}while(0)
#define MOTORA_BACKOFF(pwm)    do{digitalWrite(DIRA1,HIGH);digitalWrite(DIRA2,LOW); analogWrite(PWMA,pwm);}while(0)

#define MOTORB_FORWARD(pwm)    do{digitalWrite(DIRB1,LOW); digitalWrite(DIRB2,HIGH);analogWrite(PWMB,pwm);}while(0)
#define MOTORB_STOP(x)         do{digitalWrite(DIRB1,LOW); digitalWrite(DIRB2,LOW); analogWrite(PWMB,0);}while(0)
#define MOTORB_BACKOFF(pwm)    do{digitalWrite(DIRB1,HIGH);digitalWrite(DIRB2,LOW); analogWrite(PWMB,pwm);}while(0)

#define MOTORC_FORWARD(pwm)    do{digitalWrite(DIRC1,LOW); digitalWrite(DIRC2,HIGH);analogWrite(PWMC,pwm);}while(0)
#define MOTORC_STOP(x)         do{digitalWrite(DIRC1,LOW); digitalWrite(DIRC2,LOW); analogWrite(PWMC,0);}while(0)
#define MOTORC_BACKOFF(pwm)    do{digitalWrite(DIRC1,HIGH);digitalWrite(DIRC2,LOW); analogWrite(PWMC,pwm);}while(0)

#define MOTORD_FORWARD(pwm)    do{digitalWrite(DIRD1,LOW); digitalWrite(DIRD2,HIGH);analogWrite(PWMD,pwm);}while(0)
#define MOTORD_STOP(x)         do{digitalWrite(DIRD1,LOW); digitalWrite(DIRD2,LOW); analogWrite(PWMD,0);}while(0)
#define MOTORD_BACKOFF(pwm)    do{digitalWrite(DIRD1,HIGH);digitalWrite(DIRD2,LOW); analogWrite(PWMD,pwm);}while(0)

#define SERIAL  Serial
#define BTSERIAL Serial3

#define LOG_DEBUG

#ifdef LOG_DEBUG
  #define M_LOG SERIAL.print
#else
  #define M_LOG BTSERIAL.println
#endif

//PWM Definition
#define MAX_PWM   2000
#define MIN_PWM   300

int Motor_PWM = 0;

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
/*
  BELOW BY GROUP C4-C
*/

void AUTO_Control();
void Alignment();
void Rotation_90degree();
void Rotate();
void MoveAndRotate();
void Measurement();
void Parking();
void DataUpdate();
void calibrate();


  // sonar
  #define trigPinL 29
  #define trigPinR 25
  #define echoPinL 30
  #define echoPinR 28
  double L_sonar_dist,R_sonar_dist;
  int done = 1;
  unsigned long start_time = 0;

  //gyro
  MPU6050 mpu(Wire);
  float pitch, roll, yaw, angle_set;

  //enum class for the state machine
  enum class MOVEMENTTYPE{
    LOCATING1,
    LOCATING2,
    NEXTSTAGE_TRANSITION,
    ROTATING1,
    ROTATING2,
    ROTATING3,
    REALIGNMENT1,
    REALIGNMENT2,
    END_MOVEMENT
  };
  enum class TASKTYPE{
    INIT,
    ALIGNMENT,
    MOVEANDROTATE,
    MEASUREMENT,
    PARKING,
    END_STATE
  };
  enum class PRAKINGSTATE{
    FORWARD,
    TRANSIT,
    ROTATE,
    END_PARKING
  };
  TASKTYPE STATE = TASKTYPE::INIT;
  MOVEMENTTYPE MOVEMENT = MOVEMENTTYPE::LOCATING1;
  PRAKINGSTATE PARKING = PRAKINGSTATE::FORWARD;

  // alignment
  int flag = 0;
  int alignemnt_cnt = 0;
  int locating_cnt = 0;
  int rotate_cnt = 0;
  int park_cnt = 0;

  // light
  #define TOL   50           // tolerance for adc different, avoid oscillation
  #define K   5              // Step size
  #define LightPinL A8
  #define LightPinR A9
  //variables for light intensity to ADC reading equations 
  int int_adc0, int_adc0_m, int_adc0_c;
  int int_adc1, int_adc1_m, int_adc1_c;
  int int_left, int_right, avg_light_intensity, last_avg_light_intensity;
  int light_left, light_right;

  //measurment
  float distance_to_wall_buf, angle_of_vehicle_buf, distance_to_wall, angle_of_vehicle;
  int cnt = 0;

  //parking
  float max_light_intensity = 0;
  float yaw_at_max_light_intensity = 0;

  // PWM
  uint8_t motion_mode;
  uint8_t motion_last_mode;
  #define MOTION_MODE_ADVANCE 0
  #define MOTION_MODE_BACK 1
  #define MOTION_MODE_LEFT 2
  #define MOTION_MODE_RIGHT 3
  #define MOTION_MODE_ROTATE 4
  #define MOTION_MODE_STOP 5
  #define MOTION_MODE_ROTATE_CW 6
  #define MOTION_MODE_ROTATE_CCW 7
  #define MOTION_MODE_ROTATE_180 8
  #define MOTION_MODE_ROTATE_90 9
  #define MOTION_MODE_ROTATE_270 10

/*
  ABOVE BY GROUP C4-C
*/
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////


//    ↑A-----B�??????????????
//     |  �??????????????  |
//     |  |  |
//    ↑C-----D�??????????????
void BACK(uint8_t pwm_A, uint8_t pwm_B, uint8_t pwm_C, uint8_t pwm_D)
{
  MOTORA_BACKOFF(pwm_A); 
  MOTORB_FORWARD(pwm_B);
  MOTORC_BACKOFF(pwm_C); 
  MOTORD_FORWARD(pwm_D);
}

//    ↓A-----B�??????????????
//     |  |  |
//     |  �??????????????  |
//    ↓C-----D�??????????????
void ADVANCE(uint8_t pwm_A, uint8_t pwm_B, uint8_t pwm_C, uint8_t pwm_D)
{
  MOTORA_FORWARD(pwm_A); 
  MOTORB_BACKOFF(pwm_B);
  MOTORC_FORWARD(pwm_C); 
  MOTORD_BACKOFF(pwm_D);
}
//    =A-----B�??????????????
//     |   �?????????????? |
//     | �??????????????   |
//    ↑C-----D=
void LEFT_1(uint8_t pwm_A, uint8_t pwm_B, uint8_t pwm_C, uint8_t pwm_D)
{
  MOTORA_STOP(pwm_A); 
  MOTORB_FORWARD(pwm_B);
  MOTORC_BACKOFF(pwm_C); 
  MOTORD_STOP(pwm_D);
}

//    ↓A-----B�??????????????
//     |  �??????????????  |
//     |  �??????????????  |
//    ↑C-----D�??????????????
void RIGHT_2(uint8_t pwm_A, uint8_t pwm_B, uint8_t pwm_C, uint8_t pwm_D)
{
  MOTORA_FORWARD(pwm_A); 
  MOTORB_FORWARD(pwm_B);
  MOTORC_BACKOFF(pwm_C); 
  MOTORD_BACKOFF(pwm_D);
}
//    ↓A-----B=
//     | �??????????????   |
//     |   �?????????????? |
//    =C-----D�??????????????
void LEFT_3(uint8_t pwm_A, uint8_t pwm_B, uint8_t pwm_C, uint8_t pwm_D)
{
  MOTORA_FORWARD(pwm_A); 
  MOTORB_STOP(pwm_B);
  MOTORC_STOP(pwm_C); 
  MOTORD_BACKOFF(pwm_D);
}
//    ↑A-----B=
//     | �??????????????   |
//     |   �?????????????? |
//    =C-----D�??????????????
void RIGHT_1(uint8_t pwm_A, uint8_t pwm_B, uint8_t pwm_C, uint8_t pwm_D)
{
  MOTORA_BACKOFF(pwm_A); 
  MOTORB_STOP(pwm_B);
  MOTORC_STOP(pwm_C); 
  MOTORD_FORWARD(pwm_D);
}
//    ↑A-----B�??????????????
//     |  �??????????????  |
//     |  �??????????????  |
//    ↓C-----D�??????????????
void LEFT_2(uint8_t pwm_A, uint8_t pwm_B, uint8_t pwm_C, uint8_t pwm_D)
{
  MOTORA_BACKOFF(pwm_A); 
  MOTORB_BACKOFF(pwm_B);
  MOTORC_FORWARD(pwm_C); 
  MOTORD_FORWARD(pwm_D);
}
//    =A-----B�??????????????
//     |   �?????????????? |
//     | �??????????????   |
//    ↓C-----D=
void RIGHT_3(uint8_t pwm_A, uint8_t pwm_B, uint8_t pwm_C, uint8_t pwm_D)
{
  MOTORA_STOP(pwm_A); 
  MOTORB_BACKOFF(pwm_B);
  MOTORC_FORWARD(pwm_C); 
  MOTORD_STOP(pwm_D);
}

//    ↑A-----B�??????????????
//     | �?????????????? �?????????????? |
//     | �?????????????? �?????????????? |
//    ↑C-----D�??????????????
void rotate_1(uint8_t pwm_A, uint8_t pwm_B, uint8_t pwm_C, uint8_t pwm_D)  // CW
{
  MOTORA_BACKOFF(pwm_A); 
  MOTORB_BACKOFF(pwm_B);
  MOTORC_BACKOFF(pwm_C); 
  MOTORD_BACKOFF(pwm_D);
}

//    ↓A-----B�??????????????
//     | �?????????????? �?????????????? |
//     | �?????????????? �?????????????? |
//    ↓C-----D�??????????????
void rotate_2(uint8_t pwm_A, uint8_t pwm_B, uint8_t pwm_C, uint8_t pwm_D)  //CCW
{
  MOTORA_FORWARD(pwm_A);
  MOTORB_FORWARD(pwm_B);
  MOTORC_FORWARD(pwm_C);
  MOTORD_FORWARD(pwm_D);
}

//    =A-----B=
//     |  =  |
//     |  =  |
//    =C-----D=
void STOP()
{
  MOTORA_STOP(Motor_PWM);
  MOTORB_STOP(Motor_PWM);
  MOTORC_STOP(Motor_PWM);
  MOTORD_STOP(Motor_PWM);
}

void UART_Control()
{
  String myString;
  char BT_Data = 0;
  // USB data
  /****
   * Check if USB Serial data contain brackets
   */

  if (SERIAL.available())
  {
    char inputChar = SERIAL.read();
    if (inputChar == '(') { // Start loop when left bracket detected
      myString = "";
      inputChar = SERIAL.read();
      while (inputChar != ')')
      {
        myString = myString + inputChar;
        inputChar = SERIAL.read();
        if (!SERIAL.available()) {
          break;
        }// Break when bracket closed
      }
    }
    int commaIndex = myString.indexOf(','); //Split data in bracket (a, b, c)
    //Search for the next comma just after the first
    int secondCommaIndex = myString.indexOf(',', commaIndex + 1);
    String firstValue = myString.substring(0, commaIndex);
    String secondValue = myString.substring(commaIndex + 1, secondCommaIndex);
    String thirdValue = myString.substring(secondCommaIndex + 1); // To the end of the string
    if ((firstValue.toInt() > servo_min and firstValue.toInt() < servo_max) and  //Convert them to numbers
        (secondValue.toInt() > servo_min and secondValue.toInt() < servo_max)) {
      pan = firstValue.toInt();
      tilt = secondValue.toInt();
      window_size = thirdValue.toInt();
    }
    SERIAL.flush();
    Serial3.println(myString);
    Serial3.println("Done");
    if (myString != "") {
      display.clearDisplay();
      display.setCursor(0, 0);     // Start at top-left corner
      display.println("Serial_Data = ");
      display.println(myString);
      display.display();
    }
  }







  //BT Control
  /*
    Receive data from app and translate it to motor movements
  */
  // BT Module on Serial 3 (D14 & D15)
  if (Serial3.available())
  {
    BT_Data = Serial3.read();
    SERIAL.print(BT_Data);
    Serial3.flush();
    BT_alive_cnt = 100;
    display.clearDisplay();
    display.setCursor(0, 0);     // Start at top-left corner
    display.println("BT_Data = ");
    display.println(BT_Data);
    display.display();
  }

  BT_alive_cnt = BT_alive_cnt - 1;
  if (BT_alive_cnt <= 0) {
    STOP();
  }
//   switch (BT_Data)
//   {
//     case 'A':  ADVANCE();  M_LOG("Run!\r\n"); break;
//     case 'B':  RIGHT_2();  M_LOG("Right up!\r\n");     break;
//     case 'C':  rotate_1();                            break;
//     case 'D':  RIGHT_3();  M_LOG("Right down!\r\n");   break;
//     case 'E':  BACK(500, 500, 500, 500);     M_LOG("Run!\r\n");          break;
//     case 'F':  LEFT_3();   M_LOG("Left down!\r\n");    break;
//     case 'G':  rotate_2();                              break;
//     case 'H':  LEFT_2();   M_LOG("Left up!\r\n");     break;
//     case 'Z':  STOP();     M_LOG("Stop!\r\n");        break;
//     case 'z':  STOP();     M_LOG("Stop!\r\n");        break;
//     case 'd':  LEFT_2();   M_LOG("Left!\r\n");        break;
//     case 'b':  RIGHT_2();  M_LOG("Right!\r\n");        break;
//     case 'L':  Motor_PWM = 1500;                      break;
//     case 'M':  Motor_PWM = 500;                       break;
//   }
}



/*Voltage Readings transmitter
Sends them via Serial3*/
void sendVolt(){
    newV = analogRead(A0);
    if(newV!=oldV) {
      if (!Serial3.available()) {
        Serial3.println(newV);
        Serial.println(newV);
      }
    }
    oldV=newV;
}







//Where the program starts
void setup()
{
  SERIAL.begin(115200); // USB serial setup
  SERIAL.println("Start");
  STOP(); // Stop the robot
  Serial3.begin(9600); // BT serial setup
  //Pan=PL4=>48, Tilt=PL5=>47
   servo_pan.attach(48);
   servo_tilt.attach(47);
  //////////////////////////////////////////////
  //OLED Setup//////////////////////////////////
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
  }
  display.clearDisplay();
  display.setTextSize(2);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.cp437(true);         // Use full 256 char 'Code Page 437' font
  display.setCursor(0, 0);     // Start at top-left corner
  display.println("AI Robot");
  display.display();


////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
  /*
    BELOW BY GROUP C4-C
  */
  pinMode(echoPinL, INPUT);
  pinMode(trigPinL, OUTPUT);
  pinMode(echoPinR, INPUT);
  pinMode(trigPinR, OUTPUT);

  calibrate();
  int_adc0_c = 1000;
  int_adc1_c = 1000;
  int_adc0_m = 10;
  int_adc1_m = 10;
}


////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
/*
  BELOW BY GROUP C4-C
*/
void calibrate() {
    Serial.begin(115200); 
    // Serial.println("*******************");
    // Serial.println("START AUTO_Control");
    // Serial.println("*******************");
    // Serial.println("Calibrating the sensors...");
    // int_adc0=analogRead(LightPinL);   // Left sensor at ambient light intensity
    // int_adc1=analogRead(LightPinR);   // Right sensor at ambient light intensity
    // Serial.println(int_adc0);
    // Serial.println(int_adc1);
    // delay(500);
    // int_adc0_c=analogRead(LightPinL);   // Left sensor at zero light intensity
    // int_adc1_c=analogRead(LightPinR);   // Right sensor at zero light intensity
    // Serial.println(int_adc0_c);
    // Serial.println(int_adc1_c);

    // // calculate the slope of light intensity to ADC reading equations  
    // int_adc0_m=(int_adc0-int_adc0_c)/100;
    // int_adc1_m=(int_adc1-int_adc1_c)/100;

    // Serial.println(int_adc0_m);
    // Serial.println(int_adc1_m);
    // while(1) {

    // }
    //calibrate the gyroscope
    Wire.begin();
    mpu.begin();
    Serial.println("\nCalculating gyro offset, do not move MPU6050");
    Serial.println("............");
    delay(1000);         //delay 1000ms waiting for calibration of gyroscope to complete
    Serial.println("Done\n*******************");
    
    // while ( 1 ) {
    //   last_avg_light_intensity = avg_light_intensity;
    //   avg_light_intensity = (analogRead(LightPinL) + analogRead(LightPinR)) / 2;
    //   Serial.println("bruhhhhhhhhhhhhhh");
    //   Serial.println(analogRead(LightPinL));
    //   Serial.println(analogRead(LightPinR));
    //   Serial3.println("bruhhhhhhhhhhhhhh");
    //   Serial3.println(analogRead(LightPinL));
    //   Serial3.println(analogRead(LightPinR));
    //   if ( analogRead(LightPinL) - analogRead(LightPinR) > 150 ) {
    //     RIGHT_2(320, 320, 300, 300);
    //   }
    //   else if (analogRead(LightPinR) - analogRead(LightPinL) > 70 ) {
    //     LEFT_2(320, 320, 300, 300);
    //   } else {
    //     if (abs(avg_light_intensity - last_avg_light_intensity) < 20){
    //       if ( avg_light_intensity < 600 ) {
    //         LEFT_2(320, 320, 300, 300);
    //       } else {
    //         park_cnt++;
    //         STOP();
    //       }
    //     }
    //     else if (avg_light_intensity > last_avg_light_intensity){
    //       park_cnt = 0;
    //       LEFT_2(320, 320, 300, 300);
    //     }
    //     else if (avg_light_intensity < last_avg_light_intensity){
    //       park_cnt = 0;
    //       RIGHT_2(320, 320, 300, 300);
    //     }
    //   }
    //   delay(80);
    //   STOP();
    // }
    // while( 1 ) {
      
    // }
    STATE = TASKTYPE::ALIGNMENT;
    // STATE = TASKTYPE::MOVEANDROTATE;
    // MOVEMENT = MOVEMENTTYPE::ROTATING1;
    // STATE = TASKTYPE::MEASUREMENT;
    // STATE = TASKTYPE::PARKING;
    // PARKING = PRAKINGSTATE::TRANSIT;
}

void get_gyro() {
    //once call Dataupdate() update the gyroscope info 
    mpu.update();
    pitch = mpu.getAngleX();
    roll = mpu.getAngleY();
    yaw = mpu.getAngleZ();
}

void get_distance() {
    long durationL;
    long durationR;
    if (done) {
        // reset start_time only if the distance has been measured 
        // in the last invocation of the method
        done = 0;
        start_time = millis();
        digitalWrite(trigPinL, LOW);
        digitalWrite(trigPinR, LOW);
    }
    
    if (millis() > start_time + 2) { 
        digitalWrite(trigPinL, HIGH);
        digitalWrite(trigPinR, HIGH);
    }
    
    if (millis() > start_time + 10) {
        digitalWrite(trigPinL, LOW);
        durationL = pulseIn(echoPinL, HIGH);
        L_sonar_dist = (durationL / 2.0) / 29.1;
        if (L_sonar_dist > 30.0)
          L_sonar_dist = (durationL / 2.0) / 29.1 + 1.5;
        else if (L_sonar_dist < 30.0)
          L_sonar_dist = (durationL / 2.0) / 29.1 + 2.0;

        digitalWrite(trigPinR, LOW);
        durationR = pulseIn(echoPinR, HIGH);
        R_sonar_dist = (durationR / 2.0) / 29.1;
        done = 1;
    }

}

void get_light() {
  // calculate the light intensity of the sensors
  // in the range of [0, 100]
  if (avg_light_intensity > max_light_intensity){
    max_light_intensity = avg_light_intensity;
    yaw_at_max_light_intensity = yaw;
  }

  last_avg_light_intensity = avg_light_intensity;
  light_left = analogRead(LightPinL);
  light_right = analogRead(LightPinR);
  avg_light_intensity = (light_left+light_right) / 2;
  Serial.println(light_left);
  Serial.println(light_right);
  if (light_left < int_adc0_c) {
    int_adc0_c = light_left;
  }
  if (light_right < int_adc1_c) {
    int_adc1_c = light_right;
  }
  if (light_left > int_adc0_m) {
    int_adc0_m = light_left;
  }
  if (light_right > int_adc1_m) {
    int_adc1_m = light_right;
  }
  int_left = (light_left - int_adc0_c) / int_adc0_m;
  int_right = (light_right - int_adc1_c) / int_adc1_m;
  Serial.print("max: ");
  Serial.println(max_light_intensity);
  Serial.print("yaw: ");
  Serial.println(yaw);
  Serial.print("avg: ");
  Serial.println(avg_light_intensity);
  Serial.print("int_left: ");
  Serial.println(int_left);
  Serial.print("int_right: ");
  Serial.println(int_right);
}

//TODO
// align along the wall
void Alignment(){
    DataUpdate();
    // alignment_cnt is for checking 5 consective correct alignment
    if (alignemnt_cnt >= 5){
      STOP();
      STATE = TASKTYPE::MOVEANDROTATE;
      delay(2000);
      return;
    }
    if (abs(L_sonar_dist - R_sonar_dist) < 1.4){
      STOP();
      Serial.println("Alignment done!"); // change to "Alignment done once?"
      alignemnt_cnt++;

      // maybe can delete, because this if only for faster return
      if (alignemnt_cnt >= 5){
        STOP();
        return;
      }
    }
    else if ((L_sonar_dist - R_sonar_dist) > 1.4){
        alignemnt_cnt = 0;
        Motor_PWM = 300;
        rotate_2(300, 300, 300, 300);
    }
    else if ((L_sonar_dist - R_sonar_dist) < -1.4){
        alignemnt_cnt = 0;
        Motor_PWM = 300;
        rotate_1(300, 300, 300, 300);
    }

    delay(150);
    STOP();
}

// rotate 90 degree
void Rotation_90degree(bool is_CW){
  // if (avg_light_intensity > max_light_intensity){
  //   max_light_intensity = avg_light_intensity;
  //   yaw_at_max_light_intensity = yaw;
  // }
  Serial.print("angle_set: ");
  Serial.println(angle_set);
  if ( is_CW ) {
    rotate_2(300, 300, 300, 300);
    delay(1250);
  } else {
    rotate_1(300, 300, 300, 300);
    delay(1150);
  }
  STOP();
}

// rotate based on the angle_set
void Rotate(bool is_CW){
  // if (avg_light_intensity > max_light_intensity){
  //   max_light_intensity = avg_light_intensity;
  //   yaw_at_max_light_intensity = yaw;
  // }
  // Serial.print("angle_set: ");
  // Serial.println(angle_set);
  if ( is_CW ) {
    rotate_2(300, 300, 300, 300);
    // delay(80);
  } else {
    rotate_1(300, 300, 300, 300);
    // delay(1150);
  }
  delay(200);
  STOP();
}


/**   *@TODO: PWM control   **/
void MoveAndRotate(){
  DataUpdate();
  switch (MOVEMENT)
  {
  case MOVEMENTTYPE::LOCATING1: // get 25cm close to the wall and rough locating (+-1.5)
    Serial.println("MOVEMENT: LOCATING1");
    Serial.println(locating_cnt);
    if (locating_cnt >= 5){
      STOP();
      Serial.println(locating_cnt);
      MOVEMENT = MOVEMENTTYPE::LOCATING2;
      locating_cnt = 0;
      return;
    }
    // if consectively pass this condition 5 times, trans to next stage LOCATING2
    if (((23.0 < L_sonar_dist) && (L_sonar_dist < 27.0)) || ((23.0 < R_sonar_dist) && ( R_sonar_dist < 27.0))){ // 23.5 < left_dist&right_dist < 26.5 
      STOP(); // maybe can remove this line due to line 642?
      locating_cnt++;
    }
    else if ((L_sonar_dist > 26.0) || (R_sonar_dist > 26.0)){ // too close
      locating_cnt = 0;
      ADVANCE(300, 300, 300, 300);
    }
    else if ((L_sonar_dist < 24.0) || (R_sonar_dist < 24.0)){ // too far away
      locating_cnt = 0;
      BACK(300, 300, 300, 300);
    }
    delay(100);
    STOP();
    break;
  case MOVEMENTTYPE::LOCATING2: // fine locating (+- 0.5)
    Serial.println("MOVEMENT: LOCATING2");
    if (locating_cnt >= 5){
      STOP();
      MOVEMENT = MOVEMENTTYPE::NEXTSTAGE_TRANSITION;
      locating_cnt = 0;
      delay(2000);
      return;
    }
    if (abs(L_sonar_dist - R_sonar_dist) < 1.0){
      STOP();
      Serial.println(locating_cnt);
      Serial.println("Alignment done!");
      locating_cnt++;
      if (locating_cnt >= 5){
        STOP();
        return;
      }
    }
    else if ((L_sonar_dist - R_sonar_dist) > 0.5){
        locating_cnt = 0;
        rotate_2(300, 300, 300, 300);
    }
    else if ((L_sonar_dist - R_sonar_dist) < -0.5){
        locating_cnt = 0;
        Serial.println("ROTATE1");
        rotate_1(300, 300, 300, 300);
    }
    delay(40);
    STOP();
    break;
  case MOVEMENTTYPE::NEXTSTAGE_TRANSITION:
    Serial.println("MOVEMENT: NEXTSTAGE_TRANSITION");
    STOP();
    delay(2000);
    MOVEMENT = MOVEMENTTYPE::ROTATING1;
    break;

  /**    @TODO: calibrate the light sensors during the rotation   **/ 
  case MOVEMENTTYPE::ROTATING1: // do the rotation tasks: CW 90 degree
    Serial.println("MOVEMENT: ROTATING1");
    // CW 90
    // init angle_set
    if (rotate_cnt < 1) {
      angle_set = yaw-90.0;
      Rotation_90degree(true);
      rotate_cnt++;
    } else {
      MOVEMENT = MOVEMENTTYPE::ROTATING2;
      rotate_cnt = 0;
      STOP();
      delay(2000);
    }
    delay(100);
    STOP();
    break;
  case MOVEMENTTYPE::ROTATING2: // do the rotation tasks: CCW 270 degree
    Serial.println("MOVEMENT: ROTATING2");
    // CCW 270
    // init new angel_set
    if (rotate_cnt < 1){
      angle_set = yaw+270.0;
      Rotation_90degree(false);
      rotate_cnt++;
      delay(100);
    }
    if ( rotate_cnt < 3 ) {
      Rotation_90degree(false);
      rotate_cnt++;
    } else {
      // // ambient light intensity as dark intensity
      // int_adc0_c=analogRead(LightPinL);   // Left sensor at zero light intensity
      // int_adc1_c=analogRead(LightPinR);   // Right sensor at zero light intensity
      // Serial.println(int_adc0);
      // Serial.println(int_adc1);
      MOVEMENT = MOVEMENTTYPE::ROTATING3;
      rotate_cnt = 0;
      STOP();
      delay(2000);
    }
    delay(100);
    STOP();
    break;
  case MOVEMENTTYPE::ROTATING3: // do the rotation tasks: CW 180 degree
    Serial.println("MOVEMENT: ROTATING3");
    // CW 180
    if (rotate_cnt < 1){
      angle_set = yaw-180.0;
      Rotation_90degree(true);
      rotate_cnt++;
      delay(500);
    }
    if ( rotate_cnt < 2 ) {
      Rotation_90degree(true);
      rotate_cnt++;
    } else {
      // // light intensity as ambient light intensity
      // int_adc0=analogRead(LightPinL);   // Left sensor at ambient light intensity
      // int_adc1=analogRead(LightPinR);   // Right sensor at ambient light intensity
      // Serial.println(int_adc0);
      // Serial.println(int_adc1);
      // int_adc0_m=(int_adc0-int_adc0_c)/10;
      // int_adc1_m=(int_adc1-int_adc1_c)/10;
      // Serial.println(int_adc0_m);
      // Serial.println(int_adc1_m);
      MOVEMENT = MOVEMENTTYPE::REALIGNMENT1;
      rotate_cnt = 0;
      STOP();
      delay(2000);
      Serial.println(abs(yaw - angle_set));
      Serial.println(angle_set);
      Serial.println(yaw);
    }
    delay(100);
    STOP();
    break;  
  case MOVEMENTTYPE::REALIGNMENT1:
    Serial.println("MOVEMENT: REALGINMENT1");
    Serial.println(locating_cnt);
    if (locating_cnt >= 5){
      STOP();
      Serial.println(locating_cnt);
      MOVEMENT = MOVEMENTTYPE::REALIGNMENT2;
      locating_cnt = 0;
      return;
    }
    // if consectively pass this condition 5 times, trans to next stage REALIGNMENT2
    if (((23.5 < L_sonar_dist) && (L_sonar_dist < 26.5)) || ((23.5 < R_sonar_dist) && ( R_sonar_dist < 26.5))){ // 23.5 < left_dist&right_dist < 26.5 
      STOP(); // 
      locating_cnt++;
    }
    else if ((L_sonar_dist > 26.0) || (R_sonar_dist > 26.0)){ // too far away
      locating_cnt = 0;
      ADVANCE(300, 300, 300, 300);
    }
    else if ((L_sonar_dist < 24.0) || (R_sonar_dist < 24.0)){ // too close
      locating_cnt = 0;
      BACK(300, 300, 300, 300);
    }
    delay(50);
    STOP();
    break;

  case MOVEMENTTYPE::REALIGNMENT2:
    Serial.println("MOVEMENT: REALIGNMENT2");
    Serial.println(locating_cnt);
    if (locating_cnt >= 5){
      Serial.println(locating_cnt);
      MOVEMENT = MOVEMENTTYPE::END_MOVEMENT;
      STOP();
      delay(2000);
      return;
    } 
    if (abs(L_sonar_dist - R_sonar_dist) < 1.0){
      STOP();
      Serial.println("Movement done!");
      locating_cnt++;
      if (locating_cnt >= 5){
        STOP();
        return;
      }
    }
    else if ((L_sonar_dist - R_sonar_dist) > 0.5){
      locating_cnt = 0;
      rotate_2(300, 300, 300, 300);
    }
    else if ((L_sonar_dist - R_sonar_dist) < -0.5){
      locating_cnt = 0;
      rotate_1(300, 300, 300, 300);
    }
    delay(50);
    STOP();
    break;

  case MOVEMENTTYPE::END_MOVEMENT:
    Serial.println("MOVEMENT: END_MOVEMENT");
    STOP();
    STATE = TASKTYPE::MEASUREMENT;
    break;
  default:
    break;
  }
}

void Measurement(){
  if (cnt == 10){
    delay(2000); // wait 2s after the measurement
    distance_to_wall = distance_to_wall_buf / cnt;
    angle_of_vehicle = angle_of_vehicle_buf / cnt;
    Serial.println("MEASUREMENT");
    Serial.println(distance_to_wall);
    Serial.println(angle_of_vehicle);

    Serial3.println("MEASUREMENT");
    Serial3.println(distance_to_wall);
    Serial3.println(angle_of_vehicle);
    // Serial3.println(cnt);

    STATE = TASKTYPE::PARKING;
    return;
  }
  STOP();
  DataUpdate();
  // measure the distance from the sonar sensors
  distance_to_wall_buf += (L_sonar_dist + R_sonar_dist) / 2;
  // measure the angle of the vehicle to the wall
  angle_of_vehicle_buf += abs(L_sonar_dist - R_sonar_dist) / 2;//arctan L-R /x
  cnt++;
  // Serial.println("MEASUREMENT");
  // Serial.println(cnt);
  // Serial3.println("MEASUREMENT");
  // Serial3.println(cnt);
}

void Parking(){
  // park the robot
  //TODO
  DataUpdate();
  switch (PARKING)
  {
  case PRAKINGSTATE::FORWARD:
    Serial.println("PARKING: FORWARD\n*******************");
    Serial.println(park_cnt);
    if (park_cnt >= 5){
      STOP();
      PARKING = PRAKINGSTATE::ROTATE;
      park_cnt = 0;
      return;
    }
    // if consectively pass this condition 5 times, trans to next stage LOCATING2
    if (((5.0 < L_sonar_dist) && (L_sonar_dist < 7.0)) || ((5.0 < R_sonar_dist) && ( R_sonar_dist < 7.0))){ // 9.0 < left_dist&right_dist < 11.0 sonar not accurate, 5cm => 10cm
      STOP(); 
      park_cnt++;
    }
    else if ((L_sonar_dist > 6.5) || (R_sonar_dist > 6.5)){ // too close
      park_cnt = 0;
      ADVANCE(300, 300, 300, 300);
    }
    else if ((L_sonar_dist < 5.5) || (R_sonar_dist < 5.5)){ // too far away
      park_cnt = 0;
      BACK(300, 300, 300, 300);
    }
    delay(70);
    STOP();
    break;

  case PRAKINGSTATE::ROTATE:
    Serial.println("PARKING: ROTATE\n*******************");
    if (park_cnt >= 5){
      STOP();
      PARKING = PRAKINGSTATE::TRANSIT;
      park_cnt = -1;
      int_adc0_c=analogRead(LightPinL);   // Left sensor at zero light intensity
      int_adc1_c=analogRead(LightPinR);   // Right sensor at zero light intensity
      int_adc0_m=analogRead(LightPinL);   // Left sensor at ambient light intensity
      int_adc1_m=analogRead(LightPinR);   // Right sensor at ambient light intensity
      delay(2000);
      return;
    }
    // if consectively pass this condition 5 times, trans to next stage NEXTSTAGE_TRANSITION
    if (abs(L_sonar_dist - R_sonar_dist) < 0.8){
      STOP();
      Serial.println(park_cnt);
      Serial.println("Parking done!");
      park_cnt++;
      if (park_cnt >= 5){
        STOP();
        return;
      }
    }
    else if (L_sonar_dist - R_sonar_dist > 0.5){
        park_cnt = 0;
        Motor_PWM = 300;
        rotate_2(300, 300, 300, 300);
    }
    else if (L_sonar_dist - R_sonar_dist < -0.5){
        park_cnt = 0;
        Motor_PWM = 300;
        Serial.println("ROTATE1");
        rotate_1(300, 300, 300, 300);
    }
    delay(40);
    STOP();
    break;
  case PRAKINGSTATE::TRANSIT:
    Serial.println("PARKING: TRANSIT\n*******************");
    if (park_cnt == -1){
      // LEFT_2(320, 320, 300, 300);
      // delay(900);
      park_cnt = 0;
      Rotate(false);
      STOP();
    }
    // if ( park_cnt == 0 )
    if (park_cnt >= 5){
      STOP();
      PARKING = PRAKINGSTATE::END_PARKING;
      park_cnt = 0;
      delay(2000);
      return;
    }
    if ( analogRead(LightPinL) - analogRead(LightPinR) > 150 ) {
      RIGHT_2(320, 320, 300, 300);
      // delay(100);
      // Rotate(false);
    }
    else if (analogRead(LightPinR) - analogRead(LightPinL) > 70 ) {
      LEFT_2(320, 320, 300, 300);
      // delay(100);
      // Rotate(true);
    } else {
      if (abs(avg_light_intensity - last_avg_light_intensity) < 20){
        if ( avg_light_intensity < 600 ) {
          LEFT_2(320, 320, 300, 300);
          // delay(100);
          // Rotate(true);
        } else {
          park_cnt++;
          STOP();
        }
      }
      else if (avg_light_intensity > last_avg_light_intensity){
        park_cnt = 0;
        LEFT_2(320, 320, 300, 300);
        // delay(100);
        // Rotate(true);
      }
      else if (avg_light_intensity < last_avg_light_intensity){
        park_cnt = 0;
        RIGHT_2(320, 320, 300, 300);
        // delay(100);
        // Rotate(false);
      }
    }
    delay(100);
    STOP();
    // if (int_left - int_right > 10){
    //   park_cnt = 0;
    //   LEFT_2(320, 320, 300, 300);
    // }
    // else if (int_right - int_left > 10){
    //   park_cnt = 0;
    //   RIGHT_2(320, 320, 300, 300);
    // }
    // else {
    //   park_cnt++;
    //   STOP();
    // }
    // delay(100);
    // STOP();
    break;

  case PRAKINGSTATE::END_PARKING:
    Serial.println("PARKING: END_PARKING\n*******************");
    if (abs(L_sonar_dist - R_sonar_dist) < 0.8){
      STOP();
      Serial.println(park_cnt);
      Serial.println("Parking done!");
      park_cnt++;
      if (park_cnt >= 5){
        STOP();
        STATE = TASKTYPE::END_STATE;
        delay(2000);
        return;
      }
    }
    else if (L_sonar_dist - R_sonar_dist > 0.5){
        park_cnt = 0;
        // Motor_PWM = 300;
        rotate_2(300, 300, 300, 300);
    }
    else if (L_sonar_dist - R_sonar_dist < -0.5){
        park_cnt = 0;
        // Motor_PWM = 300;
        // Serial.println("ROTATE1");
        rotate_1(300, 300, 300, 300);
    }
    delay(40);
    STOP();
    break;
  }
}

void DataUpdate(){
  // update the data
  // get the distance from the sonar sensors
  get_distance();
  // get the light intensity from the light sensors
  get_light();
  // get the gyroscope axis x, y and z information
  get_gyro();

  //debug
  Serial.print("Left sonar distance = ");
  Serial.print(L_sonar_dist);
  Serial.print(";  Right sonar distance = ");
  Serial.println(R_sonar_dist);

  Serial.print("PWM: ");
  Serial.println(Motor_PWM);

  // Serial.print("Left sensor intensity = ");
  // Serial.print(int_right);
  // Serial.print(";  Right sensor intensity = ");
  // Serial.println(int_left);

  Serial.print("Pitch (Angle_X) : ");
  Serial.print(pitch);
  Serial.print("  Roll  (Angle_Y) : ");
  Serial.print(roll);
  Serial.print("  Yaw   (Angle_Z) : ");
  Serial.println(yaw);



  // BT Serial
  if (!Serial3.available()){
  Serial3.print("\nL_sonar_dist: ");
  Serial3.print(L_sonar_dist);
  Serial3.print(",");
  Serial3.print("R_sonar_dist: ");
  Serial3.println(R_sonar_dist);
  
  Serial3.print("\nLeft_light: ");
  Serial3.print(int_left);
  Serial3.print(",");
  Serial3.print("Right_light: ");
  Serial3.println(int_right);

  Serial3.print("\nGyro_pitch: ");
  Serial3.print(pitch);
  Serial3.print(",");
  Serial3.print("Gyro_roll: ");
  Serial3.print(roll);
  Serial3.print(",");
  Serial3.print("Gyro_yaw: ");
  Serial3.println(yaw);
  }
}

void AUTO_Control(){
//   // Basic structure of the AUTO_Control Start
  switch (STATE)
  {
    case TASKTYPE::INIT:
      Serial.print("*******************\nSTATE: INIT\n");
      break;
    case TASKTYPE::ALIGNMENT: 
      Serial.print("*******************\nSTATE: Alignment\n");
      Alignment();
      break;
    case TASKTYPE::MOVEANDROTATE:
      Serial.print("*******************\nSTATE: MoveAndRotate\n");
      MoveAndRotate();
      break;
    case TASKTYPE::MEASUREMENT:
      Serial.print("*******************\nSTATE: Measurement\n");
      Measurement();
      break;
    case TASKTYPE::PARKING:
      Serial.print("*******************\nSTATE: Parking\n");
      Parking();
      break;
    case TASKTYPE::END_STATE:
      STOP();
      break;
  default:
    STOP();
    break;
  }
  // delay(100);
  // Basic structure of the AUTO_Control End
}

/*
  ABOVE BY GROUP C4-C  
*/
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////



void loop()
{
  // run the code in every 20ms
  if (millis() > (time + 15)) {
    voltCount++;
    time = millis();
//    UART_Control(); //get USB and BT serial data
  //  DataUpdate();
  //  delay(500);
   AUTO_Control();
//test
// DataUpdate();
// if (rotate_cnt == 0){
//   rotate_cnt = 1;
//   angle_set = yaw-90.0;
//   }
//   Rotation();

//test



    //constrain the servo movement
    pan = constrain(pan, servo_min, servo_max);
    tilt = constrain(tilt, servo_min, servo_max);
    
    //send signal to servo
    servo_pan.write(pan);
    servo_tilt.write(tilt);
  }if (voltCount>=5){
    voltCount=0;
    sendVolt();
  }
}
