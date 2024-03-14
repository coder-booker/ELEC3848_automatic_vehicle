
// light
#define TOL   50           // tolerance for adc different, avoid oscillation
#define K   5              // Step size
//variables for light intensity to ADC reading equations 
int int_adc0, int_adc0_m, int_adc0_c;
int int_adc1, int_adc1_m, int_adc1_c;
int int_left, int_right;


// Ultrasonic sensor
#define echoPin 11
#define trigPin 12
#define buzzPin 9

unsigned long start_time = 0;
int done = 1;
long distance_in_cm;


// include servo library if needed
#include <Servo.h>
#define SERVO_PIN 9        //servo connected at pin 9
Servo myservo;            // declare servo object

void calibrate() {
    // measure the sensors reading at ambient light intensity  
    Serial.println("Calibration in progress, put the sensors under the light (~ 5 sec) ......");
    Serial.println("***********************");
    delay(5000);        // delay 5000 ms

    int_adc0=analogRead(A0);   // Left sensor at ambient light intensity
    int_adc1=analogRead(A1);   // Right sensor at ambient light intensity
    Serial.print("Left : ");
    Serial.println(int_adc0);
    Serial.print("Right : ");
    Serial.println(int_adc1);
    delay(1000); 

    Serial.println("\nCalibration in progress, cover the sensors with your fingers (~ 8 sec to set)......");
    Serial.println("************ Put Fingers *****************");
    delay(5000);        // delay 5000 ms
    Serial.println("********* START Calibration **************");

    // measure the sensors reading at zero light intensity  
    int_adc0_c=analogRead(A0);   // Left sensor at zero light intensity
    int_adc1_c=analogRead(A1);   // Right sensor at zero light intensity

    // calculate the slope of light intensity to ADC reading equations  
    int_adc0_m=(int_adc0-int_adc0_c)/100;
    int_adc1_m=(int_adc1-int_adc1_c)/100;
    delay(10000);        // delay 10000 ms 
    
    Serial.println("\n******** Completed! Remove your hands ********");
    delay(4000);        // delay 4000 ms
}

void setup() 
{
    // put your setup code here, to run once:
    Serial.begin(115200);
    Serial.println();

    pinMode(echoPin, INPUT);
    pinMode(trigPin, OUTPUT);
    pinMode(buzzPin, OUTPUT);
    calibrate();
}

void get_light() {
    // calculate the light intensity of the sensors
    // in the range of [0, 100]
    int_left=(analogRead(A0)-int_adc0_c)/int_adc0_m;
    int_right=(analogRead(A1)-int_adc1_c)/int_adc1_m;  

    Serial.print("Left sensor intensity = ");
    Serial.print(int_right);
    Serial.print(";  Right sensor intensity = ");
    Serial.print(int_left);
}

void get_distance() {
    long duration;
    if (done) {
        // reset start_time only if the distance has been measured 
        // in the last invocation of the method
        done = 0;
        start_time = millis();
        digitalWrite(trigPin, LOW);
    }
    
    if (millis() > start_time + 2) { 
        digitalWrite(trigPin, HIGH);
    }
    
    if (millis() > start_time + 10) {
        digitalWrite(trigPin, LOW);
        duration = pulseIn(echoPin, HIGH);
        distance_in_cm = (duration / 2.0) / 29.1;
        done = 1;
    }
}

void loop() 
{
    get_light();
    get_distance();
    delay(100);        // delay 100 ms
  
}
