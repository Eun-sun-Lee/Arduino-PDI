#include <Servo.h>

/////////////////////////////
// Configurable parameters //
/////////////////////////////

// Arduino pin assignment
#define PIN_LED 9 
#define PIN_SERVO 10 
#define PIN_IR A0 

// Framework setting
#define _DIST_TARGET 255
#define _DIST_MIN 100 
#define _DIST_MAX 410 


// Distance sensor
#define _DIST_ALPHA 0.4

//filter
#define LENGTH 30
#define k_LENGTH 8

// Servo range
#define _DUTY_MIN 200  // servo full clockwise position (0 degree)
#define _DUTY_NEU 1100  // servo neutral position (90 degree)
#define _DUTY_MAX 2000  // servo full counterclockwise position (180 degree)

// Servo speed control
#define _SERVO_ANGLE 30 
#define _SERVO_SPEED 200

// Event periods
#define _INTERVAL_DIST 20  // 거리측정주기 (ms)
#define _INTERVAL_SERVO 20 // 서보제어주기 (ms)
#define _INTERVAL_SERIAL 100 // Serial제어주기 (ms)

// PID parameters
#define _KP 0.9
#define _KD 10.0
#define _KI 0.01





//////////////////////
// global variables //
//////////////////////

// Servo instance
Servo myservo;

// Distance sensor
float dist_target; // location to send the ball
float dist_raw, dist_ema;  
float last_sampling_dist; 
int correction_dist, iter;
float dist_list[LENGTH], sum;
const float coE[] = { 0.0000289, -0.0104888, 2.1965047, -5.3746664 };
//{-0.0000154, 0.0097404, -0.4662654, 98.6667701};
//{-0.0000195, 0.0114817, -0.6870302, 108.4335836};
//{-0.0000070, 0.0038146, 0.6865941, 40.9151084};

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial;
bool event_dist, event_servo, event_serial;

// Servo speed control
int duty_chg_per_interval;
int duty_target, duty_curr;

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;

void setup() {
    // initialize GPIO pins for LED and attach servo
    pinMode(PIN_LED, OUTPUT);  
    myservo.attach(PIN_SERVO); 
    // initialize global variables
    dist_target = _DIST_TARGET;
    control = _DUTY_NEU;
    duty_curr = _DUTY_NEU;
    last_sampling_dist = 0;
    dist_raw = 0;
    dist_ema = 0;
    iterm = 0;

    correction_dist = 0;
    iter = 0; sum = 0;

    // move servo to neutral position
    myservo.writeMicroseconds(_DUTY_NEU);
    // initialize serial port
    Serial.begin(57600); 
    // convert angle speed into duty change per interval.
    duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED / 180.0) * (_INTERVAL_SERVO / 1000.0); // 서보 업데이트 1주기에 증감 가능한 duty 의 최댓값

   
    last_sampling_time_dist = 0;
    last_sampling_time_servo = 0;
    last_sampling_time_serial = 0;

    event_dist = false;
    event_servo = false;
    event_serial = false;
}

void loop() {
    /////////////////////
    // Event generator //
    /////////////////////
    
    if (millis() >= last_sampling_time_dist + _INTERVAL_DIST)
        event_dist = true;

  
    if (millis() >= last_sampling_time_servo + _INTERVAL_SERVO)
        event_servo = true;

  
    if (millis() >= last_sampling_time_serial + _INTERVAL_SERIAL)
        event_serial = true;

    ////////////////////
    // Event handlers //
    ////////////////////

    if (event_dist) {
        event_dist = false;

        // get a distance reading from the distance sensor
        dist_ema = ir_distence_filter();
        //update error_prev
        error_prev = error_curr;
        // PID control logic
        error_curr = dist_target - dist_ema; 
        pterm = _KP * error_curr; 
        dterm = _KD * (error_curr - error_prev);
        iterm += _KI * error_curr;
        control = pterm + dterm + iterm;

        duty_curr = _DUTY_NEU + control;

        // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
        duty_curr = min(max(duty_curr, _DUTY_MIN), _DUTY_MAX);

        
        last_sampling_time_dist += _INTERVAL_DIST;
    }

    if (event_servo) {
        event_servo = false;

       
        if (duty_target > duty_curr) {
            duty_curr += duty_chg_per_interval;
            if (duty_target < duty_curr) duty_curr = duty_target;
        }
        else {
            duty_curr -= duty_chg_per_interval;
            if (duty_target > duty_curr) duty_curr = duty_target;
        }
        // update servo position
        myservo.writeMicroseconds(duty_curr);
        delay(50);

        
        last_sampling_time_servo += _INTERVAL_SERVO;
    }

    if (event_serial) {
        event_serial = false;
        Serial.print("IR:");
        Serial.print(dist_ema);
        Serial.print(",T:");
        Serial.print(dist_target);
        Serial.print(",P:");
        Serial.print(map(pterm, -1000, 1000, 510, 610));
        Serial.print(",D:");
        Serial.print(map(dterm, -1000, 1000, 510, 610));
        Serial.print(",I:");
        Serial.print(map(iterm, -1000, 1000, 510, 610));
        Serial.print(",DTT:");
        Serial.print(map(duty_target, 1000, 2000, 410, 510));
        Serial.print(",DTC:");
        Serial.print(map(duty_curr, 1000, 2000, 410, 510));
        Serial.println(", -G:245,+G:265,m:0,M:800");

        
        last_sampling_time_serial += _INTERVAL_SERIAL;
    }
}
float ir_distance(void) { // return value unit: mm
    float val;
    float volt = float(analogRead(PIN_IR));
    val = ((6762.0 / (volt - 9.0)) - 4.0) * 10.0;
    dist_raw = coE[0] * pow(val, 3) + coE[1] * pow(val, 2) + coE[2] * val + coE[3];

    if (dist_raw >= _DIST_MIN && dist_raw <= _DIST_MAX) {
        last_sampling_dist = dist_raw;
    }
    else {
        dist_raw = last_sampling_dist;
    }
    return dist_raw;
}

float ir_distence_filter() {
    sum = 0;
    iter = 0;
    while (iter < LENGTH)
    {
        dist_list[iter] = ir_distance();
        sum += dist_list[iter];
        iter++;
    }

    for (int i = 0; i < LENGTH - 1; i++) {
        for (int j = i + 1; j < LENGTH; j++) {
            if (dist_list[i] > dist_list[j]) {
                float tmp = dist_list[i];
                dist_list[i] = dist_list[j];
                dist_list[j] = tmp;
            }
        }
    }

    for (int i = 0; i < k_LENGTH; i++) {
        sum -= dist_list[i];
    }
    for (int i = 1; i <= k_LENGTH; i++) {
        sum -= dist_list[LENGTH - i];
    }

    float dist_cali = sum / (LENGTH - 2 * k_LENGTH);

    return _DIST_ALPHA * dist_cali + (1 - _DIST_ALPHA) * dist_ema;
}
