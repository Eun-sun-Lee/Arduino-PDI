#include <Servo.h>

#define PIN_LED 9 
#define PIN_SERVO 10 
#define PIN_IR A0 

#define _DIST_TARGET 190
#define _DIST_MIN 100 
#define _DIST_MAX 410 

#define LENGTH 40
#define k_LENGTH 8

// Servo range
#define _DUTY_MIN 1160  // servo full clockwise position (0 degree)
#define _DUTY_NEU 1390  // servo neutral position (90 degree)
#define _DUTY_MAX 1900  // servo full counterclockwise position (180 degree)

// Servo speed control
#define _SERVO_ANGLE 30 
#define _SERVO_SPEED 100 

#define _INTERVAL_DIST 20  // 거리측정주기 (ms)
#define _INTERVAL_SERVO 20 // 서보제어주기 (ms)
#define _INTERVAL_SERIAL 100 // Serial제어주기 (ms)

#define _KP 0.7
#define _UP_KP 1.2
#define _DOWN_KP 0.4



#define EMA_ALPHA 0.5
#define DELAY_MICROS  1500

Servo myservo;

// Distance sensor
float dist_target; // location to send the ball
int correction_dist, iter;
float dist_list[LENGTH], sum;
float dist_raw, dist_ema;  
const float coE[] = {-0.0000004, -0.0004444, 1.2066632, 24.0073521};

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; 
bool event_dist, event_servo, event_serial;

// Servo speed control
int duty_chg_per_interval; 
int duty_target, duty_curr;

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;

float filtered_dist;
float ema_dist = 0;
float samples_num = 3;

float servoAngle = _DUTY_NEU;

float ir_distance(void) 
{
    float val;
    float volt = float(analogRead(PIN_IR));
    val = ((6762.0 / (volt - 9.0)) - 4.0) * 10.0;
    return val;
}

float under_noise_filter(void) 
{
    int currReading;
    int largestReading = 0;
    for (int i = 0; i < samples_num; i++)
    {
        currReading = ir_distance();
        if (currReading > largestReading)
            largestReading = currReading;

        delayMicroseconds(DELAY_MICROS);
    }
    return largestReading;
}

float filtered_ir_distance(void) {
    int currReading;
    int lowestReading = 1024;
    for (int i = 0; i < samples_num; i++) {
        currReading = under_noise_filter();
        if (currReading < lowestReading) { lowestReading = currReading; }
    }
    // ema 필터 추가
    ema_dist = EMA_ALPHA * lowestReading + (1 - EMA_ALPHA) * ema_dist;
    return ema_dist;
}

void setup()
{
    Serial.begin(57600);
    myservo.attach(PIN_SERVO);
    myservo.writeMicroseconds(_DUTY_NEU);
    delay(1000);

    last_sampling_time_dist = last_sampling_time_serial = 0;
    event_dist = event_serial = false;
}
  
void loop() 
{
    unsigned long time_curr = millis();
    float pTerm = 0;

    if (time_curr >= last_sampling_time_dist + _INTERVAL_DIST) {
        last_sampling_time_dist += _INTERVAL_DIST;
        event_dist = true;
    }
    if (time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL) {
        last_sampling_time_serial += _INTERVAL_SERIAL;
        event_serial = true;
    }

    if (event_dist)
    {
        event_dist = false;
        dist_raw = filtered_ir_distance();

        error_curr = _DIST_TARGET - dist_raw;

        float kp = 1.0;

        if (error_curr < -150)
            kp = 1.0;
        else if (error_curr > 50)
            kp = 1.0;
        else
            kp = 0.3;

        pterm = _KP * error_curr;

        servoAngle = pterm;
        myservo.writeMicroseconds(servoAngle + _DUTY_NEU);
    }
  if(event_serial) {
    event_serial = false; 
    Serial.print("dist_ir:");
    Serial.print(dist_raw);
    Serial.print(",pterm:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",duty_target:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",duty_curr:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410");
  }
}
