#include <Servo.h>
#include <FastLED.h>

#define LEFT_SERVO_UP 55
#define LEFT_SERVO_DOWN 18
#define RIGHT_SERVO_UP 50
#define RIGHT_SERVO_DOWN 85

#define SHOOTER_DIRECTION 1
#define KICKER_DIRECTION 1

#define RIGHT_BUTTON_PIN 22     // A4
#define BUTTON_STARTUP 50       // minimum ms needed of button push
#define BUTTON_COOLDOWN 90      // minimum ms between button presses

#define SHOOTER_PIN 9
#define KICKER_PIN 10
#define LEFT_SERVO_PIN 5
#define RIGHT_SERVO_PIN 6
#define SENSOR_PIN 19

#define LED_PIN 3
#define NUM_LEDS 3

#define ZERO_STREAK    20       // number of zeroes in a row required
#define ON_THRESH 1500          // high peak length in microseconds
#define RC_UPPER_THRESH 2500    // max high peak length in microseconds
#define RECEIVER_THRESH 100    // maximum milliseconds between high peaks

#define BEAM_BREAK_THRESH 50    // milliseconds of beam break required

#define SHOOT_TIME  500         // milliseconds after beam connected

#define STARTUP 1000            // milliseconds until code runs

#define DEFAULT_STAGE 0
#define INTAKE_STAGE 1
#define SPINUP_STAGE 2
#define SHOOT_STAGE 3

Servo LeftServo, RightServo;
Servo Shooter, Kicker;

int stage = DEFAULT_STAGE;

// Buttons
bool right_pressed = false;
bool right_prev_pressed = false;

unsigned long start_of_press = 0;
unsigned long last_release = 0;

bool stage_switch_ready = true;
unsigned long last_stage_switch = 0;

// Puck sensor
bool sensor_prev_reading = true;
unsigned long beam_first_tripped = 0;
unsigned long beam_first_connected = 0;

// PWM reading
unsigned long zero_count = 1;
unsigned long high_start = 0;
unsigned long high_end = 0;
unsigned long last_cycle = 0;

bool receiver_connected = false;

CRGBArray<NUM_LEDS> leds;


void setShooter(float percent) {
  Shooter.writeMicroseconds(1520 + 400 * percent * SHOOTER_DIRECTION);
}

void setKicker(float percent) {
  Kicker.writeMicroseconds(1520 + 400 * percent * KICKER_DIRECTION);
}

void nextStage() {
  stage++;
  if (stage == SHOOT_STAGE) {
    beam_first_connected = millis();
  } else if (stage > SHOOT_STAGE) {
    stage = DEFAULT_STAGE;
  }
  last_stage_switch = millis();
  stage_switch_ready = false;
}

void setup() {
  Shooter.attach(SHOOTER_PIN);
  Kicker.attach(KICKER_PIN);
  analogWrite(SHOOTER_PIN, 0);
  analogWrite(KICKER_PIN, 0);

  LeftServo.attach(LEFT_SERVO_PIN);
  RightServo.attach(RIGHT_SERVO_PIN);
  analogWrite(LEFT_SERVO_PIN, 0);
  analogWrite(RIGHT_SERVO_PIN, 0);


  pinMode(RIGHT_BUTTON_PIN, INPUT);
  pinMode(SENSOR_PIN, INPUT);

  pinMode(SHOOTER_PIN, OUTPUT);
  pinMode(KICKER_PIN, OUTPUT);
  pinMode(LEFT_SERVO_PIN, OUTPUT);
  pinMode(RIGHT_SERVO_PIN, OUTPUT);

  pinMode(LED_PIN, OUTPUT);
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
}

void loop() {
  // Receiver to button press
  if (digitalRead(RIGHT_BUTTON_PIN)) {
    if (zero_count >= ZERO_STREAK) {
      high_start = micros();
    }
    zero_count = 0;
  } else {
    if (zero_count == 0) {
      high_end = micros();
      last_cycle = millis();
      if (high_end > high_start) {
        unsigned long high_cycle = high_end - high_start;
        right_pressed = (high_cycle >= ON_THRESH && high_cycle <= RC_UPPER_THRESH);
        if (right_pressed) {
          leds[0] = CHSV(85, 255, 255);
        } else {
          leds[0] = CHSV(85, 255, 0);
        }
      }
      FastLED.show();
    }
    zero_count = min(zero_count + 1, ZERO_STREAK);
  }

  // Button press filtering
  if (right_pressed) {
    if (!right_prev_pressed) {
      start_of_press = millis();
    }
  }
  if (!right_pressed) {
    if (right_prev_pressed) {
      last_release = millis();
    }
  }
  right_prev_pressed = right_pressed;

  if (stage == SPINUP_STAGE && right_pressed && stage_switch_ready) {
    nextStage();
  } else {
    if (right_pressed && millis() - start_of_press >= BUTTON_STARTUP) {
      if (stage_switch_ready) {
        nextStage();
      }
    }
  }

  if (!right_pressed && millis() - last_release > BUTTON_COOLDOWN) {
    stage_switch_ready = true;
  }

  // Sensor readings
  if (!digitalRead(SENSOR_PIN)) {
    if (sensor_prev_reading) {
      beam_first_tripped = millis();
    }
    if (millis() - beam_first_tripped >= BEAM_BREAK_THRESH && stage == INTAKE_STAGE) {
      stage++;
    }
  } else {
    if (!sensor_prev_reading && beam_first_connected <= last_stage_switch) {
      beam_first_connected = millis();
    }
    if (millis() - beam_first_connected >= SHOOT_TIME && stage == SHOOT_STAGE) {
      stage = DEFAULT_STAGE;
    }
  }
  sensor_prev_reading = digitalRead(SENSOR_PIN);
  leds[2] = CHSV(170, 255, 255 * digitalRead(SENSOR_PIN));

  // Safety
  if (millis() < STARTUP || millis() - last_cycle > RECEIVER_THRESH) {
    if (receiver_connected) {
      leds[1] = CHSV(0, 255, 40);
      FastLED.show();
    }
    receiver_connected = false;
    stage = DEFAULT_STAGE;
  } else {
    receiver_connected = true;
  }

  // Robot Actions
  switch (stage) {
    default:
      setShooter(0.0);
      setKicker(0.0);

      LeftServo.write(LEFT_SERVO_DOWN);
      RightServo.write(RIGHT_SERVO_DOWN);
      leds[1] = CHSV(0, 0, 255);
      break;

    case INTAKE_STAGE:
      setShooter(-0.6);
      setKicker(-1.0);

      // Shooter intaking spinup
      if (millis() - last_stage_switch <= 300) {
        setShooter(-1.0);
      }

      LeftServo.write(LEFT_SERVO_UP);
      RightServo.write(RIGHT_SERVO_UP);
      leds[1] = CHSV(30, 255, 255);
      break;

    case SPINUP_STAGE:
      setShooter(1);
      setKicker(-0.7);

      LeftServo.write(LEFT_SERVO_UP);
      RightServo.write(RIGHT_SERVO_UP);
      leds[1] = CHSV(127, 255, 255);
      break;

    case SHOOT_STAGE:
      setShooter(1);
      setKicker(1);

      LeftServo.write(LEFT_SERVO_UP);
      RightServo.write(RIGHT_SERVO_UP);
      leds[1] = CHSV(212, 255, 255);
      break;
  }
}
