// Three independent stepper driver channels.
// Motor A: PUL+ 2, PUL- 3, DIR+ 4, DIR- 5
// Motor B: PUL+ 6, PUL- 7, DIR+ 8, DIR- 9
// Motor C: PUL+ 10, PUL- 11, DIR+ 12, DIR- 13
//
// Serial commands (newline terminated, 115200 baud):
//   A P <steps>                   set motor A absolute position
//   B V <steps/s>                 set motor B rotation speed
//   C G <kp> <zeta> <vmax> <amax> set motor C tuning
//   A S                           stop motor A
//   B Z                           zero motor B at its stopping point
//   M P <a> <b> <c>               set all positions in one command
//   M V <a> <b> <c>               set all speeds in one command
//   M S                           stop all motors
//   M Z                           zero all motors at their stopping points

#include <stdlib.h>

struct Motor {
  uint8_t pul;
  uint8_t pulGnd;
  uint8_t dir;
  uint8_t dirGnd;
  float kp;
  float zeta;
  float vmax;
  float amax;
  volatile long pos;
  long target;
  float vel;
  float vcmd;
  bool velMode;
  int8_t dirState;
  unsigned long lastCtrl;
  unsigned long lastStep;
};

Motor motors[3] = {
  {2, 3, 4, 5, 150, 1.0, 2000, 8000, 0, 0, 0, 0, false, 1, 0, 0},
  {6, 7, 8, 9, 150, 1.0, 2000, 8000, 0, 0, 0, 0, false, 1, 0, 0},
  {10, 11, 12, 13, 150, 1.0, 2000, 8000, 0, 0, 0, 0, false, 1, 0, 0}
};

char buf[80];
uint8_t len = 0;

void handleMotorCommand(Motor &motor, char command, char *params) {
  switch (command) {
    case 'P':
      motor.target = atol(params);
      motor.velMode = false;
      break;
    case 'V':
      motor.vcmd = atof(params);
      motor.velMode = true;
      break;
    case 'S':
      motor.vcmd = 0;
      motor.velMode = true;
      break;
    case 'G': {
      char *p = params;
      float kp = strtod(p, &p);
      float zeta = strtod(p, &p);
      float vmax = strtod(p, &p);
      float amax = strtod(p, &p);
      if (kp > 0) motor.kp = kp;
      if (zeta >= 0) motor.zeta = zeta;
      if (vmax > 0) motor.vmax = vmax;
      if (amax > 0) motor.amax = amax;
      break;
    }
    case 'Z': {
      long stopDist = (long)(motor.vel * fabs(motor.vel) / (2 * motor.amax));
      motor.pos = -stopDist;
      motor.target = 0;
      break;
    }
  }
}

void handleGroupCommand(char command, char *params) {
  if (command == 'S') {
    for (uint8_t i = 0; i < 3; i++) handleMotorCommand(motors[i], 'S', params);
  } else if (command == 'Z') {
    for (uint8_t i = 0; i < 3; i++) handleMotorCommand(motors[i], 'Z', params);
  } else if (command == 'P') {
    long values[3];
    char *p = params;
    for (uint8_t i = 0; i < 3; i++) {
      char *end;
      values[i] = strtol(p, &end, 10);
      if (end == p) return;
      p = end;
    }
    for (uint8_t i = 0; i < 3; i++) {
      motors[i].target = values[i];
      motors[i].velMode = false;
    }
  } else if (command == 'V') {
    float values[3];
    char *p = params;
    for (uint8_t i = 0; i < 3; i++) {
      char *end;
      values[i] = strtod(p, &end);
      if (end == p) return;
      p = end;
    }
    for (uint8_t i = 0; i < 3; i++) {
      motors[i].vcmd = values[i];
      motors[i].velMode = true;
    }
  }
}

void handle(char *line) {
  if (line[0] == 'M' && line[1] == ' ') {
    handleGroupCommand(line[2], line + 4);
    return;
  }
  if (line[0] >= 'A' && line[0] <= 'C' && line[1] == ' ') {
    handleMotorCommand(motors[line[0] - 'A'], line[2], line + 4);
    return;
  }

  // Keep the original one-motor protocol working for older controllers.
  handleMotorCommand(motors[0], line[0], line + 1);
}

void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (len) {
        buf[len] = 0;
        handle(buf);
        len = 0;
      }
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = c;
    }
  }
}

void stepMotor(Motor &motor) {
  int8_t direction = motor.vel > 0 ? 1 : -1;
  if (direction != motor.dirState) {
    motor.dirState = direction;
    digitalWrite(motor.dir, direction > 0 ? HIGH : LOW);
    delayMicroseconds(200);  // CL57Y requires DIR stable >125 us before a pulse.
  }
  digitalWrite(motor.pul, HIGH);
  delayMicroseconds(3);
  digitalWrite(motor.pul, LOW);
  motor.pos += direction;
}

void setup() {
  Serial.begin(115200);
  for (uint8_t i = 0; i < 3; i++) {
    Motor &motor = motors[i];
    pinMode(motor.pul, OUTPUT);     digitalWrite(motor.pul, LOW);
    pinMode(motor.pulGnd, OUTPUT);  digitalWrite(motor.pulGnd, LOW);
    pinMode(motor.dir, OUTPUT);     digitalWrite(motor.dir, HIGH);
    pinMode(motor.dirGnd, OUTPUT);  digitalWrite(motor.dirGnd, LOW);
    motor.lastCtrl = micros();
  }
}

void loop() {
  readSerial();
  unsigned long now = micros();

  for (uint8_t i = 0; i < 3; i++) {
    Motor &motor = motors[i];
    if (now - motor.lastCtrl >= 1000) {
      float dt = (now - motor.lastCtrl) * 1e-6;
      motor.lastCtrl = now;
      float acceleration;
      if (motor.velMode) {
        acceleration = (constrain(motor.vcmd, -motor.vmax, motor.vmax) - motor.vel) / dt;
      } else {
        float kd = 2 * motor.zeta * sqrt(motor.kp);
        acceleration = motor.kp * (motor.target - motor.pos) - kd * motor.vel;
      }
      acceleration = constrain(acceleration, -motor.amax, motor.amax);
      motor.vel = constrain(motor.vel + acceleration * dt, -motor.vmax, motor.vmax);
      if (!motor.velMode && motor.pos == motor.target &&
          fabs(motor.vel) < sqrt(2 * motor.amax)) {
        motor.vel = 0;
      }
    }

    if (fabs(motor.vel) > 0.5) {
      unsigned long interval = 1e6 / fabs(motor.vel);
      if (now - motor.lastStep >= interval) {
        motor.lastStep = now;
        stepMotor(motor);
      }
    }
  }
}
