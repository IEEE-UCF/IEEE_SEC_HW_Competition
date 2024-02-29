#define IN1   4
#define IN2   5
#define PWM1 10
#define ENCA1 2
#define ENCB1 3

#define IN3    6
#define IN4    7
#define PWM2  12
#define ENCA2  8
#define ENCB2  9

#define FORWARD   "move_forward"
#define BACKWARD  "move_backward"
#define LEFT      "move_left"
#define RIGHT     "move_right"

#define LEFT_MOTOR  1
#define RIGHT_MOTOR 2

// variables for PID calculations
int kp =1, kd = 0, ki = 0;
volatile  int temp, pos = 0;
long prev_t = 0;
double eprev = 0;


void setup() {
  pinMode (IN1, OUTPUT);
  pinMode (IN2, OUTPUT);
  pinMode (PWM1, OUTPUT);
  Serial.begin(19200);
  Serial.println("beginning");

  attachInterrupt(0, readEncoder1, RISING);
  attachInterrupt(1, readEncoder2, RISING);
}

void readEncoder1() {
  if(digitalRead(3)==LOW) {
  pos++;
  }else{
  pos--;
  }
}
void readEncoder2() {
  if(digitalRead(2)==LOW) {
  pos--;
  }else{
  pos++;
  }

}
void updatePos() {
  if( pos != temp ){
  Serial.println (pos);
  temp = pos;
  }
}

void setMotor(int dir, int pwmVal, int motor) {
  if (motor == LEFT_MOTOR) {
    analogWrite(10,pwmVal);
    if (dir == 0) {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      return;
    }
    else if (dir == 1) {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      return;
    }
    else if (dir == -1) {
      digitalWrite (IN1, LOW);
      digitalWrite (IN2, HIGH);
      return;
    }
  }

  else if (motor == RIGHT_MOTOR) {
    analogWrite(PWM2, pwmVal);
    if (dir == 0) {
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
      return;
    }

    else if (dir == 1) {
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
    }
    else if (dir == -1) {
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
    }
  }

}
double getPower(int target) {
  long curr_t = micros();
  double delta_t = ((double)(curr_t - prev_t)) / (1.0e6);
  prev_t = curr_t;
  updatePos();
  long e = pos - target;

  double dedt = ((double)e - eprev) / (delta_t);

  double eintegral = e * delta_t;

  double u = kp * (double)e + kd * dedt + ki * eintegral;

  double pwr = fabs(u);

  //Serial.print("e = "); Serial.println(e);
  //Serial.print("u = "); Serial.println(u);
  // delay(10000);

  if (pwr <= 130) return 130; // ONLY USED DUE TO NOT HAVING 12V POWER SUPPLY
  if (pwr > 255) return 255;
  return pwr;
}


// global variables for processing incoming commands
String input;
String command;

int index, speed, loop_iteration;
void loop() {
  // encoder test
    // while (pos < 500) {
    //   //Serial.println(getPower(50));
    //   setMotor(1, getPower(500), 1);
    //   updatePos();
    // }
    // while(pos > 0) {
    //   setMotor(-1, getPower(0), 1);
    //   updatePos();
    // }
  
  // serial test
    if (Serial.available() > 0) {
      input = Serial.readStringUntil('\n');
      Serial.println(input);
      // speed = input.substring(input.length()-4);
      index = input.length() - 2;
      speed = 0;
      loop_iteration = 1;
      // extracts the speed from the command
      while (isDigit(input.charAt(index))) {
        speed += (((int)input.charAt(index) - '0') * loop_iteration);
        loop_iteration *= 10;
        index--;
      }

      command = input.substring(0, index);
      if (command.equals(FORWARD)) {
        updatePos();
        setMotor(1, getPower(pos + speed), LEFT_MOTOR);
        setMotor(1, getPower(pos + speed), RIGHT_MOTOR);

      }
      else if (command.equals(BACKWARD)) {
        updatePos();
        setMotor(-1, getPower(pos + speed), LEFT_MOTOR);
        setMotor(-1, getPower(pos + speed), RIGHT_MOTOR);
      }
      else if (command.equals(LEFT)) {
        updatePos();
        setMotor(-1, getPower(pos + speed), LEFT_MOTOR);
        setMotor(1, getPower(pos + speed), RIGHT_MOTOR);
      }
      else if (command.equals(RIGHT)) {
        updatePos();
        setMotor(1, getPower(pos + speed), LEFT_MOTOR);
        setMotor(-1, getPower(pos + speed), RIGHT_MOTOR);
      }
      else {
        updatePos();
        setMotor(0, getPower(pos + speed), LEFT_MOTOR);
        setMotor(0, getPower(pos + speed), RIGHT_MOTOR);
      }
    }

}