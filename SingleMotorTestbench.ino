#define IN1   4
#define IN2   5
#define PWM1 10
// #define ENCA1 10
// #define ENCB1 11

#define IN3    6
#define IN4    7
#define PWM2  11

#define LEFT_MOTOR  1
#define RIGHT_MOTOR 2

#define ServoPin 12
#define SmallIntakeAngle 0
#define LargeIntakeAngle 180

#define turn_off_intake 6
#define enable_outtake 5
#define enable_intake_motor 4
#define OWWW 3
#define Shamona 2
#define halal_inshallah_mashallah 1



#include <Servo.h>
Servo ServoServo;
//#define DISTANCE_PER_ENCODER_COUNT 4.3223 // in mm
#define DISTANCE_PER_ENCODER_COUNT 0.0043223 // in mm
// variables for PID calculations
int kp =1, kd = 0, ki = 0;
volatile int temp, pos = 0;
long prev_t = 0;
double eprev = 0;
long currentVelocity = 0;


void setup() {
  // put your setup code here, to run once:
  pinMode (IN1, OUTPUT);
  pinMode (IN2, OUTPUT);
  pinMode (PWM1, OUTPUT);
  pinMode (6, OUTPUT);
  pinMode (7, OUTPUT);
  Serial.begin(57600);
  Serial.println("beginning");

  ServoServo.attach(ServoPin);
  attachInterrupt(10, readEncoder1, RISING);
  attachInterrupt(11, readEncoder2, RISING);
}

void readEncoder1() {
  if(digitalRead(10)==LOW) {
  pos++;
  }else{
  pos--;
  }
}
void readEncoder2() {
  if(digitalRead(11)==LOW) {
  pos--;
  }else{
  pos++;
  }

}
void updatePos() {
  if( pos != temp ){
    Serial.println(pos);
  temp = pos;
  }
}

void setMotor(int dir, int pwmVal, int motor) {
  // left motor = 1, right motor = 2
  analogWrite(10,pwmVal);
  if (motor == 1) {
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

  else if (motor == 2) {
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

 // if (pwr <= 130) return 130; // ONLY USED DUE TO NOT HAVING 12V POWER SUPPLY
  if (pwr > 255) return 255;
  return pwr;
}

void extendZipline() {
  // timeout function
  double time = millis();
  Serial.print("time = ");
  Serial.println(time);

  // move motor to position 500
  while (pos < 500 && (time + 3000) > millis())  {
    updatePos();
    setMotor(1, getPower(500), LEFT_MOTOR);
  }
  // wait a bit until it attaches to the zipline
  delay(1000);
  time = millis();
  // move motor to position 0
  while (pos >  0 && (time + 3000) > millis()) {
    updatePos();
    setMotor(-1, getPower(0), LEFT_MOTOR);
  }
  setMotor(0, 0, LEFT_MOTOR);
  Serial.println("OK");
}
void intakeSmallBox() {
  ServoServo.write(SmallIntakeAngle);
  delay(100);
  Serial.println("OK");
}

void intakeBigBox() {
  ServoServo.write(LargeIntakeAngle);
  delay(100);
  Serial.println("OK");
}

void enableIntakeMotor() {
  setMotor(1, 50, RIGHT_MOTOR);
  Serial.println("OK");
}

void enableOuttake() {
  setMotor(-1, 50, RIGHT_MOTOR);
  Serial.println("OK");
}

void turnOffIntake() {
  setMotor(0, 0, RIGHT_MOTOR);  
  Serial.println("OK");
}

// global variables for processing incoming commands


int v1 = 0, v2 = 0;
int inputInt = 0;
String inputString;
void loop() {

  inputString = "";


  updatePos();


// setMotor(1, 500, 1);
// calculateVel();
// delay(1000);

//Serial.println(calculateVel());


// updatePos();
//   // encoder test //
    //   while (pos < 500) {
    //   //Serial.println(getPower(50));
    //   setMotor(1, getPower(500), 1);
    //   updatePos();
    // }
    // while(pos > 0) {
    //   setMotor(-1, getPower(0), 1);
    //   updatePos();
    // }
  // if (Serial.available() > 0) {
  //   inputString = Serial.readStringUntil('\n');
  //   inputInt =  inputString.toInt();
  //   Serial.println(inputInt);
  // }
  // switch (inputInt) {
  //   case halal_inshallah_mashallah:
  //     //extendZipline();
  //     digitalWrite(6, HIGH);
  //     digitalWrite(7, LOW);
  //     break;

  //   case Shamona:
  //     //intakeSmallBox();
  //     digitalWrite(6, LOW);
  //     digitalWrite(7, LOW);
  //     break;
  //   case OWWW:
  //     intakeBigBox();
  //     break;
  //   case enable_intake_motor:
  //     enableIntakeMotor();
  //     break;
  //   case enable_outtake:
  //     enableOuttake();
  //     break;
  //   case turn_off_intake:
  //     turnOffIntake();
  //     break;
  // }

  if (inputString.equals("1")) {
    extendZipline();
  }

  else if (inputString.equals("2")) {
    intakeSmallBox();
  }

  else if (inputString.equals("3")) {
    intakeBigBox();
  }

  else if (inputString.equals("4")) {
    enableIntakeMotor();
  }

  else if (inputString.equals("5")) {
    enableOuttake();
  }

  else if (inputString.equals("6")) {
    turnOffIntake();
  }
  

}
