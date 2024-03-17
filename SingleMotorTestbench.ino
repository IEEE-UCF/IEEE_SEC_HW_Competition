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

//#define DISTANCE_PER_ENCODER_COUNT 4.3223 // in mm
#define DISTANCE_PER_ENCODER_COUNT 0.0043223 // in mm
// variables for PID calculations
int kp =1, kd = 0, ki = 0;
volatile int temp, pos = 0;
//long prev_t = 0;
double eprev = 0;
long currentVelocity = 0;


void setup() {
  // put your setup code here, to run once:
  pinMode (IN1, OUTPUT);
  pinMode (IN2, OUTPUT);
  pinMode (PWM1, OUTPUT);
  Serial.begin(115200);
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
  Serial.print (pos);
  Serial.print(" ");
  Serial.println(pos);
  temp = pos;
  }
  else {
    Serial.print('\r');
    Serial.print(temp);
    Serial.print(" ");
    Serial.println(temp);
  }
}

void setMotor(int dir, int pwmVal, int motor) {
  // left motor = 1, right motor = 2
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

int calculateVel() {
  //Serial.println();

  int prevPos = pos;
  long prev_t = micros();
  delay(100);
  long curr_t = micros();
  double delta_t = ((double)(curr_t - prev_t)) / (1.0e6);
  updatePos();
  // Serial.print ("pos = ");
  // Serial.println(pos);

  // Serial.print("prevPos = ");
  // Serial.println(prevPos);
  
  // Serial.print("delta_t = ");
  // Serial.println(delta_t);
  

  // Serial.print("pos diff = ");
  // Serial.println(pos - prevPos);

  double velocity = ((pos - prevPos) * DISTANCE_PER_ENCODER_COUNT) / delta_t;

  // Serial.print ("Velocity = ");
  // Serial.println(velocity);
  // Serial.println();
  currentVelocity = velocity;

  return velocity;

}

double getPower(int targetVelocity) {
  long prev_t = micros();
  delay(100);
  long curr_t = micros();
  double delta_t = ((double)(curr_t - prev_t)) / (1.0e6);
  currentVelocity = calculateVel();

  long e = targetVelocity - currentVelocity;

  double dedt = ((double)e - eprev) / (delta_t);

  double eintegral = e * delta_t;

  double u = kp * (double)e + kd * dedt + ki * eintegral;

  double pwr = fabs(u);

  //Serial.print("e = "); Serial.println(e);
  //Serial.print("u = "); Serial.println(u);
  // delay(10000);

  //if (pwr <= 130) return 130; // ONLY USED DUE TO NOT HAVING 12V POWER SUPPLY
  if (pwr > 255) return 255;
  return pwr;
}


// global variables for processing incoming commands
String input;
String command;


int val1, val2;
int index, speed, loop_iteration;
String subString1;
void motorValues(String inputString){
  // only enter this function if the input string starts with an m

  // Serial.print("motorValues function string = ");
  // Serial.println(inputString);

  index = inputString.indexOf(' ');
  subString1 = inputString.substring(0, index + 1);
  inputString = inputString.substring(index + 1, inputString.length());

  // Serial.print("should be m = ");
  // Serial.println(subString1);

  // Serial.print("should be number 1 and 2 = ");
  // Serial.println(inputString);
  index = inputString.indexOf(' ');
  subString1 = inputString.substring(0, index + 1);
  val1 = subString1.toInt();

  // Serial.print("substring 1 = ");
  // Serial.print(subString1);

  // Serial.print("val 1  = ");
  // Serial.print(subString1);

  inputString = inputString.substring(index+1,inputString.length());
  val2 = inputString.toInt();
  // Serial.print("final inputString = ");
  // Serial.println(inputString);

  // Serial.print("val2 = ");
  // Serial.println(val2);

  // inputString = inputString.substring(index+2,inputString.length());
  // Serial.print("separated inputString = ");
  // Serial.println(inputString);

  // Serial.print("val 2 = ");
  // Serial.println(val2);
}
int v1 = 0, v2 = 0;
String makeTestString() {
  delay(200);
  String in = "m ";
  v1++;
  v2++;

  in += v1;
  in += ' ';
  in += v2;
  in += '\n';
  return in;
}

String inputString;
void loop() {
  updatePos();
// setMotor(1, 500, 1);
// calculateVel();
// delay(1000);

//Serial.println(calculateVel());


// updatePos();
//   // encoder test //
//     //   while (pos < 500) {
//     //   //Serial.println(getPower(50));
//     //   setMotor(1, getPower(500), 1);
//     //   updatePos();
//     // }
//     // while(pos > 0) {
//     //   setMotor(-1, getPower(0), 1);
//     //   updatePos();
//     // }
  if (Serial.available() > 0) {
    inputString = Serial.readStringUntil('\n');
    // Serial.print("Input String = ");
    // Serial.println(inputString.c_str());
    
    motorValues(inputString);
    // if (motorValues(inputString) == 1) {
    //   updatePos();
    //   return;
    // } 

    
    // setMotor(DIRECTION, POWER, MOTOR_SELECTION)
    // m val1 val2
  bool left;
  bool right;
  if (val1 > 0) left= true;
  else left = false;

  if (val2 > 0) right = true;
  else right = false;
  if(left && right) {
    // move forward
    // Serial.println("Moving forwards");
    // Serial.print("LEFT MOTOR = ");
    // Serial.println(abs(val1));

    // Serial.print("RIGHT MOTOR = ");
    // Serial.println(abs(val2));
    setMotor(1, getPower(abs(val1)), LEFT_MOTOR);
    setMotor(1, getPower(abs(val2)), RIGHT_MOTOR);
  }
    
  else if (!left && !right)
  {
    //move backwards
    //Serial.println("Moving backwards");
    //updatePos();
    setMotor(-1, getPower(abs(val1)), LEFT_MOTOR);
    setMotor(-1, getPower(abs(val2)), RIGHT_MOTOR);
  }

  else if (!left && right) {
    // move left
    //Serial.println("Moving left");
    //updatePos();
    setMotor(-1, getPower(abs(val1)), LEFT_MOTOR);
    setMotor(1, getPower(abs(val2)), RIGHT_MOTOR);
  }


  else if (left && !right) {
    // move right
    //Serial.println("Moving right");
    //updatePos();
    setMotor(1, getPower(abs(val1)), LEFT_MOTOR);
    setMotor(-1, getPower(abs(val2)), RIGHT_MOTOR);
  }        

  }
}
