#include <ros.h>
#include <std_msgs/Int16.h> 
#include <geometry_msgs/Twist.h>


#define EN_L 9
#define IN1_L 8
#define IN2_L 7

#define EN_R 3
#define IN1_R 4
#define IN2_R 5

// Handles startup and shutdown of ROS
ros::NodeHandle nh;
 #define outputA1 48
 #define outputB1 49
 #define outputA2 51
 #define outputB2 50
// Minumum and maximum values for 16-bit integers
const int encoder_minimum = -32768;
const int encoder_maximum = 32767;
// Keep track of the number of wheel ticks
std_msgs::Int16 right_wheel_tick_count;
ros::Publisher rightPub("right_ticks", &right_wheel_tick_count);

std_msgs::Int16 left_wheel_tick_count;
ros::Publisher leftPub("left_ticks", &left_wheel_tick_count);

int aState1;
int aLastState1; 
int aState2;
int aLastState2;    
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;
unsigned long time1 = 100;


double w_r = 0, w_l = 0;

//wheel_rad is the wheel radius ,wheel_sep is
double wheel_rad = 0.05, wheel_sep = 0.22;

double speed_ang = 0, speed_lin = 0;

void messageCb( const geometry_msgs::Twist& msg) {
  speed_ang = msg.angular.z;
  speed_lin = msg.linear.x;
  w_r = (speed_lin / wheel_rad) + ((speed_ang * wheel_sep) / (2.0 * wheel_rad));
  w_l = (speed_lin / wheel_rad) - ((speed_ang * wheel_sep) / (2.0 * wheel_rad));
}



ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb );
void Motors_init();
void MotorL(int Pulse_Width1);
void MotorR(int Pulse_Width2);



void setup() { 
 pinMode (outputA1,INPUT);
 pinMode (outputB1,INPUT);
 pinMode (outputA2,INPUT);
 pinMode (outputB2,INPUT);

  Motors_init();
 
 //Serial.begin (9600);
 // Reads the initial state of the outputA1
 aLastState1 = digitalRead(outputA1);  
 // Reads the initial state of the outputA2
 aLastState2 = digitalRead(outputA2); 
  nh.initNode();
  nh.advertise(rightPub);
  nh.advertise(leftPub);
  nh.subscribe(sub);    
} 
void loop() { 
currentMillis = millis();
interrupt_encoder_left();
interrupt_encoder_right();
if (currentMillis - previousMillis >= time1) {
      previousMillis = currentMillis;
    rightPub.publish( &right_wheel_tick_count );
    leftPub.publish( &left_wheel_tick_count );
}

MotorL(w_l * 10);
MotorR(w_r * 10);
nh.spinOnce();
reset_encoders();
}

void interrupt_encoder_left(){
 aState1 = digitalRead(outputA1); // Reads the "current" state of the outputA1
 // If the previous and the current state of the outputA1 are different, that means a Pulse has occured
 if (aState1 != aLastState1){     
   // If the outputB1 state is different to the outputA1 state, that means the encoder is rotating clockwise
   if (digitalRead(outputB1) != aState1) { 
     left_wheel_tick_count.data  ++;
   } else {
     left_wheel_tick_count.data  --;
   }
 }
 aLastState1 = aState1; // Updates the previous state of the outputA1 with the current state
}

void interrupt_encoder_right(){
 aState2 = digitalRead(outputA2); // Reads the "current" state of the outputA2
 // If the previous and the current state of the outputA2 are different, that means a Pulse has occured
 if (aState2 != aLastState2){     
   // If the outputB2 state is different to the outputA2 state, that means the encoder is rotating clockwise
   if (digitalRead(outputB2) != aState2) { 
     right_wheel_tick_count.data ++;
   } else {
     right_wheel_tick_count.data --;
   }
 }
 aLastState2 = aState2; // Updates the previous state of the outputA2 with the current state
}

void reset_encoders(){
if (left_wheel_tick_count.data  == encoder_maximum) {
  left_wheel_tick_count.data  = encoder_minimum;
}else if (left_wheel_tick_count.data  == encoder_minimum) {
  left_wheel_tick_count.data  = encoder_maximum;
}
if (right_wheel_tick_count.data == encoder_maximum) {
  right_wheel_tick_count.data = encoder_minimum;
}else if (right_wheel_tick_count.data == encoder_minimum) {
  right_wheel_tick_count.data = encoder_maximum;
}

}


void Motors_init() {

pinMode(EN_L, OUTPUT);

pinMode(EN_R, OUTPUT);

pinMode(IN1_L, OUTPUT);

pinMode(IN2_L, OUTPUT);

pinMode(IN1_R, OUTPUT);

pinMode(IN2_R, OUTPUT);

digitalWrite(EN_L, LOW);

digitalWrite(EN_R, LOW);

digitalWrite(IN1_L, LOW);

digitalWrite(IN2_L, LOW);

digitalWrite(IN1_R, LOW);

digitalWrite(IN2_R, LOW);

}

void MotorL(int Pulse_Width1) {
if (Pulse_Width1 > 0) {

  analogWrite(EN_L, Pulse_Width1);

  digitalWrite(IN1_L, HIGH);

  digitalWrite(IN2_L, LOW);

}

if (Pulse_Width1 < 0) {

  Pulse_Width1 = abs(Pulse_Width1);

  analogWrite(EN_L, Pulse_Width1);

  digitalWrite(IN1_L, LOW);

  digitalWrite(IN2_L, HIGH);

}

if (Pulse_Width1 == 0) {

  analogWrite(EN_L, Pulse_Width1);

  digitalWrite(IN1_L, LOW);

  digitalWrite(IN2_L, LOW);

}

}


void MotorR(int Pulse_Width2) {


if (Pulse_Width2 > 0) {

  analogWrite(EN_R, Pulse_Width2);

  digitalWrite(IN1_R, LOW);

  digitalWrite(IN2_R, HIGH);

}

if (Pulse_Width2 < 0) {

  Pulse_Width2 = abs(Pulse_Width2);

  analogWrite(EN_R, Pulse_Width2);

  digitalWrite(IN1_R, HIGH);

  digitalWrite(IN2_R, LOW);

}

if (Pulse_Width2 == 0) {

  analogWrite(EN_R, Pulse_Width2);

  digitalWrite(IN1_R, LOW);

  digitalWrite(IN2_R, LOW);

}

}
