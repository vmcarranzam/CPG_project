/*
* File: Quad_spider_cpg.ino
* Purpose: Implementation of a Central Pattern Generator controlled quadruped spider-like robot
* Version: 1.0.0
* Date: 19-07-2021
* Adapted by: Victor Carranza (https://github.com/vmcarranzam/CPG_project/), from the work of 
* Martin Stokroos (https://github.com/MartinStokroos/RoFish)
* License: MIT License
*
* The CPG algorithm in the main loop of this program was taken over from Garcia-Saura[1]. The CPG
* parameters were originally chosen for generating walking patterns of a micro-hexapod. In this
* example, the pattern is modified for a 4 joint robotic quadruped.
*
* The robot model used for this particular implementation was adapted from an Hexapod Robot available on https://www.thingiverse.com/thing:3463845
* 
* The servo driver used for this implementation is the Adafruit PCA9685 16-Channel Servo Driver, usage reference: https://learn.adafruit.com/16-channel-pwm-servo-driver
* 
* The gait generating matrixes were adapted from Barron-Zambrano[4]
*
* References:
* [1] Garcia-Saura, Carlos (2015). Central Pattern Generators for the control
* of robotic systems. MSc in Computing (Spec. in Artificial Intelligence) Imperial College London.
*
* [2] J. Shao, L. Wang and J. Yu, Development of an artificial fish-like robot and its application
* in cooperative transportation, Control Engineering Practice, vol.16, no.5, pp.569-584, 2008.
*
* [3] Yu, J., Chen, S., Z. Wu, and Wang, W. (2016). On a miniature free-swimming robotic ﬁsh with
* multiple sensors. International Journal of Advanced Robotic Systems, 13(62).
* 
* [4] Barron-zambrano, Jose (2010). Hardware Implementation of a CPG-Based Locomotion Control for 
* Quadruped Robots. p.84, Universidad Autonoma de Tamaulipas
*
*/



#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>


//servomotor parameters
Adafruit_PWMServoDriver servos = Adafruit_PWMServoDriver();
#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates



//#define LED_PIN 13
#define LPERIOD 10000L  // loop period time in us.
#define N 4 // number of oscillators
#define T LPERIOD/1000000.0 // T=10ms.


int dt=100; //delay value for home function


//model parameters
unsigned long nextLoop;
char c;
int i,j;
float w[N][N], f[N][N], b[N][N];
bool forward=true;

// constant vars
float ar=20; // rad/sec
float ax=20; // rad/sec

// control inputs
float W=8, R=0.3, X=0; // common control inputs frequency/amplitude/offset of the joints
float cW[N]={0, PI, PI/2,3*PI/2}; // OHMEGAi - desired frequency rad/s.
float cR[N]={0.42, 0.31, 0.40, 0.31}; // Ri - desired amplitude in radians.
float cX[N]={0, 0, 0,0};      // Xi - desired offset in radians.

// states
float sA[N]={0,0,0,0}; // phase
float sr[N]={0,0,0,0}; // amplitude
float sx[N]={0,0,0,0}; // offset
float sA_new[N]={0,0,0,0}; // phase
float sr_new[N]={0,0,0,0}; // amplitude
float sx_new[N]={0,0,0,0}; // offset

// derivatives
float sA_d=0; // phase dot
float sr_d[N]={0,0,0,0}; // amplitude dot
float sx_d[N]={0,0,0,0}; // offset dot
float sr_d_new[N]={0,0,0,0}; // amplitude dot
float sx_d_new[N]={0,0,0,0}; // amplitude dot

// second derivatives
float sr_dd=0;
float sx_dd=0;
float out_deg[N]={0,0,0,0}; // output phase time vector

//Servo servo1;
//Servo servo2;
//Servo servo3;

// variables a imprimir
float x_1;
float x_2;
float x_3;
float x_4;
//The setup function is called once at startup of the sketch


int hip[4]={0,8,12,4};
int knee[4]={1,9,13,5};
int foot[4]={2,10,14,6};

void home() {
  
  for (int i=0; i<4;i++){
    setServo(hip[i],65);
//    delay(d);
    setServo(knee[i],90);
//    delay(d);
    setServo(foot[i],60);
//    delay(d);
    }
  
  }



void setup()
{
  servos.begin();
  servos.setOscillatorFrequency(27000000);
  servos.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

home();
delay(3000);

  delay(10);



//  pinMode(LED_PIN, OUTPUT); // for checking loop period time and loop execution time (signal high time)
  Serial.begin(9600);
//  servo1.attach(SERVO1_PIN);
//  servo2.attach(SERVO2_PIN);
//  servo3.attach(SERVO3_PIN);
//  servo1.write(90); // turn servos to the center position
//  servo2.write(90);
//  servo3.write(90);
//  delay(1500);

  // init, 1/s coupling weights
//  for walking gait
  w[1][1] = 1;
  w[1][2] = -5;
  w[1][3] = -5;
  w[1][4] = -5;
  w[2][1] = -5;
  w[2][2] = 1;
  w[2][3] = -5;
  w[2][4] = -5;
  w[3][1] = -5;
  w[3][2] = -5;
  w[3][3] = 1;
  w[3][4] = -5;
  w[4][1] = -5;
  w[4][2] = -5;
  w[4][3] = -5;
  w[4][4] = 1;    


//  //for trotting gait
//  w[1][1] = 1;
//  w[1][2] = -5;
//  w[1][3] = 5;
//  w[1][4] = -5;
//  w[2][1] = -5;
//  w[2][2] = 1;
//  w[2][3] = -5;
//  w[2][4] = 5;
//  w[3][1] = 5;
//  w[3][2] = -5;
//  w[3][3] = 1;
//  w[3][4] = -5;
//  w[4][1] = -5;
//  w[4][2] = 5;
//  w[4][3] = -5;
//  w[4][4] = 1; 


//  //for galloping gait
//  w[1][1] = 1;
//  w[1][2] = 5;
//  w[1][3] = -5;
//  w[1][4] = -5;
//  w[2][1] = -5;
//  w[2][2] = 1;
//  w[2][3] = 5;
//  w[2][4] = -5;
//  w[3][1] = -5;
//  w[3][2] = -5;
//  w[3][3] = 1;
//  w[3][4] = 5;
//  w[4][1] = 5;
//  w[4][2] = -5;
//  w[4][3] = -5;
//  w[4][4] = 1;  


  

  // forward moving, 1/s phase biases
  f[1][1] = 0.0;
  f[1][2] = -0.7;
  f[1][3] = -1.41;
  f[1][4] = -2.1;
  f[2][1] = -f[1][2];
  f[2][2] = 0.0;
  f[2][3] = f[1][3] - f[1][2];
  f[2][4] = f[1][4] - f[1][2];
  f[3][1] = -f[1][3];
  f[3][2] = -f[2][3];
  f[3][3] = 0.0;
  f[3][4] = f[1][4] - f[1][3];
  f[4][1] = -f[1][4];
  f[4][2] = -f[2][4];
  f[4][3] = -f[3][4];
  f[4][4] = 0.0;

  // backward moving, 1/s phase biases
  b[1][1] = 0.0;
  b[1][2] = 0.7;
  b[1][3] = 1.41;
  b[1][4] = 2.1;
  b[2][1] = -f[1][2];
  b[2][2] = 0.0;
  b[2][3] = f[1][3] - f[1][2];
  b[2][4] = f[1][4] - f[1][2];
  b[3][1] = -f[1][3];
  b[3][2] = -f[2][3];
  b[3][3] = 0.0;
  b[3][4] = f[1][4] - f[1][3];
  b[4][1] = -f[1][4];
  b[4][2] = -f[2][4];
  b[4][3] = -f[3][4];
  b[4][4] = 0.0;

  nextLoop = micros() + LPERIOD; // Set the loop timer variable.
}

void setServo(uint8_t n, int angle) {
  int duty;
  duty=map(angle,0,180,SERVOMIN,SERVOMAX);
  servos.setPWM(n,0,duty);
}
// The loop function is called in an endless loop
void loop()
{
//digitalWrite(LED_PIN, true);

for(i=0; i<N; i++){
  cW[i] = W;
  cR[i] = R;
  cX[i] = X;

  sA_d = cW[i];
  for(j=0;j<N;j++) {
    if (forward==true) {
      sA_d += w[i][j] * sr[j] * sin(sA[j] - sA[i] - f[i][j]);
    }
    else {
      sA_d += w[i][j] * sr[j] * sin(sA[j] - sA[i] - b[i][j]);
    }
  }
  sA_new[i] = sA[i] + T*sA_d;

  sr_dd = ar * ( (ar/4) * (cR[i] - sr[i]) - sr_d[i] );
  sr_d_new[i] = sr_d[i] + T*sr_dd;
  sr_new[i] = sr[i] + T*sr_d[i];

  sx_dd = ax * ( (ax/4) * (cX[i] - sx[i]) - sx_d[i] );
  sx_d_new[i] = sx_d[i] + T*sx_dd;
  sx_new[i] = sx[i] + T*sx_d[i];
  }

for(i=0; i<N; i++){
  sA[i] = sA_new[i];
  sr[i] = sr_new[i];
  sx[i] = sx_new[i];
  sr_d[i] = sr_d_new[i];
  sx_d[i] = sx_d_new[i];
  //out[i] = sx[i] + sr[i] * sin(sA[i]);
  out_deg[i] = 360.0 * ( sx[i] + sr[i] * sin(sA[i]) )/(2*PI);
  }


x_1=65 + round(out_deg[0]);
x_2=65 - round(out_deg[1]);
x_3=65 + round(out_deg[2]);
x_4=65 - round(out_deg[3]);

setServo(hip[0], x_1 );
setServo(foot[0], 30 + round(out_deg[0]) );
//setServo(knee[0], 130 + round(out_deg[0]) );


setServo(hip[1], x_2 );
setServo(foot[1], 30 - round(out_deg[1]) );
//setServo(knee[1], 130 - round(out_deg[1]) );

setServo(hip[2], x_3 );
setServo(foot[2], 30 + round(out_deg[2]) );
//setServo(knee[2], 130 + round(out_deg[2]) );

setServo(hip[3], x_4 );
setServo(foot[3], 30 - round(out_deg[3]) );
//setServo(knee[3], 130 - round(out_deg[3]) );




// for serial printing and plotting purposes
Serial.print(x_1);
Serial.print(",");
Serial.print(x_2);
Serial.print(",");
Serial.print(x_3);
Serial.print(",");
Serial.println(x_4);

if(Serial.available()){
  c = Serial.read();
    if(c == 'q') {  // use the 'q' and 'a' keys to set the frequency of oscillation.
        W += 0.5;
        W = constrain(W, 0, 8);
      Serial.print("speed [rad/s]: ");
      Serial.println(W,2);
    }
    if(c == 'a') {
        W -= 0.5;
        W = constrain(W, 0, 8);
      Serial.print("speed [rad/s]: ");
      Serial.println(W,2);
    }
    if(c == 'w') { // use the 'w' and 's' keys to set the amplitude.
        R += 0.05;
        R = constrain(R, 0, 0.5);
      Serial.print("amplitude [rad]: ");
      Serial.println(R,2);
    }
    if(c == 's') {
        R -= 0.05;
        R = constrain(R, 0, 0.5);
      Serial.print("amplitude [rad]: ");
      Serial.println(R,2);
    }
    if(c == 'e') { // use the 'e' and 'd' keys to set the offset (steering control)
        X += 0.05;
        X = constrain(X, -0.5, 0.5);
      Serial.print("offset [rad]: ");
      Serial.println(X,2);
    }
    if(c == 'd') {
        X -= 0.05;
        X = constrain(X, -0.5, 0.5);
      Serial.print("offset [rad]: ");
      Serial.println(X,2);
    }
    if(c == 'f') { // swim forward
      forward = true;
        Serial.println("forwards");
    }
    if(c == 'b') { // swim backward
          forward = false;
            Serial.println("backwards");
        }
}
//digitalWrite(LED_PIN, false); //loop execution time, about 2ms.


while(nextLoop > micros());  //wait until the end of the time interval
  nextLoop += LPERIOD;  //set next loop time at current time + LOOP_PERIOD
}
