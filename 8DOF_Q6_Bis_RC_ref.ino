// 8DOF_Q6_Bis_RC_ref - reference version - 12/10/2023 -

void(* resetFunc) (void) = 0;                               // soft reset function
#include <Servo.h>                                          // Servo library
int pb=A0;                                                  // stop button and start time
int Speed=5000;                                             // delay between 2 orders for left and right legs
unsigned long ch1, ch2;                                     // radio controller channel 1 and channel 2
float S=1, C=1;
int Ax = 90, Bx=-Ax, YH=30, YL=0, withers=290, lg_Itr=5, F_tr=0, B_tr=-40;     // Ax Bx YH YL rectangle coordinates, iteration length , front and back translation
const int nb=8;                                             // number of servos
Servo Srv[nb];                                              // Servos table
int Smin[nb]={ 550, 500, 540, 500, 500, 590, 510, 510};     // all servos values for 0°, you must modify these values according to your servos
int Smax[nb]={2650,2560,2550,2500,2500,2520,2550,2600};     // all servos values for 180°, you must modify these values according to your servos
int OnOff[nb]={1,1,1,1,1,1,1,1};                            // on/off Servos table
int FRLS=0, FRRS=1, FLLS=2, FLRS=3, BRLS=4, BRRS=5, BLLS=6, BLRS=7;
//servos      FRLS  FRRS  FLLS  FLRS  BRLS  BRRS  BLLS  BLRS          // you must modify these values according to your servos
int Err[nb]={   -2,   -5,    1,   -3,    1,   -2,    2,   -4    };    // 90° correction error

void  setup() {
  delay(400);                             // for reset consideration
  Serial.begin(19200); delay(400);
  pinMode(pb,INPUT_PULLUP);               // start/stop/reset button attachment
  pinMode(A1,INPUT);                      // channel 1 receiver input
  pinMode(A2,INPUT);                      // channel 2 receiver input
  
  Serial.print("\n\t To initialize, click on the Start button");
  while( digitalRead(pb) );  delay(400);

  Srv[FRLS].attach( 2,Smin[FRLS],Smax[FRLS]); // FR Front Right leg - LS Left Servo
  Srv[FRRS].attach( 3,Smin[FRRS],Smax[FRRS]); // FR Front Right leg - RS Right Servo
  Srv[FLLS].attach( 4,Smin[FLLS],Smax[FLLS]); // FL Front Left  leg - LS Left Servo
  Srv[FLRS].attach( 5,Smin[FLRS],Smax[FLRS]); // FL Front Left  leg - RS Right Servo
  Srv[BRLS].attach(10,Smin[BRLS],Smax[BRLS]); // BR Back  Right leg - LS Left Servo
  Srv[BRRS].attach(11,Smin[BRRS],Smax[BRRS]); // BR Back  Right leg - RS Right Servo
  Srv[BLLS].attach(12,Smin[BLLS],Smax[BLLS]); // BL Back  Left  leg - LS Left Servo
  Srv[BLRS].attach(13,Smin[BLRS],Smax[BLRS]); // BL Back  Left  leg - RS Right Servo

  IK(0+F_tr,0,FRLS,FRRS,withers,Speed); IK(0-B_tr,0,BLLS,BLRS,withers,Speed); IK(0-F_tr,0,FLLS,FLRS,withers,Speed); IK(0+B_tr,0,BRLS,BRRS,withers,Speed);
  Serial.print("\n\t To start, click on the Start button"); while( digitalRead(pb) );  delay(400); Serial.print("\n\t Started");
}

void loop() {
  if (! digitalRead(pb)) resetFunc();
  Walk();
}

int RC_ch1(){
  int a;
  a = map( (pulseIn(A1,HIGH,30000)), 960, 1950, 90, 0 );  // from 0 to 44 turn right, 45 straight forward, from 90 to 46 turn left
  S=sin(a*PI/180); C=cos(a*PI/180); //Serial.print( String() + "\n\t a = " + a  + "\t Sin = " + S + "  Cos = " + C );
}

void Walk(){
  if ( (Speed = map(pulseIn(A2,HIGH,30000),1450,1970,5000,1) ) >= 4000 ) { Serial.print( String() + "\n\t Speed = stop " + Speed ); delay(200); return; } // RC channel 2
  RC_ch1();
  for( int i=Ax;i>=Bx;i=i-lg_Itr){
    IK( (+i+F_tr)*S, YL, FRLS, FRRS, withers, Speed);
    IK( (+i-F_tr)*C, YH, FLLS, FLRS, withers, Speed);
    IK( (-i+B_tr)*S, YH, BRLS, BRRS, withers, Speed);
    IK( (-i-B_tr)*C, YL, BLLS, BLRS, withers, Speed);
  }
  if ( (Speed = map(pulseIn(A2,HIGH,30000),1450,1970,5000,1) ) >= 4000 ) { Serial.print( String() + "\n\t Speed = stop " + Speed ); delay(200); return; } // RC channel 2
  RC_ch1();
  for( int i=Ax;i>=Bx;i=i-lg_Itr){
    IK( (-i+F_tr)*S, YH, FRLS, FRRS, withers, Speed);
    IK( (-i-F_tr)*C, YL, FLLS, FLRS, withers, Speed);
    IK( (+i+B_tr)*S, YL, BRLS, BRRS, withers, Speed);
    IK( (+i-B_tr)*C, YH, BLLS, BLRS, withers, Speed);
  }
}

void IK(int Px, int Py, int LS, int RS, int Ay, int speed){  // Inverse Kinematics function
  if (! digitalRead(pb)) resetFunc();         // for Nano

  float Ax=0, c=168;                          // position of the main paw axis and length of femur and tibia
  float d=Ay-Py, e=Ax-Px;                     // d and e, sides of rectangle triangle 
  float h=sqrt((d*d)+(e*e));                  // h, hypotenuse of rectangle triangle
  float B=asin(d/h);    if(e<0)B=(PI-B);      // B is the top angle of the rectangle triangle
  float A=acos(h/(2*c));                      // A is the Diamond half top angle (cosin law)
  int S1=round(degrees(B-A))+Err[LS];         // S1 is the left servo angle in degrees +Err
  int S2=round(degrees(B+A))+Err[RS];         // S2 is the right servo angle in degrees +Err

//DEBUG
//  Serial.print( String()  + "\n\t Px=" + Px + "  Py=" + Py + "\t\tLeg servos :  LS=" + LS + "  RS=" + RS );
//  Serial.print("\n\t Inverse Kinematics values :     B°= ");Serial.print(round(degrees(B)));Serial.print("     A°= ");Serial.print(round(degrees(A)));
//  Serial.print("\n\t Servo angles results : S1= ");Serial.print(S1);Serial.print("°\t\tS2= ");Serial.print(S2);Serial.print("°");

// - RESTRICTED AREA CONTROL - RESTRICTED AREA CONTROL - RESTRICTED AREA CONTROL - RESTRICTED AREA CONTROL - RESTRICTED AREA CONTROL - RESTRICTED AREA CONTROL - RESTRICTED AREA CONTROL -
  if (h>(c+c)){Serial.print("\n\t Target : Px=");Serial.print(Px);Serial.print(" Py=");Serial.print(Py);Serial.print("\t h=");Serial.print(h);Serial.print(" > ");Serial.print(c+c);Serial.print(" is too long !!!!!");return;}
  if (S1<0)   {Serial.print("\n\t Target : Px=");Serial.print(Px);Serial.print(" Py=");Serial.print(Py);Serial.print("\t LS=");Serial.print(LS);Serial.print("\t angle S1=");Serial.print(S1);Serial.print(" <0° is not reachable !!!!!");return;}
  if (S2>180) {Serial.print("\n\t Target : Px=");Serial.print(Px);Serial.print(" Py=");Serial.print(Py);Serial.print("\t RS=");Serial.print(RS);Serial.print("\t angle S2=");Serial.print(S2);Serial.print(" >180° is not reachable !!!!!");return;}
// - RESTRICTED AREA CONTROL - RESTRICTED AREA CONTROL - RESTRICTED AREA CONTROL - RESTRICTED AREA CONTROL - RESTRICTED AREA CONTROL - RESTRICTED AREA CONTROL - RESTRICTED AREA CONTROL -

  if (OnOff[LS]) Srv[LS].writeMicroseconds(map(S1,0,180,Smin[LS],Smax[LS]));  // set target Left servo if servo switch is On
  if (OnOff[RS]) Srv[RS].writeMicroseconds(map(S2,0,180,Smin[RS],Smax[RS]));  // set target Right servo if servo switch is On
  delayMicroseconds(speed);                                                   // speed of action
}
