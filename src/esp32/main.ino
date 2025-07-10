#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Servo.h>

// --------------------------------------------------------------------- Constants --------------------------------------------------------------------------
//#define ENABLE_PIN           8
#define DEBOUNCE_DELAY       10
#define HOMING_SPEED         600
#define BACKOFF_STEPS        100

#define GRIPPER_SERVO_PIN     5
#define ROTATION_SERVO_PIN    3
#define DC_MOTOR_PIN          A3
#define PROXIMITY_PIN         A0

#define MOVING_SPEED          1500
#define MOVING_ACCELERATION   1000

#define DIR_UP   HIGH
#define DIR_DOWN LOW

const int STEP_PINS[3] = {4, 12, 10};
const int DIR_PINS[3]  = {2, 11, 9};
const int LIMIT_PINS[3] = {13, A5, A4};

// -------------------------------------------------------------------------- Objects -----------------------------------------------------------------------
AccelStepper steppers[3] = {
  AccelStepper(AccelStepper::DRIVER, STEP_PINS[0], DIR_PINS[0]),
  AccelStepper(AccelStepper::DRIVER, STEP_PINS[1], DIR_PINS[1]),
  AccelStepper(AccelStepper::DRIVER, STEP_PINS[2], DIR_PINS[2])
};

MultiStepper multiStepper;
Servo gripperServo;
Servo rotationServo;

bool homed[3] = {false, false, false};
unsigned long lastDebounceTime[3] = {0, 0, 0};
bool lastSwitchState[3] = {HIGH, HIGH, HIGH};

// -------------------------------------------------------------------------- Stored Points ------------------------------------------------------------------
long pointMain[3] = {-2364, -2308, -236};
long pointP[3]    = {-121,  -2077, -2457};
long pointF[3]    = {-2496, -213,  -2419};
long CenterPoint[3];

//*********************************************************************** Proximity and Convyer ****************************************************
byte sensorState = HIGH;
byte ColorValue = 0;

// -------------------------------------------------------------------------- Setup -------------------------------------------------------------------------
void setup() 
{
  for (int i = 0; i < 3; i++) {
    pointMain[i] += BACKOFF_STEPS;
    pointP[i]    += BACKOFF_STEPS;
    pointF[i]    += BACKOFF_STEPS;
    CenterPoint[i]= -BACKOFF_STEPS;
  }

  Serial.begin(9600);

  for (int i = 0; i < 3; i++) 
  {
    pinMode(LIMIT_PINS[i], INPUT_PULLUP);
    pinMode(DIR_PINS[i], OUTPUT);
    steppers[i].setMaxSpeed(MOVING_SPEED);
    steppers[i].setAcceleration(MOVING_ACCELERATION);
    multiStepper.addStepper(steppers[i]);
  }

  pinMode(PROXIMITY_PIN, INPUT);
  pinMode(DC_MOTOR_PIN, OUTPUT);
  pinMode(7, INPUT);
  pinMode(8, INPUT);


  homeDeltaRobot();

  gripperServo.attach(GRIPPER_SERVO_PIN);
  rotationServo.attach(ROTATION_SERVO_PIN);
  gripperServo.write(0);
  rotationServo.write(115);

  Serial.println("Ready. Running stored point sequence...");
}

void loop() 
{
//***********************************************
 // StartMovingConvayer();
  WaittingForObject();
  delay(90);
  //StopMovingConvayer();
//************************************************
  GetColor();

  if( ColorValue == 1 ) //For Green
  {
    GetObjectFromMain();
    MoveObjectToFabricConvayer();
  }
  else if( ColorValue == 2 )  //For Blue
  {
    GetObjectFromMain();
    MoveObjectToPlasticConvayer();
  }
  else
  {
    //StartMovingConvayer();
  }
 
}

// ------------------------------------------------------------------ Homing Function -------------------------------------------------------------------
void homeDeltaRobot() 
{
  Serial.println("Homing...");

  for (int i = 0; i < 3; i++) 
  {
    digitalWrite(DIR_PINS[i], DIR_UP);
    steppers[i].setSpeed(HOMING_SPEED);
  }

  while (!(homed[0] && homed[1] && homed[2])) 
  {
    for (int i = 0; i < 3; i++) 
    {
      if (!homed[i]) 
      {
        bool reading = digitalRead(LIMIT_PINS[i]);
        if (reading != lastSwitchState[i]) 
        {
          lastDebounceTime[i] = millis();
        }
        if ((millis() - lastDebounceTime[i]) > DEBOUNCE_DELAY) 
        {
          if (reading == LOW) 
          {
            homed[i] = true;
            steppers[i].stop();
          } 
          else 
          {
            steppers[i].runSpeed();
          }
        }
        lastSwitchState[i] = reading;
      }
    }
  }

  for (int i = 0; i < 3; i++) 
  {
    digitalWrite(DIR_PINS[i], DIR_DOWN);
    steppers[i].move(-BACKOFF_STEPS);
  }

  bool moving = true;
  while (moving) 
  {
    moving = false;
    for (int i = 0; i < 3; i++) 
    {
      if (steppers[i].distanceToGo() != 0) 
      {
        steppers[i].run();
        moving = true;
      }
    }
  }

  for (int i = 0; i < 3; i++) 
  {
    steppers[i].setCurrentPosition(0);
  }

  Serial.println("Homing complete.");
  delay(100);
}

// ------------------------------------------------------------------ Move to Point -------------------------------------------------------------------
void moveTo_MainPoint() 
{
  delay(100);
  multiStepper.moveTo(pointMain);
  multiStepper.runSpeedToPosition();
}

void moveTo_P_Point() 
{
  delay(100);
  multiStepper.moveTo(pointP);
  multiStepper.runSpeedToPosition();
}

void moveTo_F_Point() 
{
  delay(100);
  multiStepper.moveTo(pointF);
  multiStepper.runSpeedToPosition();
  
}

void moveTo_Home()
{
  delay(100);
  multiStepper.moveTo(CenterPoint);
  multiStepper.runSpeedToPosition();

  for (int i = 0; i < 3; i++) 
  {
    homed[i] = false;
  }

  homeDeltaRobot();

}

void OpenGripper()
{   
  gripperServo.write(0);  
}

void CloseGripper()
{   
  gripperServo.write(45); 
}

void MainRotation()
{   
  rotationServo.write(115);  
}

void F_Rotation()
{   
  rotationServo.write(0);  
}

void P_Rotation()
{   
  rotationServo.write(50);  
}

void StartMovingConvayer()
{  
  digitalWrite(DC_MOTOR_PIN, HIGH); 
}

void StopMovingConvayer()
{  
  digitalWrite(DC_MOTOR_PIN, LOW); 
}

void WaittingForObject()
{
  while(sensorState = digitalRead(PROXIMITY_PIN));
}
  
void GetObjectFromMain()
{
    delay(500);
  MainRotation();
  moveTo_MainPoint();
  CloseGripper();
  delay(1000);
  moveTo_Home();
}
  
void MoveObjectToFabricConvayer()
{
  delay(500);
  F_Rotation();
  moveTo_F_Point();
  OpenGripper();
  delay(1000);
  moveTo_Home();
}
  
void MoveObjectToPlasticConvayer()
{
  delay(1000);
  P_Rotation();
  moveTo_P_Point();
  OpenGripper();
  delay(1000);
  moveTo_Home();
}

void GetColor()
{
  delay(2000);
  if(digitalRead(7) && digitalRead(8))
  {
      ColorValue = 1;   //for Green
  }
  else if (!(digitalRead(7) && digitalRead(8)))
  {
    ColorValue = 2;   //for Blue
  }
  else
  {
    ColorValue = 3; // for metal
  }
}













































