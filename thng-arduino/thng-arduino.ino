/*
THNG Droid Building

template by: Prof. McLaughlin

further modified by: Blake Caven

*/
// ---------------------------------------------------------------------------------------
//                          Libraries
// ---------------------------------------------------------------------------------------
#include <PS3BT.h>
#include <usbhub.h>
#include <Sabertooth.h>
#include <Adafruit_TLC5947.h>
#include <MP3Trigger.h>
#include <Servo.h> 
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

// ---------------------------------------------------------------------------------------
//                 Servo setup
// ---------------------------------------------------------------------------------------

Servo myServo;
int servo_angle = 10; // where the servo is supposed to be
int servo_max_angle = 170;
int servo_min_angle = 10;

// --------------------------------------------------------------------------------------
//                Sabertooth setup
// --------------------------------------------------------------------------------------
int driveDeadBandRange = 10;
#define SABERTOOTH_ADDR 128
Sabertooth *ST = new Sabertooth(SABERTOOTH_ADDR, Serial1);

// ---------------------------------------------------------------------------------------
//                 Setup for USB, Bluetooth Dongle, & PS3 Controller
// ---------------------------------------------------------------------------------------
USB Usb;
BTD Btd(&Usb);
PS3BT *PS3Controller=new PS3BT(&Btd);

// ---------------------------------------------------------------------------------------
//    Used for PS3 Fault Detection
// ---------------------------------------------------------------------------------------
uint32_t msgLagTime = 0;
uint32_t lastMsgTime = 0;
uint32_t currentTime = 0;
uint32_t lastLoopTime = 0;
int badPS3Data = 0;
byte joystickDeadZoneRange = 15;

boolean isPS3ControllerInitialized = false;
boolean mainControllerConnected = false;
boolean WaitingforReconnect = false;
boolean isFootMotorStopped = true;

// ---------------------------------------------------------------------------------------
//    Used for PS3 Controller Request Management
// ---------------------------------------------------------------------------------------
long previousRequestMillis = millis();
boolean extraRequestInputs = false;

// ---------------------------------------------------------------------------------------
//    Request State Machine Variables for PS3 Controller
// ---------------------------------------------------------------------------------------

// Main state varable to determine if a request has been made by the PS3 Controller
boolean reqMade = false;
boolean reqLeftJoyMade = false;
boolean reqRightJoyMade = false;

// LEFT & RIGHT Joystick State Request Values
int turn_direction = -1;
int MAX_CONTROLLER = 50;
int MAX_TURN = 30;
int CONTROLLER_RAMP = 4;
boolean reqLeftJoyUp = false;
boolean reqLeftJoyDown = false;
int reqLeftJoyYValue = 0;

boolean reqLeftJoyLeft = false;
boolean reqLeftJoyRight = false;
int reqLeftJoyXValue = 0;

boolean reqRightJoyUp = false;
boolean reqRightJoyDown = false;
int reqRightJoyYValue = 0;

boolean reqRightJoyLeft = false;
boolean reqRightJoyRight = false;
int reqRightJoyXValue = 0;

// PS3 Controller Button State Variables
boolean reqArrowUp = false;
boolean reqArrowDown = false;
boolean reqArrowLeft = false;
boolean reqArrowRight = false;
boolean reqCircle = false;
boolean reqCross = false;
boolean reqTriangle = false;
boolean reqSquare = false;
boolean reqL1 = false;
boolean reqL2 = false;
boolean reqR1 = false;
boolean reqR2 = false;
boolean reqSelect = false;
boolean reqStart = false;
boolean reqPS = false;

// ---------------------------------------------------------------------------------------
//    Used for Pin 13 Main Loop Blinker
// ---------------------------------------------------------------------------------------
long blinkMillis = millis();
boolean blinkOn = false;

// ---------------------------------------------------------------------------------------
//    Used for drive chain
// ---------------------------------------------------------------------------------------
int currentSpeed = 0;
int currentTurn = 0;
#define MAX_SPEED 70
boolean droidMoving = false;

// --------------------------------------------------------------------------------------
//  Color Sensor
// --------------------------------------------------------------------------------------

#define s0 30
#define s1 28
#define s2 26
#define s3 24
#define LED 34
#define out 22

long rValue = 0;
long bValue = 0;
long gValue = 0;
long data = 0;
int currentColor = 0; // 0=unknown 1=red 2=green 3=blue
int currentRead = 1; // 1=red 2=green = 3=blue

// ---------------------------------------------------------------------------------------
//  Autonomous
// ---------------------------------------------------------------------------------------
// this boolean should be unnecessary if the program is written using modes instead
// TODO: confirm and remove all mentions
boolean autonomous = false;
int mode = 0;
#define COLOR_RANGE 30
int action = 1; // 1 = forward 2 = right 3 = left
#define AUTO_SPEED 40

// =======================================================================================
//                                 Main Program
// =======================================================================================
// =======================================================================================
//                                Setup Function
// =======================================================================================
void setup()
{
  
    //Initialize Serial @ 115200 baud rate for Serial Monitor Debugging
    Serial.begin(115200);
    while (!Serial);
    
    //Initialize the USB Dongle and check for errors
    if (Usb.Init() == -1)
    {
        Serial.println("OSC did not start");
        while (1); //halt
    }
    
    Serial.println("Bluetooth Library Started");
    
    //PS3 Controller - sets the interrupt function to call when PS3 controller tries to connect
    PS3Controller->attachOnInit(onInitPS3Controller); 

    //Setup PIN 13 for Arduino Main Loop Blinker Routine
    pinMode(13, OUTPUT);
    digitalWrite(13, LOW);

   // ----------------------------------------------
   // YOUR SETUP CONTROL CODE SHOULD START HERE
   // ----------------------------------------------

   // Servo PWM setup
   // TODO: make these useful
   myServo.attach(9);

   // ---------------------------------------------
   // Drive chain setup
   // ---------------------------------------------
   Serial1.begin(9600);
   ST->autobaud();
   ST->setTimeout(200);
   ST->setDeadband(driveDeadBandRange);
   
   // ---------------------------------------------
   // color sensor
   // ---------------------------------------------
   pinMode(s0, OUTPUT);
   pinMode(s1, OUTPUT);
   pinMode(s2, OUTPUT);
   pinMode(s3, OUTPUT);
   pinMode(LED, OUTPUT);
   pinMode(out, INPUT);

   digitalWrite(s0, HIGH); // 20% scaling
   digitalWrite(s1, LOW);
   
   // test LEDs
   // TODO: write this into its own function
   digitalWrite(LED, HIGH);
   delay(1000);
   digitalWrite(LED, LOW);
   delay(1000);
   digitalWrite(LED, HIGH);
   delay(1000);
   digitalWrite(LED, LOW);
}

// =======================================================================================
//    Main Program Loop - This is the recurring check loop for entire sketch
// =======================================================================================
void loop()
{   
   // Make sure the PS3 Controller is working - skip main loop if not
   if ( !readUSB() )
   {
     return;
   }

   // If the PS3 controller has been connected - start processing the main controller routines
   if (PS3Controller->PS3Connected) {
   
       // Read the PS3 Controller and set request state variables for this loop
       readPS3Request();

       // listen to input to change the current mode
       if (reqCross) {
        turn_direction *= -1;
        Serial.print("switched turn direction: ");
        Serial.println(turn_direction);
       }
       if (reqCircle) {
        if (mode == 1) {
          Serial.println("mode 0: no movement");
          stop();
          mode = 0;
        } else {
          autonomous = false;
          digitalWrite(LED, LOW);

          mode = 1;
          Serial.println("mode 1: manual driving");
        }
       }
       
       if (reqTriangle) {
        if (autonomous) {
          Serial.println("autonomous turned off");
          Serial.println("mode 0");
          autonomous = false;
          digitalWrite(LED, LOW);

          mode = 0;
          stop();
        } else {
          Serial.println("autonomous turned on");
          digitalWrite(LED, HIGH);

          autonomous = true;
          mode = 2;
          Serial.println("mode 2: autonomous");
        }
       }

       // do things based on what mode we are in
       if (mode == 1) {
        moveDroidManual(); 
       }
       if (mode == 2) {
        //Serial.println("running autonomous");
        if (millis() % 20 == 0) {
          autonomousDriving();
        }
       }
       if (mode == 0) {
        if (currentTurn != 0 || currentSpeed != 0) {
          Serial.println("stopping the droid");
          currentTurn = 0;
          currentSpeed = 0;
          ST->stop();
        }
        // stuff
       }

       
       // Ignore extra inputs from the PS3 Controller for 1/2 second from prior input
       if (extraRequestInputs)
       {
          if ((previousRequestMillis + 500) < millis())
          {
              extraRequestInputs = false;
          }
       }
    
       // If there was a PS3 request this loop - reset the request variables for next loop
       if (reqMade) {
           resetRequestVariables();
           reqMade = false;
       } 
   }

   // Blink to show working heart beat on the Arduino control board
   // If Arduino LED is not blinking - the sketch has crashed
   if ((blinkMillis + 500) < millis()) {
      if (blinkOn) {
        digitalWrite(13, LOW);
        blinkOn = false;
      } else {
        digitalWrite(13, HIGH);
        blinkOn = true;
      }
      blinkMillis = millis();
   }
}

// =======================================================================================
//      ADD YOUR CUSTOM DROID FUNCTIONS STARTING HERE
// =======================================================================================
void stop() {
  ST->turn(0);
  ST->drive(0);
  droidMoving = false;
}

int distance(int a, int b) {
  int c = a - b;
  if (b < 0 && a > 0) {
    c = b - a;
  } else if (b > 0 && a < 0) {
    c = a - b;
  }
  return abs(c);
}

void moveDroidManual() {
  if (reqLeftJoyMade) {
    // remap controller value to a custom max value
    int desiredSpeed = (reqLeftJoyYValue / MAX_CONTROLLER) * MAX_SPEED * turn_direction * -1;
    int desiredTurn = (reqLeftJoyXValue / MAX_CONTROLLER) * MAX_TURN * turn_direction; // invert turn so we turn the way our joystick moves
    // need both of these values (turn, drive) for the motorcontroller
    
    Serial.println(distance(currentTurn, desiredTurn));
    if (distance(currentSpeed, desiredSpeed) < 10) {
      currentSpeed = desiredSpeed;
    } else {
      if (desiredSpeed > currentSpeed) {
        // accelerating
        currentSpeed += distance(desiredSpeed, currentSpeed) / CONTROLLER_RAMP;
      } else {
        // decelrating
        currentSpeed -= distance(desiredSpeed, currentSpeed) / CONTROLLER_RAMP;
      }
    }
    if (distance(currentTurn, desiredTurn) < 10) {
      currentTurn = desiredTurn;
    } else {
      if (desiredTurn > currentTurn) {
        currentTurn += distance(desiredTurn, currentTurn) / CONTROLLER_RAMP;
      } else {
        currentTurn -= distance(desiredTurn, currentTurn) / CONTROLLER_RAMP;
      }
    }
    Serial.print("current turn:");
    Serial.print(currentTurn);
    Serial.print(" current speed:");
    Serial.println(currentSpeed);
    Serial.print("desired turn:");
    Serial.print(desiredTurn);
    Serial.print(" desired speed:");
    Serial.println(desiredSpeed);
    ST->turn(currentTurn);
    ST->drive(currentSpeed);
    if (!droidMoving) {
      droidMoving = true;
    }
  } else {
    if (droidMoving) {
      if (currentSpeed == 0 && currentTurn == 0) {
        ST->stop();
        droidMoving = false;
      } else {
        currentSpeed = currentSpeed / 4;
        currentTurn = currentTurn / 4;
        if (distance(0, currentSpeed) < 5) {
          currentSpeed = 0;
        }
        if (distance(0, currentTurn) < 5) {
          currentTurn = 0;
        }
      }
    }
  }
}

// ---------------------------------------------------------------------------------------
//    Autonomous Functions + Light Sensor
//        if red turn right
//        if green turn left
//        if blue go straight
//        if it isnt one of those, stop
//        red = 1, green = 3, blue = 2
// --------------------------------------------------------------------------------------
void autoDrive() {
  droidMoving = true;
  if (action == 1) {
    if (currentColor == 1) {
      action = 2;
      Serial.println("turning in the positive direction");
    } else if (currentColor == 3) {
      action = 3;
      Serial.println("turning in the negative direction");
    }
  } else if (action == 2) {
    // turn right
    currentSpeed = 0;
    currentTurn = AUTO_SPEED * -1;
  } else if (action == 3) {
    // turn left
    currentSpeed = 0;
    currentTurn = AUTO_SPEED;
  }
  if (currentColor == 2) {
    action = 1;
    currentTurn = 0;
    currentSpeed = AUTO_SPEED * -1; // technically its "backwards"...
    Serial.println("going forward");
  }
  Serial.print("currentTurn: ");
  Serial.println(currentTurn);
  Serial.print("CurrentSpeed: ");
  Serial.println(currentSpeed);
  ST->turn(currentTurn);
  ST->drive(currentSpeed);
}

// TODO: Calibrate map
void readRedValue() {
  int low_bound = 46;
  int high_bound = 129;
  digitalWrite(s2, LOW);
  digitalWrite(s3, LOW);
  data = pulseIn(out, LOW);
  
  rValue = data; // map(data, low_bound, high_bound, 255, 0);
  
}
void readBlueValue() { // this is consistently detecting blue
  int low_bound = 51;
  int high_bound = 129;

  digitalWrite(s2, LOW);
  digitalWrite(s3, HIGH);
  data = pulseIn(out, LOW);
  
  bValue = data; // map(data, low_bound, high_bound, 255, 0);
  
}
void readGreenValue() { // this is consistently detecting green
  int low_bound = 51;
  int high_bound = 129;
  digitalWrite(s2, HIGH);
  digitalWrite(s3, HIGH);
  data = pulseIn(out, LOW);
  
  gValue = data; // map(data, low_bound, high_bound, 255, 0);
  
}

void readColor() {
  /*
  read each r,g,b value one at a time, if we have looped back around - set the current color

  from early tests, it looks like the lowest raw value is the dominant color
  */
  int oldColor = currentColor;
  if (currentRead == 1) {
    readRedValue();
    currentRead = 2;
  } else if (currentRead == 2) {
    readGreenValue();
    currentRead = 3;
  } else if (currentRead == 3) {
    readBlueValue();
    currentRead = 1;
  }
  if (currentRead == 1) {
    // default color is red even though this might not be the case
    // TODO: add detection for colors that mostly white (the floor) 
    int currentLowest = rValue;
    currentColor = 1;
    if (bValue < currentLowest) {
      currentLowest = bValue;
      currentColor = 2;
    }
    if (gValue < currentLowest) {
      currentLowest = gValue;
      currentColor = 3;
    }
    if (distance(gValue, bValue) < COLOR_RANGE && distance(gValue, rValue) < COLOR_RANGE && distance(bValue, rValue) < COLOR_RANGE) {
      Serial.print("unknown color: ");
      Serial.print(rValue);
      Serial.print(", ");
      Serial.print(gValue);
      Serial.print(", ");
      Serial.println(bValue);
    }
    // TODO: if the colors are similar to each other, say the color is invalid
  }
  if (oldColor != currentColor) {
    Serial.print("new color: ");
    Serial.println(currentColor);
    Serial.print("rgb: ");
    Serial.print(rValue);
    Serial.print(", ");
    Serial.print(gValue);
    Serial.print(", ");
    Serial.println(bValue);
  }
}

/*
  Autonomous driving:
    The idea is that you go forward when the color is blue and turn when you hit a side.
    The right side is always green
    The left side is alwasy red
*/
void autonomousDriving() {
  // if STOP: return
  // if green turn left
  // if red turn right
  // if blue go straight
  readColor();
  // going to need to do a check for "real" color or we can just set the current color to invalid (-1)
  if (currentColor != -1) {
    autoDrive();
  } else {
    Serial.println("stopped");
    droidMoving = false;
    currentSpeed = 0;
    currentTurn = 0;
    ST->stop();
  }
  
  
}

// =======================================================================================
//      YOUR CUSTOM DROID FUNCTIONS SHOULD END HERE
// =======================================================================================

// =======================================================================================
//      CORE DROID CONTROL FUNCTIONS START HERE - EDIT WITH CAUTION
// =======================================================================================
// Read the PS3 Controller and set request state variables
void readPS3Request()
{
     if (!extraRequestInputs) {
      
         if (PS3Controller->getButtonPress(UP))
         {              
                Serial.println("Button: UP Selected");
    
                reqArrowUp = true;
                reqMade = true;
                
                previousRequestMillis = millis();
                extraRequestInputs = true;
                
         }
      
         if (PS3Controller->getButtonPress(DOWN))
         {
                Serial.println("Button: DOWN Selected");
    
                reqArrowDown = true;
                reqMade = true;
                
                previousRequestMillis = millis();
                extraRequestInputs = true;
           
         }
    
         if (PS3Controller->getButtonPress(LEFT))
         {
                Serial.println("Button: LEFT Selected");
    
                reqArrowLeft = true;
                reqMade = true;
                
                previousRequestMillis = millis();
                extraRequestInputs = true;
    
         }
         
         if (PS3Controller->getButtonPress(RIGHT))
         {
                Serial.println("Button: RIGHT Selected");
    
                reqArrowRight = true;
                reqMade = true;
                
                previousRequestMillis = millis();
                extraRequestInputs = true;
                         
         }
         
         if (PS3Controller->getButtonPress(CIRCLE))
         {
                Serial.println("Button: CIRCLE Selected");
    
                reqCircle = true;
                reqMade = true;
                
                previousRequestMillis = millis();
                extraRequestInputs = true;
               
         }
    
         if (PS3Controller->getButtonPress(CROSS))
         {
                Serial.println("Button: CROSS Selected");
    
                reqCross = true;
                reqMade = true;
                
                previousRequestMillis = millis();
                extraRequestInputs = true;
                  
         }
         
         if (PS3Controller->getButtonPress(TRIANGLE))
         {
                Serial.println("Button: TRIANGLE Selected");
    
                reqTriangle = true;
                reqMade = true;
                
                previousRequestMillis = millis();
                extraRequestInputs = true;
                  
         }
         
    
         if (PS3Controller->getButtonPress(SQUARE))
         {
                Serial.println("Button: SQUARE Selected");
    
                reqSquare = true;
                reqMade = true;
                
                previousRequestMillis = millis();
                extraRequestInputs = true;
                  
         }
         
         if (PS3Controller->getButtonPress(L1))
         {
                Serial.println("Button: LEFT 1 Selected");
    
                reqL1 = true;
                reqMade = true;
                
                previousRequestMillis = millis();
                extraRequestInputs = true;
         }
    
         if (PS3Controller->getButtonPress(L2))
         {
                Serial.println("Button: LEFT 2 Selected");
    
                reqL2 = true;
                reqMade = true;
                
                previousRequestMillis = millis();
                extraRequestInputs = true;
         }
    
         if (PS3Controller->getButtonPress(R1))
         {
                Serial.println("Button: RIGHT 1 Selected");
    
                reqR1 = true;
                reqMade = true;
                
                previousRequestMillis = millis();
                extraRequestInputs = true;
         }
    
         if (PS3Controller->getButtonPress(R2))
         {
                Serial.println("Button: RIGHT 2 Selected");
    
                reqR2 = true;
                reqMade = true;
                
                previousRequestMillis = millis();
                extraRequestInputs = true;
         }
    
         if (PS3Controller->getButtonPress(SELECT))
         {
                Serial.println("Button: SELECT Selected");
    
                reqSelect = true;
                reqMade = true;
                
                previousRequestMillis = millis();
                extraRequestInputs = true;
         }
    
         if (PS3Controller->getButtonPress(START))
         {
                Serial.println("Button: START Selected");
    
                reqStart = true;
                reqMade = true;
                
                previousRequestMillis = millis();
                extraRequestInputs = true;
         }
    
         if (PS3Controller->getButtonPress(PS))
         {
                Serial.println("Button: PS Selected");
    
                reqPS = true;
                reqMade = true;
                
                previousRequestMillis = millis();
                extraRequestInputs = true;
         }
     }

     if (((abs(PS3Controller->getAnalogHat(LeftHatY)-128) > joystickDeadZoneRange) || (abs(PS3Controller->getAnalogHat(LeftHatX)-128) > joystickDeadZoneRange)))
     {    
            reqLeftJoyUp = false;
            reqLeftJoyDown = false;
            reqLeftJoyLeft = false;
            reqLeftJoyRight = false;
            reqLeftJoyYValue = 0;
            reqLeftJoyXValue = 0;
            reqLeftJoyMade = true;

            int currentValueY = PS3Controller->getAnalogHat(LeftHatY) - 128;
            int currentValueX = PS3Controller->getAnalogHat(LeftHatX) - 128;
            
            char yString[5];
            itoa(currentValueY, yString, 10);

            char xString[5];
            itoa(currentValueX, xString, 10);

            Serial.print("LEFT Joystick Y Value: ");
            Serial.println(yString);
            Serial.print("LEFT Joystick X Value: ");
            Serial.println(xString);

            if (currentValueY > joystickDeadZoneRange) {
                Serial.println("Left Joystick DOWN");
                reqLeftJoyDown = true;
                reqLeftJoyYValue = currentValueY;
            }

            if (currentValueY < (-1 * joystickDeadZoneRange)) {
                Serial.println("Left Joystick UP");
                reqLeftJoyUp = true;
                reqLeftJoyYValue = currentValueY;
            }

            if (currentValueX > joystickDeadZoneRange) {
                Serial.println("Left Joystick RIGHT");
                reqLeftJoyRight = true;
                reqLeftJoyXValue = currentValueX;
            }
            
            if (currentValueX < (-1 * joystickDeadZoneRange)) {
                Serial.println("Left Joystick LEFT");
                reqLeftJoyLeft = true;
                reqLeftJoyXValue = currentValueX;
            }
     } else {
          if (reqLeftJoyMade) {
              reqLeftJoyUp = false;
              reqLeftJoyDown = false;
              reqLeftJoyLeft = false;
              reqLeftJoyRight = false;
              reqLeftJoyYValue = 0;
              reqLeftJoyXValue = 0;
              reqLeftJoyMade = false;
          }
     }

     if (((abs(PS3Controller->getAnalogHat(RightHatY)-128) > joystickDeadZoneRange) || (abs(PS3Controller->getAnalogHat(RightHatX)-128) > joystickDeadZoneRange)))
     {
            reqRightJoyUp = false;
            reqRightJoyDown = false;
            reqRightJoyLeft = false;
            reqRightJoyRight = false;
            reqRightJoyYValue = 0;
            reqRightJoyXValue = 0;
            reqRightJoyMade = true;
            
            int currentValueY = PS3Controller->getAnalogHat(RightHatY) - 128;
            int currentValueX = PS3Controller->getAnalogHat(RightHatX) - 128;

            char yString[5];
            itoa(currentValueY, yString, 10);

            char xString[5];
            itoa(currentValueX, xString, 10);

            Serial.print("RIGHT Joystick Y Value: ");
            Serial.println(yString);
            Serial.print("RIGHT Joystick X Value: ");
            Serial.println(xString);

            if (currentValueY > joystickDeadZoneRange) {
                Serial.println("Right Joystick DOWN");
                reqRightJoyDown = true;
                reqRightJoyYValue = currentValueY;
            }

            if (currentValueY < (-1 * joystickDeadZoneRange)) {
                Serial.println("Right Joystick UP");
                reqRightJoyUp = true;
                reqRightJoyYValue = currentValueY;
            }

            if (currentValueX > joystickDeadZoneRange) {
                Serial.println("Right Joystick RIGHT");
                reqRightJoyRight = true;
                reqRightJoyXValue = currentValueX;
            }
            
            if (currentValueX < (-1 * joystickDeadZoneRange)) {
                Serial.println("Right Joystick LEFT");
                reqRightJoyLeft = true;
                reqRightJoyXValue = currentValueX;
            }
     } else {
          if (reqRightJoyMade) {
              reqRightJoyUp = false;
              reqRightJoyDown = false;
              reqRightJoyLeft = false;
              reqRightJoyRight = false;
              reqRightJoyYValue = 0;
              reqRightJoyXValue = 0;
              reqRightJoyMade = false;
          }
     }    
}

// Reset the PS3 request variables on every processing loop when needed
void resetRequestVariables()
{
    reqArrowUp = false;
    reqArrowDown = false;
    reqArrowLeft = false;
    reqArrowRight = false;
    reqCircle = false;
    reqCross = false;
    reqTriangle = false;
    reqSquare = false;
    reqL1 = false;
    reqL2 = false;
    reqR1 = false;
    reqR2 = false;
    reqSelect = false;
    reqStart = false;
    reqPS = false;
}

// Initialize the PS3 Controller Trying to Connect
void onInitPS3Controller()
{
    PS3Controller->setLedOn(LED1);
    isPS3ControllerInitialized = true;
    badPS3Data = 0;

    mainControllerConnected = true;
    WaitingforReconnect = true;

    Serial.println("We have the controller connected");
    Serial.print("Dongle Address: ");
    String dongle_address = String(Btd.my_bdaddr[5], HEX) + ":" + String(Btd.my_bdaddr[4], HEX) + ":" + String(Btd.my_bdaddr[3], HEX) + ":" + String(Btd.my_bdaddr[2], HEX) + ":" + String(Btd.my_bdaddr[1], HEX) + ":" + String(Btd.my_bdaddr[0], HEX);
    Serial.println(dongle_address);
}

// Determine if we are having connection problems with the PS3 Controller
boolean criticalFaultDetect()
{
    if (PS3Controller->PS3Connected)
    {
        
        currentTime = millis();
        lastMsgTime = PS3Controller->getLastMessageTime();
        msgLagTime = currentTime - lastMsgTime;            
        
        if (WaitingforReconnect)
        {   
            if (msgLagTime < 200)
            {          
                WaitingforReconnect = false;         
            }
            lastMsgTime = currentTime;    
        } 
        
        if ( currentTime >= lastMsgTime)
        {
              msgLagTime = currentTime - lastMsgTime;      
        } else
        {
             msgLagTime = 0;
        }
        
        if ( msgLagTime > 5000 )
        {
            Serial.println("It has been 5s since we heard from Controller");
            Serial.println("Disconnecting the controller");
            
            PS3Controller->disconnect();
            WaitingforReconnect = true;
            return true;
        }

        //Check PS3 Signal Data
        if(!PS3Controller->getStatus(Plugged) && !PS3Controller->getStatus(Unplugged))
        {
            //We don't have good data from the controller.
            //Wait 15ms - try again
            delay(15);
            Usb.Task();   
            lastMsgTime = PS3Controller->getLastMessageTime();
            
            if(!PS3Controller->getStatus(Plugged) && !PS3Controller->getStatus(Unplugged))
            {
                badPS3Data++;
                Serial.println("**Invalid data from PS3 Controller. - Resetting Data**");
                return true;
            }
        }
        else if (badPS3Data > 0)
        {

            badPS3Data = 0;
        }
        
        if ( badPS3Data > 10 )
        {
            Serial.println("Too much bad data coming from the PS3 Controller");
            Serial.println("Disconnecting the controller");
            
            PS3Controller->disconnect();
            WaitingforReconnect = true;
            return true;
        }
    }
    
    return false;
}

// USB Read Function - Supports Main Program Loop
boolean readUSB()
{
    Usb.Task();    
    //The more devices we have connected to the USB or BlueTooth, the more often Usb.Task need to be called to eliminate latency.
    if (PS3Controller->PS3Connected) 
    {
        if (criticalFaultDetect())
        {
            //We have a fault condition that we want to ensure that we do NOT process any controller data!!!
            return false;
        }
        
    } 
    return true;
}
