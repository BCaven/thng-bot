/*
TODO when you get back to this
update functions to have "sleep" periods after running
make screen not clear unless necessary

*/


//************************************************
// ND Jukebox
//************************************************
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

//************************************************
// Juke Box Variables
//************************************************
// State Variables
//************************************************
bool requestSongStart = false;
bool requestSongStop = false;
bool requestScrollUp = false;
bool requestScrollDown = false;
bool requestVolUp = false;
bool requestVolDown = false;

int currentSelectedSongNumber = 1;
int currentSelectedSongLength = 166;
String currentSelectedSongTitle = "Walk the Line";
long currentSelectedSongMillisStart = millis();
float currentSelectedSongPercentComplete = 0;
bool currentSelectedSongAtEnd = false;
bool currentSelectedSongPlaying = false;

int currentVolumeNumber = 50;  // from 0 HIGH to 100 LOW
float currentVolumePercentage = .5;

bool onMainMenu = true;
bool initMainScreenComplete = false;
bool onSongDetailScreen = false;
bool initSongDetailComplete = false;
bool onVolDetailScreen = false;
bool initVolumeScreenComplete = false;
int currentTopScrollSongNumber = 1;

long scrollScreenDelayMillis = millis();
int scrollScreenDelayInterval = 500;
long refreshPercentCompleteMillis = millis();
int refreshPercentCompleteInterval = 500;
long refreshPercentVolumeMillis = millis();
int refreshPercentVolumeInterval = 500;

String songTitle[36] = {"Walk the Line",
                    "Ring of Fire",
                    "Blue Suede Shoes",
                    "So Lonesome",
                    "Folsom Prison",
                    "Cheatin Heart",
                    "Jolene",
                    "Big River",
                    "Blues Eyes Cryin",
                    "Imagine",
                    "Long Tall Sally",
                    "Pretty Woman",
                    "Peggy Sue",
                    "Everyday",
                    "La Bamba",
                    "Sweet Dreams",
                    "Desperado",
                    "The Twist",
                    "Respect",
                    "People Get Ready",
                    "Dock of the Bay",
                    "Dancing Streets",
                    "My Imagination",
                    "Stay Together",
                    "Papa New Bag",
                    "Stany By Me",
                    "Who Do You Love",
                    "My Generation",
                    "Yesterday",
                    "Mr Tambourine",
                    "Fighting Man",
                    "Paranoid",
                    "Highway to Hell",
                    "Roxanne",
                    "Lola",
                    "Love Rock N Roll"};
                    
int songTrack[36]={1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36};

long songLength[36]={166,157,136,169,165,163,162,168,140,184,129,180,150,128,124,211,121,156,148,159,162,158,181,198,128,178,150,199,127,150,139,173,208,195,250,175};

// 


// mp3 player
MP3Trigger mp3trigger;
#define MAX_VOLUME 110
#define MIN_VOLUME 1
int speaker_volume = 110;

// tft board
#define MISO 50
#define MOSI 51
#define SCK 52
#define TCS 7
#define RST 4
#define TDC 5
#define CARD_CS 6
Adafruit_ST7735 tft = Adafruit_ST7735(TCS, TDC, RST);

/*
PLAN:

pages:
song select
    list of the songs (at least five on screen)
volume
    current volume
    NOTE: only shows up when adjusting volume
current song info
    when a song is playing:
    name
    % complete

buttons:
start
stop
change song (joystick)
change volume (joystick)

So,
do not need modes for buttons

do need modes for pages

modes:
    song select
    volume
    current song info

*/
// 0 = main page
// 1 = current song
// 2 = volume popup
int page_mode = 0;
int old_mode = 0;
// how long to hold volume popup
long vol_timeout = 1000;
// when the volume button was last triggered
long vol_triggered = 0;
#define DISPLAY_REFRESH 300
#define VOLUME_REFRESH 250
#define TRACK_REFRESH 250

// sleep triggers
long last_display_trigger = 0;
long last_volume_trigger = 0;
long last_track_refresh_trigger = 0;

// graphics triggers
bool clear_screen = false;


void setup() {
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


    // sound
    mp3trigger.setup(&Serial2);
    Serial2.begin(MP3Trigger::serialRate());
    mp3trigger.setVolume(speaker_volume);

    // tft board
    tft.initR(INITR_BLACKTAB);
    testlines(ST77XX_BLUE);
}

void loop() {
    if ( !readUSB() ) {
        return;
    }

    if (clear_screen) {
        // clear the screen
        tft.fillScreen(ST77XX_BLACK);

        clear_screen = false;
    }
    // If the PS3 controller has been connected - start processing the main controller routines
    if (PS3Controller->PS3Connected) {
    
        // Read the PS3 Controller and set request state variables for this loop
        readPS3Request();
        /*
        Controller actions
        PLAY
        PAUSE
        joystick song change
        joystick volume change

        */
        if (reqTriangle) {
            // play
            // start the song
            clear_screen = true;
            page_mode = 1;
            currentSelectedSongMillisStart = millis();
            currentSelectedSongLength = 1000 * songLength[currentSelectedSongNumber];
            mp3trigger.trigger(currentSelectedSongNumber);
        }
        if (reqCross) {
            clear_screen = true;
            // stop the song
            mp3trigger.stop();
            page_mode = 0;
        }
        if (reqLeftJoyMade) {
            // volume control
            if (millis() - last_volume_trigger > VOLUME_REFRESH) {
                last_volume_trigger = millis();
                clear_screen = true;
                old_mode = page_mode;
                page_mode = 2;
                vol_triggered = millis();
                change_volume();
            }
        }
        if (reqRightJoyMade) {
            // track selection
            if (millis() - last_track_refresh_trigger > TRACK_REFRESH) {
                last_track_refresh_trigger = millis();
                clear_screen = true;
                select_track();
            }
        }
        if (millis() - last_display_trigger > DISPLAY_REFRESH) {
            last_display_trigger = millis();
            if (page_mode == 0) {
                main_page();
            } else if (page_mode == 1) {
                current_page();
                if (millis() - currentSelectedSongMillisStart > currentSelectedSongLength) {
                    clear_screen = true;
                    page_mode = 0;
                }
            } else if (page_mode == 2) {
                if (millis() - vol_triggered > vol_timeout) {
                    page_mode = old_mode;
                    clear_screen = true;
                }
                // display volume page
                volume_page();
            }
        }


        // Ignore extra inputs from the PS3 Controller for 1/2 second from prior input
        if (extraRequestInputs) {
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
   // update mp3trigger
   mp3trigger.update();
}
void change_volume() {
    old_mode = page_mode;
    int vol_direction = 0;
    if (reqLeftJoyYValue > 0) {
        vol_direction = 1;
    } else {
        vol_direction = -1;
    }
    speaker_volume += vol_direction;
    if (speaker_volume > MAX_VOLUME) {
        speaker_volume = MAX_VOLUME;
    } else if (speaker_volume < MIN_VOLUME) {
        speaker_volume = MIN_VOLUME;
    }
    vol_triggered = millis();
    mp3trigger.setVolume(speaker_volume);
}

void select_track() {
    // remember cannot go past 0 or max len
    // no infinite scrolling
    int track_direction = 0;
    if (reqRightJoyYValue > 0) {
        track_direction = 1;
    } else {
        track_direction = -1;
    }
    currentSelectedSongNumber += track_direction;
    if (currentSelectedSongNumber < 0) {
        currentSelectedSongNumber = 0;
    } else if (currentSelectedSongNumber > 35) {
        currentSelectedSongNumber = 35;
    }
}

void main_page() {
    // main page has the following contents:
    // five songs - tick on the current song
    // clear the screen:
    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(0, 0);
    tft.setTextWrap(false);
        
    // edge cases: overflow to the edge
    // main case
    int start = -2;
    if (currentSelectedSongNumber < 2) {
        start = -1 * currentSelectedSongNumber;
    }
    int end = start + 5;
    if (currentSelectedSongNumber > 32) {
        end = 35 - currentSelectedSongNumber;
        start = end - 5;
    }
    Serial.print("start: ");
    Serial.print(start);
    Serial.print(" end: ");
    Serial.println(end);

    // TODO: check if the tft library clears the space into which the character is being written
    // if it does not, you will have to manually clear it
    for (int i = start; i < end; i++) {
        // display each line
        String name = songTitle[currentSelectedSongNumber + i];
        int inv_start = start * -1;
        if (i != 0) {
            tft.println(name);
        } else {
            String tag = ">>>";
            tag.concat(name);
            tft.println(tag);
        }
    }
}

void current_page() {
    // display the current song and percent complete
    String currentSongName = songTitle[currentSelectedSongNumber];
    // TODO: fix this percentage
    float real_percent_complete = (millis() - (currentSelectedSongLength + currentSelectedSongMillisStart)) / currentSelectedSongLength;
    int percent_complete = 100 * real_percent_complete;
    String bars = "";
    for (int i = 0; i < percent_complete / 10; i++) {
        bars += '=';
    }
    if (percent_complete % 10 > 5) {
        bars += '-';
    }
    tft.setCursor(0, 0);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextWrap(false);
    tft.println(currentSongName);
    tft.print("percent complete: ");
    tft.print(percent_complete);
    tft.println("%");
    tft.println("~~~~~");
    tft.println(bars);
}

void volume_page() {
    int vol_range = MAX_VOLUME - MIN_VOLUME;
    float real_vol_percent = speaker_volume / vol_range;
    int vol_percent = 100 * real_vol_percent;
    String bars = "";
    for (int i = 0; i < vol_percent / 10; i++) {
        bars += '=';
    }
    if (vol_percent % 10 > 5) {
        bars += '-';
    }

    // display the current volume
    tft.setCursor(0, 0);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextWrap(false);
    tft.print("volume: ");
    tft.print(vol_percent);
    tft.println("%");
    tft.println("");
    tft.println(bars);
    // remember: higher volume number is actually quieter volume
}

// video tft
void testlines(uint16_t color) {
  tft.fillScreen(ST77XX_BLACK);
  for (int16_t x=0; x < tft.width(); x+=6) {
    tft.drawLine(0, 0, x, tft.height()-1, color);
    delay(0);
  }
  for (int16_t y=0; y < tft.height(); y+=6) {
    tft.drawLine(0, 0, tft.width()-1, y, color);
    delay(0);
  }

  tft.fillScreen(ST77XX_BLACK);
  for (int16_t x=0; x < tft.width(); x+=6) {
    tft.drawLine(tft.width()-1, 0, x, tft.height()-1, color);
    delay(0);
  }
  for (int16_t y=0; y < tft.height(); y+=6) {
    tft.drawLine(tft.width()-1, 0, 0, y, color);
    delay(0);
  }

  tft.fillScreen(ST77XX_BLACK);
  for (int16_t x=0; x < tft.width(); x+=6) {
    tft.drawLine(0, tft.height()-1, x, 0, color);
    delay(0);
  }
  for (int16_t y=0; y < tft.height(); y+=6) {
    tft.drawLine(0, tft.height()-1, tft.width()-1, y, color);
    delay(0);
  }

  tft.fillScreen(ST77XX_BLACK);
  for (int16_t x=0; x < tft.width(); x+=6) {
    tft.drawLine(tft.width()-1, tft.height()-1, x, 0, color);
    delay(0);
  }
  for (int16_t y=0; y < tft.height(); y+=6) {
    tft.drawLine(tft.width()-1, tft.height()-1, 0, y, color);
    delay(0);
  }
}


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
