/*
THNG Droid Building

template by: Prof. McLaughlin

further modified by: Blake Caven

Tasks:
[DONE] redo control structure into modes and function delays (like jukebox)
[DONE] add video triggers to autonomous mode
[DONE] add audio triggers to autonomous mode
[TODO] record/find audio
[DONE] clean up manual driving
[TODO] time how long it takes for the tft to refresh/draw shapes (specifically filling rectangles)
[TODO] popup windows
[TODO] custom event 1
  [DONE] planning
  [TODO] code
  [TODO] hardware
[TODO] custom event 2
  [TODO] planning
  [TODO] code
  [TODO] hardware
[TODO] custom event 3
  [TODO] planning
  [TODO] code
  [TODO] hardware
[TODO] hidden feature: jukebox (space/time permitting)
[TODO] Physical build
[TODO] servo wiring/setup
[TODO] remove unused variables
[TODO] make sure all constants are #define
[TODO] random() for making ambient routines more interesting


PLAN FOR PHYSICAL BUILD:
  [TODO] design bones
  [TODO] print bones
  [TODO] holes for LEDs
  [TODO] paths for wiring

PLAN FOR ROUTINE 1
  The creature comes to life
    motion:
      starts normal then starts lurching
    video:
      mock boot sequence followed by screen corruption
    audio:
      grab the noises from the original movie (or just make some lol)
    lights:
      start low, pulse as the creature is coming to life, breathe as the creature is now ALIVE

PLAN FOR ROUTINE 2
  the creature *doing something*
  "vibing"

PLAN FOR ROUTINE 3
  the creature dying
    motion:
      irratic shaking
    audio:
      death screech
    video:
      fire
    lights:
      fire (if you can do it)

PLAN FOR AMBIENT:
  Sound:
    look at movie for idle noises
  Video:
    figure out where you are going to mount the tft board

  Lights:
    breathing
    maybe get eyes in there too

Design consideration:
Video:
  for animations only update the pixels that change so that way we dont have the screen flash as
  the animation plays

  also look into displaying images

Physical build considerations:
using buttons/stuff to give the back of the droid a "control panel"
would be funny to actually wire them to the pico or something

panels:
  put them on a 3d printed skeleton
  would be cool to have them "loosely" mounted so they can shift and move a little bit
    maybe this is where the servo comes in
back panel:
  "exposed" (fake) circuitry that looks half finished (maybe use the dead board?)
  button array near the tft to serve as a fake UI

bones:
  have as few as possible
  maybe print "joints" that give the panels a little bit of wiggle room
    will need to cut the panels to fit pretty well so they can support each other
  experiment with vibrational motors to see how well they can "shake" the panels

servo motor:
  have it "push" the panels to make it look like the top of the droid is opening

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
PS3BT *PS3Controller = new PS3BT(&Btd);

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
long last_controller_refresh = 0;
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
long last_manual_trigger = 0;

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
int currentRead = 1;  // 1=red 2=green = 3=blue

// ---------------------------------------------------------------------------------------
//  Autonomous
// ---------------------------------------------------------------------------------------
int mode = 0;
#define COLOR_RANGE 30
int action = 1; // 1 = forward 2 = right 3 = left
#define AUTO_SPEED 40
long last_autonomous_trigger = 0;

/*
Sound

TODO:
EVA dashboard sounds
record sounds
add sounds to sd card

*/
MP3Trigger mp3trigger;

// length of audio tracks
// TODO: patch library to ask controller if sound is playing
#define AUDIO_LENGTH 1000
#define CELEBRATION 1
#define BLUE_TO_GREEN 2
#define BLUE_TO_RED 3
#define BACK_TO_BLUE 4
// time at which last sound was triggered
long last_played = 0;
bool playing = false;
// effective range 1-110
// higher number is quieter
#define MAX_SPEAKER_VOLUME 1
#define MIN_SPEAKER_VOLUME 110
int speaker_volume = 50;

// dealing with audio tracks
int num_audio_tracks = 0;
long last_audio_trigger = 0;
long last_audio_mode_refresh = 0;
// length in millis
long track_lengths[10] = {
  10, 10, 10, 10, 10, 10, 10, 10, 10, 10
};


// --------------------------------------------------------------------------------------
// VIDEO
// --------------------------------------------------------------------------------------
#define MISO 50
#define MOSI 51
#define SCK 52
#define TCS 7
#define RST 4
#define TDC 5
#define CARD_CS 6
Adafruit_ST7735 tft = Adafruit_ST7735(TCS, TDC, RST);

/*
LEDs

*/
#define led_clock 9
#define led_data 8
#define led_latch 11
Adafruit_TLC5947 LEDControl = Adafruit_TLC5947(1, led_clock, led_data, led_latch);
#define LED_MAX_BRIGHT 4000
#define NUM_LEDS 3
int current_led = 0;
// --------------------------------------------------------------------------------------
// MODES (audio, video, control)
// --------------------------------------------------------------------------------------
// define how often each mode refreshes
#define CONTROL_MODE_REFRESH 100
#define AUTO_MODE_REFRESH 20
#define MANUAL_MODE_REFRESH 50
long last_control_mode_trigger = 0;

int audio_mode = 0;   // 0 = quiet, 1 = ambient
int video_mode = 0;   // TODO: define these
int control_mode = 0; // 0 = standby, 1 = manual, 2 = autonomous

// video control
#define CLEAR_REFRESH 100
bool clear_screen = true;
long last_clear_trigger = 0;
long last_video_mode_refresh = 0;

// motor control
bool twitchy = false;
#define MOTOR_MODE_REFRESH 1000
long last_motor_mode_refresh = 0;
int motor_mode = 0;

// LED modes
#define LED_MODE_REFRESH 1000
long last_led_mode_refresh = 0;
int led_mode = 0;

/*
Timing control
*/
#define AUDIO_MODE_REFRESH 300
#define VIDEO_MODE_REFRESH 250
#define CONTROL_MODE_REFRESH 20
#define TFT_DISPLAY_REFRESH 400
#define CONTROLLER_REFRESH 10
#define CUSTOM_ROUTINE_MIN 1000
#define CUSTOM_ROUTINE_MAX 2000
#define DOT_MIN_REFRESH 500
#define DOT_MAX_REFRESH 3000

/*
Ambient tracks:
uhhhhhhhhh
*/
#define AMBIENT_SOUND_REFRESH 2000
long last_ambient_sound = 0;
int FIRST_AMBIENT_TRACK = 0;
int LAST_AMBIENT_TRACK = 10;

/*
Custom sequences
*/
// picking sequences
// TODO: do this
// timing control for custom sequences
// control for which part of the sequence we are on
int sequence_location = 1;
#define SEQUENCE_1_NUM_LOCATIONS 19
bool go_to_next_sequence_location = false;
// loading dots when booting (...)
long boot_dot_refresh = 1000;
long last_boot_dot_trigger = 0;
int num_dots = 0;
bool printing_dots = false;
int custom_routine_sound_track = 0;
bool custom_rotuine_start_new_sound = false;
long custom_routine_refresh = 0;
// corruption animation

// twitching motors
#define MIN_TWITCH 70
#define MAX_TWITCH 90
#define MIN_TWITCH_TURN 40
#define MAX_TWITCH_TURN 50


// =======================================================================================
//                                 Main Program
// =======================================================================================
/*
Setup function:
*/
void setup()
{
  // Initialize Serial @ 115200 baud rate for Serial Monitor Debugging
  Serial.begin(115200);
  while (!Serial)
    ;

  // Initialize the USB Dongle and check for errors
  if (Usb.Init() == -1)
  {
    Serial.println("OSC did not start");
    while (1)
      ; // halt
  }

  Serial.println("Bluetooth Library Started");

  // PS3 Controller - sets the interrupt function to call when PS3 controller tries to connect
  PS3Controller->attachOnInit(onInitPS3Controller);

  // Setup PIN 13 for Arduino Main Loop Blinker Routine
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  // sound
  mp3trigger.setup(&Serial2);
  Serial2.begin(MP3Trigger::serialRate());
  mp3trigger.setVolume(speaker_volume);

  // tft board
  tft.initR(INITR_BLACKTAB);
  testlines(ST77XX_BLUE);

  // LED board
  LEDControl.begin();
  // turn them all off
  Serial.println("Turning the lights off");
  for (int i = 0; i < NUM_LEDS; i++) {
    LEDControl.setPWM(i, 0);
  }
  LEDControl.write();
  delay(1000);
  Serial.println("Turning the lights on");
  // turn them all on
  for (int i = 0; i < NUM_LEDS; i++) {
    LEDControl.setPWM(i, LED_MAX_BRIGHT);
  }
  LEDControl.write();
  Serial.println("PWM values:");
  for (int i = 0; i < NUM_LEDS; i++) {
    Serial.println(LEDControl.getPWM(i));
  }
  delay(1000);
  Serial.println("Turning the lights off");
  // turn them all back off
  for (int i = 0; i < NUM_LEDS; i++) {
    LEDControl.setPWM(i, 0);
  }
  LEDControl.write();
}

/*
Main loop:
primary modes:
  manual
    manual driving
  autonomous
    autonomous driving
  standby
    pick mode / routine to run

background tasks:
  video
  audio
  misc
  color sensor (if in autonomous mode)

*/
void loop()
{
  // Make sure the PS3 Controller is working - skip main loop if not
  if (!readUSB())
  {
    return;
  }
  if (clear_screen)
  {
    
    // clear the screen
    if (millis() - last_clear_trigger > CLEAR_REFRESH)
    {
      last_clear_trigger = millis();
      tft.fillScreen(ST77XX_BLACK);
      clear_screen = false;
    }
  }
  // If the PS3 controller has been connected - start processing the main controller routines
  if (PS3Controller->PS3Connected)
  {
    if (millis() - last_controller_refresh > CONTROLLER_REFRESH) {
      // Read the PS3 Controller and set request state variables for this loop
      last_controller_refresh = millis();
      readPS3Request();
      if (control_mode == 0)
      {
        // inputs in control_mode 0
        if (reqCircle)
        {
          digitalWrite(LED, LOW);
          control_mode = 1;
        }
        if (reqTriangle)
        {
          control_mode = 2;
          digitalWrite(LED, HIGH);
        }
        if (reqCross) {
          // start the selected ambient routine
          Serial.println("Switching to control mode 3");
          control_mode = 3;
        }
        if (reqArrowUp) {
          // go up one routine
        }
        if (reqArrowDown) {
          // go down one routine
        }
      }
      else if (control_mode == 1)
      {
        // inputs in control_mode 1
        if (reqCross)
        {
          turn_direction *= -1;
        }
        if (reqCircle)
        {
          control_mode = 1;
          stop();
        }
      }
      else if (control_mode == 2)
      {
        // inputs in control_mode 2
        if (reqTriangle)
        {
          stop();
          // trigger celebration sound
          mp3trigger.trigger(CELEBRATION);
          control_mode = 0;
        }
      }
    }
    // run tasks based on the mode we are in
    // control mode stuff
    // do things based on what mode we are in
    if (control_mode == 1)
    {
      if (millis() - last_manual_trigger > MANUAL_MODE_REFRESH) {
        last_manual_trigger = millis();
        moveDroidManual();
        motor_mode = -1;
      }
    } else if (control_mode == 2)
    {
      // Serial.println("running autonomous");
      if (millis() - last_autonomous_trigger > AUTO_MODE_REFRESH)
      {
        last_autonomous_trigger = millis();
        autonomousDriving();
        motor_mode = -1;
      }
    } else if (control_mode == 0)
    {
      // TODO: make sure all of the frame counters are reset
      sequence_location = 0;
      num_dots = 0;
      if (millis() - last_control_mode_trigger > CONTROL_MODE_REFRESH) {
        last_control_mode_trigger = millis();
        if (currentTurn != 0 || currentSpeed != 0)
        {
          Serial.println("stopping the droid");
          currentTurn = 0;
          currentSpeed = 0;
          ST->stop();
        }
        // TODO: decide if there is going to be a special idling thing
      }
    } else if (control_mode == 3) {
      // custom routine 1
      if (millis() - last_control_mode_trigger > custom_routine_refresh) {
        last_control_mode_trigger = millis();
        // make the custom_routine_refresh random
        custom_routine_refresh = random(CUSTOM_ROUTINE_MIN, CUSTOM_ROUTINE_MAX); // TODO: make this random
        custom_routine_1();
        if (go_to_next_sequence_location) {
          go_to_next_sequence_location = false;
          sequence_location += 1;
          Serial.print("sequence location: ");
          Serial.println(sequence_location);
        }
        if (sequence_location > SEQUENCE_1_NUM_LOCATIONS) {
          // go back to the normal mode
          control_mode = 0;
          Serial.println("going back to the normal mode");
        }

      }
    }
    
    if (millis() - last_motor_mode_refresh > MOTOR_MODE_REFRESH) {
      last_motor_mode_refresh = millis();
      if (motor_mode == 0) {
        ST->stop();
      } else if (motor_mode == 1) {
        // twitchy
        motor_twitch();
      } else if (motor_mode == 2) {
        // something else
      } else if (motor_mode == 3) {
        // a third thing ig
      } else {
        // designed for autonomous and manual modes, this will just do nothing
      }

    }
    if (millis() - last_audio_mode_refresh > AUDIO_MODE_REFRESH)
    {
      // audio mode stuff
      if (audio_mode == 0)
      {
        // make sure we are quiet
        mp3trigger.trigger(0);
      }
      else if (audio_mode == 1)
      {
        // ambient sound
        ambient_sound();
      }
      else if (audio_mode == 2)
      {
        // jukebox mode
        //juke_box();
      } else if (audio_mode == 3) {
        // custom routines
        // TODO; make function for this
        custom_routine_sound();
      }
    }

    if (millis() - last_video_mode_refresh > VIDEO_MODE_REFRESH)
    {
      // video mode stuff
    }

    if (millis() - last_led_mode_refresh > LED_MODE_REFRESH) {
      last_led_mode_refresh = millis();
      if (led_mode == 0) {
        // do nothing
      } else if (led_mode == 1) {
        // blink random
        led_random_blink();
      } else if (led_mode == 2) {
        // TMP turn off and go to mode 0
        led_all_off();
        led_mode = 0;
      }
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
    if (reqMade)
    {
      resetRequestVariables();
      reqMade = false;
    }
  }

  // Blink to show working heart beat on the Arduino control board
  // If Arduino LED is not blinking - the sketch has crashed
  if ((blinkMillis + 500) < millis())
  {
    if (blinkOn)
    {
      digitalWrite(13, LOW);
      blinkOn = false;
    }
    else
    {
      digitalWrite(13, HIGH);
      blinkOn = true;
    }
    blinkMillis = millis();
  }
  // update mp3trigger
  mp3trigger.update();
  // update LEDs
  LEDControl.write();
}

// =======================================================================================
//      ADD YOUR CUSTOM DROID FUNCTIONS STARTING HERE
// =======================================================================================
/*
Routine 1:
the thing comes to life

Scrolling "boot" text to start then gets corrupted

remember, you can use setTextColor(foreground, background) to clear the area behind the text

NOTE: these will have to be split into several subroutines that run one at a time so the droid
is still responsive during the routine
*/
void custom_routine_1() {
  audio_mode = 3;
  // start the boot
  // might not worry about scrolling text for now - it would be cool but its a lot of attention
  // for something that is very hard to see at the end of the day
  if (sequence_location == 1) {
    tft.setCursor(0, 0);
    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    tft.setTextSize(1);
    tft.println("Beginning boot sequence...");
    // TODO: sound "booting..."
    
    custom_routine_sound_track = 1;
    custom_rotuine_start_new_sound = true;
    go_to_next_sequence_location = true;
  } else if (sequence_location == 2) {
    tft.print("Checking peripherals");
    go_to_next_sequence_location = true;
  } else if (sequence_location == 3) {
    boot_dots();
  } else if (sequence_location == 4) {
    tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
    tft.println("DONE");
    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    go_to_next_sequence_location = true;
  } else if (sequence_location == 5) {
    tft.println("importing external libraries:");
    go_to_next_sequence_location = true;
  } else if (sequence_location == 6) {
    tft.println("motor control - IMPORTED");
    go_to_next_sequence_location = true;
  } else if (sequence_location == 7) {
    tft.println("power management - IMPORTED");

    go_to_next_sequence_location = true;
  } else if (sequence_location == 8) {
    tft.println("importing LED controller");
    go_to_next_sequence_location = true;
  } else if (sequence_location == 9) {
    // test LEDs
    test_leds();
  } else if (sequence_location == 10) {
    tft.print("LED setup ");
    tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
    tft.println("COMPLETE");
    tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
    go_to_next_sequence_location = true;
  } else if (sequence_location == 11) {
    tft.println("attempting to import soul");
    // TODO: sound "attempting to import soul"
    go_to_next_sequence_location = true;
  } else if (sequence_location == 12) {
    boot_dots();
  } else if (sequence_location == 13) {
    // maybe make this text bigger/overwrite the entire screen
    tft.setTextColor(ST7735_RED, ST7735_BLACK);

    tft.println("FAILED");
    tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
    led_mode = 1;
    // sound "FAILED"

    go_to_next_sequence_location = true;
  } else if (sequence_location == 14) {
    // attempting import a second time
    tft.println("retrying");
    go_to_next_sequence_location = true;
  } else if (sequence_location == 15) {
    boot_dots();
  } else if (sequence_location == 16) {
    // corruption begins to occur but isnt super serious
    // bigger text
    tft.setTextSize(3);
    tft.setCursor(50, 50);
    tft.setTextColor(ST7735_RED);

    tft.println("FAILED");    
    // sound "FAILED"
    go_to_next_sequence_location = true;
  } else if (sequence_location == 17) {
    // corruption animation
    corruption_animation();
  } else if (sequence_location == 18) {
    tft.setTextSize(2);
    tft.setCursor(20, 20);
    tft.setTextColor(ST7735_WHITE);
    tft.println("CRITICAL ERROR ENCOUNTERED");
    motor_mode = 1;
    tft.setCursor(0, 0);
    tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
    tft.println("EXITING");
    // and then it dips
    custom_routine_sound_track = 2;
    custom_rotuine_start_new_sound = true;
    
    // sound "CRITICAL ERROR ENCOUNTERED"
    // corruption complete
    // this should include motors twitching, lights flashing and the tft spazing out
    go_to_next_sequence_location = true;
  } else if (sequence_location == 19) {
    // uhh something, this is probably the end
    // sound "the thing"
    motor_mode = 0;
    led_mode = 2;
    audio_mode = 0;
    go_to_next_sequence_location = true;
    tft.fillScreen(ST7735_BLACK);
  }
}
void boot_dots() {
  if (millis() - last_boot_dot_trigger > boot_dot_refresh) {
    // TODO: add "beep"
    num_dots += 1;
    tft.print(".");
    boot_dot_refresh = random(DOT_MIN_REFRESH, DOT_MAX_REFRESH);
    if (num_dots >= 3) {
      go_to_next_sequence_location = true;
      // reset the function
      num_dots = 0;
    }
  }
}
void test_leds() {
  // turn the previous one off
  if (current_led > 0) {
    LEDControl.setPWM(current_led - 1, 0);
  }
  // TODO: add a tick noise when the led turns on
  LEDControl.setPWM(current_led, LED_MAX_BRIGHT);

  current_led += 1;
  if (current_led > NUM_LEDS) {
    go_to_next_sequence_location = true;
  }
}
void led_random_blink() {
  int rled = random(0, NUM_LEDS);
  if (LEDControl.getPWM(rled) > 0) {
    // turn the led off because it is already on
    LEDControl.setPWM(rled, 0);
  } else {
    LEDControl.setPWM(rled, LED_MAX_BRIGHT);
  }
}
void led_all_off() {
  for (int i = 0; i < NUM_LEDS; i++) {
    LEDControl.setPWM(i, 0);
  }
}
/*
TODO: decide how you want to store/render the animation

could make the animation then use a python script to convert it to a string
then have the animation script write the next pixel
it would probably only be a couple frames anyways so it should be fine
*/
void corruption_animation() {
  // TODO
  for (int16_t x = 0; x < tft.width(); x += 6)
  {
    tft.drawLine(0, 0, x, tft.height() - 1, ST7735_RED);
    delay(0);
  }
  for (int16_t x = 0; x < tft.width(); x += 6)
  {
    tft.drawLine(x, 0, x, tft.height() - 1, ST7735_RED);
    delay(0);
  }
  go_to_next_sequence_location = true;
}
void corruption_complete() {
  // this might be better as its own sequence

  /*
  Things that should be happening:
  LEDs going crazy
  Wheels twitching
  servo doing *something*
  maybe hook up some vibrational motors to the panels so the panels shake

  */
}
void motor_twitch() {
  int invert_twitch = MAX_TWITCH * -1;
  int invert_twitch_turn = MAX_TWITCH_TURN * -1;
  int drive = random(invert_twitch, MAX_TWITCH);
  int rot = random(invert_twitch_turn, MAX_TWITCH_TURN);
  // make sure we are above the minimum
  if (drive > 0 && drive < MIN_TWITCH) {
    drive = MIN_TWITCH;
  }
  if (drive < 0 && drive > MIN_TWITCH * -1) {
    drive = -1 * MIN_TWITCH;
  }
  if (rot > 0 && rot < MIN_TWITCH_TURN) {
    rot = MIN_TWITCH_TURN;
  }
  if (rot < 0 && rot > MIN_TWITCH_TURN * -1) {
    rot = -1 * MIN_TWITCH_TURN;
  }
  currentTurn = rot;
  currentSpeed = drive;
  ST->turn(currentTurn);
  ST->drive(currentSpeed);
}

/*
Routine 2:
the thing tries to eat you

*/
void custom_routine_2() {


}

/*
Routine 3:
the thing dies a fiery death

*/
void custom_routine_3() {

}

/*
TFT board

*/
void popupWindow(String text) {
  /*
  Ideas for the popup window: just clear a portion of the screen

  Problems:
  if this happens during an animation the screen would get cleared anyways

  


  */
}
void testlines(uint16_t color)
{
  tft.fillScreen(ST77XX_BLACK);
  for (int16_t x = 0; x < tft.width(); x += 6)
  {
    tft.drawLine(0, 0, x, tft.height() - 1, color);
    delay(0);
  }
  for (int16_t y = 0; y < tft.height(); y += 6)
  {
    tft.drawLine(0, 0, tft.width() - 1, y, color);
    delay(0);
  }

  tft.fillScreen(ST77XX_BLACK);
  for (int16_t x = 0; x < tft.width(); x += 6)
  {
    tft.drawLine(tft.width() - 1, 0, x, tft.height() - 1, color);
    delay(0);
  }
  for (int16_t y = 0; y < tft.height(); y += 6)
  {
    tft.drawLine(tft.width() - 1, 0, 0, y, color);
    delay(0);
  }

  tft.fillScreen(ST77XX_BLACK);
  for (int16_t x = 0; x < tft.width(); x += 6)
  {
    tft.drawLine(0, tft.height() - 1, x, 0, color);
    delay(0);
  }
  for (int16_t y = 0; y < tft.height(); y += 6)
  {
    tft.drawLine(0, tft.height() - 1, tft.width() - 1, y, color);
    delay(0);
  }

  tft.fillScreen(ST77XX_BLACK);
  for (int16_t x = 0; x < tft.width(); x += 6)
  {
    tft.drawLine(tft.width() - 1, tft.height() - 1, x, 0, color);
    delay(0);
  }
  for (int16_t y = 0; y < tft.height(); y += 6)
  {
    tft.drawLine(tft.width() - 1, tft.height() - 1, 0, y, color);
    delay(0);
  }
}

void testdrawtext(char *text, uint16_t color)
{
  tft.setCursor(0, 0);
  tft.setTextColor(color);
  tft.setTextWrap(true);
  tft.print(text);
}

void testfastlines(uint16_t color1, uint16_t color2)
{
  tft.fillScreen(ST77XX_BLACK);
  for (int16_t y = 0; y < tft.height(); y += 5)
  {
    tft.drawFastHLine(0, y, tft.width(), color1);
  }
  for (int16_t x = 0; x < tft.width(); x += 5)
  {
    tft.drawFastVLine(x, 0, tft.height(), color2);
  }
}

// sound

void ambient_sound()
{
  if (millis() - last_ambient_sound > AMBIENT_SOUND_REFRESH) {
    int raw_sound = random(0, LAST_AMBIENT_TRACK + 1);
    if (raw_sound < FIRST_AMBIENT_TRACK) {
      // give the option for the ambient sound to be nothing
      raw_sound = 0;
    }

    mp3trigger.trigger(raw_sound);
    // TODO: might not want to track ambient sounds so you can interupt them
    last_audio_trigger = millis(); 
    last_ambient_sound = millis();
  }
}
void custom_routine_sound() {
  if (custom_rotuine_start_new_sound) {
    custom_rotuine_start_new_sound = false;
    mp3trigger.trigger(custom_routine_sound_track);
    last_audio_trigger = millis();
  }
}

// movement

void stop()
{
  ST->turn(0);
  ST->drive(0);
  droidMoving = false;
}

int distance(int a, int b)
{
  int c = a - b;
  if (b < 0 && a > 0)
  {
    c = b - a;
  }
  return abs(c);
}

void moveDroidManual()
{
  if (reqLeftJoyMade)
  {
    // remap controller value to a custom max value
    int desiredSpeed = (reqLeftJoyYValue / MAX_CONTROLLER) * MAX_SPEED * turn_direction * -1;
    int desiredTurn = (reqLeftJoyXValue / MAX_CONTROLLER) * MAX_TURN * turn_direction; // invert turn so we turn the way our joystick moves
    // need both of these values (turn, drive) for the motorcontroller

    Serial.println(distance(currentTurn, desiredTurn));
    if (distance(currentSpeed, desiredSpeed) < 10)
    {
      currentSpeed = desiredSpeed;
    }
    else
    {
      if (desiredSpeed > currentSpeed)
      {
        // accelerating
        currentSpeed += distance(desiredSpeed, currentSpeed) / CONTROLLER_RAMP;
      }
      else
      {
        // decelrating
        currentSpeed -= distance(desiredSpeed, currentSpeed) / CONTROLLER_RAMP;
      }
    }
    if (distance(currentTurn, desiredTurn) < 10)
    {
      currentTurn = desiredTurn;
    }
    else
    {
      if (desiredTurn > currentTurn)
      {
        currentTurn += distance(desiredTurn, currentTurn) / CONTROLLER_RAMP;
      }
      else
      {
        currentTurn -= distance(desiredTurn, currentTurn) / CONTROLLER_RAMP;
      }
    }
    // move!
    ST->turn(currentTurn);
    ST->drive(currentSpeed);
    if (!droidMoving)
    {
      droidMoving = true;
    }
  }
  else
  {
    if (droidMoving)
    {
      if (currentSpeed == 0 && currentTurn == 0)
      {
        ST->stop();
        droidMoving = false;
      }
      else
      {
        currentSpeed = currentSpeed / 4;
        currentTurn = currentTurn / 4;
        if (distance(0, currentSpeed) < 5)
        {
          currentSpeed = 0;
        }
        if (distance(0, currentTurn) < 5)
        {
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
void autoDrive()
{
  droidMoving = true;
  if (action == 1)
  {
    if (currentColor == 1)
    {
      // trigger sound blue to color 1
      mp3trigger.trigger(BLUE_TO_RED);
      tft.fillScreen(ST77XX_RED);
      action = 2;
      Serial.println("turning in the positive direction");
    }
    else if (currentColor == 3)
    {
      // trigger sound blue to color 2
      mp3trigger.trigger(BLUE_TO_GREEN);
      tft.fillScreen(ST77XX_GREEN);

      action = 3;
      Serial.println("turning in the negative direction");
    }
  }
  else if (action == 2)
  {
    // turn right
    currentSpeed = 0;
    currentTurn = AUTO_SPEED * -1;
    if (currentColor == 2)
    {
      // trigger color 1 to blue
      mp3trigger.trigger(BACK_TO_BLUE);
      tft.fillScreen(ST77XX_BLUE);

    }
  }
  else if (action == 3)
  {
    // turn left
    currentSpeed = 0;
    currentTurn = AUTO_SPEED;
    if (currentColor == 2)
    {
      // trigger color 2 to blue
      mp3trigger.trigger(BACK_TO_BLUE);
      tft.fillScreen(ST77XX_BLUE);

    }
  }
  if (currentColor == 2)
  {
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
void readRedValue()
{
  int low_bound = 46;
  int high_bound = 129;
  digitalWrite(s2, LOW);
  digitalWrite(s3, LOW);
  data = pulseIn(out, LOW);

  rValue = data; // map(data, low_bound, high_bound, 255, 0);
}
void readBlueValue()
{ // this is consistently detecting blue
  int low_bound = 51;
  int high_bound = 129;

  digitalWrite(s2, LOW);
  digitalWrite(s3, HIGH);
  data = pulseIn(out, LOW);

  bValue = data; // map(data, low_bound, high_bound, 255, 0);
}
void readGreenValue()
{ // this is consistently detecting green
  int low_bound = 51;
  int high_bound = 129;
  digitalWrite(s2, HIGH);
  digitalWrite(s3, HIGH);
  data = pulseIn(out, LOW);

  gValue = data; // map(data, low_bound, high_bound, 255, 0);
}

void readColor()
{
  /*
  read each r,g,b value one at a time, if we have looped back around - set the current color

  from early tests, it looks like the lowest raw value is the dominant color
  */
  int oldColor = currentColor;
  if (currentRead == 1)
  {
    readRedValue();
    currentRead = 2;
  }
  else if (currentRead == 2)
  {
    readGreenValue();
    currentRead = 3;
  }
  else if (currentRead == 3)
  {
    readBlueValue();
    currentRead = 1;
  }
  if (currentRead == 1)
  {
    // default color is red even though this might not be the case
    // TODO: add detection for colors that mostly white (the floor)
    int currentLowest = rValue;
    currentColor = 1;
    if (bValue < currentLowest)
    {
      currentLowest = bValue;
      currentColor = 2;
    }
    if (gValue < currentLowest)
    {
      currentLowest = gValue;
      currentColor = 3;
    }
    if (distance(gValue, bValue) < COLOR_RANGE && distance(gValue, rValue) < COLOR_RANGE && distance(bValue, rValue) < COLOR_RANGE)
    {
      Serial.print("unknown color: ");
      Serial.print(rValue);
      Serial.print(", ");
      Serial.print(gValue);
      Serial.print(", ");
      Serial.println(bValue);
    }
    // TODO: if the colors are similar to each other, say the color is invalid
  }
  if (oldColor != currentColor)
  {
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
void autonomousDriving()
{
  // if STOP: return
  // if green turn left
  // if red turn right
  // if blue go straight
  readColor();
  // going to need to do a check for "real" color or we can just set the current color to invalid (-1)
  if (currentColor != -1)
  {
    autoDrive();
  }
  else
  {
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
  if (!extraRequestInputs)
  {

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

  if (((abs(PS3Controller->getAnalogHat(LeftHatY) - 128) > joystickDeadZoneRange) || (abs(PS3Controller->getAnalogHat(LeftHatX) - 128) > joystickDeadZoneRange)))
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

    if (currentValueY > joystickDeadZoneRange)
    {
      Serial.println("Left Joystick DOWN");
      reqLeftJoyDown = true;
      reqLeftJoyYValue = currentValueY;
    }

    if (currentValueY < (-1 * joystickDeadZoneRange))
    {
      Serial.println("Left Joystick UP");
      reqLeftJoyUp = true;
      reqLeftJoyYValue = currentValueY;
    }

    if (currentValueX > joystickDeadZoneRange)
    {
      Serial.println("Left Joystick RIGHT");
      reqLeftJoyRight = true;
      reqLeftJoyXValue = currentValueX;
    }

    if (currentValueX < (-1 * joystickDeadZoneRange))
    {
      Serial.println("Left Joystick LEFT");
      reqLeftJoyLeft = true;
      reqLeftJoyXValue = currentValueX;
    }
  }
  else
  {
    if (reqLeftJoyMade)
    {
      reqLeftJoyUp = false;
      reqLeftJoyDown = false;
      reqLeftJoyLeft = false;
      reqLeftJoyRight = false;
      reqLeftJoyYValue = 0;
      reqLeftJoyXValue = 0;
      reqLeftJoyMade = false;
    }
  }

  if (((abs(PS3Controller->getAnalogHat(RightHatY) - 128) > joystickDeadZoneRange) || (abs(PS3Controller->getAnalogHat(RightHatX) - 128) > joystickDeadZoneRange)))
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

    if (currentValueY > joystickDeadZoneRange)
    {
      Serial.println("Right Joystick DOWN");
      reqRightJoyDown = true;
      reqRightJoyYValue = currentValueY;
    }

    if (currentValueY < (-1 * joystickDeadZoneRange))
    {
      Serial.println("Right Joystick UP");
      reqRightJoyUp = true;
      reqRightJoyYValue = currentValueY;
    }

    if (currentValueX > joystickDeadZoneRange)
    {
      Serial.println("Right Joystick RIGHT");
      reqRightJoyRight = true;
      reqRightJoyXValue = currentValueX;
    }

    if (currentValueX < (-1 * joystickDeadZoneRange))
    {
      Serial.println("Right Joystick LEFT");
      reqRightJoyLeft = true;
      reqRightJoyXValue = currentValueX;
    }
  }
  else
  {
    if (reqRightJoyMade)
    {
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

    if (currentTime >= lastMsgTime)
    {
      msgLagTime = currentTime - lastMsgTime;
    }
    else
    {
      msgLagTime = 0;
    }

    if (msgLagTime > 5000)
    {
      Serial.println("It has been 5s since we heard from Controller");
      Serial.println("Disconnecting the controller");

      PS3Controller->disconnect();
      WaitingforReconnect = true;
      return true;
    }

    // Check PS3 Signal Data
    if (!PS3Controller->getStatus(Plugged) && !PS3Controller->getStatus(Unplugged))
    {
      // We don't have good data from the controller.
      // Wait 15ms - try again
      delay(15);
      Usb.Task();
      lastMsgTime = PS3Controller->getLastMessageTime();

      if (!PS3Controller->getStatus(Plugged) && !PS3Controller->getStatus(Unplugged))
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

    if (badPS3Data > 10)
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
  // The more devices we have connected to the USB or BlueTooth, the more often Usb.Task need to be called to eliminate latency.
  if (PS3Controller->PS3Connected)
  {
    if (criticalFaultDetect())
    {
      // We have a fault condition that we want to ensure that we do NOT process any controller data!!!
      return false;
    }
  }
  return true;
}
