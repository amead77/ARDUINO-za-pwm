/*
Motor PWM Controller for a small GR-18 ZA-EC Blue fan. (Z size, 240v)

                        *** WARNING *** 
        as of 2023-06-07 it is currently untested on a fan.
    Everything works on the bench, but I haven't tested it on a fan yet.
            Or even built a non-breadboard version.
                        *** WARNING ***

the fan uses 10v on the internal pwm controller, I'm going to use 5v on the arduino pwm input in the hopes it'll
still read it on the pwm input.
I will need a 10v power supply for the fan pwm controller, and drop it to 5v for the arduino, while linking the grounds.

uses a 20x4 rgb lcd (rgb not used) with attached LCD backpack, and a rotary encoder with pushbutton (button not used).
purpose is to create a pwm output for a motor controller and to display value to lcd.
I may change the LCD for a 16x2, but I had a 20x4 lying around. Or maybe just use LEDS for the speed indicator.

this code was from another project of mine (FS game controller) with multiple encoders, buttons and axis, i've just chopped it and 
mangled it to fit this project.
after spending what seemed like forever chopping and mangling, I realised starting from scratch would have been quicker.

I was going to write the last running speed to eeprom, but decided that's a bit dangerous, so it's not implemented.
Instead the fan should start at 25%.

TODO:
    - test on a fan
    - build a non-breadboard version
    - tidy up the code
    - set the encoder button to set speed to zero if pressed
    - read motor encoder pulse and display rpm (literally forgot this until now)
    - only if issues - switch from loop() to interrupts

REALITY:
    - I'll probably never get around to doing any of the above, except testing on a fan, thereby proving it works.
    - it'll make it to my "projects" box and never be seen again.
    - If it doesn't work I'll probably still not do the TODOs, but I will make it work.
*/

// include the adafruit lib for the LCD backpack
#include "Adafruit_LiquidCrystal.h"

 // Rotary Encoder Inputs
#define enc1CLK 4
#define enc1DT 5
#define enc1button 6
#define EncoderMinVal 0
#define EncoderMaxVal 100
#define EncoderStartVal 25 //start the encoder at this value which is 25% duty cycle for the fan on startup (mapped 0..255)
#define PWMpin A0

int xMap = 0;

int counter1 = EncoderStartVal; 

int counter1prev = 0;

int currentStateEnc1CLK;
int previousStateEnc1CLK; 
int input_debounce;
int LCDupdate = 999; //start with a value that will force an update
bool encChanged = true; //force an update on startup

// Connect via i2c, default address #0 (A0-A2 not jumpered)
Adafruit_LiquidCrystal lcd(0);



void setup() {
    Serial.begin(115200);
    Serial.println("LCD Character Backpack I2C Test.");

    // set up the LCD's number of rows and columns:
    if (!lcd.begin(20, 4)) {
        Serial.println("Could not init backpack. Check wiring.");
        while(1); //go into infinite loop to stop the program
    }
    //got this far means backpack works
    lcd.setBacklight(HIGH);
    Serial.println("Backpack init'd.");

    // Print first message to the LCD.
    lcd.print("ZA-PWM Controller r8");

    // Set encoder pins as inputs  
    pinMode (enc1CLK,INPUT);
    pinMode (enc1DT,INPUT);
    pinMode(enc1button, INPUT);

    //set the PWM pin as an output
    pinMode(PWMpin, OUTPUT);

    // Read the initial state of enc1CLK and Assign to previousStateCLK variable
    previousStateEnc1CLK = digitalRead(enc1CLK);

    //no real reason for the delay, but it feels right
	delay(100);
}


bool readEncoderPos() {
    // Read the current state of enc1CLK
    bool result = false;
    currentStateEnc1CLK = digitalRead(enc1CLK);
    // If the previous and the current state of the enc1CLK are different then a pulse has occurred
    if (currentStateEnc1CLK != previousStateEnc1CLK){ 
        result = true;
        // If the enc1DT state is different than the enc1CLK state then the encoder is rotating counterclockwise
        if (digitalRead(enc1DT) != currentStateEnc1CLK) { 
            counter1 ++;
			if (counter1 > EncoderMaxVal) {
				counter1 = EncoderMaxVal;
			}
		} else {
            counter1 --;
 			if (counter1 < EncoderMinVal) {
				counter1 = EncoderMinVal;
			}       
        }
		//Serial.print("(1) Direction: ");
		//Serial.print(" -- Value: ");
		//Serial.println(counter1);
    }

    previousStateEnc1CLK = currentStateEnc1CLK; 
	if (counter1prev != counter1) {
		counter1prev = counter1;
	}
    return result;
}

void updateLCD() {

    //map counter1 (0..255) to 0..100 value in xMap
    xMap = map(counter1, 0, 100, 0, 255);
//    String counter1str = "xMap: "+String(xMap)+" counter1: "+String(counter1)+"%  ";
//    Serial.print("xMap: ");
//    Serial.println(xMap);
//    Serial.print("counter1: ");
//    Serial.println(counter1);
    
    lcd.setCursor(0, 1);
    lcd.print("SPEED: ");
    lcd.setCursor(8, 1);
    String counter1str = String(counter1)+"%  ";
    lcd.print(counter1str);

}

void loop() {
    //lcd.setBacklight(HIGH);
    //delay(500);
    //lcd.setBacklight(LOW);
    //delay(500);

    //no delay = some encoder read errors due to bounce, delay of 50 = lots of errors due to slow read interval
    //everything in loop() must be fast, or encoder read errors occur
    delay(1);

    LCDupdate++;
    //when the counter reaches 1000, reset the counter and update the lcd if the encoder has changed
    //this is to prevent blocking the loop() with lcd updates (1000x 1ms delay = 1 second LCD updates if req.)
    if (LCDupdate == 1000) {
        if (encChanged) {
            updateLCD();
            encChanged = false;
            analogWrite(PWMpin, xMap);
            Serial.println("PWM changed to "+String(xMap));
        }
        LCDupdate = 0;
        //Serial.println("LCDupdate reset");
    }

    //read the encoder and set encChanged to true if it has changed, this will be reset in if/LCDupdate above
    if (readEncoderPos()) {
        encChanged = true;
        //Serial.println("readEncoderPos() = true");
    }
}
