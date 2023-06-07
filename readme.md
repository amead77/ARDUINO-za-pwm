# Motor PWM Controller for a small GR-18 ZA-EC Blue fan. (Z size, 240v)

##                        *** WARNING *** 
##        as of 2023-06-07 it is currently untested on a fan.
##    Everything works on the bench, but I haven't tested it on a fan yet.
##            Or even built a non-breadboard version.
##                        *** WARNING ***


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
- [ ] test on a fan
- [ ] build a non-breadboard version
- [ ] tidy up the code
- [ ] set the encoder button to set speed to zero if pressed
- [ ] read motor encoder pulse and display rpm (literally forgot this until now)
- [ ] only if issues - switch from loop() to interrupts


REALITY:
- I'll probably never get around to doing any of the above, except testing on a fan, thereby proving it works.
- it'll make it to my "projects" box and never be seen again.
- If it doesn't work I'll probably still not do the TODOs, but I will make it work.
