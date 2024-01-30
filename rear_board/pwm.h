#include <PWM.h> //PWM librarey for controlling freq. of PWM signal
#define signal_pin  9
//Set duty cycle to 50% by default -> for 16-bit 65536/2 = 32768
int32_t frequency =1000;
int32_t duty_cycle;




void  PWM_Setup(){
InitTimersSafe(); //Initialize timers without disturbing timer 0
pinMode(signal_pin,OUTPUT);
}


void loop(){

    // 0 --> 65536

pwmWriteHR(signal_pin, constrain(duty_cycle,0,65530));
SetPinFrequencySafe(signal_pin, frequency);


}
