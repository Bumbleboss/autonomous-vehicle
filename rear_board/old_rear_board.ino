#include <SPI.h>
#include <mcp2515.h>
#include <PWM.h> //PWM librarey for controlling freq. of PWM signal
//body
#define Left_Pin 16
#define Right_Pin 19
#define Brakes_Pin 17
                                  #ifndef UINT16_MAX
                                  #define UINT16_MAX 65535
                                  #endif
                                  #ifndef UINT8_MAX
                                  #define UINT8_MAX 255
                                  #endif
                                  #define counterpin 3 // check the pins
                                  

                  //COUNTER
                  const int int0Pin = counterpin; // INT1 pin on Arduino 
                  // Counter variable
                  volatile int CurrentPulseCount = 0;
                  // Interrupt Service Routine (ISR) for INT0
                  void int0ISR() {
                    CurrentPulseCount++;
                    if (CurrentPulseCount >= 65000)
                    {
                      CurrentPulseCount = 0;
                    }
                  }

long sentt;

struct can_frame canMsg1;
struct can_frame canMsg2;

MCP2515 mcp2515(10);

long flag = 0;
          int decimalToBinary(int decimal_num) {
                  int binary_num = 0, i = 1, remainder;
              
                  while (decimal_num != 0) {
                      remainder = decimal_num % 2;
                      decimal_num /= 2;
                      binary_num += remainder * i;
                      i *= 10;
                  }
              
                  return binary_num;
              }

long TH_PWM;
bool Left_Out,Right_Out,Brakes_Out;
long throttle_old;
void throttle_send(int value)
{
  if (throttle_old > value)
  {
    for (int loopp = throttle_old; loopp > value; loopp--)
    {

      if (value >= 0 && value <= 255)
      {
        sentt=constrain(map(loopp,0,255,0,65535),0,65535);
        pwmWriteHR(9,sentt );
        SetPinFrequencySafe(9, 1000);
      }
    }

  } else
  {
    for (int loopp = throttle_old; loopp < value; loopp++)
    {

      if (value >= 0 && value <= 255)
      {
        sentt=constrain(map(loopp,0,255,0,65535),0,65535);
        pwmWriteHR(9,sentt );
        SetPinFrequencySafe(9, 1000);
      }
    }

  }
}

void setup() {
                                canMsg2.can_id  = 0x02;
                                canMsg2.can_dlc = 2;
                                canMsg2.data[0]=0x00;
                                canMsg2.data[1]=0x00;

  
  Serial.begin(115200);
  
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS);
  mcp2515.setNormalMode();
  
  Serial.println("------- CAN Read ----------");
  Serial.println("ID  DLC   DATA");
  InitTimersSafe(); //Initialize timers without disturbing timer 0
  pinMode(9,OUTPUT);
  
  pinMode(Left_Pin,OUTPUT);
  pinMode(Right_Pin,OUTPUT);
  pinMode(Brakes_Pin,OUTPUT);



                                      //counter
                                    // Configure INT0 pin as input
                                    pinMode(int0Pin, INPUT);
                                    // Enable external interrupt INT0
                                    attachInterrupt(digitalPinToInterrupt(int0Pin), int0ISR, RISING);

}


void loop() {

  if (mcp2515.readMessage(&canMsg1) == MCP2515::ERROR_OK) {

      Serial.print(canMsg1.data[0]);
      TH_PWM=canMsg1.data[0];
      
      Serial.print("  ");
      Serial.println(TH_PWM);
      
      if(abs(TH_PWM-throttle_old)>1)
      {
        throttle_send(TH_PWM);  
      }else
      {
      sentt=constrain(map(canMsg1.data[0],0,255,0,65535),0,65535);
      
      pwmWriteHR(9,sentt );
      SetPinFrequencySafe(9, 1000);
      }
      throttle_old=TH_PWM;
        

      Left_Out = bitRead(canMsg1.data[1],0);
      Right_Out = bitRead(canMsg1.data[1],1);
      Brakes_Out = bitRead(canMsg1.data[1],2);

       digitalWrite(Left_Pin,Left_Out);
       //digitalWrite(Left_Pin,HIGH);
       //digitalWrite(Right_Pin,HIGH);     
       digitalWrite(Right_Pin,Right_Out);
       digitalWrite(Brakes_Pin,!Brakes_Out);
                                Serial.println(CurrentPulseCount);

  }
                                canMsg2.data[0] = (CurrentPulseCount >> 0) & 0xFF;
                                canMsg2.data[1] = (CurrentPulseCount >> 8) & 0xFF;
                                mcp2515.sendMessage(&canMsg2);
}
