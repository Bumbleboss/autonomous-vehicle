#include <SPI.h>
#include <math.h>  
#include <mcp2515.h>
#include <Wire.h>
                          #include <Servo.h> // esc
                          #include <PWM.h> // throttle
                          #include <NewPing.h> // ultrasonic
                          #ifndef UINT16_MAX
                          #define UINT16_MAX 65535
                          #endif
                          #ifndef UINT8_MAX
                          #define UINT8_MAX 255
                          #endif

unsigned long previousMillisss = 0;
unsigned long intervalll = 2000; // 2 seconds
bool brakebool=LOW;
unsigned long currentMillisss;
struct can_frame canMsg1;
                        struct can_frame canMsg2;

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

MCP2515 mcp2515(4);

//IO_Pins
#define Blinking_LED 0
#define CAN_INT 2
#define CAN_CS 4
#define INT0 10
#define EN_R 11
#define ESC_PIN 12
#define TRIGGER_PIN 13
#define ECHO_PIN 14
#define AEB_LED 15 // j19
#define ACC_LED 18 //j18
#define LKA_LED 20 //// REVERSED
#define ST_LED 19 ////  REVERSED
#define HL_OUT 21
#define Left_OUT 22
#define Right_OUT 23
#define Horn_OUT 24

#define Steering_Angle A7
#define Throttle A6
#define LDR A5
#define ADC4 A4
#define Accident_LED A1
                                      #define MAX_DISTANCE 450 // ultrasonic


// Variables
int throttle_old;

int esc_flag = 0;
                                     volatile int CurrentPulseCount = 0;
                                      int speed_value = 70 ; // value sent to the motor
                                      int accelration = 1 ;
                                      int Deceleration = 0 ;
                                      
                                      int Ultra_distance; // distance infront of the car
                                      int OneMeter_pulses = 52; // the number of pulseCount sent during pass 1 meter 
                                      long sentt; // the values send to the motor 
                                      int x, y; // the values that the user enter to achive the goal 
                                      int Y_Axis_Meters_Used_to_Turn = 2 ; // distance wast in Y axis during turn
                                      int x_Axis_Meters_Used_to_Turn = 2 ; 
                                      int Right_torque = 1600 ; // torque needed to turn right
                                      int left_torque = 1400 ;
                                      int time_for_full_torque = 3000; // time of powerful torque
                                      int little_Right_torque = 1550; // torque needed to remain the steering in full turn
                                      int little_left_torque = 1450; 
                                      int time_for_little_torque = 5500 ; // time of little torque
                                      int Pluses_After_turn ; 
                                      int distance_Travel_along_y;
                                      int distance_Travel_along_x;
                                      int pulses_of_y_axis ;
                                      int pulses_of_x_axis ;
                                      int contsspeed_state_flag =0;
                                      int AUTO_state_flag = 0;
                                      int manual_state_flag = 0;
                                      int brakes_state_flag =0;
                                     int CurrentPulseCount_inbit= 1;

                                     
                                      
long interval = 500;           // Blink interval in milliseconds

unsigned long previousMillis_left = 0;   // Will store the last time LED was updated
unsigned long currentMillis_left = 0 ;

unsigned long previousMillis_Right = 0;   // Will store the last time LED was updated
unsigned long currentMillis_Right = 0 ;

unsigned long previousMillis_both = 0;   // Will store the last time LED was updated
unsigned long currentMillis_both = 0 ;

unsigned long previousMillis_internal = 0;   // Will store the last time LED was updated
unsigned long currentMillis_internal = 0 ;

byte I2C_B1, I2C_B2;
int Throttle_IN;


typedef enum
{
  idle_state = 0,
  left_state = 1,
  right_state = 2,
  hazard_state = 3,
} state;


state current_state;
state previous_state;


                                          typedef enum
                                          {
                                            manual_state = 0,
                                            brakes_state = 1,
                                            AUTO_state = 2,
                                            contsspeed_state = 3,
                                            
                                          } throttle_status;

throttle_status throttle_state;

int uart_intake;
bool serialflag;

bool Wait_flag, Right_flag, Left_flag, HL_flag,ACC_flag,ACC_loop,AEB_flag,AEB_loop,LKA_flag,LKA_loop;;

bool Wait, Right, Left;

bool  Seat_Belt_SW;
bool  Braking_SW;
bool  Accident_SW;
bool  Right_SW;
bool  Left_SW;
bool  Wait_SW;
bool  HL_SW;
bool  Horn_SW;
bool  ST_SW;
bool  LKA_SW;
bool  ACC_SW;
bool  AEB_SW;

unsigned long fb_displacement = 0;
unsigned long fb_speed = 0;

//functions

                           NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

                          // MAIN MOTOR
                          /* 
                           - minimum value is 70 for movement
                           - incrementally increase or decrease by using increase condition
                           - increase == 1
                           - decrease == 0
                           - AND MAKE GOD DAMN SURE THAT YOU SWITCHED THE CABLE TO CONTROLLER INSTEAD OF MANUAL THROTTLE
                          */
                           void throttle_send(int value, int increase) 
                          {
                             if(increase == 1) 
                             {
                               canMsg1.data[0]= value;
                             }
                              else if (increase == 0)
                             {
                               canMsg1.data[0]= 40;
                             }
                          }


                          // ESC
                          Servo servo;
                          // set the steering Direction
                           void steerESC(int Direction) 
                          {
                            if (Direction > 1) // full turn 90 degree 
                             {
                              servo.writeMicroseconds(Right_torque); // powerful turn
                              delay(time_for_full_torque);
                          //    servo.writeMicroseconds(little_Right_torque); 
                               servo.writeMicroseconds(1500);

                               delay(time_for_little_torque);

                              servo.writeMicroseconds(left_torque);
                              delay(time_for_full_torque);
                            //  delay(time_for_little_torque);
                              servo.writeMicroseconds(1500); 
                                                            delay(50);

                              Pluses_After_turn = CurrentPulseCount ; 
                              pulses_of_x_axis = ( distance_Travel_along_x * OneMeter_pulses ) + Pluses_After_turn ; 
                                                                    esc_flag = 1;


                             }
                            else if (Direction < -1) // turn left
                             {
                              servo.writeMicroseconds(left_torque);
                              delay(time_for_full_torque);
//                              servo.writeMicroseconds(little_left_torque);
                               servo.writeMicroseconds(1500);
                              delay(time_for_little_torque);
                              servo.writeMicroseconds(Right_torque); // powerful turn
                              delay(time_for_full_torque);
                              servo.writeMicroseconds(1500);
                                                            delay(10);

                              Pluses_After_turn = CurrentPulseCount ;
                              pulses_of_x_axis = ( distance_Travel_along_x * OneMeter_pulses ) + Pluses_After_turn ; 
 
                             }
                          
                            else if (Direction = 1){
                              servo.writeMicroseconds(Right_torque);
                              delay(time_for_full_torque);
//                              servo.writeMicroseconds(little_Right_torque);
                               servo.writeMicroseconds(1500);
                              delay(time_for_little_torque/2);
                              servo.writeMicroseconds(1500);
                              delay(10);
                              
                              Pluses_After_turn = CurrentPulseCount ; 
                              pulses_of_x_axis = ( distance_Travel_along_x * OneMeter_pulses ) + Pluses_After_turn ; 
//                               throttle_send(speed_value,Deceleration) ;

                              }
                         
                            else if (Direction = -1)
                              {
                               servo.writeMicroseconds(left_torque);
                               delay(time_for_full_torque);
//                               servo.writeMicroseconds(little_left_torque);
                               servo.writeMicroseconds(1500);

                               delay(time_for_little_torque/2);
                               servo.writeMicroseconds(1500);
                                                             delay(10);

                               Pluses_After_turn = CurrentPulseCount ; 
                               pulses_of_x_axis = ( distance_Travel_along_x * OneMeter_pulses ) + Pluses_After_turn ; 
//                               throttle_send(speed_value,Deceleration) ;

                              }  
                            else if ( Direction = 0 )
                              {
                               servo.writeMicroseconds(1500);
                               Pluses_After_turn = CurrentPulseCount ; 
                                                             delay(10);

                               pulses_of_x_axis = ( distance_Travel_along_x * OneMeter_pulses ) + Pluses_After_turn ; 
//                               throttle_send(speed_value,Deceleration) ;
                              }
                          }

     
                                
                                 void ultrasonic_CHECK(int distance) 
                                {
                                if (20 <distance < 100)  
                                  {
                                    throttle_send(speed_value, Deceleration);
                                    delay (1000);
                                  }
                                }
                                
                                
                                void determine_goal_coordinate(void)
                                {
                                  while(true)
                                   {
                                  Serial.println("Enter the value of x coordinate of the goal:");
                                  while (!Serial.available()) {} // Wait for user input
                                  x = Serial.parseInt(); // Read the value entered by the user
                                
                                  Serial.println("Enter the value of y coordinate of the goal:");
                                  while (!Serial.available()) {} // Wait for user input
                                  y = Serial.parseInt(); // Read the value entered by the user
                                
                                  // Check if both x and y % = 0 ,,and y >= 1 to prevent negative values 
                                  if (x == (int) x && y == (int) y && y >= 1) {
                                    Serial.println("Values accepted");
                                    break; 
                                   }
                                  else {
                                    Serial.println("Values are not valid. Both x and y should be greater than or equal to 2.");
                                    Serial.println("Please enter the values again.");
                                    continue;
                                  }
                                   }
                                }



                                void initialize_move_plan(int x , int y)
                                {
                                  if (x=!0){
                                    distance_Travel_along_y = y - Y_Axis_Meters_Used_to_Turn;
                                  }
                                  else {
                                    distance_Travel_along_y = y;
                                    Serial.println(y);
                                  }
                                  distance_Travel_along_x = x - x_Axis_Meters_Used_to_Turn;
                                  pulses_of_y_axis = distance_Travel_along_y * OneMeter_pulses ;
                                  Serial.println("move plan has been initialized");
                                }


                                
                                void MOVE_CAR(void)
                                {
                                  //ultrasonic_CHECK( Ultra_distance) ;
                                   if (pulses_of_y_axis >= CurrentPulseCount )
                                   {
                                     throttle_send(speed_value,accelration) ;

                                   }
                                    else if ( pulses_of_y_axis < CurrentPulseCount && esc_flag == 0)
                                   {

                                      steerESC(x);
                                //      delay (50);

                                   } 
                                     else if (pulses_of_x_axis >= CurrentPulseCount && esc_flag == 1) 
                                      {
                                       throttle_send(speed_value,accelration);
                                       
                                      }
                                    else 
                                   {
                                       //throttle_send(speed_value,Deceleration);
                                   }
                                }
                                

void blinkLed_Right(void);
void blinkLed_Left(void);
void blinkLed_both(void);

void blinkLed_Left(void) {
  // Set the digital pin as output:
  if (currentMillis_left - previousMillis_left > interval) {
    // Save the last time the LED was toggled 
    previousMillis_left = currentMillis_left;
    digitalWrite(Left_OUT, !digitalRead(Left_OUT));
  }
}

void blinkLed_Right(void) {
  // Set the digital pin as output:
  if (currentMillis_Right - previousMillis_Right > interval) {
    // Save the last time the LED was toggled 
    previousMillis_Right = currentMillis_Right;
    digitalWrite(Right_OUT, !digitalRead(Right_OUT));
  }
}

void blinkLed_both(void) {
  // Set the digital pin as output:
  if (currentMillis_both - previousMillis_both > interval) {
    // Save the last time the LED was toggled 
    previousMillis_both = currentMillis_both;
    digitalWrite(Right_OUT, !digitalRead(Right_OUT));
    digitalWrite(Left_OUT, digitalRead(Right_OUT));
    //digitalWrite(Blinking_LED, !digitalRead(Blinking_LED));

  }
}

void I2C_Read(int howMany) {
  // loop through all 

  I2C_B1 = Wire.read();
  I2C_B2 = Wire.read();

  Seat_Belt_SW = bitRead(I2C_B1, 0);
  Braking_SW = bitRead(I2C_B1, 1);
  Accident_SW = bitRead(I2C_B1, 2);
  Right_SW = bitRead(I2C_B1, 3);
  Left_SW = bitRead(I2C_B1, 4);
  Wait_SW = bitRead(I2C_B1, 5);
  HL_SW = bitRead(I2C_B1, 6);
  Horn_SW = bitRead(I2C_B1, 7);

  ST_SW = bitRead(I2C_B2, 1); /// REVERSD
  LKA_SW = bitRead(I2C_B2, 0);/// REVERSD
  ACC_SW = bitRead(I2C_B2, 2);
  AEB_SW = bitRead(I2C_B2, 3);

}

void Blinking_Internal(void) {

  if (currentMillis_internal - previousMillis_internal > interval) {
    // Save the last time the LED was toggled 
    previousMillis_internal = currentMillis_internal;
    digitalWrite(Blinking_LED, !digitalRead(Blinking_LED));
  }

                                     

}


void setup() {
  canMsg1.can_id  = 0x00;
  canMsg1.can_dlc = 2;
  canMsg1.data[0] = 0x00;
  canMsg1.data[1] = 0x00;


  Wire.begin(8); // join I2C bus with address #8
  Wire.onReceive(I2C_Read); // register event

  Serial.begin(115200);
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS);
  mcp2515.setNormalMode();
// InitTimersSafe(); //Initialize timers without disturbing timer 0

  current_state = idle_state;
  pinMode(Blinking_LED, OUTPUT);
  pinMode(EN_R, OUTPUT);
  //pinMode(PWM_R, OUTPUT);
  //pinMode(EN_L, OUTPUT);
 // pinMode(PWM_L, OUTPUT);
  pinMode(AEB_LED, OUTPUT);
  pinMode(ACC_LED, OUTPUT);
  pinMode(LKA_LED, OUTPUT);
  pinMode(ST_LED, OUTPUT);
  pinMode(HL_OUT, OUTPUT);
  pinMode(Left_OUT, OUTPUT);
  pinMode(Right_OUT, OUTPUT);
  pinMode(Horn_OUT, OUTPUT);

                                       delay(7000); // for esc
                                      // init ESC
                                      servo.attach(ESC_PIN);
                                    //   determine_goal_coordinate();
                                   //   initialize_move_plan(x, y) ;
}

void loop() { 

                                      if (mcp2515.readMessage(&canMsg2) == MCP2515::ERROR_OK) {
                                        fb_displacement = (canMsg2.data[0] & 0xFF) | ((canMsg2.data[1] & 0xFF) << 8) | ((canMsg2.data[2] & 0xFF) << 16) | ((canMsg2.data[3] & 0xFF) << 24);
                                        fb_speed = (canMsg2.data[4] & 0xFF) | ((canMsg2.data[5] & 0xFF) << 8) | ((canMsg2.data[6] & 0xFF) << 16) | ((canMsg2.data[7] & 0xFF) << 24);
                                      }

                                            // example of how receiver should deal with data
                                      Serial.print("Displacement: ");
                                      Serial.print(fb_displacement / 1000.0, 2);
                                      Serial.print("\n");

                                      Serial.print("Speed: ");
                                      Serial.print(fb_speed * (1000 / 100000000.0), 2);
                                      Serial.print("\n");
                                                                            
                                    // ultrasonic 
                                    // delay(50);
                                    // unsigned int uS = sonar.ping_cm();
                                    // Serial.print(uS);
                                    // Serial.println("cm");

   
  
    currentMillisss = millis();

  if (ACC_SW == 1 && ACC_flag == 1) {
    delay(50);
    if (ACC_SW == 1) {
      ACC_loop = !ACC_loop;
      ACC_flag = 0;
    }
  }
  if (ACC_SW == 0) {
    ACC_flag = 1;
  }

  if (AEB_SW == 1 && AEB_flag == 1) {
    delay(50);
    if (AEB_SW == 1) {
      AEB_loop = !AEB_loop;
      AEB_flag = 0;
    }
  }
  if (AEB_SW == 0) {
    AEB_flag = 1;
  }

  digitalWrite(AEB_LED,AEB_loop);
  digitalWrite(ACC_LED,ACC_loop);
                          
                                 if (ACC_loop == LOW && AEB_loop == HIGH ) 
                              { 

                                  throttle_state = contsspeed_state;
                                   
                                    Serial.println("contsspeed_state activated");
                                    
                              }
                              
                                  else if(ACC_loop == HIGH && AEB_loop == LOW)
                                  {
                                     
                                   throttle_state = AUTO_state;
                                   
                               //     Serial.println("AUTO_state activated");
                                    
                                  }
                                  else if (ACC_loop == HIGH && AEB_loop == HIGH)
                                  {
                                    
                                    throttle_state = brakes_state;
                                   
                                 //   Serial.println("brakes_state activated");
                                     
                                  }
                                
                              else if (ACC_loop == LOW && AEB_loop == LOW)
                              {
                                      
                                throttle_state = manual_state;
                                
                                 //   Serial.println("manual_state activated");
                                    
                              }
                            
                              else
                              {
                               throttle_state = manual_state;
                                     
                               
                               //     Serial.println("manual_state activated");
                                    
                              }

                          
                              switch(throttle_state)
                            {
                              case manual_state:
                                canMsg1.data[0]=map(analogRead(Throttle),0,1024,0,255);

                                break;
                          
                              case brakes_state:
                                throttle_send(speed_value, Deceleration);                                
                                break;
                          
                              case AUTO_state:
                                    Serial.println("oh SHIT");
//                          
//                                      Ultra_distance = sonar.ping_cm(); 
//                                      delay(50); // needed for ultrasonic
//                                      ultrasonic_CHECK(Ultra_distance) ;
                                      MOVE_CAR();
                                      
                                      break;
                                
                              case contsspeed_state:
                              
                                              canMsg1.data[0]=70;
                                
                                break;
                                   
                              default:
                                canMsg1.data[0]=map(analogRead(Throttle),0,1024,0,255);
                              }


  currentMillis_Right = millis();
  currentMillis_left = millis();
  currentMillis_both = millis();
  currentMillis_internal = millis();
  // hazards

  if (Wait == HIGH) {
    current_state = hazard_state;
  } else if (Wait == LOW && Right == HIGH && Left == LOW) {
    current_state = right_state;
  } else if (Wait == LOW && Right == LOW && Left == HIGH) {
    current_state = left_state;
  } else if (Wait == LOW && Right == LOW && Left == LOW) {
    current_state = idle_state;
  }

  if (Wait_SW == 1 && Wait_flag == 1) {
    delay(50);
    if (Wait_SW == 1) {
      Wait = (!Wait);
      Wait_flag = 0;
    }
  }
  if (Wait_SW == 0) {
    Wait_flag = 1;
  }

  if (Right_SW == 1 && Right_flag == 1) {
    delay(50);
    if (Right_SW == 1) {
      Right = (!Right);
      Right_flag = 0;
    }
  }
  if (Right_SW == 0) {
    Right_flag = 1;
  }

  if (Left_SW == 1 && Left_flag == 1) {
    delay(50);
    if (Left_SW == 1 ) {
      Left = (!Left);
      Left_flag = 0;
    }
  }
  if (Left_SW == 0) {
    Left_flag = 1;
  }

  switch (current_state) {
    case (idle_state):
      digitalWrite(Left_OUT, 0);
      digitalWrite(Right_OUT, 0);

      break;
    case (left_state):
      digitalWrite(Right_OUT, 0);
      blinkLed_Left();

      break;
    case (right_state):
      blinkLed_Right();
      digitalWrite(Left_OUT, 0);

      break;
    case (hazard_state):
      blinkLed_both();

      break;
    default:
      break;
  }

  digitalWrite(Horn_OUT, Horn_SW);

  //digitalWrite(HL_OUT,HL_SW);
  if (HL_SW == 1 && HL_flag == 1) {
    delay(50);
    if (HL_SW == 1) {
      digitalWrite(HL_OUT, !digitalRead(HL_OUT));
      HL_flag = 0;
    }
  }
  if (HL_SW == 0) {
    HL_flag = 1;
  }

                                                          

  bitWrite(canMsg1.data[1], 0, digitalRead(Left_OUT));
  bitWrite(canMsg1.data[1], 1, digitalRead(Right_OUT));
  bitWrite(canMsg1.data[1], 2, Braking_SW);
                                                           
  mcp2515.sendMessage(&canMsg1);

  Blinking_Internal();
  digitalWrite(Blinking_LED,HIGH);
                                                   
}
