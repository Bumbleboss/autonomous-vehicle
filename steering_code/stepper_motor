#define step_per_rev 1000

// defines pins numbers

const int stepPin = 7; // +pulses
const int dirPin = 6; 
const int enPin = 8;
const int desired_deg=45.0;
float total_steps ;
float x = 0;

void setup() {
  
  // Sets the two pins as Outputs
  pinMode(stepPin,OUTPUT); 
  pinMode(dirPin,OUTPUT);

  pinMode(enPin,OUTPUT);
  digitalWrite(enPin,LOW);

  total_steps=((desired_deg/360.0)*step_per_rev);
  
}
void loop() {
   digitalWrite(dirPin,HIGH); // Enables the motor to move in a particular direction
   
   for(; x < total_steps; x++) {
     digitalWrite(stepPin,HIGH); 
     delayMicroseconds(800); 
     digitalWrite(stepPin,LOW); 
     delayMicroseconds(500);
   }