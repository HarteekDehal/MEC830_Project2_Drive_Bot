// Motor Pins Established
int enA = 3;
int in1 = 5;
int in2 = 6;
int enB = 11;
int in3 = 9;
int in4 = 10;

//Motor Speed Values established
int lMotorSpeed = 0;
int rMotorSpeed = 0;

// Joystick Input
int yStick = A2; 
int xStick = A3; 

//Middle of the axis of the joystick is established
int yPos = 512;
int xPos = 512;  

void setup(){

  // Set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
   
  // Left motor disabled and direction forward
  digitalWrite(enA, LOW);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  // Right motor disabled and direction forward
  digitalWrite(enB, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  
}

void loop() {

  // Analog values read from joystick and saved as positions
  yPos = analogRead(yStick); 
  xPos = analogRead(xStick);

//Right and Left Motors turned backward if the joystick is pushed in the negative y direction
  if (yPos < 400){
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    //Determine Motor Speeds mapped by the proportional position of the joystick from center
    yPos = 400-yPos ; 
    lMotorSpeed = map(yPos, 0, 400, 0, 100);
    rMotorSpeed = map(yPos, 0, 400, 0, 100);
  }

  //Right and Left Motors turned forward if the joystick is pushed in the negative y direction
  else if (yPos > 600)
  {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    //Determine Motor Speeds mapped by the proportional position of the joystick from center
    lMotorSpeed = map(yPos, 600, 1023, 0, 100);
    rMotorSpeed = map(yPos, 600, 1023, 0, 100); 
  }

  //Turn off both motors if the joystick isn't moving past center
  else
  {
    lMotorSpeed = 0;
    rMotorSpeed = 0; 
  }
  
//Right and Left Motors turned backward if the joystick is pushed in the negative y direction
  if (xPos < 400){
    //Determine Motor Speeds mapped by the proportional position of the joystick from center
    xPos = 400-xPos ; 
    xPos = map(xPos, 0, 400, 0, 100);
    //Right motor increased by same amount right motor is decreased so car turns left
    lMotorSpeed = lMotorSpeed - xPos;
    rMotorSpeed = rMotorSpeed + xPos;
    // Limit speed of motors to 100
    if (lMotorSpeed < 0)lMotorSpeed = 0;
    if (rMotorSpeed > 100)rMotorSpeed = 100;
  }

  //Turns car right
  else if (xPos > 600)
  {
    //Determine Motor Speeds mapped by the proportional position of the joystick from center
    xPos = map(xPos, 600, 1023, 0, 100);
    //Left motor increased by same amount right motor is decreased so car turns right
    lMotorSpeed = lMotorSpeed + xPos;
    rMotorSpeed = rMotorSpeed - xPos;
    // Limit speed of motors to 100
    if (lMotorSpeed > 100)lMotorSpeed = 100;
    if (rMotorSpeed < 0)rMotorSpeed = 0;      
  }

  //Write the speeds of the motors so car can move
  analogWrite(enA, lMotorSpeed);
  analogWrite(enB, rMotorSpeed);

}
