#include "NewPing.h" // library to use ultrsonic sensor
#include "Servo.h" // library to use servo motors
#include "String.h"
#include <Pixy2.h>

Pixy2 pixy; // object of pixy lbrary to use pixy camera

int tomato_width = 85; // define minimum width of tomato
int tomato_height = 60; // define minimum height of tomato

int lemon_width = 70 ; // define minimum width of lemon
int lemon_height = 40; // define minimum height of lemon

int m1a = A5; // motor pins attach to arduino
int m1b = A4;
int m2a = A2;
int m2b = A3;

int ENA = 3; // it must be pvm pins
int ENB = 5; // use to control the speed of motor car

int RIGHT = A1; //ir sensor pin
int LEFT = A0; 

//pvm pins is used to convert the digital signal into an analog by varying the width of the Pulse 

int rotationGripperPos = 1;
int groundGripperPos = 130;    // variable to store the servo position
int upperGroundGripperPos = 60;
int gripperPos = 80;

int rotationGripperPin = 4;
int groundGripperPin = 9;    // variable to attach the servo to arduino
int upperGroundGripperPin = 7;
int gripperPin = 6;

#define TRIGGER_PIN  8   //Trigger pin is an Input pin
#define ECHO_PIN     2   //Echo pin is an Output pin
#define MAX_DISTANCE 100

Servo gripper; // take Servo library objects and use for our gripper
Servo groundGripper;
Servo upperGroundGripper;
Servo rotationGripper;
 
int pos = 0; 

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // use newping library for accurate distance measure with ultrasonic sensor

String val,num;  // val variable is to get value from bluetoth app and give command to arduino manually
float distance;

int val1; // get servo position and then write in servo 

void setup() {

	pixy.init();   

	// The pinMode() function is used to configure a specific pin to behave either as an input or an output
	pinMode(m1a, OUTPUT);  // Digital pin 10 set as output Pin
	pinMode(m1b, OUTPUT);  // Digital pin 11 set as output Pin
	pinMode(m2a, OUTPUT);  // Digital pin 12 set as output Pin
	pinMode(m2b, OUTPUT);  // Digital pin 13 set as output Pin
	pinMode(ENA, OUTPUT); // initialize ENA pin as an output
	pinMode(ENB, OUTPUT); // initialize ENB pin as an output
	pinMode(RIGHT, INPUT); // initialize RIGHT pin as an input
	pinMode(LEFT, INPUT); // initialize ENA pin as an input

	Serial.begin(9600);

	groundGripper.attach(groundGripperPin); // attach the servos to arduinos
	upperGroundGripper.attach(upperGroundGripperPin); // use PWM pins of arduino for servos
	rotationGripper.attach(rotationGripperPin);
	gripper.attach(gripperPin);

	pinMode(RIGHT, INPUT); // initialize RIGHT pin as an input
	pinMode(LEFT, INPUT); // initialize ENA pin as an input

}

void loop(){

	int i; 
	pixy.ccc.getBlocks(); // intialize pixy and get detected objects
	
	manualRobotMovement(); // use to control movements by andriod app
  
	detectAndArrangeObjects(); // detect both objects and pick place 
	
	lineFollowerRobot(); // line follower robot commands
  
}

void lineFollowerRobot(){
	if (digitalRead(RIGHT)==0 && digitalRead(LEFT)==0){ //MOVE FORWARD
		moveRobot(60,60,1,0,1,0);
	}
	else if (digitalRead(RIGHT)== 0 && digitalRead(LEFT)== 1){ //MOVE RIGHT//
		moveRobot(0,200,0,0,1,0);
		delay(400);
	}
	else if (digitalRead(RIGHT)== 1 && digitalRead(LEFT) == 0){ //MOVE-LEFT//
		moveRobot(200,0,1,0,0,0);
		delay(400);    
	}
	else if (digitalRead(RIGHT)== 1 && digitalRead(LEFT)== 1){ //STOP//
		moveRobot(0,0,0,0,0,0);
	}
}

void detectAndArrangeObjects(){

	int distance = sonar.ping_cm();

	if (distance >= 5 && distance <= 10){
	
		groundGripper.write(120); // set servo to default posiiton
		upperGroundGripper.write(90);
		rotationGripper.write(90);
		gripper.write(80);
			  
		moveRobot(0,0,0,0,0,0); // stop the robot
		delay(1000);
			  
		if (pixy.ccc.numBlocks){ // check if there is an detected object or not

			int x = pixy.ccc.blocks[0].m_x;
			int y = pixy.ccc.blocks[0].m_y;
			int w = pixy.ccc.blocks[0].m_width;
			int h = pixy.ccc.blocks[0].m_height;
				
			if(pixy.ccc.getBlocks()== 1 && pixy.ccc.blocks[0].m_signature == 1 && h < tomato_height && h > (tomato_height-15) && w < tomato_width && w > (tomato_width-15)){ 
  
				// if it detect object it return 1 then we check which object for tomotoas we set signature 1
					
				moveGroundGripper(120,0); // move servos to pick tomatos
				moveGripper(80,110);
				moveGroundGripper(80,110);      
			  
				moveRobot(0,200,0,0,1,0); //turn right
				delay(750);  
				moveRobot(0,0,0,0,0,0);// stop robot 

				moveGroundGripper(120,0); // move servos to drop tomatos      
				moveGripper(110,80);
				moveGroundGripper(0,120);

				moveRobot(0,200,0,0,0,1); // turn right back

				delay(900); 
				moveRobot(0,0,0,0,0,0); // stop robot       
				return 1;	
			}
			else if(pixy.ccc.getBlocks()== 1 && pixy.ccc.blocks[0].m_signature == 2 && h < lemon_height && h > (lemon_height-15) && w < lemon_width && w > (lemon_width - 15) ){ 
				
				// for lemons we set signature on pixymon as 2
			  
				moveGroundGripper(120,0); // move servos to pick lemons
				moveGripper(80,120);
				moveGroundGripper(0,120);
				  
				moveRobot(0,0,0,0,0,0); // stop robot
							
				moveRobot(200,0,1,0,0,0);  // turn left
				delay(750);       
				moveRobot(0,0,0,0,0,0); // stop robot

				moveGroundGripper(120,0); // move servos to drop lemons
				moveGripper(120,80);
				moveGroundGripper(0,120);      
				  
				moveRobot(200,0,0,1,0,0); // turn left back

				delay(900);
				moveRobot(0,0,0,0,0,0); // stop robot      
				return 1;
			} 			   
			return 1;
		}
	}
}

void manualRobotMovement(){
	int i;

	if(Serial.available() > 0){
		val = Serial.readString();
		num = val.substring(1, 2);
		Serial.println(num);
		Serial.println(val);
  
		if(val.startsWith("F")){ // Forward
			// set speed 150 for both right left tires then give command to motor which side move with 0 and 1 and  0 means low 1 means high
			moveRobot(150,150,1,0,1,0); 
			delay(500); 
			moveRobot(0,0,0,0,0,0);//stop robot
			return 1;
		}
		else if(val.startsWith("B")){ // Backward
			// set speed 150 for both right left tires then give command to motor which side move with 0 and 1 and  0 means low 1 means high
			moveRobot(150,150,0,1,0,1);    
			delay(500);
			moveRobot(0,0,0,0,0,0); // stop robot
			return 1;
		}
		else if(val.startsWith("L")){ //Left
			// set speed 200 for both left tires then give command to motor which side move with 0 and 1 and  0 means low 1 means high
			moveRobot(200,0,1,0,0,0);   
			delay(500);
			moveRobot(0,0,0,0,0,0); //stop robot
			return 1;
		}
		else if(val.startsWith("R")){ //Right 
			// set speed 200 for both right tires then give command to motor which side move with 0 and 1 and  0 means low 1 means high
			moveRobot(0,200,0,0,1,0);      
			delay(500);
			moveRobot(0,0,0,0,0,0);//stop robot 
			return 1;
		}
		else if(val.startsWith("S")){ //Stop
			// set speed 0 for both right left tires then give command to motor which side move with 0 and 1 and  0 means low 1 means high
			moveRobot(0,0,0,0,0,0);
			delay(1000);
		}
		else if(val.startsWith("D")){      
			// write code to drop the manual picking up cup or glass
		}
		else if(val.startsWith("G")){ // pick glass 
			distance = sonar.ping_cm(); // get distance from ultrasonic sensor
		  
			if(distance == 5 || distance == 4) {  // measure the degree of motion of servos then wriite noted degree in servos
				gripper.write(gripperPos);
				groundGripper.write(groundGripperPos);
				upperGroundGripper.write(upperGroundGripperPos);
				rotationGripper.write(rotationGripperPos);

				moveGroundGripper(groundGripperPos,60);
				moveGripper(gripperPos,170);
				moveGroundGripper(gripperPos,150);
			  
				return 1;        
			}
		}
		else if(val.startsWith("C")){    
			distance = sonar.ping_cm();    
        
			if(distance == 4 || distance == 5){// measure the degree of motion of servos then wriite noted degree in servos
				groundGripper.write(170);
				rotationGripper.write(1); 
          
				for (pos = 0; pos <= 60; pos += 1) { 
					upperGroundGripper.write(pos);
					delay(50);                     
				}
				
				gripper.write(70);
				moveGroundGripper(180,60);
				moveGripper(90,170);
				moveGroundGripper(60,180);
			}       
		} 
		else if(val.startsWith("W")) { // for gripper    
			String num = val.substring(1, val.length());
			int   servo1PPos = num.toInt();
			Serial.println(servo1PPos);

			// We use for loops so we can control the speed of the servo If previous position is bigger then current position
			if (servo1PPos > gripperPos ) {
				for ( int j = servo1PPos; j > gripperPos; j--) {   // Run servo down
					gripper.write(servo1PPos);
					delay(50);    // defines the speed at which the servo rotates
				}
			}
			else if (servo1PPos < gripperPos) {       
				for ( int j = servo1PPos; j < gripperPos; j++) {   // Run servo up
					gripper.write(servo1PPos);
					delay(50);
				}
			}   
			servo1PPos = gripperPos;   // set current position as previous position
			return 1;
		}
		else if(val.startsWith("X")) { // rotation/elbow gripper rotation  
			String num = val.substring(1, val.length());
			int   servo1PPos = num.toInt();
			Serial.println(servo1PPos);

			// We use for loops so we can control the speed of the servo If previous position is bigger then current position
			if (servo1PPos > rotationGripperPos ) {
				for ( int j = servo1PPos; j > rotationGripperPos; j--) {   // Run servo down
					rotationGripper.write(servo1PPos);
					delay(50);    // defines the speed at which the servo rotates
				}
			}
			else if (servo1PPos < rotationGripperPos) {
				for ( int j = servo1PPos; j < rotationGripperPos; j++) {   // Run servo up
					rotationGripper.write(servo1PPos);
					delay(50);
				}
			}
			servo1PPos = rotationGripperPos;   // set current position as previous position
			return 1;
		}
		else if(val.startsWith("Y")) { // upper ground rotation   
			String num = val.substring(1, val.length());
			int   servo1PPos = num.toInt();
			Serial.println(servo1PPos);

			// We use for loops so we can control the speed of the servo If previous position is bigger then current position
			if (servo1PPos > upperGroundGripperPos ) {
				for ( int j = servo1PPos; j > upperGroundGripperPos; j--) {   // Run servo down
					upperGroundGripper.write(servo1PPos);
					delay(50);    // defines the speed at which the servo rotates
				}
			}
			else if (servo1PPos < upperGroundGripperPos) {
				for ( int j = servo1PPos; j < upperGroundGripperPos; j++) {   // Run servo up
					upperGroundGripper.write(servo1PPos);
					delay(50);
				}
			}
			servo1PPos = upperGroundGripperPos;   // set current position as previous position
			return 1;
		}
		else if(val.startsWith("Z")) { // ground gripper rotation    
			String num = val.substring(1, val.length());
			int   servo1PPos = num.toInt();
			Serial.println(servo1PPos);

			// We use for loops so we can control the speed of the servo If previous position is bigger then current position
			if (servo1PPos > groundGripperPos ) {
				for ( int j = servo1PPos; j > groundGripperPos; j--) {   // Run servo down
					groundGripper.write(servo1PPos);
					delay(50);    // defines the speed at which the servo rotates
				}	
			}
			else if (servo1PPos < groundGripperPos) {
				for ( int j = servo1PPos; j < groundGripperPos; j++) {   // Run servo up
					groundGripper.write(servo1PPos);
					delay(50);
				}
			}
			servo1PPos = groundGripperPos;   // set current position as previous position
			return 1;
		}
		else{
			moveRobot(0,0,0,0,0,0);// stop robot
		}
	}
}

void moveRobot(int enaPos,int enbPos,int dcMotor1,int dcMotor2,int dcMotor3,int dcMotor4){
	analogWrite(ENA, enaPos);
	analogWrite(ENB, enbPos);
	digitalWrite(m1a, dcMotor1);
	digitalWrite(m1b, dcMotor2);
	digitalWrite(m2a, dcMotor3);
	digitalWrite(m2b, dcMotor4);   
}

void moveGroundGripper(int startPos,int endPos){
	if(startPos>endPos){
		for (pos = startPos; pos >= endPos; pos -= 1) {      
			val1 = map(pos, 0, 1023, 0, 180);  // get value from map function which later write on servo as degree to move    
			groundGripper.write(val1);  
			groundGripper.write(pos);             
			delay(25); 
		}  
	}
	else{
		for (pos = startPos; pos <= endPos; pos += 1) {
			val1 = map(pos, 0, 1023, 0, 180);  // get value from map function which later write on servo as degree to move    
			groundGripper.write(val1);  
			groundGripper.write(pos);             
			delay(25);                     
		}
	}
}

void moveGripper(int startPos,int endPos){
	if(startPos<endPos){
		for (pos = startPos; pos <= endPos; pos += 1) {
			val1 = map(pos, 0, 1023, 0, 180);   // get value from map function which later write on servo as degree to move   
			gripper.write(val1);  
			gripper.write(pos);             
			delay(25); 
		}
	}
	else{
		for (pos = startPos; pos >= endPos; pos -= 1) {
			val1 = map(pos, 0, 1023, 0, 180);    // get value from map function which later write on servo as degree to move 
			gripper.write(val1);  
			gripper.write(pos);             
			delay(25); 
		}   
	}
}
  
