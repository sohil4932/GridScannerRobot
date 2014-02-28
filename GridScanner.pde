/*
 8x8 Maze solver
 PIN CONFIGURATION
 
 A0-Line following sensor
 A1-  "
 A2-  "
 A3-  "
 A4-A5-A6-A7-""
 
 pwm 6 - motor
 pwm 9 - motor
 pwm 10- motor
 pwm 11- motor
 
 initial direction = +y (left = +x, right= -x, up = +y, down = -y)
 +y=01,-y=10,+x=00,-x=11
 */

#include <EEPROM.h>

int val=0;

// Motor connection - L293D connection
const int lf=6;         // Left Motor - High Value runs robot in Forward direction
const int lr=9;         // Left Motor - High Value runs robot in Reverse direction 
const int rf=11;        // Right Motor - High Value runs robot in Forward direction
const int rr=10;        // Right Motor - High Value runs robot in Reverse direction

boolean flag=0;          // denotes direction from grid to maze 

int curx=0;              // Current potition of robot - X cordinate
int cury=0;              // Current potition of robot - Y cordinate

// Speed of motor
int fast=175;            
int slow1=100;
int slow2=50;
int slow3=0;
int slow=125;

// Store value of line follower sensors
int sensor=0;

int initialvar=1;

byte dir=0;                // indicates direction in which bot is moving

// We defined directions as numeric value
const byte py=01;        // positive y
const byte ny=10;        // negative y
const byte px=00;        // positive x
const byte nx=11;        // negative x

int a=0;
int temp=1;
byte sen_off=0;
int addr=0;
byte sen_on=0;
byte csen_on=0;
byte node_value=0;

void setup()
{  
  pinMode(6,OUTPUT);   // 6 and 9 for left motor
  pinMode(9,OUTPUT);   
  pinMode(10,OUTPUT);   // 10 and 11 for right motor
  pinMode(11,OUTPUT);

  // Line follower sensor on PORT-F of Arduino Mega.
  DDRF=B00000000; 

  // Initial direction of robot is Positive Y.
  dir = py;        
}

void loop()
{
  // Read values of Line follower sensor.
  a = PINF;
  sensor = a & 0xFF;

  // Call line follower function.
  linefollow();

  // Check if robot reached to cross section or not.
  linecount();

  // If robot reaches to boundry change direction.
  if (cury == 7) 
  {
    if (curx%2==0 && flag==0)
    {
      turnleft();
      flag=1;
    }
    if (curx%2==1 && flag==1)
    {
      turnleft();
      flag=0;
    }
  }
  if (cury==0)
  {
    if (curx%2==1 && flag==0)
    {
      if (curx==7)
      {
        while(1)
          stopbot();
      } 
      turnright();
      flag=1;
    }
    if (curx%2==0 && flag==1)
    {
      turnright();
      flag=0;
    }
  }


}

//line follow function
void linefollow()
{
  //straight, centre 2 and individual centre 2
  if ((sensor==B00011000) || (sensor==B00001000) || (sensor==B00010000))
  {
    analogWrite(lf,fast);
    analogWrite(lr,0);
    analogWrite(rf,fast);
    analogWrite(rr,0);
  } 

  //left side offset, right fast left slow, 3-3 2-2 1-1
  //slow 1
  else if ((sensor==B00111000) || (sensor==B00110000) || (sensor==B00100000))
  {
    analogWrite(lf,slow1);
    analogWrite(lr,0);
    analogWrite(rf,fast);
    analogWrite(rr,0);
  }

  //right side offset, left fast right slow, 3-3 2-2 1-1
  //slow 1
  else if ((sensor==B00011100) || (sensor==B00001100) || (sensor==B00000100))
  {
    analogWrite(lf,fast);
    analogWrite(lr,0);
    analogWrite(rf,slow1);
    analogWrite(rr,0);
  } 

  //left side offset, right fast left slow, 3-3 2-2 1-1
  //slow 2
  else if ((sensor==B01110000) || (sensor==B01100000) || (sensor==B01000000) )
  {
    analogWrite(lf,slow2);
    analogWrite(lr,0);
    analogWrite(rf,fast);
    analogWrite(rr,0);
  }

  //right side offset, left fast right slow, 3-3 2-2 1-1
  //slow 2
  else if ((sensor==B00001110) || (sensor==B00000110)  || (sensor==B00000010))
  {
    analogWrite(lf,fast);
    analogWrite(lr,0);
    analogWrite(rf,slow2);
    analogWrite(rr,0);
  } 

  //left side offset, right fast left slow, 3-3 2-2 1-1
  //slow 3 
  else if ((sensor==B11100000) || (sensor==B11000000)  || (sensor==B10000000))
  {
    analogWrite(lf,slow3);
    analogWrite(lr,0);
    analogWrite(rf,fast);
    analogWrite(rr,0);
  }
  //right side offset, left fast right slow, 3-3 2-2 1-1
  //slow 3 
  else if ((sensor==B00000011) || (sensor==B00000001) || (sensor==B00000111))
  {
    analogWrite(lf,fast);
    analogWrite(lr,0);
    analogWrite(rf,slow3);
    analogWrite(rr,0);
  }
  else
  {
    analogWrite(lf,fast);
    analogWrite(lr,0);
    analogWrite(rf,fast);
    analogWrite(rr,0);
  }  
}


//line count function
void linecount()
{
  sen_off=0;
  sen_on=0;
  csen_on=0;
  if ((sensor&B00000001) == B00000001)
  {
    sen_on++;
  }
  if ((sensor&B00000010) == B00000010)
  {
    sen_on++;
  }
  if ((sensor&B00000100) == B00000000)
  {
    sen_off++;
  }
  if ((sensor&B00001000) == B00000000)
  {
    sen_off++;
  }
  else
  {
    csen_on++;
  } 
  if ((sensor&B00010000) == B00000000)
  {
    sen_off++;
  }
  else
  {
    csen_on++;
  } 
  if ((sensor&B00100000) == B00000000)
  {
    sen_off++;
  }
  if ((sensor&B01000000) == B01000000)
  {
    sen_on++;
  }
  if ((sensor&B10000000) == B10000000)
  {
    sen_on++;
  }

  // addr is variable to store value of node into EEPROM
  if (sensor == B11111111)
  {
    // Change current potition of robot according to direction of robot
    if(dir == px)
    {
      curx++;
      addr = addr + 8;
    }
    else if (dir == nx)
    {
      curx--;
      addr = addr - 8;
    }
    else if (dir==py)
    {
      cury++;
      addr = addr + 1;
    }
    else if (dir == ny)
    {
      cury--;
      addr = addr - 1;
    }

    // Get value from EEPROM and change it according to presece of node.
    // In this condition Node is not present at intersection point, So we put 0.
    node_value = EEPROM.read(addr);
    node_value = node_value + 0;
    EEPROM.write(addr,node_value);      //no node detected

    // Run robot in same condition untill it crosses the intersection point.
    do
    {
      a=PINF;
      sensor=a&0xFF;
      sen_off=0;
      if ((sensor&B00000001) == B00000000)
      {
        sen_off++;
      }
      if ((sensor&B00000010) == B00000000)
      {
        sen_off++;
      }
      if ((sensor&B00000100) == B00000000)
      {
        sen_off++;
      }
      if ((sensor&B00100000) == B00000000)
      {
        sen_off++;
      }
      if ((sensor&B01000000) == B00000000)
      {
        sen_off++;
      }
      if ((sensor&B10000000) == B00000000)
      {
        sen_off++;
      }
    }
    while(sen_off < 5);

    sen_off=0; 
  }

  else if ((sen_off>=1) && (sen_on>=3) && (csen_on<2))
  {
    // Change current potition of robot according to direction of it.
    if(dir == px)
    {
      curx++;
      addr = addr + 8;
    }
    else if (dir == nx)
    {
      curx--;
      addr = addr - 8;
    }
    else if (dir==py)
    {
      cury++;
      addr = addr + 1;
    }
    else if (dir == ny)
    {
      cury--;
      addr = addr - 1;
    }

    // Get value from EEPROM and change it according to presece of node.
    // In this condition Node is present at intersection point, So we put 1.
    node_value=EEPROM.read(addr);
    node_value=node_value + 1;
    EEPROM.write(addr,node_value);      //node detected

    // Run robot in same condition untill it crosses the intersection point.
    do
    {
      a=PINF;
      sensor=a&0xFF;
      sen_off=0;
      if ((sensor&B00000001) == B00000000)
      {
        sen_off++;
      }
      if ((sensor&B00000010) == B00000000)
      {
        sen_off++;
      }
      if ((sensor&B00000100) == B00000000)
      {
        sen_off++;
      }
      if ((sensor&B00100000) == B00000000)
      {
        sen_off++;
      }
      if ((sensor&B01000000) == B00000000)
      {
        sen_off++;
      }
      if ((sensor&B10000000) == B00000000)
      {
        sen_off++;
      }
    }
    while(sen_off < 5);
    sen_off=0; 
  }  
}

//breaking function
void stopbot()
{
  analogWrite(lf,0);
  analogWrite(lr,0);
  analogWrite(rf,0);
  analogWrite(rr,0);
}

//turn left
void turnleft()
{
  analogWrite(lf,0);
  analogWrite(lr,175);
  analogWrite(rf,175);
  analogWrite(rr,0);

  // Turn untill sensor reach to perticular state.
  do
  {
    a=PINF;
    sensor=a&0xFF;
  }
  while(sensor != B01100000); 

  // Stop for while  
  stopbot();

  // Change Current direction of robot according to previous direction
  if (dir == py)
  {
    dir = px;
  }
  else if (dir == px)
  {
    dir = ny;
  }
  else if (dir == ny)
  {
    dir = nx;
  }
  else if (dir == nx)
  {
    dir = py;
  }

  delay(100);
}

//turn right
void turnright()
{
  analogWrite(lf,175);
  analogWrite(lr,0);
  analogWrite(rf,0);
  analogWrite(rr,175);

  // Turn untill sensor reach to perticular state.
  do
  {
    a=PINF;
    sensor=a&0xFF;
  }
  while(sensor != B00000110);  

  // Stop for while 
  stopbot();

  // Change Current direction of robot according to previous direction
  if (dir == py)
  {
    dir = nx;
  }
  else if (dir == nx)
  {
    dir = ny;
  }
  else if (dir == ny)
  {
    dir = px;
  }
  else if (dir == px)
  {
    dir = py;
  }     
  delay(100);  
}
