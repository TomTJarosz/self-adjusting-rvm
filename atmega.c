#include <Arduino.h>
#include <HardwareSerial.h>

bool[3] limit={false,false,false};
int8_t[4] read_buf;
int8_t[4] write_buf;
bool serial_ret=false;
int limit_pins[3]={2,3,4};
int led_pin=9;
int button_pin=8;
int step_pin0=7;
int step_pin1=11;
int step_pin2=10;
int steps[3]={step_pin0,step_pin1,step_pin2}
int dir_pin0=13;
int dir_pin1=6;
int dir_pin2=5;
int dirs[3]={dir_pin0,dir_pin1,dir_pin2}

int limit_pin0=4;
int limit_pin1=3;
int limit_pin2=2;
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50; 
unsigned long last_button_press=0;
int double_press_time=1000;
int shutoffDelay=5000;
bool unsent_button=false;
int8_t button_state=0x00;
bool on=true;
void setup() 
{
  Serial.begin(115200);//we can probs go faster
  pinMode(led_pin, OUTPUT);
  pinMode(button_pin, INPUT);
  pinMode(step_pin0, OUTPUT);
  pinMode(step_pin1, OUTPUT);
  pinMode(step_pin2, OUTPUT);
  pinMode(dir_pin0, OUTPUT);
  pinMode(dir_pin1, OUTPUT);
  pinMode(dir_pin2, OUTPUT);
  pinMode(limit_pin0, INPUT);
  pinMode(limit_pin1, INPUT);
  pinMode(limit_pin2, INPUT);
  pinMode(A0,OUTPUT);
  digitalWrite(led_pin,HIGH);
  //TODO
}

void loop() 
{

update_button_state();
if (Serial.available() >3)
{
Serial.readBytes(read_buf,4);
if (read_buf[0]&0x03==1)
  {//rotate
  rotate(&(read_buf[1]),0x04&read_buf[0],0x08&read_buf[0],0x10&read_buf[0]);
  }
  
if (read_buf[0]==2)
  {
  //send state
  send_state();
  //set button state to 0
  button_state=0;
  }
if (read_buf[0]==3)
  {
  //move to limit
  move_to_limit();
  }
}
}
//returns [31-8=number of steps from previous orientation for motors 0,1,and 2 (negated). 5-3=limit, 2-0=overheat]
void move_to_limit(int8_t* dTarg)
  {
  int8_t[3] dT={0,0,0};
  int i;
  write_buf[0]=0;
  write_buf[1]=0;
  write_buf[2]=0;
  write_buf[3]=0;

        digitalWrite(dir_pin2, LOW);
        while (!digitalRead(limit_pin2))
          {
          digitalWrite(step_pin2,HIGH);
          delay(1);
          digitalWrite(step_pin2,LOW);
          delay(1);          
          }
          //delay(50);


        digitalWrite(dir_pin1, LOW);
        while (!digitalRead(limit_pin1))
          {
          digitalWrite(step_pin1,HIGH);
          delay(22);
          digitalWrite(step_pin1,LOW);
          delay(13);
          }
          //Serial.println(score-70);
          //delay(50);


        digitalWrite(dir_pin0, HIGH);
        while (!digitalRead(limit3))
          {

          digitalWrite(step_pin0,HIGH);
          delay(1);
          digitalWrite(step_pin0,LOW);
          delay(15);
          
          }
          //Serial.println(score-70);
          //delay(100);

  update_limit(0);
  update_limit(1);
  update_limit(2);
  for (i=0;i<3;i++)
    {
    write_buf[0]=write_buf[0]|(int8_t(!limit[i]))<<(i+3);
    }
  for (i=0;i<3;i++)
    {
    write_buf[i+1]=dT[i];
    }
  Serial.Write(write_buf,4);
  }

void rotate(int8_t* dTarg,bool negative0, bool negative1, bool negative2)
  {
  int i;
  bool dir;
    
    for (i=0; i<3; i++)
      {
                 
        if (i==0)
        {dir=!negative0}
        if (i==1)
        {dir=!negative0}
        else (i==2)
        {dir=negative0}
        rotate_motor(i,dir,dTarg[i]);
        update_limit(i);
        
      }
    

  write_buf[0]=0;
  write_buf[1]=0;
  write_buf[2]=0;
  write_buf[3]=0;

    write_buf[0]=0;
    for (i=0;i<3;i++)
      {
      write_buf[0]=write_buf[0]|(int8_t(limit[i]))<<(i+3);
      }

    
  Serial.Write(write_buf,4);
  }


//state-> bits23-16=button state, 8=limit0, 9=limit1, 10=limit2, 0=overeat0, 1=overheat1, 2=overheat2
void send_state()
{
  update_limit();
  int i;
  write_buf[0]=0x00;
  write_buf[0]= write_buf[0]|button_state;
  write_buf[1]=0x00;
  write_buf[2]=0x00;
  write_buf[3]=0x00;
  Serial.Write(write_buf,4);
}






void update_limit(int i)
{
limit[i]=digitalRead(limit_pins[i]);
}

void update_button_state()
  {
  unsigned long cur=millis();
  bool button = digitalRead(button_pin);

  if (button != lastButtonState) 
    {
    if (((cur - lastDebounceTime) > debounceDelay)&&!button) 
      {
      if (cur-last_button_press<double_press_time)
        {//double click
        unsent_button=false;
        button_state=1;
        }
      else
        { 
        unsent_button=true;
        last_button_press=cur;
        }
      }
    lastDebounceTime = cur;
    }
    
  else if (button)
    {
    if (((cur - lastDebounceTime) > shutoffDelay)) 
      {//shutdown
      digitalWrite(led_pin,0);
      on=!on;
      digitalWrite(A0,on);
      button_state=2;
      }
    }


  else if (!button&&unsent_button&&cur-last_button_press>double_press_time)
    {//single click
    unsent_button=false;
      button_state=3;
    }
  }



void rotate_motor(int motor,bool dir, int8_t dist)
{int i;
if (motor==0)
  {
    digitalWrite(dir_pin0, dir);
    for (i=0; i<dist; i++)
    {
    digitalWrite(step_pin0,HIGH);
    delay(1);
    digitalWrite(step_pin0,LOW);
    delay(15);
    }
  }
if (motor==1)
  {
    digitalWrite(dir_pin1, dir);
    for (i=0; i<dist; i++)
    {
    digitalWrite(step_pin1,HIGH);
    delay(22);
    digitalWrite(step_pin1,LOW);
    delay(13);
    }
  }
if (motor==2)
  {
    digitalWrite(dir_pin2, dir);
    
    for (i=0; i<dist; i++)
    {
    digitalWrite(step_pin2,HIGH);
    delay(1);
    digitalWrite(step_pin2,LOW);
    delay(1);
    }
  }

}



