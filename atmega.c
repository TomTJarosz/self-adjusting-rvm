#include <Arduino.h>
#include <HardwareSerial.h>

bool[3] overheat={false,false,false};
bool[3] limit={false,false,false};
int8_t[4] read_buf;
int8_t[4] write_buf;
bool serial_ret=false;
int[3]overheat_pins;
int[3]limit_pins;
int led_pin;
int button_pin;
int motor_pin0;
int motor_pin1;
int motor_pin2;
int limit_pin0;
int limit_pin1;
int limit_pin2;
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50; 
unsigned long last_button_press=0;
int double_press_time=1000;
int shutoffDelay=5000;
bool unsent_button=false;
int8_t button_state=0x00;

void setup() 
{
  Serial.begin(115200);//we can probs go faster
  pinMode(led_pin, OUTPUT);
  pinMode(button_pin, INPUT);
  pinMode(motor_pin0, OUTPUT);
  pinMode(motor_pin1, OUTPUT);
  pinMode(motor_pin2, OUTPUT);
  pinMode(limit_pin0, INPUT);
  pinMode(limit_pin1, INPUT);
  pinMode(limit_pin2, INPUT);
  //TODO
}

void loop() 
{
update_button_state();
if (Serial.available() >3)
{
Serial.readBytes(read_buf,4);
if (read_buf[0]==1)
  {//rotate
  rotate(&(read_buf[1]));
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
  int dir=0;
  write_buf[0]=0;
  write_buf[1]=0;
  write_buf[2]=0;
  write_buf[3]=0;
  bool[3] done={false,false,false};
  while (!done[0]|!done[1]|!done[2]) 
    {
    for (i=0; i<3; i++)
      {
      update_overheat(i);
      update_limit(i);
      if (overheat[i])
        {
        write_buf[0]=write_buf[0]|(int8_t(overheat[i]))<<i;
        done[i]=true;
        }
      if (limit[i])
        {
        done[i]=true;
        }
      if (!done[i])
        {          
        rotate_motor(i,-1);
        if (dt[i]<127)
          {
          dT[i]+=1;
          }
        }
      }
    }
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

void rotate(int8_t* dTarg)
  {
  int8_t[3] dT={0,0,0};
  int i;
  int dir=0;
  bool[3] done={false,false,false};
  while (!done[0]|!done[1]|!done[2]) 
    {
    for (i=0; i<3; i++)
      {
      update_overheat(i);
      update_limit(i);
      if (dT[i]==dTarg[i] | overheat[i] | limit[i])
        {
        done[i]=true;
        }
      if (!done[i])
        {          
        if (dTarg[i]<0)
          {
          dir=-1;
          }
        else
          {
          dir=1;
          }
        rotate_motor(i,dir);
        dT[i]+=dir;
        }
      }
    }

  write_buf[0]=0;
  write_buf[1]=0;
  write_buf[2]=0;
  write_buf[3]=0;
  if (dT[0]==dTarg[0]&&dT[1]==dTarg[1]&&dT[2]==dTarg[2])
    {
    Serial.Write(write_buf,4);
    }
  else
    {
    write_buf[0]=0;
    for (i=0;i<3;i++)
      {
      write_buf[0]=write_buf[0]|(int8_t(overheat[i]))<<i;
      }
    for (i=0;i<3;i++)
      {
      write_buf[0]=write_buf[0]|(int8_t(limit[i]))<<(i+3);
      }
    for (i=0;i<3;i++)
      {
      write_buf[i+1]=dT[i];
      }
    }
  Serial.Write(write_buf,4);
  }


//state-> bits23-16=button state, 8=limit0, 9=limit1, 10=limit2, 0=overeat0, 1=overheat1, 2=overheat2
void send_state()
{
  update_overheat();
  update_limit();
  int i;
  write_buf[0]=0x00;
  for (i=0;i<3;i++)
    {
    write_buf[0]=write_buf[0]|(int8_t(overheat[i]))<<i;
    }
  write_buf[1]=0x00;
  for (i=0;i<3;i++)
    {
    write_buf[1]=write_buf[1]|(int8_t(limit[i]))<<i;
    }
  write_buf[2]=0x00;
  write_buf[2]= write_buf[2]|button_state;
  write_buf[3]=0x00;
  Serial.Write(write_buf,4);
}





void update_overheat(int i)
{
overheat[i]=digitalRead(overheat_pins[i]);
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
      button_state=2;
      }
    }


  else if (!button&&unsent_button&&cur-last_button_press>double_press_time)
    {//single click
    unsent_button=false;
      button_state=3;
    }
  }



void rotate_motor(int motor,int direction)
{//todo
}



