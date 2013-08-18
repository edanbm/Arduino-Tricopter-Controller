/*  Tricopter Controller Code for Arduino

    Copyright (C) 2013 Steven C. Parker

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>
 */

#include <Wire.h>
#include <Servo.h>

//  Gyro Related Settings and Variables
#define ITG3200_ADDRESS          0x68
#define ITG3200_WHO_AM_I         0x00
#define ITG3200_SMPLRT_DIV       0x15 
#define ITG3200_DLPF_FS          0x16
#define ITG3200_INT_CFG          0x17
#define ITG3200_INT_STATUS       0x1A
#define ITG3200_TEMP_OUT_H       0x1B
#define ITG3200_TEMP_OUT_L       0x1C
#define ITG3200_GYRO_XOUT_H      0x1D
#define ITG3200_GYRO_XOUT_L      0x1E
#define ITG3200_GYRO_YOUT_H      0x1F
#define ITG3200_GYRO_YOUT_L      0x20
#define ITG3200_GYRO_ZOUT_H      0x21
#define ITG3200_GYRO_ZOUT_L      0x22
#define ITG3200_PWR_MGM          0x3E
#define ITG_DLPF_CFG_0           0x1;
#define ITG_DLPF_CFG_1           0x2;
#define ITG_DLPF_CFG_2           0x4;
#define ITG3200_DLPF_FS_SEL_0    0x8;
#define ITG3200_DLPF_FS_SEL_1    0x10;

int32_t     xRate_drift, yRate_drift, zRate_drift;

//  Receiver Related Signals
uint8_t      portb_Now, portb_Last;
//
//  pulseLength[0]  Throttle
//  pulseLength[1]  Elevator
//  pulseLength[2]  Rudder
//  pulseLength[3]  Aileron (reversed)

int16_t      riseEdge[4], fallEdge[4], pulseLength[4];

//  Loop timer
int32_t      timeNow;
int32_t      lastUpdateTime;


//  SuperRX State Machine Variables
#define      STATE_INIT         0
#define      STATE_UNARMED      1
#define      STATE_ARMCMD       2
#define      STATE_ARMED        3

uint8_t      main_state = STATE_INIT;
uint32_t     arm_start_time;

//  PID Controller signals
int16_t      yaw_error;
int16_t      yaw_newerror;
int16_t      yaw_Kp = 20;
int16_t      yaw_Ki = 0;
int16_t      yaw_Kd = 0;
int16_t      yaw_output = 0;
int16_t      yawP, yawI, yawD;
int16_t      yaw_pulselength;

int16_t      roll_error;
int16_t      roll_newerror;
int16_t      roll_Kp = 30;
int16_t      roll_Ki = 0;
int16_t      roll_Kd = 0;
int16_t      roll_output = 0;
int16_t      rollP, rollI, rollD;

int16_t      pitch_error;
int16_t      pitch_newerror;
int16_t      pitch_Kp = 20;
int16_t      pitch_Ki = 0;
int16_t      pitch_Kd = 0;
int16_t      pitch_output = 0;
int16_t      pitchP, pitchI, pitchD;

//  Servo Related Signals
Servo        yaw_servo;
Servo        port_esc;
Servo        star_esc;
Servo        aft_esc;

//  Debug Port
uint8_t      tx_buf[256];

void setup ()
{
  Serial.begin(115200);
  Wire.begin();
  gyroInit();

  //  Enable PC Change Interriupt for digital pins 8,9,10,11
  PCICR=(1<<PCIE0);
  PCMSK0=(1<<PCINT0) | (1<<PCINT1) |(1<<PCINT2) | (1<<PCINT3); 

  //  Timers 1 on instead of millis() - greater than 4us accuracy 
  //  Divide by 8 - 2 Mhz operation
  TCCR1A = 0x00;
  TCCR1B = 0x02;
  
  //  Servo / ESC attachment
  yaw_servo.attach(4);
  port_esc.attach(5);
  star_esc.attach(6);
  aft_esc.attach(7);
}

void loop ()
{  
  unsigned char data;
  int16_t temp, xRate, yRate, zRate;
  unsigned long result;

//  Time the loop to 50 Hz
  timeNow = micros();
  if (timeNow > lastUpdateTime + 20000)
  {
    lastUpdateTime = lastUpdateTime + 20000;

    //  Get latest Gyro sample
    data = gyroRead(ITG3200_WHO_AM_I);
    temp =  gyroGetAxis(ITG3200_TEMP_OUT_H);
    xRate = gyroGetAxis(ITG3200_GYRO_XOUT_H) >> 2 ;
    yRate = gyroGetAxis(ITG3200_GYRO_YOUT_H) >> 2 ;
    zRate = gyroGetAxis(ITG3200_GYRO_ZOUT_H) >> 2 ;
      
    switch(main_state)
    {
      case STATE_INIT:
        // TBD - maybe some reset values

        main_state = STATE_UNARMED;
        break;
      case STATE_UNARMED:
      {
        //  Initialize Drift Counters
        xRate_drift = 0;
        yRate_drift = 0;
        zRate_drift = 0;
        
        //  Check for arm state
        if(pulseLength[0] < 1000 && pulseLength[2] > 1700)
        {  
          arm_start_time = millis();
          main_state = STATE_ARMCMD;
        }     
     
        //  Center Servos
        yaw_servo.writeMicroseconds(1450);
        port_esc.writeMicroseconds(950);
        star_esc.writeMicroseconds(950);
        aft_esc.writeMicroseconds(950);
        
        break; 
      }
      
      case STATE_ARMCMD:
      {
        //  Low pass filter drift
        
        xRate_drift = (xRate_drift - (xRate_drift>>4)) + (int32_t)xRate;
        yRate_drift = (yRate_drift - (yRate_drift>>4)) + (int32_t)yRate;
        zRate_drift = (zRate_drift - (zRate_drift>>4)) + (int32_t)zRate;

        if(pulseLength[0] > 1000 || pulseLength[2] < 1700)
        {  
          main_state = STATE_UNARMED;
          break;
        }        
        
        //  Arm Command for 5 seconds
        if (millis() > arm_start_time + 5000)
          // Scale drift
          xRate_drift = xRate_drift >> 4; 
          yRate_drift = yRate_drift >> 4; 
          zRate_drift = zRate_drift >> 4; 
          main_state = STATE_ARMED;
          break;
      }

      case STATE_ARMED:
      {
        //  Add PID here

        yaw_newerror = (zRate - zRate_drift) - (pulseLength[2] - 1500);
        roll_newerror = -(yRate - yRate_drift) - (pulseLength[3] - 1500);
        pitch_newerror = (xRate - xRate_drift) - (pulseLength[1] - 1500);

        //  Cap all error terms
        
        if(yaw_newerror > 400) yaw_newerror = 400;
        if(yaw_newerror < -400) yaw_newerror = -400;
        if(roll_newerror > 400) roll_newerror = 400;
        if(roll_newerror < -400) roll_newerror = -400;
        if(pitch_newerror > 400) pitch_newerror = 400;
        if(pitch_newerror < -400) pitch_newerror = -400;


        yawP = yaw_newerror >> 2;
        yawI = yawI + (yaw_Ki * yaw_newerror);
        yawD = yaw_Kd * (yaw_newerror - yaw_error);

        rollP = roll_newerror >> 2;
        rollI = rollI + (roll_Ki * roll_newerror>>5);
        rollD = roll_Kd * (roll_newerror - roll_error);

        pitchP = pitch_newerror >> 2;
        pitchI = pitchI + (pitch_Ki * pitch_newerror);
        pitchD = pitch_Kd * (pitch_newerror - pitch_error);

        yaw_output = (yawP);
        roll_output = (rollP + rollI + rollD);
        pitch_output = (pitchP + pitchI + pitchD);

        yaw_error = yaw_newerror;
        roll_error = roll_newerror;
        pitch_error = pitch_newerror;
       
        yaw_pulselength = 1500  - ((3*yaw_newerror)>>2) + 100;
        
        yaw_servo.writeMicroseconds(yaw_pulselength);
        port_esc.writeMicroseconds(pulseLength[0]  - ((5*pitch_newerror)>>5)  + ((5*roll_newerror)>>5));
        star_esc.writeMicroseconds(pulseLength[0]  - ((5*pitch_newerror)>>5)  - ((5*roll_newerror)>>5));
        aft_esc.writeMicroseconds(pulseLength[0] + ((5*pitch_newerror)>>5));
        break;
      }
    }

  //  Debug Port

  tx_buf[0] = 0x7E;
  tx_buf[1] = roll_newerror >> 8;
  tx_buf[2] = roll_newerror & 0xFF;
  tx_buf[3] = pitch_newerror >> 8;
  tx_buf[4] = pitch_newerror & 0xFF;
  tx_buf[5] = roll_newerror >> 8;;
  tx_buf[6] = roll_newerror & 0xFF; 
  tx_buf[7] = 0x00;
  tx_buf[8] = 0x00;
  tx_buf[9] = 0x00;
  tx_buf[10] = 0x00;
  tx_buf[11] = 0x00;
  tx_buf[12] = 0x00;
  tx_buf[13] = 0x00;
  tx_buf[14] = main_state;
  tx_buf[15] = 0x7E;

//  Serial.write(tx_buf, 16);
//  Serial.println((tx_buf[1] <<8) | (tx_buf[2] & 0xFF));
  }
}

void gyroInit()
{
  //  Set low pass filter bandwidth to 98Hz
  gyroWrite(ITG3200_DLPF_FS, 0x16 );
  //  Divide sample rate by 1
  gyroWrite(ITG3200_SMPLRT_DIV, 1);
}  

void gyroWrite(char registerAddress, char data)
{
  Wire.beginTransmission(ITG3200_ADDRESS);
  Wire.write(registerAddress);
  Wire.write(data);
  Wire.endTransmission();  
}

unsigned char gyroRead(char registerAddress)
{
  unsigned char data=0;

  Wire.beginTransmission(ITG3200_ADDRESS);
  Wire.write(registerAddress);
  Wire.endTransmission();  

  Wire.requestFrom(ITG3200_ADDRESS, 1);
  while (!Wire.available()) {
  }
  data = Wire.read();

  return data;
}

int gyroGetAxis(char registerAddress)
{
  int data=0;

  data = gyroRead(registerAddress)<<8;
  data |= gyroRead(registerAddress+1);

  return data;
}

ISR(PCINT0_vect)
{
  timeNow = TCNT1;
  portb_Now = PINB;
  int PulseLength_tmp;
  int edgeCount;

  for(edgeCount=0;edgeCount<4;edgeCount++)
  {
    //  Check for rising edge
    if (!(portb_Last & 1<<edgeCount) && (portb_Now & 1<<edgeCount))
    {
      riseEdge[edgeCount] = timeNow;
    }

    //  Check for falling edge
    if ((portb_Last & 1<<edgeCount) && !(portb_Now & 1<<edgeCount))
    {
      fallEdge[edgeCount] = timeNow;
      PulseLength_tmp = (fallEdge[edgeCount] - riseEdge[edgeCount]) >> 1;
      if (PulseLength_tmp > 900 && PulseLength_tmp < 2200) 
        pulseLength[edgeCount] = PulseLength_tmp;
    }
  } 
  portb_Last = portb_Now;       
}



