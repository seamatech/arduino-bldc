#include <FlexiTimer2.h>
#include <PWM.h>
#include <avr/wdt.h>

//sema bldc v1.0
//we use pwm lib for high speed pwm
//we use flexitimer2 for detect the hall sensor
//The MosFET status:
//1-UH(Pin7)  3-VH(Pin8)  5-WH;(Pin9)
//2-UL(Pin3)  4-VL(Pin4)  6-WL;(Pin6)
#define MosPin_RUN1   digitalWrite(7,1);digitalWrite(8,0);digitalWrite(9,0); digitalWrite(3,0);digitalWrite(4,1);digitalWrite(6,0) //101 CW Hall=5; (1,4) 10-01-00  U-V-W   AB
#define MosPin_RUN2   digitalWrite(7,1);digitalWrite(8,0);digitalWrite(9,0); digitalWrite(3,0);digitalWrite(4,0);digitalWrite(6,1) //100 CW Hall=4; (1,6) 10-00-01  U-V-W   AC
#define MosPin_RUN3   digitalWrite(7,0);digitalWrite(8,1);digitalWrite(9,0); digitalWrite(3,0);digitalWrite(4,0);digitalWrite(6,1) //110 CW Hall=6; (3,6) 00-10-01  U-V-W   BC
#define MosPin_RUN4   digitalWrite(7,0);digitalWrite(8,1);digitalWrite(9,0); digitalWrite(3,1);digitalWrite(4,0);digitalWrite(6,0) //010 CW Hall=2; (3,2) 01-10-00  U-V-W   BA
#define MosPin_RUN5   digitalWrite(7,0);digitalWrite(8,0);digitalWrite(9,1); digitalWrite(3,1);digitalWrite(4,0);digitalWrite(6,0) //011 CW Hall=3; (5,2) 01-00-10  U-V-W   CA
#define MosPin_RUN6   digitalWrite(7,0);digitalWrite(8,0);digitalWrite(9,1); digitalWrite(3,0);digitalWrite(4,1);digitalWrite(6,0) //001 CW Hall=1; (5,4) 00-10-01  U-V-W   CB
#define MosPin_STOP   digitalWrite(7,0);digitalWrite(8,0);digitalWrite(9,0); digitalWrite(3,0);digitalWrite(4,0);digitalWrite(6,0) //All MosFet Close     00-00-00  U-V-W
#define MosPin_BRK    digitalWrite(7,0);digitalWrite(8,0);digitalWrite(9,0); digitalWrite(3,1);digitalWrite(4,1);digitalWrite(6,1) //The Low MosFet Open  01-01-01  U-V-W

#define _MotorPhasePairs 5
#define _UpdatePWMTime 1  //The Value more big, the motor will start up more slow.
#define _PWMUpStep 2
#define _PWMDownStep 1
#define _DelayTimes 1
#define _UpOffSetDelayTime 10
#define _MaxBRKTimes 1000
#define _MaxPinChangeTime 500
#define _KeyMaxCount  20

#define _MaxIshunt 500  //Limit Normal run current about 12A  ( the resistance value 10momega)
#define _MaxUpIshunt 580 //Limit Startup Current about 3A
#define _MaxMaxIshunt 780 //Limit Peeak Current about 16A 
#define _MaxIshuntCheck 220 //Check the mosfet status. Limit the current.
#define _MaxIshuntTime 2

#define _MaxUpPWM 900 //Limit Startup the PWM
#define _MaxLocationPWM 150 //Location the rotor position.
#define _MaxLocationTime 100
#define _MaxMotorUpStep  5

#define _BEMFLow 0
#define _BEMFHigh 1

#define _MaxUpdateStallTime 20000 //The motor stall time
#define _MaxSensorLessUpdateStallTime 5000 //The sensorless time

#define M_LoopRun 0
#define M_KP 8
#define M_KI 2
#define M_KD 0
#define M_Scale 30
#define M_MaxMaxPWM  1016
#define M_MinMinPWM  200
#define M_MaxSpeed 3000
#define M_MinSpeed 200

//int UpDelayTime[]= {2000,1900,1800,1600,1400,1200,1000,800,600,550,500,480,460,440,420,400,380,360,355,350,345,340,335,330,325,320,315,310,305,300,298,292};
int UpDelayTime[]= {50,50,50,49,49,49,43,42,40,40,39,39,38,38,37,37,36,35,35,34,34,34,33,33,32,31,31,30};
//int UpDelayTime[]= {1650,1600,1550,1500,1500,1450,1450,1400,1400,1400,1380,1370,1360,1360,1355,1355,1350,1350,1340,1340,1330,1330,1320,1320,1320,1320,1320,1310};
int frequency = 20000; //pwm frequency in Hz 16khz
int HallValue,LastHallValue,RunPin,DirPin,BEMFPinA,BEMFPinB,BEMFPinC;
int RunEnable=1,DirStatus=1;
int ReadPoint,NowPwm=0;
unsigned int UpdatePwm=0;
unsigned int UpdateStallTime=0;
int OutStatus, Ishunt,AvgIshunt=0,SubIshunt,ErrIshunt,CountErr=0,Error=0,HallErr=0,CountRun=0,RunStatus=0,CurrentDirect;
int RunPinChangeCount=0,DirPinChangeCount=0;
int FlashBegan,FlashErr;
unsigned long AvgMotorStepTime,MotorStepTime,LastStepTime,ThisFlashTime,lastrtime;
unsigned long BEMFTimeStamp,LastRunCycleTime,RunTime;
int MotorPhase,LastPhase,SensorLess=0;
int Hall_1=0,Hall_2=0,Hall_3=0,Hall_4=0,Hall_5=0,Hall_6=0,RightRunStatus=0,SensorLessSound=1;
int MotorActualSpeed,TargetSpeed;
int EK, EK1,EK2,CEK,CEK1,CEK2,YK,DealLoop;
float EnvTemperature;

void setup(){
  Serial.begin(9600); 
  pinMode(2, INPUT);  //Over Current for Hardware 硬件保护
  pinMode(3, OUTPUT);  //U-L
  pinMode(4, OUTPUT);  //V-L
  pinMode(6, OUTPUT);  //W-L
  pinMode(7, OUTPUT);  //U-H
  pinMode(8, OUTPUT);  //V-H
  pinMode(9, OUTPUT);  //W-H
 //the hall sensor abc
  pinMode(11, INPUT); //Hall A
  pinMode(12, INPUT); //Hall B
  pinMode(13, INPUT); //Hall C
  pinMode(18, INPUT); //Dir
  pinMode(19, INPUT); // Enable
  //
  pinMode(14, INPUT_PULLUP); //BEMFPinA
  pinMode(15, INPUT_PULLUP); // BEMFPinB
  pinMode(16, INPUT_PULLUP); // BEMFPinC
  
  //setP16( ); 
  MotorStop();
  InitTimersSafe();
  bool success = SetPinFrequencySafe(10, frequency); //pin 10 for high speed pwm,
  digitalWrite(5,1);
  NowPwm=0;
  pwmWrite(10,0);
  CheckMos();
  MotorSound();
  FlexiTimer2::set(1, 1.0/15000, MotorRUN);
  FlexiTimer2::start();
  digitalWrite(5,0);
  wdt_enable(WDTO_120MS);
}

void setP32( ) {
  Serial.println("ADC Prescaler = 32");  // 101
  ADCSRA |=  (1 << ADPS2);  // 1
  ADCSRA &=  ~(1 << ADPS1);  // 0
  ADCSRA |=  (1 << ADPS0);  // 1
}


void setP128( ) { // 
  Serial.println("ADC Prescaler = 128");  // 111
  ADCSRA |=  (1 << ADPS2);  // 1
  ADCSRA |=  (1 << ADPS1);  // 1
  ADCSRA |=  (1 << ADPS0);  // 1
} // setP128


void setP16( ) {
  Serial.println("ADC Prescaler = 16");  // 100
  ADCSRA |=  (1 << ADPS2);  // 1
  ADCSRA &=  ~(1 << ADPS1);  // 0
  ADCSRA &=  ~(1 << ADPS0);  // 0
}

void MotorRUN(){
         wdt_reset();
         UpdatePwm++;
        
         if(RunEnable==0&&Error==0){
              if(SensorLess==1&&RunStatus==2){
                    if(CurrentDirect==1){
                          SenseBEMFCW();
                          UpdateStallTime++;
                         if(MotorPhase!=LastPhase){
                            if(RunStatus<3)CountRun++;
                                  MosPin_STOP;
                                  delayMicroseconds(2);
                                  SensorLessCW();
                                  LastPhase=MotorPhase;
                                  UpdateStallTime=0;
                                  digitalWrite(5,OutStatus);
                                  if(MotorPhase==1){
                                       MotorStepTime=micros()-LastStepTime;
                                       LastStepTime=micros();
                                       AvgMotorStepTime=(AvgMotorStepTime+MotorStepTime)>>1;
                                       DealLoop=1;
                                           OutStatus=!OutStatus;
                                  }
                          }
                   }
                   
                   else{
                           SenseBEMFCCW();
                           UpdateStallTime++;
                           if(MotorPhase!=LastPhase){
                              if(RunStatus<3)CountRun++;
                                MosPin_STOP;
                                delayMicroseconds(2);
                                 SensorLessCCW();
                                 LastPhase=MotorPhase;
                                 UpdateStallTime=0;
                                 digitalWrite(5,OutStatus);
                           
                                 if(MotorPhase==1){
                                      MotorStepTime=micros()-LastStepTime;
                                     LastStepTime=micros();
                                    AvgMotorStepTime=(AvgMotorStepTime+MotorStepTime)>>1;
                                       DealLoop=1;
                                           OutStatus=!OutStatus;
                                  }
                            }
                   }
                   
            }//SelesorLess Status
            if(SensorLess==0){
                    UpdateStallTime++;
                    HallValue= digitalRead(11)*4+(int)digitalRead(12)*2+digitalRead(13);
                    if(RunStatus>0)if(HallValue==0||HallValue==7)Error=3;
                    if(HallValue!=LastHallValue){
                            MosPin_STOP;
                            //delayMicroseconds(2);
                            HallValue= digitalRead(11)*4+(int)digitalRead(12)*2+digitalRead(13);           
                            if(CurrentDirect==1){ 
                                     MotorCCW(); 
                               }
                               else{ 
                                     MotorCW();   
                              }
                             LastHallValue=HallValue;
                             UpdateStallTime=0;
                             if(RunStatus<3)CountRun++;
                             digitalWrite(5,OutStatus);
                         
                             if(HallValue==2){
                                 MotorStepTime=micros()-LastStepTime;
                                 LastStepTime=micros();
                                 AvgMotorStepTime=(AvgMotorStepTime+MotorStepTime)>>1;
                                 DealLoop=1;
                                     OutStatus=!OutStatus;
                             }    
                      }
                }//Sensored Status 
      }
      else{
         MosPin_STOP;
      }
}

void SensorLessCW(){
 switch(MotorPhase){
             case 1:   
                   MosPin_RUN1; 
                   break; 
             case 2:   
                  MosPin_RUN2; 
                  break; 
             case 3:  
                   MosPin_RUN3; 
                   break;  
             case 4:   
                   MosPin_RUN4; 
                   break; 
             case 5:   
                   MosPin_RUN5; 
                   break; 
             case 6: 
                   MosPin_RUN6; 
                   break;  
             default: MosPin_STOP;
       } 
}

void SensorLessCCW(){
 switch(MotorPhase){
             case 1:  
                   MosPin_RUN1; 
                   break; 
             case 2:   //100
                  MosPin_RUN6; 
                  break; 
             case 3:  //110
                   MosPin_RUN5; 
                   break;  
             case 4:   //010
                   MosPin_RUN4; 
                   break; 
             case 5:   //011
                   MosPin_RUN3; 
                   break; 
             case 6: //001
                   MosPin_RUN2; 
                   break;  
             default: MosPin_STOP;
       } 
}

void SenseBEMFCW(){
       BEMFPinA=digitalRead(14);
       BEMFPinB=digitalRead(15);
       BEMFPinC=digitalRead(16);
       switch(MotorPhase){
             case 1:    
                   if(BEMFPinC==_BEMFLow){
                     RunTime=micros()-BEMFTimeStamp;
                     delayMicroseconds(2);
                      BEMFPinC=digitalRead(16);
                      if((BEMFPinC==_BEMFLow)&&(RunTime>(LastRunCycleTime/2))){
                            MotorPhase++; 
                            LastRunCycleTime=micros()-BEMFTimeStamp;
                            BEMFTimeStamp=micros();
                      }   
                   }
                    break; 
             case 2:   
                    if(BEMFPinB==_BEMFHigh){
                       RunTime=micros()-BEMFTimeStamp;
                       delayMicroseconds(2);
                        BEMFPinB=digitalRead(15);
                        if((BEMFPinB==_BEMFHigh)&&(RunTime>(LastRunCycleTime/2))){
                              MotorPhase++; 
                              LastRunCycleTime=micros()-BEMFTimeStamp;
                              BEMFTimeStamp=micros();                      
                         }
                      }
                  break; 
             case 3:     
                   if(BEMFPinA==_BEMFLow){
                        RunTime=micros()-BEMFTimeStamp;
                       delayMicroseconds(2);
                       BEMFPinA=digitalRead(14);
                       if((BEMFPinA==_BEMFLow)&&(RunTime>(LastRunCycleTime/2))){
                             MotorPhase++; 
                             LastRunCycleTime=micros()-BEMFTimeStamp;
                             BEMFTimeStamp=micros();
                         }
                  
                  }
                   break;  
             case 4:              
                       if(BEMFPinC==_BEMFHigh){
                             RunTime=micros()-BEMFTimeStamp;
                            delayMicroseconds(2);
                            BEMFPinC=digitalRead(16);
                            if((BEMFPinC==_BEMFHigh)&&(RunTime>(LastRunCycleTime/2))){
                                  MotorPhase++;
                                  LastRunCycleTime=micros()-BEMFTimeStamp;
                                  BEMFTimeStamp=micros();
                              }
                    }
                   break; 
             case 5:   
                     if(BEMFPinB==_BEMFLow){
                           RunTime=micros()-BEMFTimeStamp;
                           delayMicroseconds(2);
                            BEMFPinB=digitalRead(15);
                            if((BEMFPinB==_BEMFLow)&&(RunTime>(LastRunCycleTime/2))){
                                  MotorPhase++; 
                                  LastRunCycleTime=micros()-BEMFTimeStamp;
                                  BEMFTimeStamp=micros();
                         }
                      }
                   break; 
             case 6: 
                     if(BEMFPinA==_BEMFHigh){
                          RunTime=micros()-BEMFTimeStamp;
                         delayMicroseconds(2);
                          BEMFPinA=digitalRead(14);
                          if((BEMFPinA==_BEMFHigh)&&(RunTime>(LastRunCycleTime/2))){
                                  MotorPhase=1;
                                  LastRunCycleTime=micros()-BEMFTimeStamp;
                                  BEMFTimeStamp=micros();
                          }
                     }
                   break;  
             default: MosPin_STOP;
       } 
             
}


void SenseBEMFCCW(){
       BEMFPinA=digitalRead(14);
       BEMFPinB=digitalRead(15);
       BEMFPinC=digitalRead(16);

      
       switch(MotorPhase){
             case 1:   
                      if(BEMFPinC==_BEMFHigh){
                         RunTime=micros()-BEMFTimeStamp;
                          delayMicroseconds(2);
                          BEMFPinC=digitalRead(16);
                          if((BEMFPinC==_BEMFHigh)&&(RunTime>(LastRunCycleTime/2))){
                               MotorPhase++; 
                               LastRunCycleTime=micros()-BEMFTimeStamp;
                               BEMFTimeStamp=micros();
                         }
                  }
                   break; 
             case 2:  
                     if(BEMFPinA==_BEMFLow){
                       RunTime=micros()-BEMFTimeStamp;
                          delayMicroseconds(2);
                          BEMFPinA=digitalRead(14);
                          if((BEMFPinA==_BEMFLow)&&(RunTime>(LastRunCycleTime/2))){
                              MotorPhase++; 
                              LastRunCycleTime=micros()-BEMFTimeStamp;
                              BEMFTimeStamp=micros();                      
                          }
                   }
                  break; 
             case 3:   
                   if(BEMFPinB==_BEMFHigh){
                     RunTime=micros()-BEMFTimeStamp;
                        delayMicroseconds(2);
                       BEMFPinB=digitalRead(15);
                       if((BEMFPinB==_BEMFHigh)&&(RunTime>(LastRunCycleTime/2))){
                           MotorPhase++;
                           LastRunCycleTime=micros()-BEMFTimeStamp;
                           BEMFTimeStamp=micros();
                      }
                  }
                   break;  
             case 4:             
                   if(BEMFPinC==_BEMFLow){
                     RunTime=micros()-BEMFTimeStamp;
                         delayMicroseconds(2);
                          BEMFPinC=digitalRead(16);
                          if((BEMFPinC==_BEMFLow)&&(RunTime>(LastRunCycleTime/2))){
                                MotorPhase++; 
                                LastRunCycleTime=micros()-BEMFTimeStamp;
                               BEMFTimeStamp=micros();
                           }
                    }
                   break; 
             case 5:   
                        if(BEMFPinA==_BEMFHigh){
                           RunTime=micros()-BEMFTimeStamp;
                           delayMicroseconds(2);
                           BEMFPinA=digitalRead(14);
                           if((BEMFPinA==_BEMFHigh)&&(RunTime>(LastRunCycleTime/2))){
                                 MotorPhase++;
                                 LastRunCycleTime=micros()-BEMFTimeStamp;
                                 BEMFTimeStamp=micros();
                            }
                      }
                   break; 
             case 6: 
                   if(BEMFPinB==_BEMFLow){
                         RunTime=micros()-BEMFTimeStamp;
                         delayMicroseconds(2);
                          BEMFPinB=digitalRead(15);
                          if((BEMFPinB==_BEMFLow)&&(RunTime>(LastRunCycleTime/2))){
                                MotorPhase=1; 
                                LastRunCycleTime=micros()-BEMFTimeStamp;
                                BEMFTimeStamp=micros();
                              }
                        }
                   break;  
             default: MosPin_STOP;
       } 
             
}
/*

void SenseBEMF(){
       BEMFPinA=digitalRead(14);
       BEMFPinB=digitalRead(15);
       BEMFPinC=digitalRead(16);
       HallValue= digitalRead(14)*4+(int)digitalRead(15)*2+digitalRead(16);
      
       switch(MotorPhase){
             case 1:   //101
           
                  if(sense==1){
                      if(BEMFPinC==_BEMFLow){
                          delayMicroseconds(1);
                          BEMFPinC=digitalRead(16);
                          if(BEMFPinC==_BEMFLow){ LastRunCycleTime=(micros()-BEMFTimeStamp)/3;BEMFTimeStamp=micros();sense=0;}}
                          else{
                            //nothing
                          }
                  }else{
                      if(micros()-BEMFTimeStamp>LastRunCycleTime){
                        MotorPhase++; sense=1;
                        }
                  }
               
                   break; 
             case 2:   //100
               
                    if(sense==1){
                        if(BEMFPinB==_BEMFHigh){
                         delayMicroseconds(1);
                          BEMFPinB=digitalRead(15);
                          if(BEMFPinB==_BEMFHigh){LastRunCycleTime=(micros()-BEMFTimeStamp)/3;BEMFTimeStamp=micros();sense=0;}}
                      else{
                      }
                      
                  }else{
                      if(micros()-BEMFTimeStamp>LastRunCycleTime){
                          MotorPhase++; sense=1;
                      }
                  }
                  break; 
             case 3:  //110
             
                   if(sense==1){
                         
                   if(BEMFPinA==_BEMFLow){
                           delayMicroseconds(1);
                          BEMFPinA=digitalRead(14);
                          if(BEMFPinA==_BEMFLow){LastRunCycleTime=(micros()-BEMFTimeStamp)/3;BEMFTimeStamp=micros();sense=0;RunTest++;}}
                    else{

                    }
                  }else{
                   if(micros()-BEMFTimeStamp>LastRunCycleTime){
                        MotorPhase++; sense=1;
                      }
                  }
                   break;  
             case 4:   //010
          
                     if(sense==1){                        
                        if(BEMFPinC==_BEMFHigh){
                           delayMicroseconds(1);
                          BEMFPinC=digitalRead(16);
                          if(BEMFPinC==_BEMFHigh){LastRunCycleTime=(micros()-BEMFTimeStamp)/3;BEMFTimeStamp=micros();sense=0;}}
                      else{
                      }
                  }else{
                    if(micros()-BEMFTimeStamp>LastRunCycleTime){
                        MotorPhase++; sense=1;
                      }
                    }
                   break; 
             case 5:   //011
               
                     if(sense==1){
                                            
                        if(BEMFPinB==_BEMFLow){
                           delayMicroseconds(1);
                           BEMFPinB=digitalRead(15);
                          if(BEMFPinB==_BEMFLow){LastRunCycleTime=(micros()-BEMFTimeStamp)/3;BEMFTimeStamp=micros();sense=0;}}
                      else{
                      }
                  }else{
                      if(micros()-BEMFTimeStamp>LastRunCycleTime){
                        MotorPhase++; sense=1;
                        }
                  }
                   break; 
             case 6: //001
               
                     if(sense==1){
                         if(BEMFPinA==_BEMFHigh){
                           delayMicroseconds(1);
                          BEMFPinA=digitalRead(14);
                          if(BEMFPinA==_BEMFHigh){LastRunCycleTime=(micros()-BEMFTimeStamp)/3;BEMFTimeStamp=micros();sense=0;}}
                      else{

                      }
                  }else{
                     if(micros()-BEMFTimeStamp>LastRunCycleTime){
                        MotorPhase=1; sense=1;
                        }
                  }
                   break;  
             default: MosPin_STOP;
       } 
             
}
*/
void MotorCW(){
        switch(HallValue){
             case 5:   //101
                   MosPin_RUN1; 
                   break; 
             case 4:   //100
                  MosPin_RUN2; 
                  break; 
             case 6:  //110
                   MosPin_RUN3; 
                   break;  
             case 2:   //010
                   MosPin_RUN4; 
                   break; 
             case 3:   //011
                   MosPin_RUN5; 
                   break; 
             case 1: //001
                   MosPin_RUN6; 
                   break;  
             default: MosPin_STOP;
        } 
}
/*
//Same special motor need this sequence, 有些电机比较特殊，需要用这个反向表
void MotorCCW(){
        switch(HallValue){
              case 5:   //101
                   MosPin_RUN3; 
                   break; 
             case 4:   //100
                  MosPin_RUN4; 
                  break; 
             case 6:  //110
                   MosPin_RUN5; 
                   break;  
             case 2:   //010
                   MosPin_RUN6; 
                   break; 
             case 3:   //011
                   MosPin_RUN1; 
                   break; 
             case 1: //001
                   MosPin_RUN2; 
                   break;  
             default: MosPin_STOP;
        } 
}

*/

void MotorCCW(){
        switch(HallValue){
              case 5:   //101
                   MosPin_RUN4; 
                   break; 
             case 4:   //100
                  MosPin_RUN5; 
                  break; 
             case 6:  //110
                   MosPin_RUN6; 
                   break;  
             case 2:   //010
                   MosPin_RUN1; 
                   break; 
             case 3:   //011
                   MosPin_RUN2; 
                   break; 
             case 1: //001
                   MosPin_RUN3; 
                   break;  
             default: MosPin_STOP;
        } 
}

 
void CheckInputPin(){
     RunPin=digitalRead(19);
     DirPin=digitalRead(18);
    //RunPin Check;
    if(RunEnable!=RunPin){
       RunPinChangeCount++;
       if(RunPinChangeCount>_KeyMaxCount){ 
           RunEnable=RunPin;
           if(RunEnable==1)Error=0;   //复位错误
           RunPinChangeCount=0;
           
           }
    }
    else{
      RunPinChangeCount=0;
    }
    
    //DirPin Check
    if(DirStatus!=DirPin){
       DirPinChangeCount++;
       if(DirPinChangeCount>_KeyMaxCount){ DirStatus=DirPin;DirPinChangeCount=0;}
    }
    else{
       DirPinChangeCount=0;
    }
  
}

void FlashError(int Err){
     int TempDiff;
     if(FlashErr<(Err+1)){
          if(FlashBegan==1){ThisFlashTime=millis();FlashBegan=0;}
          TempDiff=millis()-ThisFlashTime;
          if(TempDiff<100)digitalWrite(5,1);
          if(TempDiff>100&&TempDiff<500)digitalWrite(5,0);
          if(TempDiff>500){FlashBegan=1;FlashErr++;}
     }
     else{
          if(FlashBegan==1){ThisFlashTime=millis();FlashBegan=0;}
           TempDiff=millis()-ThisFlashTime;
           if(TempDiff<1000){digitalWrite(5,0);}else{FlashErr=0;Serial.println(Error);Serial.println(ErrIshunt);}
     }
       
}

 
void CurrentCheck(){
  
   //read the Current 
    Ishunt=analogRead(A6);
    SubIshunt=SubIshunt-AvgIshunt+Ishunt;
    AvgIshunt=SubIshunt/5;
      //Startup Current
    if(AvgIshunt>_MaxUpIshunt&&RunStatus==1){Error=5;MosPin_STOP;ErrIshunt=AvgIshunt;}
     //Normal Current
    if(AvgIshunt>_MaxIshunt&&RunStatus>1){
          CountErr++;
         if(CountErr>_MaxIshuntTime){Error=6;MosPin_STOP;ErrIshunt=AvgIshunt;}
      } 
     else{
          CountErr--;
          if(CountErr<0)CountErr=0;
     }
     //The  peak current.
    if(Ishunt>_MaxMaxIshunt){Error=7;MosPin_STOP;ErrIshunt=Ishunt;}
    
    if(digitalRead(2)==0){
       Error=1;     //硬件过流保护
    }
}  

void readEnvTemp(){
  
  double Digital_Value=analogRead(A7);   //读取串联电阻上的电压值（数字量）
  double Voltage_Value=(Digital_Value/1023)*5.00;//换算成模拟量的电压值
  double Rt_Value= Voltage_Value*82/(48-Voltage_Value);  //计算出热敏电阻的阻值
  EnvTemperature=(double)1/(log(Rt_Value/10)/3980 + 1/( 25 + 273.15)) - 273.15; //计算所感知的温度并发送
  
}


void CheckMos(){
         pwmWrite(10,100);
        //check UH
         digitalWrite(7,1);digitalWrite(8,0);digitalWrite(9,0); digitalWrite(3,0);digitalWrite(4,0);digitalWrite(6,0);
         delayMicroseconds(5);
          InitIshuntCheck(6);
         
             //check VH
         MosPin_STOP;
         delayMicroseconds(5);
         digitalWrite(7,0);digitalWrite(8,1);digitalWrite(9,0); digitalWrite(3,0);digitalWrite(4,0);digitalWrite(6,0);
         delayMicroseconds(5);
          InitIshuntCheck(6);
         //Check WH
          MosPin_STOP;
         delayMicroseconds(5);
         digitalWrite(7,0);digitalWrite(8,0);digitalWrite(9,1); digitalWrite(3,0);digitalWrite(4,0);digitalWrite(6,0);
         delayMicroseconds(5);
          InitIshuntCheck(6);
         //Check UL
            MosPin_STOP;
         delayMicroseconds(5);
         digitalWrite(7,0);digitalWrite(8,0);digitalWrite(9,0); digitalWrite(3,1);digitalWrite(4,0);digitalWrite(6,0);
         delayMicroseconds(5);
          InitIshuntCheck(6);
         //Check VL
         MosPin_STOP;
         delayMicroseconds(5);
         digitalWrite(7,0);digitalWrite(8,0);digitalWrite(9,0); digitalWrite(3,0);digitalWrite(4,1);digitalWrite(6,0);
         delayMicroseconds(5);
         InitIshuntCheck(6);
         //Check WL
         MosPin_STOP;
         delayMicroseconds(5);
         digitalWrite(7,0);digitalWrite(8,0);digitalWrite(9,0); digitalWrite(3,0);digitalWrite(4,0);digitalWrite(6,1);
         delayMicroseconds(5);
         InitIshuntCheck(6);
         
       //Finish Test Close MosFET.
         MosPin_STOP;
         pwmWrite(10,0);
}
void  InitIshuntCheck(int times){
      for(int i=0;i<times;i++){
            Ishunt=analogRead(A6);
            AvgIshunt=(AvgIshunt+Ishunt)/2;    
         }
         if(AvgIshunt>_MaxIshuntCheck){while(1){MosPin_STOP;FlashError(1);};}
}
  

void  SoundFreq(int ontime, int offtime) {
	int i=0;
	for(i=0; i<ontime; i++)
	    delayMicroseconds(1);
	MosPin_BRK;
	MosPin_STOP;
	for(i=0; i<offtime; i++)
	    delayMicroseconds(1);
       
}


void Sound(int repetitions, int duration, int ontime, int offtime) {
	int i = 0;
	int q= 0;
	for (i=0; i<repetitions; i++)	 {
		for (q=0; q<duration; q++) {
			MosPin_RUN2;  InitIshuntCheck(2);
                        SoundFreq(ontime,offtime);
			MosPin_RUN5;  InitIshuntCheck(2); 
                        SoundFreq(ontime,offtime);
			MosPin_RUN6;  InitIshuntCheck(2); 
                        SoundFreq(ontime,offtime);
			MosPin_RUN3;  InitIshuntCheck(2); 
                        SoundFreq(ontime,offtime);
			MosPin_RUN4;  InitIshuntCheck(2);
                        SoundFreq(ontime,offtime);
			MosPin_RUN1;  InitIshuntCheck(2); 
                        SoundFreq(ontime,offtime);
		}
		delay(30);
		}
}

void MotorSound() {
        pwmWrite(10,20);
	Sound(1, 20, 10, 1200);
	Sound(1, 20, 40, 1200);
	Sound(2, 20, 80, 1200);
        pwmWrite(10,0);
        MosPin_STOP;
}

void ReadPointValue(){
    //read the pointmeter value for control the speed
    ReadPoint=analogRead(A3)+30;  
    if(ReadPoint>1000)ReadPoint=1016;
    if(ReadPoint<50){Error=2;MotorStop();}else{if(Error==2)Error=0;}  //when the pointmeter value <20, The motor will stop.
    TargetSpeed=map(ReadPoint,50,1016,M_MinSpeed,M_MaxSpeed);
    if(TargetSpeed<M_MinSpeed)TargetSpeed=M_MinSpeed;
   if(TargetSpeed>M_MaxSpeed)TargetSpeed=M_MaxSpeed;
}


void MotorRunUp(){
     int i;
     int j;
        if(SensorLess==1){
            //Location the Roller;
            for(i=0;i<_MaxLocationPWM;i++){
                 if(Error==0){
                         NowPwm++;
                         pwmWrite(10,NowPwm/4);
                         MosPin_RUN1;
                         delayMicroseconds(_MaxLocationTime);
                         Ishunt=analogRead(A6);
                         CurrentCheck(); 
                       
                 }else{
                  break;
                 }
             }
             MotorPhase=2; //The Next Phase;
             j=0;
             for(;;){
                   if(Error==0){
                       if(CurrentDirect==1){
                             SensorLessCW();}
                        else{
                             SensorLessCCW();
                        }
                        LastPhase=MotorPhase;
                        if(j>(_MaxMotorUpStep)){
                             LastRunCycleTime=UpDelayTime[j]*_DelayTimes+_UpOffSetDelayTime;
                             RunStatus=2;
                          
                             break;
                         }
                         for(i=0;i<_DelayTimes;i++){
                                delayMicroseconds(UpDelayTime[j]);
                               delayMicroseconds(_UpOffSetDelayTime);
                         }
             
                         MotorPhase++;
                         if(MotorPhase>6){
                              j++;MotorPhase=1;
                              NowPwm++;
                              if(NowPwm>_MaxUpPWM)NowPwm=_MaxUpPWM;
                              pwmWrite(10,NowPwm/4);
                         }      
                        CurrentCheck();   
                        BEMFTimeStamp=micros();   
                }else
                {
                  break;
                }
             }
        }
        else{  //Motor Up by Sensored method.
           if (UpdatePwm>_UpdatePWMTime){
                MotorActualSpeed=(60000000/AvgMotorStepTime)/_MotorPhasePairs;
                NowPwm=NowPwm+_PWMUpStep;          
                if(NowPwm>_MaxUpPWM)NowPwm=_MaxUpPWM;
                pwmWrite(10,NowPwm/4);
                UpdatePwm=0;
            }
             ForceRun();
             RightRun();
             if(CountRun>18){
                  if(RightRunStatus==1){
                      RunStatus=2;                    
                    }
                   else{
                      Error=8;  //Hall sequence wrong,        
                   }
             }
     
             if(NowPwm>_MaxUpPWM)NowPwm=_MaxUpPWM;
             if(CurrentDirect!=DirStatus){RunStatus=3; }
         }
   
}

void ForceRun(){
     HallValue= digitalRead(11)*4+digitalRead(12)*2+digitalRead(13);
     if(HallValue==0||HallValue==7)Error=3;   
     if(CurrentDirect==1){ 
              MotorCCW(); 
          }
      else{ 
               MotorCW();
           }
      CurrentCheck(); 
      LastHallValue=HallValue;
 }

 
void RightRun(){
      switch(HallValue){
           case 1:
              Hall_1=1;
              break;
           case 2:
              Hall_2=1;
              break;
           case 3:
              Hall_3=1;
              break;
           case 4:
              Hall_4=1;
              break;
           case 5:
              Hall_5=1;
              break;
           case 6:
              Hall_6=1;
              break;
           default:
              RightRunStatus=0;
              break;
      }
     RightRunStatus=Hall_1&&Hall_2&&Hall_3&&Hall_4&&Hall_5&&Hall_6;
}

void OpenRun(){
     if (UpdatePwm>_UpdatePWMTime){
              MotorActualSpeed=(60000000/AvgMotorStepTime)/_MotorPhasePairs;
              if(ReadPoint>NowPwm){
                 NowPwm=NowPwm+_PWMUpStep;
              }
              else{
                 NowPwm=NowPwm-_PWMDownStep;
               }
              pwmWrite(10,NowPwm/4);
              UpdatePwm=0;
            if(CountRun>80&&M_LoopRun&&SensorLess==0)RunStatus=5;
       
      }
}


void LoopRun(){
    
    if(DealLoop==1){
          MotorActualSpeed=(60000000/AvgMotorStepTime)/_MotorPhasePairs;    // 获得以RPM 为单位的速度，//MotorParameters[2]是极对数
      
           CEK = M_KP + M_KI + M_KD ;
           CEK1 = 0 - M_KP - 2*M_KD ;
           CEK2 = M_KD ;
           EK2 = EK1 ;
           EK1 = EK ;
           EK = TargetSpeed - MotorActualSpeed;
    
            YK = EK * CEK + EK1 * CEK1 + EK2 *CEK2 ;
            YK=YK/M_Scale;
            if(YK>50)YK=50;
            if(YK<-50)YK=-50;
            NowPwm=NowPwm+YK;

             if(NowPwm <= M_MinMinPWM)
                   NowPwm = M_MinMinPWM;
             else if(NowPwm > M_MaxMaxPWM)
                  NowPwm = M_MaxMaxPWM;
             pwmWrite(10,NowPwm/4);
            DealLoop=0;
   }
}

void MotorStop(){
    NowPwm=0;
    pwmWrite(10,0);
    MosPin_STOP;
    RunStatus=0;
    CountRun=0;
    UpdateStallTime=0;
    if(Error==0)digitalWrite(5,0);
}

void loop(){

    CheckInputPin();
    if(RunEnable==0){
         
          ReadPointValue();
          if(Error==0){ 
               switch(RunStatus){
                     case 0:
                //           setP32( );  
                           HallValue=0;                    
                           HallValue= digitalRead(11)*4+digitalRead(12)*2+digitalRead(13);
                           if(HallValue==0||HallValue==7){
                               SensorLess=1;
                               if(SensorLessSound==1){
                                   pwmWrite(10,60);
                                   Sound(3, 20, 80, 1200);
                                   pwmWrite(10,0);
                                  SensorLessSound=0;
                               }
                           }
                           else{
                                SensorLess=0;
                                SensorLessSound=1;
                           }
                 //          FlexiTimer2::start();
                           CurrentDirect=DirStatus;
                           UpdateStallTime=0;
                           CountRun=0;
                           NowPwm=0;
                           Ishunt=3;  //假设电流采样值
                           SubIshunt=15; //假设电流采样值
                           AvgIshunt=3; //假设电流采样值
                           MotorStepTime=0;
                           AvgMotorStepTime=0;
                           Hall_1=0;Hall_2=0;Hall_3=0;Hall_4=0;Hall_5=0;Hall_6=0;RightRunStatus=0;
                           RunStatus=1;
                           break;
                      case 1:
                           MotorRunUp();
                              
                           break;
                      case 2:
                            OpenRun();
                            if(CurrentDirect!=DirStatus)RunStatus=3;
                            break;
                      case 3:
                     //        FlexiTimer2::stop();
                             NowPwm=0;
                             pwmWrite(10,0);
                             MosPin_STOP;
                             delay(800);  
                             MosPin_BRK;
                             delay(400);
                             MosPin_STOP;
                             delay(100);
                             CurrentDirect=DirStatus;
                             RunStatus=0;  
                             break;
                    case 5:
                           LoopRun();
                           if(CurrentDirect!=DirStatus)RunStatus=3;
                           break;
                     default:
                            Error=9;
                            MosPin_STOP;
                            break;                          
                  }
          }
          else{
             pwmWrite(10,0);
             MosPin_STOP;
             FlashError(Error);  //flash the Error statues
        }
       if(UpdateStallTime>_MaxUpdateStallTime)Error=4;
       if(UpdateStallTime>_MaxSensorLessUpdateStallTime&&SensorLess==1)Error=4;
    }
    else{
       MotorStop();
   }
   CurrentCheck();  
   readEnvTemp();
    //Debug for output data;
    if((millis()-lastrtime)>1000){
       
        Serial.print("ReadPoint");
        Serial.println(ReadPoint);
      //  Serial.print("Ishunt");
       // Serial.println(AvgIshunt);
      //  Serial.print("Cunrrent");
       // float tempi=AvgIshunt*125/256;
     //   Serial.println(tempi);
        lastrtime=millis();
    }
     
}


 
