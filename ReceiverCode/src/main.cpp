
#define              RXVERSIONNUMBER   64.5       // Sept 19th 2021  ****************************************

    //    #define DEBUG     
    //    #define DB_SENSORS          
    //    #define DB_PID          
    //    #define DB_BIND  
    //    #define DB_FAILSAFE
    //    #define  SECOND_TRANSCEIVER
          #define  RX_DELAY                 0  // for new library  and extra tranceiver
          #define  RECEIVE_TIMEOUT         66  //  15 milliseconds was too short
          #define  PacketsPerHop           20  // *********************
          #define  CHANNELSUSED            16 
          #define  SERVOSUSED              10
          #define  SBUSRATE                10  // SBUS frame every 10 milliseconds
          #define  SBUSPORT                Serial3   
                  
      
 bool USE_BMP280             =   false;    //  Pressure BMP280
 bool USE_INA219             =   false;    //  Volts INA219 
 bool USE_BNO055             =   false;    //  Cheap BNO055 gyro
 bool USE_BNO055A            =   false;    //  Adafruit BNO055 gyro
 bool USE_MPU6050            =   false;    //  Gyro MPU6050

        #define EXTRAMICROS 500                      // for extra resolution driving servos
        #define MINMICROS 1000 - EXTRAMICROS     
        #define MAXMICROS 2000 + EXTRAMICROS

        
        
        #define UNCOMPRESSEDWORDS  20                        //   16 Channels plus extra 4 16 BIT values
        #define COMPRESSEDWORDS    UNCOMPRESSEDWORDS * 3 / 4 // = 16 WORDS  with no extra 

//  WORKS ON TEENSY 4.0, LC and 4.1 
//  Detects and uses INA219 to read volts
//  Detects and uses uses MPU6050 gyro
//  Detects and uses BMP280 pressure sensor for altitude
//  Detects and uses BNO055 gyro at 28 (Adafruit) or 29 (Cheapo) hex
//  Add Simple Kalman filter for PID.
//  Gyro Angular velocity used for Quadcopter... a work in progress
//  Binding implemented 
//  SBUS WORKING!
//  Failsafe working (after two seconds) 
//  MODEL MEMORY AUTO SELECTION
//  RESOLUTION INCREASED TO 12 BIT
//  Channels incleased to 16, but only 12 PWM outputs.  SBUS can handle all.

//*********************************
/* *****  TEENSY 4.0 PINS *******
 * 0...7  PWM SERVOS Channels 1 - 8 
 * 8      PWM SERVO Channel 9
 * 9      SPI CE1 
 * 10     SPI CSN1
 * 11     SPI MOSI
 * 12     SPI MISO
 * 13     SPI SCK
 * 14     SBUS output (TX3) 
 * 15     N/A (RX3)
 * 16     PWM Channel 10
 * 18     I2C SDA
 * 19     I2C SCK
 * 20     SPI CSN2     
 * 21     SPI CE2      
 * 22     IRQ1 
 * 23      RQ2 
 */
 //*******************************

#include "SBUS.h"
#include <SPI.h>  
#include <RF24.h>  
#include <Servo.h> 
#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_INA219.h>
#include <BMP280_DEV.h> 
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <MPU6050_tockn.h>
#include <SimpleKalmanFilter.h>
#include <Compress.h>

#define  STICKSRATE               60 * 100    // Max 
#define  MAXCORRECTION            15
#define  DeadBand                  4
#define  KK                       15
 SimpleKalmanFilter RollRateKalman (KK, KK, 0.001);
 SimpleKalmanFilter PitchRateKalman(KK, KK, 0.001);
 SimpleKalmanFilter YawRateKalman  (KK, KK, 0.001);
 uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
 Adafruit_BNO055 BNO055_Cheapo   = Adafruit_BNO055(55,0x29);   // Cheapo version
 Adafruit_BNO055 BNO055_Adafruit = Adafruit_BNO055(55,0x28);   // Adafruit version
 sensors_event_t orientationData, angVelocityData; 
 Adafruit_INA219 ina219;
 BMP280_DEV bmp280;
 MPU6050 mpu6050(Wire);
 Servo MCMServo[SERVOSUSED];
 
 byte PWMPins[SERVOSUSED] = {0,1,2,3,4,5,6,7,8,16};  // ten now


#define  FHSS_RESCUE_BOTTOM      118       
#define  FHSS_RESCUE_TOP         125 


#define  pinCE1                  9   // NRF1
#define  pinCSN1                 10  // NRF1

#define  pinCSN2                 20  // NRF2  
#define  pinCE2                  21  // NRF2   

#define  FAILSAFE_TIMEOUT        2000
#define  LED_PIN                 13

#define  ACKPAYLOADLENGTH        12 

#define  AEROPLANE                1 
#define  HELICOPTER               2
#define  QUADCOPTER               3 
#define  HEXACOPTER               4
#define  OCTOCOPTER               5 

uint64_t ThisPipe = 0xBABE1E5420LL; // default startup
uint64_t NewPipe  = 0;
uint64_t OldPipe  = 0;
int tt;

SBUS MySbus(SBUSPORT);
RF24 Radio1(pinCE1, pinCSN1); 
RF24 Radio2(pinCE2, pinCSN2); 
RF24 CurrentRadio(pinCE1, pinCSN1); 
Compress compress;

char payLoad[ACKPAYLOADLENGTH];  // must have spare space
byte PacketNumber;
byte NextFrequency;

uint16_t ReceivedData[UNCOMPRESSEDWORDS];               //  20  words
uint16_t PreviousData[UNCOMPRESSEDWORDS]; 

bool Connected=false;
float PacketStartTime;
int i;
int LastConnectionMoment = 0;
int SearchStartTime      = 0;
int StillSearchingTime   = 0;
double LastVoltMoment    = 0;

float PitchRate;            // Filtered versions
float RollRate;
float YawRate;
float Pitch;
float Roll;
float Yaw;
float Temperature6050; 

float MeasuredPitchRate;   // Measured versions
float MeasuredRollRate;
float MeasuredYawRate;
float MeasuredPitch;
float MeasuredRoll;
float MeasuredYaw; 

float OldPitchRate=0;      // Previous versions
float OldRollRate=0;
float OldYawRate=0;
float OldPitch=0;
float OldRoll=0;
float OldYaw=0; 

float RollRateIntegral=0;
float OldRollRateIntegral=0;
float RollRateDerivative=0;
float OldRollRateError=0;

float PitchRateIntegral=0;
float OldPitchRateIntegral=0;
float PitchRateDerivative=0;
float OldPitchRateError=0;

float temperature280, pressure, altitude,StartAltitude,CurrentAltitude;           
bool  Swash_DisplayStarted     = false;
char  ModelAltitude[12]="N/A";
char  ModelRoll[12]="N/A";
char  ModelPitch[12]="N/A";
char  ModelYaw[12] ="N/A";
char  volt[12]="N/A";
float  Swash_P=0;
float  Swash_I=0;
float  Swash_D=0;
float  yawp=0;
float  yawi=0;
float  yawd=0;
byte  ModelType = 0; // 1=Aeroplane 2=Heli 3=Quadcopter 4=Hexacopter 5=Octocopter
byte  Throttle[8];
float TargetYawRate = 0;
float YawError = 0;
float YawRateCorrection=0;
float TargetRollRate = 0;
float TargetPitchRate = 0;
float RollRateCorrection = 0;
float PitchRateCorrection = 0;
int   LoopsPS=0;
int   TimeThis=0;
int   MainLoopTime=0;
long int   DeltaTime=0;
byte  BindNow = 0;
bool  BoundFlag = false;
byte  SavedPipeAddress[8]; 
int   BindOKTimer=0;
bool  SaveNewBind =true;
bool  ServosAttached = false;
float rxv  = RXVERSIONNUMBER;
uint16_t   SbusChannels[CHANNELSUSED+8] ;  // a few spare
int   SBUSTimer = 0;
bool  FailSaveSafe = false;
bool  FailSafeChannel[17];
bool  FailSafeDataLoaded = false;
byte  ModelNumber=0;
bool  ModelNumberSaved=false;
byte  PowerSetting=4;
byte  DataRate=1;
bool  ReInit=false;
unsigned int   RX_TimeOut =RECEIVE_TIMEOUT; // 50 MS by default
bool FailSafeSent = false;
uint8_t  byte1=0;
uint8_t  byte2=0;
byte  reconnect_attempts=0;
byte  Rnumber = 1;

// ************************************************************************************************************
// ************************************************************************************************************

byte EEPROMReadByte(int p_address){
     byte bt = EEPROM.read(p_address);
     return bt;
}
void LoadFailSafeData(){
   byte FS_Offset=10;
 uint16_t s[CHANNELSUSED];                                
 
     for (i=0;i<CHANNELSUSED;i++){
           s[i]=map(EEPROMReadByte(i+FS_Offset),0,180,MINMICROS,MAXMICROS); // load failsafe values and simulate better resloution
     } 
       FS_Offset+=CHANNELSUSED; 
       for (i=0;i<CHANNELSUSED;i++){
              if (EEPROMReadByte(i+FS_Offset)) {ReceivedData[i] = s[i];} 
     } 
    FailSafeDataLoaded=true;
    #ifdef  DB_FAILSAFE
    Serial.println ("Fail safe settings are loaded!");  
    #endif
}
void MapToSBUS(){

int RangeMax  = 2047; // = Frsky at 150 %
int RangeMin  = 0;

                      for (int jj=0;jj<CHANNELSUSED;jj++) {SbusChannels[jj] = map(ReceivedData[jj],MINMICROS,MAXMICROS,RangeMin,RangeMax);}                       
}  

float DoDeadBand(float tr){
          if (tr<=DeadBand && tr >=-DeadBand) tr=0;         
          if (tr>DeadBand) tr-=DeadBand;
          if (tr<  (-DeadBand)) tr+=DeadBand;
          return tr;
}

void StabilizeYawRate(){
                 YawError           =  (YawRate-TargetYawRate); 
                 YawRateCorrection  =  (YawError * yawp);           // P  
}

float GetRollRateIntegral(float rre){      
          RollRateIntegral=(rre*DeltaTime)+OldRollRateIntegral;      
          OldRollRateIntegral=RollRateIntegral;
          return RollRateIntegral;
}

float GetPitchRateIntegral(float rre){      
          PitchRateIntegral=(rre*DeltaTime)+OldPitchRateIntegral;      
          OldPitchRateIntegral=PitchRateIntegral;
          return PitchRateIntegral;
}

float GetRollRateDerivative(float rre){
       RollRateDerivative = (rre-OldRollRateError)/DeltaTime;
       OldRollRateError=rre; 
       return RollRateDerivative;
}

float GetPitchRateDerivative(float rre){
       PitchRateDerivative = (rre-OldPitchRateError)/DeltaTime;
       OldPitchRateError=rre; 
       return PitchRateDerivative;
}


void  StabilizeRollRate(){  
float RollRateError = 0;
float I;
float D;
              RollRateError       =  RollRate-TargetRollRate; 
              RollRateCorrection  =  RollRateError * Swash_P;                            // P                      
              I = GetRollRateIntegral(RollRateError);
              RollRateCorrection +=  I * Swash_I;                                        // I
              D = GetRollRateDerivative(RollRateError);
              RollRateCorrection +=  D * Swash_D;                                        // D      
              RollRateCorrection= constrain(RollRateCorrection,-MAXCORRECTION,MAXCORRECTION);
}

void StabilizePitchRate(){
float PitchRateError = 0;
float I;
float D;                     
              PitchRateError       =   (PitchRate-TargetPitchRate); 
              PitchRateCorrection  =    PitchRateError * Swash_P;                          // P                      
              I = GetPitchRateIntegral(PitchRateError);
              PitchRateCorrection +=  I * Swash_I;                                         // I
              D = GetPitchRateDerivative(PitchRateError);
              PitchRateCorrection +=  D * Swash_D;                                         // D      
              PitchRateCorrection= constrain(PitchRateCorrection,-MAXCORRECTION,MAXCORRECTION);
}

void ShowPid(){
 
 #ifdef DB_PID // *********************************************
           LoopsPS++;   
           if(millis()-TimeThis>=1000){
                           // Serial.print ("LoopsPS = ");
                           // Serial.println(LoopsPS);   
                            Serial.print ("Swash_P: ");
                            Serial.println (Swash_P);
                            Serial.print ("Swash_I: ");
                            Serial.println (Swash_I,10);
                            Serial.print ("Swash_D: ");
                            Serial.println (Swash_D);
                            Serial.print ("RollRate: ");
                            Serial.println (RollRate);
                            Serial.print ("DeltaTime: ");
                            Serial.println (DeltaTime);
                            Serial.println (" "); 
                            TimeThis=millis();
                            LoopsPS = 0;    
                   }
  #endif   // ***************************************************


}

void  MoveServos(){
   int j=0;
   int k = 0;
          DeltaTime=micros()-DeltaTime;   
          if (millis()-SBUSTimer>=SBUSRATE){   
            SBUSTimer=millis();              // timer starts before send starts....
            MySbus.write(&SbusChannels[0]);
          }         
           if (1){    //(ModelType==AEROPLANE ){                         // !! fix Later ***************************************
            for (j=0;j<SERVOSUSED;j++) {
              if (PreviousData[j] != ReceivedData[j]) {
                       MCMServo[j].writeMicroseconds(ReceivedData[j]);   
                       PreviousData[j] = ReceivedData[j];              
              } 
          }
          return;                                                       //  !! remove  later ***************************************
       }
       if (ModelType==HELICOPTER){
           Serial.println ("HELICOPTER!"); 
       }
   // ************************** QUADZONE **************************************       
       if (ModelType==QUADCOPTER){    
          
          TargetRollRate  = (map((ReceivedData[0]),0,180,-STICKSRATE,STICKSRATE)) /100;      // get Aileron stick 
          TargetRollRate  = DoDeadBand(TargetRollRate);
          TargetPitchRate = (map((ReceivedData[1]),0,180,-STICKSRATE,STICKSRATE))/100;       // get elevator stick 
          TargetPitchRate = DoDeadBand(TargetPitchRate);
          TargetYawRate   = (map((ReceivedData[3]),0,180,-STICKSRATE,STICKSRATE))/100;       // get Rudder stick 
          TargetYawRate   = DoDeadBand(TargetYawRate);
               
          for (k=0;k<=3;k++){Throttle[k] = ReceivedData[2];}                                 // get throttle stick 
          
          StabilizeRollRate();
          StabilizePitchRate();
          StabilizeYawRate();

          ShowPid();

          Throttle[0] -= YawRateCorrection;
          Throttle[1] += YawRateCorrection;
          Throttle[2] -= YawRateCorrection;
          Throttle[3] += YawRateCorrection;   
  
          Throttle[0] += RollRateCorrection;
          Throttle[1] += RollRateCorrection;
          Throttle[2] -= RollRateCorrection;
          Throttle[3] -= RollRateCorrection;
             
          Throttle[0] -= PitchRateCorrection;
          Throttle[1] += PitchRateCorrection;
          Throttle[2] += PitchRateCorrection;
          Throttle[3] -= PitchRateCorrection;
          
          Throttle[0] = constrain(Throttle[0],20,180);  
          Throttle[1] = constrain(Throttle[1],20,180);
          Throttle[2] = constrain(Throttle[2],20,180);
          Throttle[3] = constrain(Throttle[3],20,180);
          
          if (ReceivedData[2]<15){                                    //  no throttle on very low throttle stick input    
              for (k=0;k<=3;k++){
                Throttle[k] = 2;
                } 
              OldRollRateError=0;
              OldPitchRateError=0;
              }                  
          for (k=0;k<=3;k++){MCMServo[k].write(Throttle[k]);}        //  set four throttles
 }

  // ******************************************************************************     
       if (ModelType==HEXACOPTER){
           Serial.println ("HEXACOPTER!");
       }
       if (ModelType==OCTOCOPTER){
           Serial.println ("OCTOCOPTER!");
      }
}

void FailSafe(){

     if(BoundFlag){          
          if (!FailSafeDataLoaded) {LoadFailSafeData();}
          if (!FailSafeSent){
                    MapToSBUS();
                    MoveServos(); 
                    #ifdef DB_FAILSAFE
                              Serial.println ("FAILSAFE SENT");
                    #endif   
                    FailSafeSent=true;
          }
                
     }

}              



void ShowHopDurationEtc(){
float OnePacketTime=0;
                 OnePacketTime=(millis()-PacketStartTime)/PacketsPerHop;
                 Serial.print ("Hop duration: ");
                 Serial.print ((millis()-PacketStartTime)/1000);
                 Serial.print ("s  Packets per hop: ");
                 Serial.print (PacketsPerHop);
                 Serial.print ("  Average Time per packet: ");
                 Serial.print (OnePacketTime);
                 Serial.print ("ms  Next channel: ");
                 Serial.println(NextFrequency);
                 PacketStartTime=millis();
}


void HopToNextFrequency(){ 

                 CurrentRadio.stopListening();
                 CurrentRadio.setChannel(NextFrequency);
                 CurrentRadio.startListening();
                 LastConnectionMoment=millis(); 
                 #ifdef DEBUG
                   ShowHopDurationEtc();     
                 #endif     
               
}


void InitCurrentRadio(){
  CurrentRadio.begin();    
  CurrentRadio.setAutoAck(1);           
  CurrentRadio.enableAckPayload();                 
  CurrentRadio.maskIRQ(1,1,1);          // no interrupts      
  CurrentRadio.enableDynamicPayloads(); 
  CurrentRadio.setAddressWidth(4);  
  CurrentRadio.setCRCLength(RF24_CRC_8); // could be 16 or disabled
  

  switch (PowerSetting){
      case 1:
            CurrentRadio.setPALevel(RF24_PA_MIN);
            break;
      case 2:
            CurrentRadio.setPALevel(RF24_PA_LOW);
            break;
      case 3:
            CurrentRadio.setPALevel(RF24_PA_HIGH);
            break;
      case 4:
            CurrentRadio.setPALevel(RF24_PA_MAX);
            break;
      default:
            break;
  }
  switch (DataRate){
      case 1:
          CurrentRadio.setDataRate(RF24_250KBPS);     
          break;
      case 2:
          CurrentRadio.setDataRate(RF24_1MBPS);
          break;
      case 3:
          CurrentRadio.setDataRate(RF24_2MBPS);
          break;
      default:
          break;
  }
  
  CurrentRadio.openReadingPipe(1,ThisPipe);  
  SaveNewBind=true;                       
  CurrentRadio.startListening();

}



void Reconnect(){   
            SearchStartTime = millis();
            reconnect_attempts++;
            if (reconnect_attempts>0){
             reconnect_attempts=0;
             CurrentRadio.stopListening();
             if (Rnumber==1){
                  Rnumber=2;
                  CurrentRadio=Radio2;
             }else{
                  Rnumber=1;
                  CurrentRadio=Radio1;
             }
            InitCurrentRadio();
            }
             #ifdef DEBUG 
                 Serial.print ("Reconnection attempt: ");
                 Serial.print (reconnect_attempts);
                 Serial.print ("  Radio: ");
                 Serial.println (Rnumber);
            #endif
            
            while (!Connected){
              StillSearchingTime = millis() - SearchStartTime;
              
              #ifdef DEBUG  
                  Serial.print  (millis());
                  Serial.print  ("  Pipe=");
                  Serial.print (abs(int(ThisPipe)));
                  Serial.print  ("  ");
                  Serial.println ("Searching (Reconnecting)...");  
              #endif
                 
              i=FHSS_RESCUE_BOTTOM;
              while(!CurrentRadio.available() && i <= FHSS_RESCUE_TOP) { // Search for frequency
                        CurrentRadio.stopListening(); 
                        CurrentRadio.setChannel(i);              
                        CurrentRadio.startListening();
                        delay(4);        
                        i++; 
              }
            
              if(CurrentRadio.available()) {
                   Connected=true; FailSafeSent=false;
                   reconnect_attempts=0;
                     #ifdef DEBUG
                       Serial.println ("*****************************************************************************************************************");  
                    #endif  
               }       
              else if (StillSearchingTime >= FAILSAFE_TIMEOUT) {FailSafe();}
       }                  
}




void LoadExtraPayload(){
char a[] = "A";               // Altitude
char v[] = "v";               // Volts
char r[] = "R";               // Roll
char p[] = "P";               // Pitch
char y[] = "Y";               // Yaw
char V[] = "V";               // Software version
char M[] = "M";               // Model Number
char D[] = "D";               // Data rate changed

char vbuf[12];

 switch (PacketNumber){
               case 5: 
                  strcpy(payLoad,y);          
                  strcat(payLoad,ModelYaw);
                  break;
              
              case 6: 
                  strcpy(payLoad,p);          
                  strcat(payLoad,ModelPitch);
                  break;
              
               case 7: 
                  strcpy(payLoad,a);          
                  strcat(payLoad,ModelAltitude);
                  break;
              
              case 8: 
                  strcpy(payLoad,r);          
                  strcat(payLoad,ModelRoll);
                  break;
                             
              case 9:  
                    strcpy(payLoad,v);
                    strcat(payLoad,volt);      //rx volts
                    break;        
             case 10:                          // ... 42                    
                    dtostrf(rxv, 2, 2, vbuf);  // rx software version number
                    strcpy(payLoad,V);
                    strcat(payLoad,vbuf); 
                    break; 
             case 11:  
                    strcpy(payLoad,M);
                    payLoad[1]= ModelNumber;  
                    break; 
             case 28:  
                    strcpy(payLoad,D);
                    payLoad[1]= ReInit ; 
                    ReInit=false;
                    break; 
               

                    
            default:
            break;
             
 }
}

bool ReadData(){
uint16_t CompressedData[COMPRESSEDWORDS] ;   // 30 bytes -> 40 bytes when uncompressed
                       Connected=false;
                       delay(RX_DELAY);
                       if (CurrentRadio.available()){
                             LoadExtraPayload();
                             Connected=true;
                             LastConnectionMoment=millis();  
                             CurrentRadio.writeAckPayload(1,&payLoad,ACKPAYLOADLENGTH);               // Send telemetry (actual length plus 0)  
                             CurrentRadio.read(&CompressedData, sizeof(CompressedData));              // Get Data
                             compress.DeComp(ReceivedData,CompressedData,UNCOMPRESSEDWORDS);   // my library to decompress ! 
                             FailSafeDataLoaded=false;
                             MapToSBUS();      
                       }        
                    return Connected;    
}

void EEPROMUpdateByte(int p_address, byte p_value){
     EEPROM.update(p_address, p_value);
}

// ***********************************************************************************
// ModelNumber occupies EEPROM at offset 28 ******************************************

void SaveModelNumber(){
        EEPROMUpdateByte(28,ModelNumber); 
}
void   LoadModelNumber(){
        ModelNumber=EEPROMReadByte(28);
}

void AttachServos(){
    if  (!ServosAttached) {for (i=0;i<SERVOSUSED;i++) {MCMServo[i].attach(PWMPins[i]);}ServosAttached=true; }     // now 12 SbusChannels, DEFAULT TRAVEL
    MySbus.begin();
}

void BindModel(){
  ThisPipe=NewPipe;
  if (SaveNewBind){
      for (i=0;i<8;i++){
        EEPROMUpdateByte(i,ReceivedData[i]);
      }
    }
  CurrentRadio.stopListening();
  InitCurrentRadio();
  BoundFlag=true; 
  BindNow=0;
  SaveNewBind=false;
  AttachServos();    // AND SBUS!!!  
      #ifdef DB_BIND
      Serial.println ("BINDING NOW");
      #endif 
}

void RebuildFlags(bool *f, uint16_t tb){  // Pass arraypointer and the two bytes to be decoded
        for (i=0;i<16;i++){   
          f[15-i]=false;                     // false is default
          if (tb & 1<<i) f[15-i]=true;       // sets true if bit was on
       }                         
}


void CheckParams(){

byte mn=0;  
uint16_t TwoBytes=0;

      PacketNumber  = (ReceivedData[CHANNELSUSED+1]);
      NextFrequency = (ReceivedData[CHANNELSUSED+2]);   
                
               
        switch(PacketNumber){
              case 3:
                    Swash_P  = ReceivedData[CHANNELSUSED+3];
                    Swash_P /= 200;
                    break;
              case 4:
                    Swash_I  = ReceivedData[CHANNELSUSED+3];
                    Swash_I /= 1000000000;
                    break;
              case 5:
                    Swash_D  =  ReceivedData[CHANNELSUSED+3];
                    Swash_D *=  900;
                    break;
              case  6:
                    if (BoundFlag){
                          mn=byte(ReceivedData[CHANNELSUSED+3]);
                          if (mn!=ModelNumber && mn>0){
                               ModelNumber=mn;
                               SaveModelNumber();
                           }
                    }        
                    break; 
              case 7:
                    PowerSetting=ReceivedData[CHANNELSUSED+3] ;
                    break;
              case 8:
                    DataRate=ReceivedData[CHANNELSUSED+3] ;
                    break;
              case 9:
                    yawp=ReceivedData[CHANNELSUSED+3] ;
                    yawp/=100;
                    break;
              case 10:
                    yawi=ReceivedData[CHANNELSUSED+3];
                    break;
              case 11:
                    yawd=ReceivedData[CHANNELSUSED+3];
                    break;
              case 12:
                    ModelType=ReceivedData[CHANNELSUSED+3];
                    break;   
               case 13:
                    BindNow=ReceivedData[CHANNELSUSED+3];
                    break; 
               case 14:
                    byte1=ReceivedData[CHANNELSUSED+3]; // These bytes are failsafe flags
                    break;
               case 15:
                    byte2=ReceivedData[CHANNELSUSED+3]; // These bytes are failsafe flags 
                    break;                   
               case 16:
                    FailSaveSafe=bool(ReceivedData[CHANNELSUSED+3]);
                    if(FailSaveSafe){
                              TwoBytes=uint16_t(byte2)+uint16_t(byte1<<8); 
                             // Serial.println (TwoBytes,BIN);
                              RebuildFlags(FailSafeChannel,TwoBytes);
                    }
                    break; 
               case 17:
                    ReInit=bool(ReceivedData[CHANNELSUSED+3]);  // must reinitialise the port if changed settings
                    if (ReInit) {InitCurrentRadio();}
                    break;
               default:
                    break;     // 
           }                
}


#ifdef DB_SENSORS 
void Sensors_Status(){   

  
        if ( USE_BNO055 || USE_BNO055A){
            Serial.print("Pitch=");
            Serial.print(int(Pitch));
            Serial.print(" Roll=");
            Serial.print(int(Roll));
            Serial.print(" Yaw=");
            Serial.print(int(Yaw)); 
            Serial.print("    PitchRate=");
            Serial.print(int(PitchRate));
            Serial.print(" RollRate=");
            Serial.print(int(RollRate));
            Serial.print(" YawRate=");
            Serial.print(int(YawRate));           
           }   
           if (USE_INA219){
             Serial.print("     Volts=");
             Serial.print(volt);
          }
         if (USE_BMP280){
            Serial.print("  Altitude=");
            Serial.print(int(CurrentAltitude*3.28084)); // convert from meters
            Serial.print(" Temp=");
            Serial.print(int(temperature280));
            
         }
         
         Serial.println (" ");
}
#endif
        
// ******************************************************
 




void DoSensors(){    

        if (USE_BMP280) {
                if (bmp280.getMeasurements(temperature280, pressure, altitude)) 
                 CurrentAltitude=int(altitude-StartAltitude);  
                 dtostrf(CurrentAltitude, 0, 0, ModelAltitude);
        }
        if ( USE_BNO055 || USE_BNO055A || USE_MPU6050){
       
                 dtostrf(Roll, 0, 0, ModelRoll);
                 dtostrf(Pitch, 0, 0, ModelPitch);
                 dtostrf(Yaw, 0, 0, ModelYaw);
        }

     if (USE_INA219) {dtostrf(ina219.getBusVoltage_V(), 1, 1, volt);}
        
          #ifdef DB_SENSORS 
            Sensors_Status();     // look if interested
           #endif
        
        if ((millis()-LastVoltMoment) > 1000){    
        LastVoltMoment  = millis();     
        
  }
}

FASTRUN void ReceiveData(){ 
     if (!Connected) if(millis()-LastConnectionMoment>=RX_TimeOut) Reconnect();
     if (ReadData()){CheckParams();
     if (PacketNumber >= PacketsPerHop){HopToNextFrequency();DoSensors();}} 
} 

FASTRUN void KalmanFilter(){
     RollRate  = RollRateKalman.updateEstimate (RollRate); 
     PitchRate = PitchRateKalman.updateEstimate(PitchRate);
     YawRate   = YawRateKalman.updateEstimate  (YawRate);
}
void ReadBNOValues(){
  Yaw   = orientationData.orientation.x;
  Roll  = orientationData.orientation.y;
  Pitch = orientationData.orientation.z;  
  PitchRate = - angVelocityData.gyro.x;
  RollRate  = - angVelocityData.gyro.y;
  YawRate   = - angVelocityData.gyro.z;
  KalmanFilter();
}

void Get_Mpu6050(){
      mpu6050.update();          
      PitchRate = mpu6050.getGyroX();
      RollRate  = mpu6050.getGyroY();
      YawRate   = mpu6050.getGyroZ();  
      Pitch     = mpu6050.getAngleX();
      Roll      = mpu6050.getAngleY();
      Yaw       = mpu6050.getAngleZ();
      KalmanFilter();
}


void Get_BNO055_Cheapo(){
  BNO055_Cheapo.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  BNO055_Cheapo.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  ReadBNOValues();
}

void Get_BNO055_Adafruit(){
  BNO055_Adafruit.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  BNO055_Adafruit.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  ReadBNOValues();
}


void ReadSavedPipe(){
    for (i=0;i<8;i++){
        SavedPipeAddress[i]=EEPROMReadByte(i); // uses first 8 bytes only.
    }
  
}

void GetOldPipe(){
  ReadSavedPipe();
  OldPipe =(uint64_t)SavedPipeAddress[0]<<56;
  OldPipe+=(uint64_t)SavedPipeAddress[1]<<48;
  OldPipe+=(uint64_t)SavedPipeAddress[2]<<40;
  OldPipe+=(uint64_t)SavedPipeAddress[3]<<32;
  OldPipe+=(uint64_t)SavedPipeAddress[4]<<24;
  OldPipe+=(uint64_t)SavedPipeAddress[5]<<16;  
  OldPipe+=(uint64_t)SavedPipeAddress[6]<<8;
  OldPipe+=(uint64_t)SavedPipeAddress[7];
  
}

void ScanI2c() {
            delay(500);                                     // allow time to wake things up
            USE_MPU6050 = false;
            USE_BMP280  = false;
            USE_INA219  = false;   
            USE_BNO055  = false;   
            USE_BNO055A = false;  
            for (i=1;i<127;i++){
                Wire.beginTransmission(i);
                if(Wire.endTransmission()==0){
                      #ifdef DB_SENSORS
                              Serial.print (i,HEX); // in case new one shows up
                              Serial.print("   ");   
                               if (i==0x68) Serial.println("MPU6050 gyro detected!");  
                               if (i==0x76) Serial.println("BMP280 barometer detected!");  
                               if (i==0x40) Serial.println("INA219 voltage meter detected!");  
                               if (i==0x29) Serial.println("Cheapo BNO055 gyro clone detected!"); 
                               if (i==0x28) Serial.println("Real Adafruit BNO055 gyro detected!"); 
                      #endif
                     
                      if (i==0x28) {USE_BNO055A=true;} 
                      if (i==0x29) {USE_BNO055=true;}   
                      if (i==0x40) {USE_INA219=true;} 
                      if (i==0x68) {USE_MPU6050=true;} 
                      if (i==0x76) {USE_BMP280=true;}               
                  }    
         }
}


void GetNewPipe(){
  NewPipe =(uint64_t)ReceivedData[0]<<56;
  NewPipe+=(uint64_t)ReceivedData[1]<<48;
  NewPipe+=(uint64_t)ReceivedData[2]<<40;
  NewPipe+=(uint64_t)ReceivedData[3]<<32;
  NewPipe+=(uint64_t)ReceivedData[4]<<24;
  NewPipe+=(uint64_t)ReceivedData[5]<<16;  
  NewPipe+=(uint64_t)ReceivedData[6]<<8;
  NewPipe+=(uint64_t)ReceivedData[7];
}



// ***************************************************************************************************
// **************************************** SETUP ****************************************************
// ***************************************************************************************************

void setup() {                                       
  pinMode(LED_PIN,OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  delay(2000);  // Needed ! - possibly for stabilising capacitors.  
  Serial.begin(9600);
  Wire.begin();
  ScanI2c();                                       // see what's connected               
  if (USE_BNO055) BNO055_Cheapo.begin();
  if (USE_BNO055A) BNO055_Adafruit.begin();
  SPI.begin();
  SPI.setClockDivider (SPI_CLOCK_DIV16);           // for Teensy - Slow SPI bus
  
  
  CurrentRadio = Radio1;
  InitCurrentRadio();
  
 #ifdef  SECOND_TRANSCEIVER
      CurrentRadio = Radio2;
      InitCurrentRadio();
#endif
#ifndef  SECOND_TRANSCEIVER
   Radio2=Radio1;
#endif

  if (USE_BMP280) {
          #ifdef DB_SENSORS
              Serial.print("Starting BMP280 ... ");
          #endif
           
            bmp280.begin(0x76);
            bmp280.setTimeStandby(TIME_STANDBY_4000MS);     
            bmp280.startNormalConversion();              
            for (i=0;i<15000;i++) {bmp280.getMeasurements(temperature280, pressure, StartAltitude);} // warm up bmp280
            while (!bmp280.getMeasurements(temperature280, pressure, StartAltitude)) {}
        
          #ifdef DB_SENSORS
            Serial.println("Done! ");
          #endif
  }

    if (USE_INA219){
      ina219.begin(); 
   }
   if (USE_MPU6050){ 
          mpu6050.begin();        
      #ifdef DB_SENSORS
               mpu6050.calcGyroOffsets(true);
               Serial.println (" ");
               Serial.println (" ");
      #else
               mpu6050.calcGyroOffsets(false);
      #endif
     
    }
   DeltaTime=0;
   MainLoopTime=millis();
   GetOldPipe(); 
   digitalWrite(LED_PIN, LOW);
   LoadModelNumber();
  
   
   
}

// **********************************************************************************************
// **********************************************************************************************
 // FailSafe data occupies EEPROM from offset 10 to 26 ******************************************
 
void SaveFailSafeData(){
     byte FS_Offset=10;                                              
     for (i=0;i<CHANNELSUSED;i++){
           EEPROMUpdateByte(i+FS_Offset,(map(ReceivedData[i],MINMICROS,MAXMICROS,0,180)));      // save servo positions lower res: 8 bits
     
     }  
     FS_Offset+=CHANNELSUSED; 
     for (i=0;i<CHANNELSUSED;i++){
           EEPROMUpdateByte(i+FS_Offset,FailSafeChannel[i]);   // save flags
     } 
}



void  SaveFailSafeSettings(){
      SaveFailSafeData(); 
      #ifdef  DB_FAILSAFE
      Serial.println ("Fail safe settings are saved!");   
      #endif
      FailSaveSafe=false;
}



void DoBinding(){

        
char BINDOK[]="BINDOK";
   GetNewPipe();
      #ifdef DB_BIND
            tt=NewPipe;
            Serial.println(tt);
            tt=OldPipe;
            Serial.println(tt);
       #endif
       if (OldPipe == NewPipe){
           strcpy(payLoad,BINDOK);
           SaveNewBind=false; 
           
           #ifdef DB_BIND  
           Serial.println(millis()-BindOKTimer);
           #endif
           
           if (BindOKTimer==0){
              BindOKTimer=millis();}
           else{
              if ((millis()-BindOKTimer)>400) { // allow .4 of a second for the TX to bind
                  BindNow=1;
              }
           }
       }
       if (BindNow==1 && !BoundFlag) {
          #ifdef DB_BIND  
            Serial.print ("Binding to: ");
            tt=NewPipe;
            Serial.println(tt);
          #endif 
         BindModel();
      }
}




// *******************************************************************************************************
void loop() {      
         ReceiveData();
         if (BoundFlag){
              if (USE_BNO055A)      Get_BNO055_Adafruit();
              if (USE_BNO055)       Get_BNO055_Cheapo();
              if (USE_MPU6050)      Get_Mpu6050();
              if (Connected)        MoveServos();
              if (FailSaveSafe)     SaveFailSafeSettings();  
              MainLoopTime=millis();
              DeltaTime=micros();
         }else{
              DoBinding();
         }    
  } 
//********************************************************************************************************