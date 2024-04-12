/********************************************************************************************
/                                      HPi Source File                                      *             
/                               copyright (c) 2024 HPi Studio                               *
/                                                                                           *
/                                                                                           *
/        des: Honda 200cc hp+ SmartECU                                                      *
/                                                                                           *
/        1402/06/01 Created By Hosein Pirani.                                               *
/                                                                                           *
/       Modified in  fri. 1402/06/17 from 15:00 To 19:00 (blinkers)                         *
/       Last modification: tue. 1403/01/13 from 13:10 To 19:15Major Bugs Fixed.Tested On M32*
/                                                                                           *
/TODO: Test Servo.                                                                          *
/TODO: Test LED FLASHERs                                                                    *
/TODO: Fix AutoStart Start Servo  Angle For Normal Start.                                   *
/TODO: Start UI Developing!!!!                                                              *
/TODO: Fuel Level !!engine temp SenSor!!                                                    *
/TODO: TEST And Debug idle Speed Adjuster                                                   *
/*******************************************************************************************/
////
///////////Fix EMERGENCY Shutdown Pin : merge  with CDI Shotdown and connect To An Normal Close Relay, and shutdown will Done by toggling the  relay.

#include <Arduino.h>
#include <EEPROM.h>
//Termocouple
#include <max6675.h>
///Servo
#include <Servo.h>


//!!!!!!!!!!!!!
//! bellow lines may removed for final upload!
//! //////////////
#if defined (__AVR_ATmega16__)|| defined(__AVR_ATmega32__)// im testing  on atmega16 & atmega32 & atmega328P
#define TIMERMASK TIMSK
#define RPM_PULSE_IN 14
#elif defined __AVR_ATmega328P__ 
#define TIMERMASK TIMSK1//Only For Atmega328p
#define RPM_PULSE_IN 12
#endif


//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!Pinouts should be Coreccted For Final Upload!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

constexpr auto EEP_FIRST_CHECK_FLAG = 2;////////////////////////////////EEPROM FLAG Should be changed For Final Upload.
///RPM in : PD6--- PD0 & PD1 are serial pins.
constexpr auto headlight_OutPin = 1;//PB1
constexpr auto frontLeftBlink_OutPin = 0;//PB0
constexpr auto backLeftBlink_OutPin = 5;//PB5
constexpr auto frontRightBlink_OutPin = 4;//PB4
constexpr auto backRightBlink_OutPin = 7;//PB7
constexpr auto LeftHorn_OutPin = 6;//PB6
constexpr auto RightHorn_OutPin = A1;//PA1(25)
constexpr auto RedS_OutPin = 3;//PB3
constexpr auto blueS_OutPin = 2;//PB2
constexpr auto SERVO_OutPin = A7;//PA7 SERVO OUT
constexpr auto AudioSwitcher_OutPin = A0;//PA0 /// Used For Swith Between  HIFI Speaker And Piezo Buzzer For Siren. 
constexpr auto CDI_ShutDown_OutPin = A6;//PA6 ////  For Auto Startup.should connected to relay (Normal close)//Open For Power Off
constexpr auto ENGINE_Start_OutPin = 16;//PC0 /// connected to relay/Power Transistor For auto start function. 
constexpr auto Switch_OutPin = 15;//PD7 // connected to relay for software controlled switch and Autostartup(Normally Open).
//Piezo Shake Sense In Pin
constexpr auto Piezo_OutPin = A4;//PA4 PIEZO ALARM OUT
constexpr auto ShakeSense_INpin = 11;//PD3//(Interrupt)For Piezo Alarm.
// Input Keys
constexpr auto RemoteStartINpin = 22;//PC6; //Remote Start
constexpr auto RemoteLSINpin = 19;//PC3//Lock or Silence if  Alarm actived or silence Lock.
constexpr auto RemoteUnlockINpin = 20;//PC4;//unlock Alarm
constexpr auto RemoteShutDownINpin = 21;//PC5;///Remote Emergency ShutDown
constexpr auto LturnINpin = A2;//PA2;
constexpr auto RturnINpin = A3;//PA3;
constexpr auto HEADLightINpin = 18;//PC2;
constexpr auto HornINpin = 10;//PD2//(Interrupt)
constexpr auto FuelGauge_IN = 17;//PC7//Connected To Fuel Gauge 
constexpr auto VBattINpin = A5;//PA5//for check the switch is Open Or No AND Measuring Battery Voltage for 90v -> r1 =220k , r2 = 13k output = 5v max. 
//OUTPUT Of Switch And Relay Should Be Connected To Two Seperate Diode wich Their Cathode Was Connected To Main DC Voltage (After Switch). 
// This Is Necessary For Detecting Switch State. 
//Temp Sensor
constexpr auto ThermoCS_OutPin = 17;//PC1
constexpr auto ThermoSCK_OutPin = 12;//PD4
constexpr auto ThermoSO_InPin = 13;//PD5 
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// /////Automatic Report means that Informations Will Reported Automatically By MCU.manual calls from ui may Not needed.*********
// Serial Commands(RX Line)
// From UI
constexpr auto InSerial_TurnOffENGINE_cmd = "EOF";//Implemented.
constexpr auto InSerial_HeadLightON_cmd = "HON";//Implemented.
constexpr auto InSerial_HeadLightOFF_cmd = "HOF";//Implemented.
constexpr auto InSerial_HeadBlinkOFF_cmd = "HBF";//Implemented.
constexpr auto InSerial_PoliceLightON_cmd = "PLO";//Implemented.
constexpr auto InSerial_PoliceLightOFF_cmd = "PLF";//Implemented.
constexpr auto InSerial_SetSirenSourceME_cmd = "SSM";//Implemented
constexpr auto InSerial_SetSirenSourceYOU_cmd = "SSU";//Implemented
constexpr auto InSerial_GetENGINEstate_cmd = "EST";//Implemented
constexpr auto InSerial_GetENGINEtemperature_cmd = "ETM";//Implemented
constexpr auto InSerial_GetENGINErpm_cmd = "ERP";///Not Implemented///////////////Automatic Report
constexpr auto InSerial_GetFuelLevel_cmd = "FUL";//Implemented
constexpr auto InSerial_GetBatteryVoltage = "BAT";
constexpr auto InSerial_MultiBlinkON_cmd = "MBO";//Implemented.
constexpr auto InSerial_MultiBlinkOFF_cmd = "MBC";//Implemented.
constexpr auto InSerial_LeftBlinkON_cmd = "LBO";//Implemented.
constexpr auto InSerial_LeftBlinkOFF_cmd = "LBC";//Implemented.
constexpr auto InSerial_RightBlinkON_cmd = "RBO";//Implemented.
constexpr auto InSerial_RightBlinkOFF_cmd = "RBC";//Implemented.
constexpr auto InSerial_BlinkerDanceON_cmd = "BDO";//Implemented.
constexpr auto InSerial_BlinkerDanceOFF_cmd = "BDC";//Implemented.
constexpr auto InSerial_AllowENGINEstart_cmd = "AES";//Implemented.
//commands Include Numerical Values.
constexpr auto InSerial_HeadBlinkON_cmd = "HBO:";//Implemented.//Substring Index!!
constexpr auto InSerial_SetBlinkInterval_cmd = "SBI:";//Implemented.//Substring Index!!
constexpr auto InSerial_AUTOStart_cmd = "ASE:";//Implemented.//Substring Index!!
constexpr auto InSerial_SetMinIdleRPM_cmd = "IDR:";//Implemented.//Substring Index!!
constexpr auto InSerial_SetMinServoAngle_cmd = "SLA:";//Implemented.//Substring Index!!
constexpr auto InSerial_SetMaxServoAngle_cmd = "SHA:";//Implemented.//Substring Index!!
constexpr auto InSerial_SetHornMode_cmd = "HRN:";//Implemented.//Substring Index!!
//Declare Flag For Each One To Prevent From Trash Calls.
//Serial Commands (TX Line)
// From ECU
constexpr auto OutSerial_STARTUP_cmd = "Im Alive^_^";//Implemented
constexpr auto OutSerial_ENGINEisOFF_cmd = "OFF";//Implemented/////////Automatic Report
constexpr auto OutSerial_ENGINEisON_cmd = "ON"; //Implemented/////////Automatic Report
constexpr auto OutSerial_HeadLightIsON_cmd = "ONH";//Implemented
constexpr auto OutSerial_HeadLightIsOFF_cmd = "OFH";//Implemented.
constexpr auto OutSerial_HeadBlinkIsON_cmd = "HBO";//Implemented
constexpr auto OutSerial_HeadBlinkIsOFF_cmd = "HBF";//Implemented.
constexpr auto OutSerial_LeftTurnIsON_cmd = "LON";//Implemented
constexpr auto OutSerial_LeftTurnIsOFF_cmd = "LOF";///Not Implemented
constexpr auto OutSerial_RightTurnIsON_cmd = "RON";//Implemented
constexpr auto OutSerial_RightTurnIsOFF_cmd = "ROF";//Implemented
constexpr auto OutSerial_AllBlinkersIsON_cmd = "ABO";//Implemented
constexpr auto OutSerial_AllBlinkersIsOFF_cmd = "ABF";//Implemented
constexpr auto OutSerial_BlinkDanceIsON_cmd = "BDO";//Implemented.
constexpr auto OutSerial_BlinkDanceIsOFF_cmd = "BDF";//Implemented
constexpr auto OutSerial_ALarmSourceIsMicro_cmd = "ASM";//Implemented.
constexpr auto OutSerial_AlarmSourceIsUI_cmd = "ASU";//Implemented
constexpr auto OutSerial_SirenIsOn_cmd = "SON";//Implemented
constexpr auto OutSerial_SirenIsOFF_cmd = "SOF";//Implemented
constexpr auto OutSerial_PoliceLightsIsOn_cmd = "PON";//Implemented
constexpr auto OutSerial_PoliceLightsIsOFF_cmd = "POF";//Implemented
constexpr auto OutSerial_ShakeDetected_cmd = "WOW";//Implemented./////////Automatic Report. Tell UI To Start CountDown Timer For Report Danger To Owner 
constexpr auto OutSerial_AlarmSilenced_cmd = "NOP";//Implemented./////Automatic Report. OK So Please Stop CountDown Timer.Allthings Is OK.
//commands Include Numerical Values.
constexpr auto OutSerial_BatteryVoltage_cmd = "VBT:";//Implemented.
constexpr auto OutSerial_ENGINErpm_cmd = "RPM:";//Implemented./////Automatic Report
constexpr auto OutSerial_ENGINEtemperature_cmd = "TMP:";//Implemented./////Automatic Report
constexpr auto OutSerial_FuelLevel_cmd = "FuL:";//Implemented./////Automatic Report

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Termocouple
MAX6675 thermocouple(ThermoSCK_OutPin, ThermoCS_OutPin, ThermoSO_InPin);
// Servo For Adjusting Idle throttle(ENGINEs Idle RPM)
Servo IdleServo;
///////////RPM Meter
bool MeasEnd = 0;
volatile uint16_t T1OVF_Counter = 0;
volatile unsigned long input_Rissing_time = 0, input_Falling_time = 0, input_TOP_time = 0;
const unsigned long Timer1_prescaler_freq = 250000;//2000000
unsigned long rpmPrvmillis = 0;
volatile unsigned long RPM = 0;
//
///Idle RPM
unsigned long idleSpeedPrevMillis = 0;
bool RPMadjusted = false;
 uint16_t previousAngle  = 0;// store prev Servo angle. waite for engine to warm up.
uint16_t eep_minServoAngle = 0, eep_maxServoAngle = 360;
///
//AUTOStart
 unsigned long AutoStartDurrationMillis = 0;//counter for Start Button Pressing. 
 bool StartCompleted = false;// if task was done
 int startPressDurration = 0;//we need this flag because we uising millis()
 //
 ///Remote Control Listener
  bool remote_unlock_flag =false;//Remote unlock KeyPress Flag.
   bool remote_start_flag =false;//Remote start KeyPress Flag.  
 bool remote_LSI_flag =false; //Remote Lock Or Silence KeyPress Flag.
 bool RemoteShutDownFlag = false;//flag for remote shutdown keyPress.
 unsigned long RemoteShutDownLastMillis = 0; // Timer For remote shutdown keyPress.
 byte RemoteLSstate = 0;// counter For Lock/SilenceAlarm
 bool lockflag = false;//for Play Single Alarm. means Locked.
 bool PiezoDetected = false;// Piezzo Used As Sensor and Now Shake Detected.
 //
 ///Alarm
 unsigned long  Alarm_prevmillis = 0, Alarm_prevmicros = 0;// counters.
 unsigned long Alarm_Timer = 0;//Timer For Loop Alarm. Disable The Alarm after 20 Seconds.
 bool flag = true;
 bool Alarm = false;//Alarm Called?
 bool single_Alarm = false;// one time  Alarm (played when Locking)
 bool Silenced = false; // Silent Mode?
 uint16_t A_freq = 768, D_freq = 700, B_freq = 768, C_freq = 170, Single_freq = 611;
 int mod3_wait = 0;
 unsigned long delayy = 550, mod_A_delay = 350, mod_D_delay = 400, mod_B_delay = 350, mod_C_delay = 3000, Single_delay = 500;
 byte Alarmstage = 1, currentCounter = 0, singlecount = 0;/// current stage of Alarm
//
// Battery Voltage 
double r1 = 10000.0;// HighResistor 10Kohms.
double r2 = 1000.0;//LowResistor 1Kohms.  10:1 Divided
// for 24 V  75K(76K) ,20K
// for 12 v 4.7k ,6.8k
//for  55 v 10K ,1K prefrred
//
//FuelGauge Resistors
double UpResistor = 6800.0; //Upper(PullUP) Resistor.
double DownResistor = 4700.0; //Lower(PullDown) Resistor.


///HORN
unsigned short debounceDelay = 200;
unsigned long Horn_delayMillis = 0,hornPrevMillis = 0, buttonPrevMillis = 0, lastDebounceTime = 0;
volatile byte hornclicks = 0;
bool HornFlag= false;
//HORN modes
byte hornCountA=0,hornCountB=0;
bool horn1Aflag = true, horn1Bflag = false;
bool hornStateA = false,hornStateB = false;
bool hornModeTwoState = true;
bool hornModThreeState = false; // current state of patern. below line is our patern 
byte hornModeCStage  =1;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////<blinkers>
#define blinkInterval  eep_blinkinterval // normal blinkers on/of delay in ms.
//
// Serial flags
// Used For One Time Call UI For Change State of someThing.
bool rturn_SerialOut_flag = false ;
bool lturn_SerialOut_flag = false ;
bool multiblink_SerialOut_flag = false ;
bool headlight_SerialOut_flag = false ;
bool policeLights_SerialOut_flag = false ;
//
unsigned long  prevMillis = 0; //for millis();. it used instead of old depricated delay() .
///MODE2
bool danceTwoFrontFlag = true,danceTwoBackFlag = false,danceTwoFrontState = false,danceTwoBackState = false; // states
byte danceTwoFrontCounter = 0,danceTwoBackCounter = 0; // blink counters
///
byte danceMode =1;// current Mode
byte danceblinkcounter =0;// how many times current mode repeated?.
byte stagecounter = 0;// current stage play counter.
//
//normal blink
unsigned long DancePrev_Millis = 0;//Store PrevMillis
bool state = false;
bool blinkerstate = false;
bool multiblink = false;
bool Leftfrontblinkerstate = false;
bool Rightfrontblinkerstate = false;
bool Leftbackblinkerstate = false;
bool Rightbackblinkerstate = false;
bool blinkdance = false;
//
// Headlight Blink
unsigned long Headblink_prevMillis = 0;
unsigned long Headblink_timeout = 0; // timeout should be Started Before Call. ////////FIX IT
unsigned long Headblink_delay = 50; /// Delay For Blink Should Be Configured From UI!!!!!!!!!!!!!!!!!!!!!!!!!
bool Headblink_flag = false; // flag for call.
bool Headblink_state = false; // flag for toggle on off.
bool headlight_wasOn = false; // remeber that headlight was on before call. 
 int Headblink_blinkmode = 0;/// Positive number For Duration Or Zero For Continuos.
//
// POLICE Siren Blinkers(RED/BLUE)LEDs
byte sirenCaseCounter = 0;// wich case is playing?
byte sirenDanceRedCounter = 0;//how many times RED LEDs toggled
byte sirenDanceBlueCounter = 0;//how many times BLUE LEDs toggled
byte SirenStageCounter = 0;//how Many times Current Case was Repeated? 
bool sirenDanceRedFLAG = true; //Now, Were We Are?
bool sirenDanceBlueFLAG = false;//Now, Were We Are?
bool sirenDanceREDState = false;//Current State Of OUTPUT Pin(High Or Low)
bool sirenDanceBLUEState = false;//Current State Of OUTPUT Pin(High Or Low)
unsigned long sirenDancePrevMillis = 0;//Last Toggle Time
bool m_doSiren = false;//Serial Command is received? so Blink.
//////////////////////////////////////////////////////////////////////////////////////////</blinkers>
//defines
constexpr auto ENGINE_IS_OFF = false;
constexpr auto ENGINE_IS_ON = true ;
/// input keys Flag
bool headlightFlag = false, lturnflag = false, rturnflag = false, brakeflag = false, engPowerFlag = ENGINE_IS_OFF; //parsing keys
//EEPROM DATA
bool freshstart = true;//first startup (start from reset Vector Or Power ON)
int eep_blinkinterval = 250; // default Delay For Nomal Blinking
byte eep_blinkintervalAddress = 1; // Address Off Interval Holder
byte eep_minimumIdleRPMAddress = eep_blinkintervalAddress + sizeof(int);  //minimum Engine's Idle RPM EEPROM Address. for Idle Adjuster. Set By UI.
byte eep_minimumIdleRPM = 150;//minimum Idle RPM
byte eep_minServoAngleAddress = eep_minimumIdleRPMAddress + sizeof(byte);
byte eep_maxServoAngleAddress = eep_minServoAngleAddress + sizeof(uint16_t);
//byte eep_HornDelayAddress = eep_maxServoAngle + sizeof()
//
//Serial
String   input = "";
/*
String SerialInputCommands[] =
{
"off","on","headlight:on","headlight:off"
,"DoSirenLight","TurnOffSiren","multiblink:on","multiblink:off",
"lturn:blink","rturn:blink","lturnblink:off","rturnblink:off","blinkdanceOn","blinkdanceOff","SetBlinkInterVal:300","engineState","EMERGENCYSHOTDOWN","AllowStart",
"AutoStart:10,"SetMinimumServoAngle","SetMaximumServoAngle",
"SetHornMode:1,2,3","BlinkHeadlight:1|2","OFFHeadblink"
};


String SerialOUTPUTCommands[] =
{"off","on","Engine Is OFF(EOF)","Engine Is ON(EON)","DANCE On ","Dance Off"
"VBATT","TEMP","FUEL_LEVEL","RPM:000,SmallLight,Uplight,DownLight,UPlightBlink","L","l","R","r","M","m",
};
*/
 volatile unsigned long  RPS = 0; ///For Debug RPM

ISR(TIMER1_OVF_vect)
    {
    T1OVF_Counter++;///RPM Meter
    }

ISR(TIMER1_CAPT_vect)
    {
      volatile unsigned long  Freq = 0;
    if (!MeasEnd)
        {
        input_Rissing_time = ICR1;
        MeasEnd = 1;
        } else
        {
        input_Falling_time = ICR1;
        if (T1OVF_Counter)
            {
            input_TOP_time = input_Falling_time + (65536 * T1OVF_Counter) - input_Rissing_time;
            } else
            {
            input_TOP_time = input_Falling_time - input_Rissing_time;
            }
            if (input_TOP_time == 0)
                {

                } else
                {
                Freq = (Timer1_prescaler_freq / input_TOP_time);
                ///freq means revolution per second(RPS) and RPM Means Revolution per Minute
                RPM = Freq * 60;
                RPS = Freq;
                }
                T1OVF_Counter = 0;
                MeasEnd = 0; 
              //                  Freq = 0;
             //   RPM = 0;
        }
       // Freq = 0;
      //  RPM = 0;
    }
void setup() 
{

  Serial.begin(115200);
  analogReference(EXTERNAL);
  //Outputs
  pinMode(headlight_OutPin,OUTPUT);
  pinMode(backLeftBlink_OutPin,OUTPUT);
  pinMode(frontLeftBlink_OutPin,OUTPUT);
  pinMode(backRightBlink_OutPin,OUTPUT);
  pinMode(frontRightBlink_OutPin,OUTPUT);
  pinMode(LeftHorn_OutPin,OUTPUT);
  pinMode(RightHorn_OutPin,OUTPUT);
  pinMode(blueS_OutPin,OUTPUT);
  pinMode(RedS_OutPin,OUTPUT);
  pinMode(AudioSwitcher_OutPin,OUTPUT);
  //pinMode(ThermoCS_OutPin, OUTPUT);
 // pinMode(ThermoSCK_OutPin, OUTPUT);
  pinMode(SERVO_OutPin, OUTPUT);
  pinMode(CDI_ShutDown_OutPin, OUTPUT);
  pinMode(ENGINE_Start_OutPin, OUTPUT);
  pinMode(Switch_OutPin, OUTPUT);
  pinMode(Piezo_OutPin, OUTPUT);
  ///Inputs
  pinMode(LturnINpin, INPUT);
  pinMode(RturnINpin, INPUT);
  pinMode(HEADLightINpin, INPUT);
  //pinMode(ThermoSO_InPin, INPUT);
  pinMode(HornINpin, INPUT);
  pinMode(VBattINpin, INPUT);
  pinMode(ShakeSense_INpin, INPUT);
  pinMode(FuelGauge_IN, INPUT);
  pinMode(RemoteLSINpin, INPUT);
  pinMode(RemoteUnlockINpin, INPUT);
  pinMode(RemoteStartINpin, INPUT);
  pinMode(RemoteShutDownINpin, INPUT);
  ///Horn Button Listener
  attachInterrupt(digitalPinToInterrupt(HornINpin), checkHornKey, CHANGE);
//servo
  IdleServo.attach(SERVO_OutPin);
  //RPM Meter 
  TCCR1A = 0;           // Init Timer1A
  TCCR1B = 0;           // Init Timer1B
  TCCR1B |= B11000011;  // (B11000010) Internal Clock, Prescaler = 16, ICU Filter EN, ICU Pin RISING
  TIMERMASK |= B00100001;  // Enable Timer OVF & CAPT Interrupts
  ////SERVO

  //
  //Blink Lights For Test!
  
  digitalWrite(headlight_OutPin, HIGH);
  digitalWrite(frontLeftBlink_OutPin, HIGH);
  digitalWrite(frontRightBlink_OutPin, HIGH);
  digitalWrite(backLeftBlink_OutPin, HIGH);
  digitalWrite(backRightBlink_OutPin, HIGH);
  delay(1000);
  digitalWrite(headlight_OutPin, LOW);
  digitalWrite(frontLeftBlink_OutPin, LOW);
  digitalWrite(frontRightBlink_OutPin, LOW);
  digitalWrite(backLeftBlink_OutPin, LOW);
  digitalWrite(backRightBlink_OutPin, LOW);
Serial.println(OutSerial_STARTUP_cmd);
//sei();
}

void loop() 
{
 /// EEPROM DATA
///Blink Delay & Minimum Idle RPM + Servo Angle
//Serial.println(RPS);
    if (freshstart == true)
    {
        byte firstcheck = 0;
        uint16_t add = 0;
        firstcheck = EEPROM[add];
        



        if (firstcheck != EEP_FIRST_CHECK_FLAG)
        {
            //Blink Interval
            EEPROM.put(add, EEP_FIRST_CHECK_FLAG);
            delay(5);
            EEPROM.put(eep_blinkintervalAddress, eep_blinkinterval);
            delay(5);
            //IDLE RPM
            EEPROM.put(eep_minimumIdleRPMAddress, eep_minimumIdleRPM);
            //SERVO Angle
            delay(5);
            EEPROM.put(eep_minServoAngleAddress, eep_minServoAngle);
            delay(5);
            EEPROM.put(eep_maxServoAngleAddress, eep_maxServoAngle);
        } else
        {
            EEPROM.get(eep_blinkintervalAddress, eep_blinkinterval);
            delay(5);
            EEPROM.get(eep_minimumIdleRPMAddress, eep_minimumIdleRPM);
        }
        freshstart = false;
    }
 //
 //Serial.println(millis());
    /// read Serial Commands
    if (Serial.available())
    {
        input = Serial.readStringUntil('\n');
        delay(1);
        Serial.print(input);
        delay(1);
    }


 ///// RPM Adjusting
 if (RPMadjusted == false) adjustIdleSpeed();




    //headlight
  if (digitalRead(HEADLightINpin) == HIGH)
   {
    digitalWrite(headlight_OutPin, HIGH);
    headlightFlag = true;
    if (headlight_SerialOut_flag == false)
    {
    headlight_SerialOut_flag = true;//Set The Flag
    Serial.print(OutSerial_HeadLightIsON_cmd);
    }
   }else  //headLight Off
    {
      if (headlightFlag == true)
      {
        
        
          headlightFlag = false;
    digitalWrite(headlight_OutPin, LOW);
    if(headlight_SerialOut_flag == true)
    {
     Serial.print(OutSerial_HeadLightIsOFF_cmd);
     headlight_SerialOut_flag = false;//Reset The Flag
    }
      }


    }
/////left turn
  if (digitalRead(LturnINpin) == HIGH )
  {
      if(blinkInterval <=0) blinkInterval = EEPROM.read(eep_blinkintervalAddress);
      Rightfrontblinkerstate = false;//turn off
      Rightbackblinkerstate = false;// the others
      blinkdance = false;//off
      multiblink = false;//off
      //

       blinkerstate = true;
    Leftfrontblinkerstate = true;
    Leftbackblinkerstate = true;
     lturnflag = true;
           if (lturn_SerialOut_flag == false)//Just Update UI One Time To Prevent From Unnecessary Calls
          {
            lturn_SerialOut_flag = true;//SetFlag
            Serial.print(OutSerial_AllBlinkersIsOFF_cmd);   //UpdateUI  
            delayMicroseconds(5);
           Serial.print(OutSerial_LeftTurnIsON_cmd);   //UpdateUI 
          }
  }  else // left turn OFF
  {
    if (lturnflag == true)
    {
            lturn_SerialOut_flag = false;//Reset the flag
        digitalWrite(backLeftBlink_OutPin,LOW);
      digitalWrite(frontLeftBlink_OutPin,LOW);
         blinkerstate = false;
      Leftfrontblinkerstate = false;
      Leftbackblinkerstate = false;
      multiblink = false;
      lturnflag = false;
      blinkdance = false;
Serial.print(OutSerial_AllBlinkersIsOFF_cmd);

    }
 
 }
////right turn

  if (digitalRead(RturnINpin) ==  HIGH)
   {
    
      if (blinkInterval <= 0) blinkInterval = EEPROM.read(eep_blinkintervalAddress);//interval
      // check for multiblink
      if ((Leftfrontblinkerstate == true) && (Leftbackblinkerstate == true))
      {
          Leftfrontblinkerstate = false;
          Rightfrontblinkerstate = false;
          Rightbackblinkerstate = false;
          Rightfrontblinkerstate = false;
         // if (blinkdance) Serial.print(OutSerial_BlinkDanceIsOFF_cmd);   //UpdateUI  
         
          blinkdance = false;//
          //
          blinkerstate = true;//
          multiblink = true;
      } else
      {
          multiblink = false;      ///the
          digitalWrite(backLeftBlink_OutPin, LOW);//other
          digitalWrite(frontLeftBlink_OutPin, LOW);//blinkers
          input = ""; //clear the buffer
         // if (blinkdance) Serial.print(OutSerial_BlinkDanceIsOFF_cmd);   //UpdateUI  
          blinkdance = false;//
          blinkerstate = true;//
          Rightbackblinkerstate = true;//same as above 
          Rightfrontblinkerstate = true;//turn off other
           rturnflag = true;
          if (rturn_SerialOut_flag == false)//Just Update UI One Time To Prevent From Unnecessary Calls
          {
            rturn_SerialOut_flag = true;//set the flag
          Serial.print(OutSerial_AllBlinkersIsOFF_cmd);
          delayMicroseconds(5);
          Serial.print(OutSerial_RightTurnIsON_cmd);
          }
         
      }
    } else // Right Turn off
   {
    if (rturnflag == true)
    {
      rturn_SerialOut_flag = false;//Reset The Serial Flag for Feature call.
      digitalWrite(backRightBlink_OutPin,LOW);
      digitalWrite(frontRightBlink_OutPin,LOW);
       blinkerstate = false;
       Rightbackblinkerstate = false;
       Rightfrontblinkerstate = false;
       multiblink = false;
       rturnflag = false;
       blinkdance = false;
       Serial.print(OutSerial_AllBlinkersIsOFF_cmd);   //UpdateUI  
    }
   }
/////////RPM METER & Battery Voltage & Engine Temp. will Update EVERY 1 Second.
   if ((millis() - rpmPrvmillis) >= 1000)
   {
       rpmPrvmillis = millis();
       //RPM
       if (RPS <= 1)
       {
           RPM = 0;
           if (engPowerFlag == ENGINE_IS_ON)
           {
               engPowerFlag = ENGINE_IS_OFF;
               Serial.print(OutSerial_ENGINEisOFF_cmd);
               TemporaryDOSwitch(false);//restore to defaults

           }
       } else
       {

        //   prevMillis = millis();
          
          
           



           if (engPowerFlag == ENGINE_IS_OFF)
           {
               engPowerFlag = ENGINE_IS_ON;
               Serial.println(OutSerial_ENGINEisON_cmd);
               Serial.print(OutSerial_ENGINErpm_cmd);
               Serial.println(RPM);
           }
       }
   }
   ////////////////////////////////////////////////////////////////
  // read incomming commands


  /// turn on main lights///////
  if (input == InSerial_HeadLightON_cmd)
     {
      digitalWrite(headlight_OutPin,HIGH);
      input ="";
      Serial.print(OutSerial_HeadLightIsON_cmd);
     }//
   //////turn off main lights///////////
   if ((input == InSerial_HeadLightOFF_cmd))
       {
       if (digitalRead(HEADLightINpin) == LOW)
       {
               digitalWrite(headlight_OutPin, LOW);
               headlightFlag = false;
               input = "";
               Serial.print(OutSerial_HeadLightIsOFF_cmd);
           
       }
       }////////
   // engine State Called.
   if (input == InSerial_GetENGINEstate_cmd)
     {
        if (engPowerFlag ==ENGINE_IS_OFF)
        {
          Serial.print(OutSerial_ENGINEisOFF_cmd);
          
        }
        else {
        Serial.print(OutSerial_ENGINEisON_cmd);
        }
        input ="";
      }////////   
   // turn on Police Lights
   if (input == InSerial_PoliceLightON_cmd)
       {
       m_doSiren = true;
       input = "";
       Serial.print(OutSerial_PoliceLightsIsOn_cmd);
       blinkSiren();

       }
   // turn off Police Lights
   if (input == InSerial_PoliceLightOFF_cmd)
       {
       m_doSiren = false;
       input = "";
       Serial.print(OutSerial_PoliceLightsIsOFF_cmd);
       }
   //Load speaker,Unload BUZZER
   if (input == InSerial_SetSirenSourceME_cmd)
       {
       Serial.print(OutSerial_AlarmSourceIsUI_cmd);
       toggleSpeakerPin(false);
       }
   //onload loud speaker,Load BUZZER
   if (input == InSerial_SetSirenSourceYOU_cmd)
       {
       Serial.print(OutSerial_ALarmSourceIsMicro_cmd);
       toggleSpeakerPin(true);
       }
   // Emergency. Called By UI (recieved Remotely)
   if (input == InSerial_TurnOffENGINE_cmd)
       {
       digitalWrite(CDI_ShutDown_OutPin, HIGH);
       }
   // Can Start Engine
   if (input == InSerial_AllowENGINEstart_cmd)
       {
       digitalWrite(CDI_ShutDown_OutPin, LOW);
       }
    //////turn off All Blinkers ///////////
  if ((input == InSerial_LeftBlinkOFF_cmd) || (input == InSerial_RightBlinkOFF_cmd)  || (input == InSerial_MultiBlinkOFF_cmd)  || (input == InSerial_BlinkerDanceOFF_cmd) )
  {
      if ((rturnflag == false) && (lturnflag == false))
      {// if physical button was not pressed
          
          Leftfrontblinkerstate = false;
          Rightfrontblinkerstate = false;
          Rightbackblinkerstate = false;
          Rightfrontblinkerstate = false;
          blinkerstate = false;
          multiblink = false;
          digitalWrite(backLeftBlink_OutPin, LOW);
          digitalWrite(frontLeftBlink_OutPin, LOW);
          digitalWrite(backRightBlink_OutPin, LOW);
          digitalWrite(frontRightBlink_OutPin, LOW);
          input = ""; //clear the buffer
         // if (blinkdance) Serial.print(OutSerial_BlinkDanceIsOFF_cmd);   //UpdateUI  
          blinkdance = false;
           Serial.print(OutSerial_AllBlinkersIsOFF_cmd);
      }
  }////////

 //////////checks for Left blinker///////////////
  if (input == InSerial_LeftBlinkON_cmd)
  {
      Serial.print(OutSerial_LeftTurnIsON_cmd);

      blinkerstate = true;
      Leftfrontblinkerstate = true;
      Leftbackblinkerstate = true;
      input = "";

  }
  //
 ////AUTOStart
  if (input.startsWith(InSerial_AUTOStart_cmd))
  {
 
      startPressDurration = input.substring(4).toDouble();
               
              AutoStart(startPressDurration);
             // input = ""; We will Clear Buffer Inside Function After Completting Task. 
  }
 //////EEPROM Blink Interval set.
  if (input.startsWith(InSerial_SetBlinkInterval_cmd))
  {
      eep_blinkinterval = input.substring(4).toInt();
      if (eep_blinkinterval > 0)
      {
          EEPROM.update(eep_blinkintervalAddress, eep_blinkinterval);
      }
  }
  //
//////EEPROM minimum idle RPM set.
  if (input.startsWith(InSerial_SetMinIdleRPM_cmd))
  {

      eep_minimumIdleRPM = input.substring(4).toInt();
      if (eep_minimumIdleRPM > 0)
      {
          EEPROM.update(eep_minimumIdleRPMAddress, eep_minimumIdleRPM);
          delay(5);
      }
  }
//
/// EEPROM Min and max Servo Angle
  if (input.startsWith(InSerial_SetMinServoAngle_cmd))
  {

      eep_minServoAngle = input.substring(4).toInt();
   
          EEPROM.update(eep_minServoAngleAddress, eep_minServoAngle);
          delay(5);
      
  }
  ///MAX
  if (input.startsWith(InSerial_SetMaxServoAngle_cmd))
  {

      eep_maxServoAngle = input.substring(4).toInt();
      if (eep_maxServoAngle > 0)
      {
          EEPROM.update(eep_maxServoAngleAddress, eep_maxServoAngle);
          delay(5);
      }
  }
 //
 // Horn Mode
  if (input.startsWith(InSerial_SetHornMode_cmd))
  {
      hornclicks = input.substring(4).toInt();  
      input = "";
  }
  //
  //HeadBlink ON
  if (input.startsWith(InSerial_HeadBlinkON_cmd))
  {
      Headblink_blinkmode = input.substring(4).toInt();
      // enable flag
      if (Headblink_blinkmode >= -1)  Headblink_flag = true;//  we didnt used signed variable so  Precaution is proof of Wisdom and Tact ..^_^..
      
      if (Headblink_blinkmode > 0)
      {
          Headblink_delay = Headblink_blinkmode;
                  input = "";
          Headblink_timeout = millis(); //Start Timeout Timer!
      }
      Serial.print(OutSerial_HeadBlinkIsON_cmd);
      BlinkHeadLight();// need to be called fast as fast because timer was actived.
  }
 //
 //HeadBlink OFF
  if (input == InSerial_HeadBlinkOFF_cmd)
  {
      
    Headblink_flag = false;
      input = "";
      Serial.print(OutSerial_HeadBlinkIsOFF_cmd);
      BlinkHeadLight();// fast call for disabling.
  }
 // 
  //Temperature
  if (input == InSerial_GetENGINEtemperature_cmd)
  {
    input = "";
      Serial.print(OutSerial_ENGINEtemperature_cmd);
      Serial.println(GetEngineTemp());
  }
  //
  //FUEL Level
  if (input == InSerial_GetFuelLevel_cmd)
  {
    input = "";
      Serial.print(OutSerial_FuelLevel_cmd);
      Serial.println(GetFuelLevel());
  }
  //
  //ENGINE Temp 
  if (input == InSerial_GetBatteryVoltage)
  {
    input = "";
           Serial.println(OutSerial_BatteryVoltage_cmd);
       Serial.println(getBatteryVoltage());

  }
  //
 ///////commands from UI//////////////////

  //Right Turn Blink
  if (input == InSerial_RightBlinkON_cmd)
   {
       blinkerstate = true;
       Rightbackblinkerstate = true;
       Rightfrontblinkerstate = true;
       input ="";   
       Serial.print(OutSerial_RightTurnIsON_cmd);
   }  
  //multi blink
  if (input == InSerial_MultiBlinkON_cmd)
  {   
     blinkerstate = true;
     multiblink = true;
     input ="";
     Serial.print(OutSerial_AllBlinkersIsON_cmd);
  }

  if (input == InSerial_BlinkerDanceON_cmd)
  {
   blinkerstate = true;
   blinkdance = true;
   Serial.print(OutSerial_BlinkDanceIsON_cmd);
  }

  /////////////////////////////////////////////////////////////////////
  // we're using millis() instead Of Delay(). So We need Call Functions rapidly. 
  // its like Multitasking and Event listening.
//call  functions  
 Blink();//for blinkers.
     Horn(); // for horn.

  ListenForRemoteControl();//Remote Control Listener.
  DoAlarm();// Lock Alarm.
  BlinkHeadLight();

}//loop

/// <summary>
/// void blink(void)
/// blinks the specified blinkers determinated by flags
/// blinkers are: Front LEFT/Right, rear LEFT/Right blinkers
/// it has 4 modes + multi and dance
/// </summary>
/// <param name="none"></param>
void Blink(void)
{
  if (blinkerstate)
      {
      //  Serial.println("blinkerState");
        if (Leftfrontblinkerstate && Leftbackblinkerstate)
        {        
          //Serial.println(millis());
          if ((millis() - prevMillis) >= blinkInterval)
             {                          
              digitalWrite(frontLeftBlink_OutPin,state);
               digitalWrite(backLeftBlink_OutPin,state);  
               /// call ui app
              // if (state)
            //   {
            //       Serial.print("L");
            //   } else
             //  {
             //      Serial.print("l");
             //  }
               state =!state;
              prevMillis = millis();
              //Serial.println("LFB_Millis");
             }
         }//Left Turn
        //Right Turn
         if (Rightbackblinkerstate && Rightfrontblinkerstate)
         {
          //Serial.println("rbf");
                  if ((millis() - prevMillis) >= blinkInterval)
           {
                   digitalWrite(backRightBlink_OutPin,state);
                   digitalWrite(frontRightBlink_OutPin,state);
                  state =!state;
                //  if (state)
                //  {
               //       Serial.print("R");
                //  } else
                //  {
             // / //       Serial.print("r");
                 // }
                 Serial.println("rmillis");
             prevMillis = millis();
           }
        }//
         //Multiblink
         if (multiblink)
          {
            if ((millis() - prevMillis) >= (blinkInterval)){
                              digitalWrite(backRightBlink_OutPin,!state);
                   digitalWrite(frontRightBlink_OutPin,!state);
                   digitalWrite(frontLeftBlink_OutPin,!state);
                   digitalWrite(backLeftBlink_OutPin,!state);
                   digitalWrite(backRightBlink_OutPin, !state);
                   if (state)//Update UI
                   {
               //        Serial.print("M");
                   } else
                   {
             //          Serial.print("m");
                   }
             state =!state; 
              prevMillis = millis();
            }
          }
        if (blinkdance)
        {
           // Serial.print("dance");   //UpdateUI  
            if (danceMode >= 5 ) danceMode = 1;
          switch (danceMode)
          {
                   case 1:
                   if ((millis() - DancePrev_Millis) >= 300)
                   {
                       DancePrev_Millis = millis(); ///////////////////////// If Somthing Went Wrong Remove This Line
                    danceblinkcounter++;
                    switch (danceblinkcounter) 
                    {             
                    case 1:         
                      digitalWrite(frontLeftBlink_OutPin,HIGH);
                      digitalWrite(backLeftBlink_OutPin,LOW);
                     break;
                     case 2:
                     digitalWrite(frontLeftBlink_OutPin,LOW);
                     digitalWrite(frontRightBlink_OutPin,HIGH);
                     break;
                     case 3:
                     digitalWrite(frontRightBlink_OutPin,LOW);
                     digitalWrite(backRightBlink_OutPin,HIGH);
                     break;
                     case 4:
                     digitalWrite(backRightBlink_OutPin,LOW);
                     digitalWrite(backLeftBlink_OutPin,HIGH);
                     danceblinkcounter = 0;
                     stagecounter++;
                     if (stagecounter >=5)
                     {
                       danceMode++;
                       stagecounter = 0;
                     }
                     break;
                    }
                   }
                         // dancecounter = 0;
                          
          break;
                        

          case 2: /// turn on Front Blinkers Rapidly etc 3 times,then turn on back blinkers rapidly too.
                      // this task wil repeated for etc 3 times


                 //    Serial.print("\n case2 \n");
              if (danceTwoFrontCounter <=5 && danceTwoFrontFlag == true)
            {            
              if ((millis() - DancePrev_Millis) >= 50)
              {
                digitalWrite(backLeftBlink_OutPin,LOW);//turn off the 
                digitalWrite(backRightBlink_OutPin,LOW);//back blinkers
                DancePrev_Millis = millis();
                danceTwoFrontState = !danceTwoFrontState;
                digitalWrite(frontLeftBlink_OutPin,danceTwoFrontState);//toggle the 
                digitalWrite(frontRightBlink_OutPin,danceTwoFrontState);// front blinkers.
                if (danceTwoFrontState == HIGH)
                {
                 // Serial.print("\n 1A \n");
                  danceTwoFrontCounter++;
                  if (danceTwoFrontCounter >=5 )
                  {
                    
                   danceTwoFrontFlag = false;
                   danceTwoBackFlag = true;
                  }

                 }
               }
            }else
            {
              if (danceTwoBackCounter <=5 && danceTwoBackFlag == true )
             {
               if ((millis() - DancePrev_Millis) >=50)
               {
                digitalWrite(frontLeftBlink_OutPin,LOW);//turn off the 
                digitalWrite(frontRightBlink_OutPin,LOW);//front blinkers
                DancePrev_Millis = millis();
                danceTwoBackState =!danceTwoBackState;
                digitalWrite(backLeftBlink_OutPin,danceTwoBackState);
                digitalWrite(backRightBlink_OutPin,danceTwoBackState);
                 if (danceTwoBackState == HIGH)
                 {
                 // Serial.print("\n 1B \n");
                  danceTwoBackCounter++;
                  if (danceTwoFrontCounter >= 5 && danceTwoBackCounter >=5)
                  {
                    danceTwoFrontCounter = 0;
                    danceTwoBackCounter = 0;
                   danceTwoFrontFlag = true;
                   danceTwoBackFlag = false;
                   stagecounter++;
                   if (stagecounter >= 5)
                    {
                     stagecounter = 0;
                     danceMode++;
                    }
                  }
                 }
                }  
              }
           }
          break;
          case 3:
                     ////blinks front and back Left blinkers etc 3 times then turns right blinkers...
                     /// Copy-Pasted From MODE TWO!!!!!
                //     Serial.print("\n case3 \n");
           if (danceTwoFrontCounter <=5 && danceTwoFrontFlag == true)
           {            
              if ((millis() - DancePrev_Millis) >= 50)
              {
                digitalWrite(frontRightBlink_OutPin,LOW);//turn off the 
                digitalWrite(backRightBlink_OutPin,LOW);//Right blinkers
                DancePrev_Millis = millis();
                danceTwoFrontState = !danceTwoFrontState;
                digitalWrite(frontLeftBlink_OutPin,danceTwoFrontState);//toggle the 
                digitalWrite(backLeftBlink_OutPin,danceTwoFrontState);// Left blinkers.
                if (danceTwoFrontState == HIGH)
                {
                //  Serial.print("\n 1A \n");
                  danceTwoFrontCounter++;
                  if (danceTwoFrontCounter >=5 )
                  {
                    
                   danceTwoFrontFlag = false;
                   danceTwoBackFlag = true;
                  }

                 }
               }
            }else
            {
             if (danceTwoBackCounter <=5 && danceTwoBackFlag == true )
             {
              if ((millis() - DancePrev_Millis) >=50)
              {
                digitalWrite(frontLeftBlink_OutPin,LOW);//turn off the 
                digitalWrite(backLeftBlink_OutPin,LOW);//Left blinkers
                DancePrev_Millis = millis();
                danceTwoBackState =!danceTwoBackState;
                digitalWrite(frontRightBlink_OutPin,danceTwoBackState);
                digitalWrite(backRightBlink_OutPin,danceTwoBackState);
                 if (danceTwoBackState == HIGH)
                 {
               //   Serial.print("\n 1B \n");
                  danceTwoBackCounter++;
                  if (danceTwoFrontCounter >= 5 && danceTwoBackCounter >=5)
                  {
                    danceTwoFrontCounter = 0;
                    danceTwoBackCounter = 0;
                   danceTwoFrontFlag = true;
                   danceTwoBackFlag = false;
                   stagecounter++;
                   if (stagecounter >= 5)
                    {
                     stagecounter = 0;
                     danceMode++;
                    }
                  }
                 }
              }
             }
           }
  
          break;
                        

          case 4:////////Blink All 4 Blinkers Rapidly
           if (danceTwoFrontCounter <=5 && danceTwoFrontFlag == true)
           {            
              if ((millis() - DancePrev_Millis) >= 50)
              {
                DancePrev_Millis = millis();
                danceTwoFrontState = !danceTwoFrontState;
                digitalWrite(frontLeftBlink_OutPin,danceTwoFrontState);///Toggle
                digitalWrite(frontRightBlink_OutPin,danceTwoFrontState);///the
                digitalWrite(backLeftBlink_OutPin,danceTwoFrontState);///All
                digitalWrite(backRightBlink_OutPin,danceTwoFrontState);//Blinkers
                if (danceTwoFrontState == HIGH)
                {
                  //Serial.print("\n 1A \n");
                  danceTwoFrontCounter++;
                  if (danceTwoFrontCounter >=5 )
                  {
                    
                   danceTwoFrontFlag = false;
                   danceTwoBackFlag = true;
                  }

                 }
               }
            }else
            {
             if (danceTwoBackCounter <=5 && danceTwoBackFlag == true )
             {
              if ((millis() - DancePrev_Millis) >=50)
              {
                digitalWrite(frontLeftBlink_OutPin,LOW);//turn off the 
                digitalWrite(backLeftBlink_OutPin,LOW);//Left blinkers
                digitalWrite(frontRightBlink_OutPin,LOW);
                digitalWrite(backRightBlink_OutPin,LOW);
                DancePrev_Millis = millis();
                danceTwoBackState =!danceTwoBackState;

                 if (danceTwoBackState == HIGH)
                 {
                  //Serial.print("\n mode4 waiting \n");
                  danceTwoBackCounter++;
                  if (danceTwoFrontCounter >= 5 && danceTwoBackCounter >=5)
                  {
                    danceTwoFrontCounter = 0;
                    danceTwoBackCounter = 0;
                   danceTwoFrontFlag = true;
                   danceTwoBackFlag = false;
                   stagecounter++;
                   if (stagecounter >=5)
                    {
                     stagecounter = 0;
                     danceMode = 1;
                    }
                  }
                 }
              }
             }
            }

                        break;                       
          }
        }
       }else
       {
       //turn off AllBlinkers
       digitalWrite(frontLeftBlink_OutPin,LOW);
       digitalWrite(frontRightBlink_OutPin,LOW);
       digitalWrite(backLeftBlink_OutPin,LOW);
       digitalWrite(backRightBlink_OutPin,LOW);
       }
}//blink


////////////////////////////////////////////
//      
//      void Horn();
// horn!
// it toggles horn relays according to  specified flags
//  it Has 4 Modes 
// modes will Change by  1 to 4 clicks Under 2seconds
/////////////////////////////////////////////

void Horn ()
{
 
 // hornclicks = 3;
    if (digitalRead(HornINpin) == HIGH)
  {
  //Serial.println("true");
        if (HornFlag == true)
        {
            //Serial.println("horNing");
           // {
             // case 1:
            if ((hornclicks == 1) && (millis() - buttonPrevMillis) > 300)
            {
                digitalWrite(RightHorn_OutPin, HIGH);
                digitalWrite(LeftHorn_OutPin, HIGH);
            } else if ((hornclicks == 2) && (millis() - buttonPrevMillis) > 300)
            {
      
                //Serial.print("\n case2 \n");
                if (hornCountA <= 5 && horn1Aflag == true)
                {
                    if ((millis() - Horn_delayMillis) >= 50)
                    {
                        digitalWrite(RightHorn_OutPin, LOW);
                        Horn_delayMillis = millis();
                        hornStateA = !hornStateA;
                        digitalWrite(LeftHorn_OutPin, hornStateA);
                        if (hornStateA == HIGH)
                        {
                            //  Serial.print("\n 1A \n");
                            hornCountA++;
                            if (hornCountA >= 5)
                            {

                                horn1Aflag = false;
                                horn1Bflag = true;
                            }

                        }
                    }
                } else
                {
                    if (hornCountB <= 5 && horn1Bflag == true)
                    {
                        if ((millis() - Horn_delayMillis) >= 50)
                        {
                            digitalWrite(LeftHorn_OutPin, LOW);
                            Horn_delayMillis = millis();
                            hornStateB = !hornStateB;
                            digitalWrite(RightHorn_OutPin, hornStateB);
                            if (hornStateB == HIGH)
                            {
                                //  Serial.print("\n 1B \n");
                                hornCountB++;
                                if (hornCountA >= 5 && hornCountB >= 5)
                                {
                                    hornCountA = 0;
                                    hornCountB = 0;
                                    horn1Aflag = true;
                                    horn1Bflag = false;

                                }
                            }
                        }
                    }
                }
                //hornclicks = 0;                                       
            } else if ((hornclicks == 3) && (millis() - buttonPrevMillis) > 300)
            {

                  //Serial.print("\n case3 \n");
                 ////mod2
                if ((millis() - Horn_delayMillis) > 100)
                {
                    Horn_delayMillis = millis();
                    if (hornModeTwoState == HIGH)
                    {
                        digitalWrite(LeftHorn_OutPin, LOW);
                        digitalWrite(RightHorn_OutPin, HIGH);
                        hornModeTwoState = false;
                    } else
                    {
                        digitalWrite(RightHorn_OutPin, LOW);
                        digitalWrite(LeftHorn_OutPin, HIGH);
                        hornModeTwoState = true;
                    }
                }
                //hornclicks = 0;
            } else if ((hornclicks == 4) && (millis() - buttonPrevMillis) > 300)
            {
                //  Serial.print("\n case4 \n");
                 //Mode 3 Wedding Mode ^_^
                if ((millis() - Horn_delayMillis) >= 100 && hornModThreeState == false && hornModeCStage == 1)
                {
                    // Serial.println("stage 1");
                    hornModeCStage = 2;
                    hornModThreeState = true;
                    Horn_delayMillis = millis();
                    digitalWrite(LeftHorn_OutPin, hornModThreeState);
                    digitalWrite(RightHorn_OutPin, hornModThreeState);
                } else if ((millis() - Horn_delayMillis) >= 50 && hornModThreeState == true && hornModeCStage == 2)
                {
                    // Serial.println("stage 2");
                    hornModeCStage = 3;
                    hornModThreeState = false;
                    Horn_delayMillis = millis();
                    digitalWrite(LeftHorn_OutPin, hornModThreeState);
                    digitalWrite(RightHorn_OutPin, hornModThreeState);
                } else if ((millis() - Horn_delayMillis) >= 100 && hornModThreeState == false && hornModeCStage == 3)
                {
                    //  Serial.println("stage 3");
                    hornModeCStage = 4;
                    hornModThreeState = true;
                    Horn_delayMillis = millis();
                    digitalWrite(LeftHorn_OutPin, hornModThreeState);
                    digitalWrite(RightHorn_OutPin, hornModThreeState);
                } else if ((millis() - Horn_delayMillis) >= 50 && hornModThreeState == true && hornModeCStage == 4)
                {
                    //  Serial.println("stage 4");
                    hornModeCStage = 5;
                    hornModThreeState = false;
                    Horn_delayMillis = millis();
                    digitalWrite(LeftHorn_OutPin, hornModThreeState);
                    digitalWrite(RightHorn_OutPin, hornModThreeState);
                } else if ((millis() - Horn_delayMillis) >= 200 && hornModThreeState == false && hornModeCStage == 5)
                {
                    //  Serial.println("stage 5");
                    hornModeCStage = 6;
                    hornModThreeState = true;
                    Horn_delayMillis = millis();
                    digitalWrite(LeftHorn_OutPin, hornModThreeState);
                    digitalWrite(RightHorn_OutPin, hornModThreeState);
                } else if ((millis() - Horn_delayMillis) >= 200 && hornModThreeState == true && hornModeCStage == 6)
                {
                    //  Serial.println("stage 6");
                    hornModeCStage = 7;
                    hornModThreeState = false;
                    Horn_delayMillis = millis();
                    digitalWrite(LeftHorn_OutPin, hornModThreeState);
                    digitalWrite(RightHorn_OutPin, hornModThreeState);
                } else if ((millis() - Horn_delayMillis) >= 250 && hornModThreeState == false && hornModeCStage == 7)
                {
                    // Serial.println("stage 7");
                    hornModeCStage = 8;
                    hornModThreeState = true;
                    Horn_delayMillis = millis();
                    digitalWrite(LeftHorn_OutPin, hornModThreeState);
                    digitalWrite(RightHorn_OutPin, hornModThreeState);
                } else if ((millis() - Horn_delayMillis) >= 250 && hornModThreeState == true && hornModeCStage == 8)
                {
                    // Serial.println("stage 8");
                    hornModeCStage = 9;
                    hornModThreeState = false;
                    Horn_delayMillis = millis();
                    digitalWrite(LeftHorn_OutPin, hornModThreeState);
                    digitalWrite(RightHorn_OutPin, hornModThreeState);
                } else if ((millis() - Horn_delayMillis) >= 250 && hornModThreeState == false && hornModeCStage == 9)
                {
                    //  Serial.println("stage 9");
                    hornModeCStage = 10;
                    hornModThreeState = true;
                    Horn_delayMillis = millis();
                    digitalWrite(LeftHorn_OutPin, hornModThreeState);
                    digitalWrite(RightHorn_OutPin, hornModThreeState);
                } else if ((millis() - Horn_delayMillis) >= 250 && hornModThreeState == true && hornModeCStage == 10)
                {
                    // Serial.println("stage 10");
                    hornModeCStage = 11;
                    hornModThreeState = false;
                    Horn_delayMillis = millis();
                    digitalWrite(LeftHorn_OutPin, hornModThreeState);
                    digitalWrite(RightHorn_OutPin, hornModThreeState);
                } else if ((millis() - Horn_delayMillis) >= 250 && hornModThreeState == false && hornModeCStage == 11)
                {
                    //  Serial.println("stage 11");
                    hornModeCStage = 1;
                }
                //hornclicks = 0;
            }
        }
  }else
  { 
 //   Serial.println("released");
   digitalWrite(LeftHorn_OutPin,LOW);
   digitalWrite(RightHorn_OutPin,LOW);
  }
} 
/// <summary>
/// Interrupt@ Horn KeyPress Listener
/// </summary>
void checkHornKey()
{

  if((digitalRead(HornINpin)) == HIGH)
  {
 
      if ((millis() - buttonPrevMillis) > 500)hornclicks = 0; //if no cicks comes after timeout so restart counter .
   if (millis() - lastDebounceTime > debounceDelay)
    {
      lastDebounceTime = millis();
     // Serial.println("intermillis");
      HornFlag = true;
      //cli();
      hornclicks++;
      if (hornclicks > 4 ) hornclicks = 1; // if we was  pressed the button more than 4 times so reset the click counter.
      buttonPrevMillis = millis();
    }
       Serial.print("pressing");
   Serial.println(hornclicks);
    Horn();
  }else if ((digitalRead(HornINpin)) == false) HornFlag = false;//this code is Unnecessary. will removed.


 
  //Serial.println("interout");
//Serial.print(hornclicks);
}

/// <summary>
/// ISR For Shake detecting Using Piezo Sensor. 
/// </summary>
void ListenForPiezo()
{
//Piezo was detected an shake.
    Serial.print(OutSerial_ShakeDetected_cmd);
    PiezoDetected = true;
    DoLock();
}

/// <summary>
/// Police Siren Lights turns on By MultiBlinker key Or HPiUI App 
/// </summary>
void blinkSiren()
    {
    if (m_doSiren)
    {


          switch (sirenCaseCounter)
          {
            case 1:


                //Serial.print("\n mod1 \n");
                if (sirenDanceRedCounter <= 5 && sirenDanceRedFLAG == true)
                    {
                    if ((millis() - sirenDancePrevMillis) >= 50)
                        {
                        digitalWrite(blueS_OutPin, LOW);//turn off the blues

                        sirenDancePrevMillis = millis();
                        sirenDanceREDState = !sirenDanceREDState;
                        digitalWrite(RedS_OutPin, sirenDanceREDState);//toggle the 

                        if (sirenDanceREDState == HIGH)
                            {
                           // Serial.print("\n 1A \n");
                            sirenDanceRedCounter++;
                            if (sirenDanceRedCounter >= 5)
                                {

                                sirenDanceRedFLAG = false;
                                sirenDanceBlueFLAG = true;
                                }

                            }
                        }
                    } else
                    {
                    if (sirenDanceBlueCounter <= 5 && sirenDanceBlueFLAG == true)
                        {
                        if ((millis() - sirenDancePrevMillis) >= 50)
                            {
                            digitalWrite(RedS_OutPin, LOW);//turn off the reds

                            sirenDancePrevMillis = millis();
                            sirenDanceBLUEState = !sirenDanceBLUEState;
                            digitalWrite(blueS_OutPin, sirenDanceBLUEState);

                            if (sirenDanceBLUEState == HIGH)
                                {
                            //    Serial.print("\n 1B \n");
                                sirenDanceBlueCounter++;
                                if (sirenDanceRedCounter >= 5 && sirenDanceBlueCounter >= 5)
                                    {
                                    sirenDanceRedCounter = 0;
                                    sirenDanceBlueCounter = 0;
                                    sirenDanceRedFLAG = true;
                                    sirenDanceBlueFLAG = false;
                                    SirenStageCounter++;
                                    if (SirenStageCounter >= 5)
                                        {
                                        SirenStageCounter = 0;
                                        sirenCaseCounter++;
                                        }
                                    }
                                }
                            }
                        }
                    }
                    break;
            case 2:
              //  Serial.print("\n mod1 \n");
                if (sirenDanceRedCounter <= 5 && sirenDanceRedFLAG == true)
                    {
                    if ((millis() - sirenDancePrevMillis) >= 50)
                        {
                        digitalWrite(blueS_OutPin, LOW);//turn off the blues

                        sirenDancePrevMillis = millis();
                        sirenDanceREDState = !sirenDanceREDState;
                        digitalWrite(RedS_OutPin, sirenDanceREDState);//toggle the 

                        if (sirenDanceREDState == HIGH)
                            {
                           // Serial.print("\n 1A \n");
                            sirenDanceRedCounter++;
                            if (sirenDanceRedCounter >= 5)
                                {

                                sirenDanceRedFLAG = false;
                                sirenDanceBlueFLAG = true;
                                }

                            }
                        }
                    } else
                    {
                    if (sirenDanceBlueCounter <= 5 && sirenDanceBlueFLAG == true)
                        {
                        if ((millis() - sirenDancePrevMillis) >= 50)
                            {
                            digitalWrite(RedS_OutPin, LOW);//turn off  red and blues
                            digitalWrite(blueS_OutPin, LOW);//
                            sirenDancePrevMillis = millis();
                            //because this code is copy-Pasted from previous case, Below Lines is unnecessary.
                            //they should be Modified!
                            // we should write code for waiting for 1000millisecond 
                           // sirenDanceBLUEState = !sirenDanceBLUEState;
                            //if (sirenDanceBLUEState == HIGH)
                               // {
                          //  Serial.print("\n 1B \n");
                            sirenDanceBlueCounter++;
                            if (sirenDanceRedCounter >= 5 && sirenDanceBlueCounter >= 5)
                                {
                                sirenDanceRedCounter = 0;
                                sirenDanceBlueCounter = 0;
                                sirenDanceRedFLAG = true;
                                sirenDanceBlueFLAG = false;
                                SirenStageCounter++;
                                if (SirenStageCounter >= 10)///because every stage runs two times for toggle
                                    {
                                    SirenStageCounter = 0;
                                    sirenCaseCounter = 1;
                                    }
                                }
                            // }
                            }
                        }
                    }
                    break;
          }
    } else
    {
    digitalWrite(RedS_OutPin, LOW);
    digitalWrite(blueS_OutPin, LOW);
    }
}


/// <summary>
/// toggles Audio Source Between UI And MCU For Certain Alarm Modes..
/// </summary>
/// <param name="ToMCU">: If True MCU Became Audio Source, UI Became Source If False.  </param>
void toggleSpeakerPin(bool ToMCU)
    {
    if (ToMCU)
        {
        digitalWrite(AudioSwitcher_OutPin, LOW);
        } else
        {
        digitalWrite(AudioSwitcher_OutPin, HIGH);
        }
    }

/// <summary>
/// !!NEED TO BE CORRECTED.TEMPORARY DEFINED!!
/// uint16_t GetEngineTemp
/// Reads Data From Engine's Temperature Sensor and Convert it to celcius.
/// </summary>
/// <returns>Current Engine's Temperature</returns>
float GetEngineTemp()
{ 
   // Serial.println(thermocouple.readCelsius());
   return thermocouple.readCelsius();
}

/// <summary>
/// Get Current Battery/Rectifier Voltage. Maximum 55vDC (can be changed To 24 Volts For More Accuracy).
/// 220K ohm's + 10Kohm's --> 22:1 divided.
/// </summary>
/// <returns>(float)0 To 90VDC </returns>
float getBatteryVoltage()
{

    double input_voltage = 0.0;
    double temp = 0.0;


    int analog_value = analogRead(VBattINpin);

    temp = (analog_value * 5.0) / 1024.0;

    input_voltage = (temp / (r2) / (r1 + r2));

    if (input_voltage < 0.1)
    {
        input_voltage = 0.0;
    }
    return input_voltage;
}

/// <summary>
/// void adjustIdleSpeed()
/// set's the Engine's idle speed. used for auto starting function.
/// and warms engine in cool temps. 
/// </summary>
/// 
void adjustIdleSpeed()
{
    uint16_t  temperature = GetEngineTemp();
    

    if ((millis() - idleSpeedPrevMillis) >= 1000)
    {
        idleSpeedPrevMillis = millis();
//Serial.println("tmp millis");

        if (temperature < 5)
        {
            // Ice
            SetIdleRPM(280);
        } else if (temperature >= 5 && temperature < 10)
        {
            // very cold temperature near ice
            SetIdleRPM(250);
        } else if (temperature >= 10 && temperature < 15)
        {
            // very cold temperature
            SetIdleRPM(230);
        } else if (temperature >= 15 && temperature < 20)
        {
            // cold temperature
            SetIdleRPM(200);
        } else if (temperature >= 20 && temperature < 25)
        {
            // cool temperature
            SetIdleRPM(180);
        } else if (temperature >= 25 && temperature < 30)
        {
            //normal temperature
            SetIdleRPM(170);
        } else if (temperature >= 30 && temperature < 35)
        {
            //warm temperature
            SetIdleRPM(160);
        } else if (temperature > 35)
        {
            SetIdleRPM(eep_minimumIdleRPM);
            RPMadjusted = true;
        }
    }

}

/// <summary>
/// void SetIdleRPM(byte _RPM)
/// Set Current Engine's RPM 
/// </summary>
/// <param name="_RPM">-> Specified RPM to Be Set. RPM Will Moltiplied By 10. Because Variable MEMORY Is too Small!</param>
void SetIdleRPM(uint16_t _RPM)
{
    uint16_t SpecifiedAngle = map(_RPM, 100, 280, eep_minServoAngle, eep_maxServoAngle);

    /// skip if Engine's Temperature not changed ( wait For Engine to warm Up).
    if (SpecifiedAngle != previousAngle)
    {
       IdleServo.write(SpecifiedAngle);
        previousAngle = SpecifiedAngle;
    }
}

/// <summary>
/// Auto Start Engine. 
/// </summary>
/// <param name="when">-> countDown Time For Start in Milliseconds.</param>
/// <param name="howmany">-> duration of pressing the Start Button in seconds. use 100 for force until engine start.  </param>
void AutoStart(unsigned int howmany)
{


        if (Switch_is_Open() == false)
        {
            TemporaryDOSwitch(true);
        }

        if (howmany > 0 )// if command comes from UI
        {
            
            
              
                if (StartCompleted == false)
                {
                    digitalWrite(ENGINE_Start_OutPin, HIGH);


                    if ((millis() - AutoStartDurrationMillis) > howmany)
                    {
                        digitalWrite(ENGINE_Start_OutPin, LOW);
                        StartCompleted = true;
                        AutoStartDurrationMillis = 0;
                        input = "";//Clear Serial Buffer Here.
                    }

                }
            
        } else if (howmany == 100)//Start Until Engine turned on.
        {
            if (StartCompleted == false)
            {
                digitalWrite(ENGINE_Start_OutPin, HIGH);
            }
            if (RPM >= 1000)
            {
                digitalWrite(ENGINE_Start_OutPin, LOW);
                StartCompleted = true;
            }

        }

}



/// <summary>
/// check for switch is open or no
/// </summary>
/// <returns>True If Switch Was Physically Opened.Else False returned.</returns>
bool Switch_is_Open()
{
  if((digitalRead(VBattINpin)) == HIGH ){
    return true;
  }else return false;
}

/// <summary>
/// Temporary Open Switch For Remote AutoStart
/// </summary>
/// <param name="open">-> Open or Close The Switch.</param>
void TemporaryDOSwitch(bool open)
{
    if (open == true)
    {
        digitalWrite(Switch_OutPin, HIGH);// Connect DC Power To System.
        digitalWrite(CDI_ShutDown_OutPin, LOW);//Disable CDI shutdown Pin.

    } else//Restore defaults
    {
        digitalWrite(Switch_OutPin, LOW);// Disconnect DC Power from System.
        if (digitalRead(VBattINpin) == LOW)//Swich is pysically Closed
        {
            digitalWrite(CDI_ShutDown_OutPin, LOW);// Switch was Closed  (physically), so restore CDI Shutdown Mode 
        }
    }
}

/// <summary>
/// Remote Control Listener.
/// </summary>
void ListenForRemoteControl()
{
    if (engPowerFlag == ENGINE_IS_ON)
    {
        // Lock Not Allowed. Only Remote Shutdown Allowed.
        if (digitalRead(RemoteShutDownINpin) == HIGH)//ShutDown ENGINE Remotely.
        {

            digitalWrite(CDI_ShutDown_OutPin, HIGH);
            RemoteShutDownFlag = true;
            RemoteShutDownLastMillis = millis();
        }
    } else
    {// Engine Is Off
        if (digitalRead(RemoteLSINpin) == HIGH)//Lock Or Silence Alarm 
        {
          if(remote_LSI_flag == false)
         {
          remote_LSI_flag = true;
            RemoteLSstate++;
            switch (RemoteLSstate)
            {
            case 1://Lock
            //Serial.println("lock");
                Silenced = false;//Disable Silence Mode If Silenced.
                lockflag = true;
                DoLock();
                break;
            case 2://SilenceAlarm
            //Serial.println("silence");
             Serial.print(OutSerial_AlarmSilenced_cmd);//Tell UI To Dont Call Owner.
                PiezoDetected = false;//Reset "Piezo Detected" Flag. 
                noTone(Piezo_OutPin);//Disable Alarm.
                Alarm = false;//Disable Alarm
                Silenced = true;
                RemoteLSstate = 0;//Reset The Counter
                break;
            
            }    
         }    
        }else if (digitalRead(RemoteLSINpin) == LOW)
        {
         //reset the flag 
         remote_LSI_flag = false;
        }
        if (digitalRead(RemoteUnlockINpin) == HIGH)//Unlock ENGINE
        {
          if (remote_unlock_flag == false)
          {
            remote_unlock_flag = true;
          RemoteLSstate = 0;//restore LSI flag
          noTone(Piezo_OutPin);//Disable Alarm.
            Alarm = false; //Disable Alarm.flags
            lockflag = false;////Reset "Lock" Flag. 
            PiezoDetected = false;//Reset "Piezo Detected" Flag. 
            detachInterrupt(digitalPinToInterrupt(ShakeSense_INpin));//disable Piezzo Interrupt
            digitalWrite(CDI_ShutDown_OutPin, LOW);//if Cdi was closed Open It.
           // pinMode(ShakeSense_INpin, OUTPUT);
          }
        }else if (digitalRead(RemoteUnlockINpin) == LOW)
        {
          remote_unlock_flag = false;//Reset The Flag.
        }
        if (digitalRead(RemoteStartINpin) == HIGH && remote_start_flag == false)//Press Start Button.
        {
          remote_start_flag  = true; //Set the Flag.
          if(Switch_is_Open() == false)
          {
          TemporaryDOSwitch(true);
          }
            digitalWrite(ENGINE_Start_OutPin, HIGH);

        } else if (digitalRead(RemoteStartINpin) == LOW)//Release Start Button.
        {
          remote_start_flag = false; //Reset The Flag.
            digitalWrite(ENGINE_Start_OutPin, LOW);
            TemporaryDOSwitch(false);
        }

    }//

    if ((digitalRead(RemoteShutDownINpin) == LOW) && RemoteShutDownFlag == true)// ShutDown Key Was Released. 
    {
        if ((millis() - RemoteShutDownLastMillis) >= 10000)//Wait For 10 Seconds To Engine's shutdown Complete.
        {
            RemoteShutDownLastMillis = millis();
            TemporaryDOSwitch(false);//close switch.
            digitalWrite(CDI_ShutDown_OutPin, LOW);
            RemoteShutDownFlag = false;
        }
    }

}

/// <summary>
/// Lock ENGINE.  
/// </summary>
void DoLock()
{
    if (lockflag == true)
    {
        //single Alarm
        single_Alarm = true;
        pinMode(ShakeSense_INpin, INPUT);//toggle Piezo to shake sensor
        attachInterrupt(digitalPinToInterrupt(ShakeSense_INpin), ListenForPiezo, CHANGE);//Attach Interrupt For Piezo sensor
        TemporaryDOSwitch(false);//close switch.
        DoAlarm();//play one time lock Alarm.
    }
    if (PiezoDetected == true)
    {
        //continous Alarm
        single_Alarm = false;
        Alarm = true;
        Alarm_Timer = millis();//Start the Alarm's Auto-Disable Timer
        DoAlarm();
    }

}

/// <summary>
/// Play's Alarm Sounds.
/// </summary>
void DoAlarm()
{
    if (Alarm == true)
    {
        if (Silenced == false)// if not silenced
        {
            blinkerstate = true;//Enable Blinkers.
            multiblink = true;//Enable Blinkers.

            switch (Alarmstage)
            {
            case 1:
                //Serial.println("1A");
                //play_tone_in_case = false;
                if ((micros() - Alarm_prevmicros) >= mod_A_delay)
                {
                    Alarm_prevmicros = micros();
                    if (flag == true)
                    {
                        A_freq++;
                        if (A_freq >= 768)
                        {
                            currentCounter++;
                            flag = false;
                        }
                    } else
                    {
                        A_freq--;
                        if (A_freq <= 511)
                            flag = true;

                    }
                }
                tone(Piezo_OutPin, A_freq);

                break;
            case 4:
               // Serial.println("4B");
                //play_tone_in_case = false;
                if ((millis() - Alarm_prevmillis) >= mod_D_delay)
                {
                    Alarm_prevmillis = millis();
                    if (D_freq <= 511)
                    {
                        D_freq = 912;
                    } else
                    {
                        D_freq = 511;
                        currentCounter++;
                    }
                }
                tone(Piezo_OutPin, D_freq);
                break;
            case 2:
               // Serial.println("2B");
                //play_tone_in_case = true;
                if ((micros() - Alarm_prevmicros) >= mod_B_delay)
                {
                    Alarm_prevmicros = micros();
                    if (flag == true)
                    {
                        mod3_wait++;
                        noTone(Piezo_OutPin);
                        if (mod3_wait >= 100)
                        {

                            B_freq = 912;
                            flag = false;
                            mod3_wait = 0;
                        }
                    } else
                    {
                        B_freq--;
                        tone(Piezo_OutPin, B_freq);
                        if (B_freq <= 511)
                        {
                            flag = true;
                            currentCounter++;
                        }
                    }
                }//micros
                break;
            case 3:
               // Serial.println("3C");
                //play_tone_in_case = true;
                if ((micros() - Alarm_prevmicros) >= mod_C_delay)
                {
                    Alarm_prevmicros = micros();
                    mod3_wait++;
                    if (mod3_wait >= 100)
                    {
                        mod3_wait = 0;
                        currentCounter++;
                        flag = !flag;
                    }
                    if (flag == true)
                    {
                        //mod3_wait++;
                        noTone(Piezo_OutPin);
                    } else
                    {
                        //freq--;
                        tone(Piezo_OutPin, C_freq);

                    }
                }//micros
                break;
            }


            if (currentCounter >= 10)
            {
                currentCounter = 0;
                Alarmstage++;
                flag = true;///reset the Flag
                mod3_wait = 0;//reset counter 
            }
            if (Alarmstage > 4)
            {
                Alarmstage = 1;
            }

        } else//Silent Mode. Just Blinkers.
        {
            blinkerstate = true;//Enable Blinkers.
            multiblink = true;//Enable Blinkers.
        }

        if ((millis() - Alarm_Timer) >= 20000)
        {
            Alarm_Timer = 0;//reset The Timer.
            Alarm = false;
            blinkerstate = false;//
            multiblink = false;
            noTone(Piezo_OutPin);
        }

    } else// Disable Alarm
    {
        Alarm = false;//Disable Alarm
        Alarm_Timer = 0;//reset The Timer.
        blinkerstate = false;//Disable Blinkers.
        multiblink = false;//Disable Blinkers.
    }
    //
    /////////Single Alarm
    if (single_Alarm == true)
    {
        blinkerstate = true;//Enable Blinkers.
        multiblink = true;//Enable Blinkers.
       /* Serial.println("s");*/
        if ((micros() - Alarm_prevmicros) >= Single_delay)
        {
            Alarm_prevmicros = micros();
            if (flag == true)
            {
                Single_freq++;
                tone(Piezo_OutPin, Single_freq);
                if (Single_freq >= 768)
                {
                    singlecount++;
                    //freq =768;
                    flag = false;
                }
            } else
            {
                Single_freq--;
                noTone(Piezo_OutPin);
                if (Single_freq <= 611)
                    flag = true;
            }
        }//micros


        if (singlecount >= 2)
        {
            blinkerstate = false;//disable Blinkers.
            multiblink = false;//disable Blinkers.
            singlecount = 0;
            noTone(Piezo_OutPin);
            single_Alarm = false;
        }
        Blink();

    }///single
}//

//Some Bad Drivers Who Turns their UPLights in Two Way Roads Forced Me To Write This Function.ICAN'T SEE MY FRONT O_o
/// <summary>
/// Blinks Headlight For Specified Duration Or Permanently Until off
/// </summary>
/// <param name="mode">Blink Duration Or ZERO For Permanent.</param>
void BlinkHeadLight()
{
    if (Headblink_flag == true)
    {
        headlightFlag = false;// disable headlight wich turned by button

        if (Headblink_blinkmode > 0)
        {
            ///TimeOut
            if ((millis() - Headblink_timeout) >= Headblink_blinkmode)/// disable blink
            {
                Headblink_timeout = millis();

                Headblink_flag = false;//disable blink
                
            }
            if ((millis() - Headblink_prevMillis) >= Headblink_delay)
            {
                Headblink_prevMillis = millis();
                Headblink_state = !Headblink_state;
                digitalWrite(headlight_OutPin,Headblink_state == true ? HIGH : LOW); //Toggle Headlight.
            }

        } else/// So Blink Until Disabled
        {
            if ((millis() - Headblink_prevMillis) >= Headblink_delay)
            {
                Headblink_prevMillis = millis();
                Headblink_state = !Headblink_state;
                digitalWrite(headlight_OutPin, Headblink_state == true ? HIGH : LOW); //Toggle Headlight.
            }
        }
    } else
    {
        headlightFlag = true;// Enable Normall Light.
        // if Headlight Was Not Turned On By button It Will Turned oFF In Loop();
    }
}


/// <summary>
/// Get Current Fuel Level.
/// </summary>
/// <returns> 0 To 100% </returns>
float GetFuelLevel()
{
    int Fuel_Level = 0;
    int in_voltage = 0;
    in_voltage = analogRead(FuelGauge_IN);
    Fuel_Level = map(in_voltage, 0, 1023, 0, 100);
    return Fuel_Level;
}

//END Of File