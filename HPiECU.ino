/********************************************************************************************
/                                      HPi Source File                                      *             
/                               copyright (c) 2023 HPi Studio                               *
/                                                                                           *
/                                                                                           *
/        des: Honda 200cc hp+ SmartECU                                                      *
/                                                                                           *
/        1402/06/01 Created By Hosein Pirani.                                               *
/                                                                                           *
/       Modified in  fri. 1402/06/17 from 15:00 To 19:00 (blinkers)                         *
/       Last modification: mon. 1402/09/27 from 15:35 To 18:40(Idle RPM,-unused variables)  *
/TODO:   !!!!!!!!!!-->     Free Up More MEMORY!!! <--!!!!                                   *
/TODO: Test LED FLASHERs                                                                    *
/TODO: Add Battery voltage Level indicator                                                  *
/TODO: !!engine temp SenSor!! + indicator                                                   *
/TODO: ,fix if() bug on loop(),Serial Communic.edit RPM Meter   ,!!engine temp SenSor!!     *
/TODO: TEST And Debug idle Speed Adjuster                                                   *
/*******************************************************************************************/


#include <EEPROM.h>
///Servo
#include <ServoTimer2.h>
// Servo For Adjusting Idle throttle(ENGINEs Idle RPM)
ServoTimer2 IdleServo;


//No pin is curently unused.


constexpr auto headlightPin = 2;
constexpr auto frontLeftBlinkPin = 4;
constexpr auto backLeftBlinkPin = 5;
constexpr auto frontRightBlinkPin = 6;
constexpr auto backRightBlinkPin = 7;
constexpr auto LeftHornPin = 8;
constexpr auto RightHornPin = 9;
constexpr auto RedSPin = 10;
constexpr auto blueSPin = 11;
constexpr auto BrakePin = 12;
constexpr auto AudioSwitcherPin = A0; /// Used For Swith Between Normal HIFI Speaker And Piezo Buzzer For Siren. 
constexpr auto EMERGENCYShutDownPin = A4; /// For Remote ShutDown
constexpr auto MotorPin = A6; /// steper|DC|servo motor for adjusting idle throttle(adjust engine's idle rpm)

// Input Keys
constexpr auto LturnINpin = A2;
constexpr auto RturnINpin = A3;
constexpr auto HEADLightINpin = A5;
constexpr auto BrakeINpin = 13;
constexpr auto HornINpin = 3;
constexpr auto VBattIN = A1;//for Measuring Battery Voltage for 15v -> r1 = 4.7k , r2 = 2.4k output = 5v2 max. for 12 v 4.7k ,6.8k
constexpr auto TempSenseIn = A7;//Engine Temprature SenSor Pin.

///////////RPM Meter
bool MeasEnd = 0;
uint16_t T1OVF_Counter = 0;
unsigned long input_Rissing_time = 0, input_Falling_time = 0, input_TOP_time = 0, Freq = 0;
const unsigned long Timer1_prescaler_freq = 250000;//2000000
unsigned long rpmPrvmillis = 0;
int RPM = 0;

///Idle RPM
unsigned long idleSpeedPrevMillis = 0;
bool RPMadjusted = false;
uint16_t Enginetemperature = 0;
//!!!!!!!!!!!!!
//! bellow lines should removed for final upload!
//! //////////////
#ifdef __AVR_ATmega16__///because im testing it on both atmega16 and atmega328
#define TIMERMASK TIMSK
#define RPM_PULSE_IN 14
#else 
#define TIMERMASK TIMSK1
#define RPM_PULSE_IN 12
#endif
///////////
///HORN
unsigned short debounceDelay = 200;
unsigned long delayMillis = 0,hornPrevMillis = 0, buttonPrevMillis = 0, lastDebounceTime = 0;
byte hornclicks = 0;
bool buttunstate= false;
//HORN modes
byte hornCountA=0,hornCountB=0;
bool horn1Aflag = true, horn1Bflag = false;
bool hornStateA = false,hornStateB = false;
bool hornModeTwoState = true;
bool hornModThreeState = false; // current state of patern. below line is our patern 
byte hornModeCStage  =1;

///////////
/////<blinkers>
unsigned int blinkInterval = 250; // normal blinkers on/of delay in ms.
unsigned long  prevMillis = 1000; //for millis();. it used instead of old depricated delay() .
///MODE2
bool danceTwoFrontFlag = true,danceTwoBackFlag = false,danceTwoFrontState = false,danceTwoBackState = false; // states
byte danceTwoFrontCounter = 0,danceTwoBackCounter = 0; // blink counters
///
byte danceMode =1;// current Mode
byte danceblinkcounter =0;// how many times current mode repeated?.
byte stagecounter = 0;// current stage play counter.
//
//normal blink
bool state = false;
bool blinkerstate = false;
bool multiblink = false;
bool Leftfrontblinkerstate = false;
bool Rightfrontblinkerstate = false;
bool Leftbackblinkerstate = false;
bool Rightbackblinkerstate = false;
bool blinkdance = false;

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
//flags
constexpr auto ENGINE_IS_OFF = false;
constexpr auto ENGINE_IS_ON = true ;
/// input keys Flag
bool headlightFlag = false, lturnflag = false, rturnflag = false, brakeflag = false, engPowerFlag = ENGINE_IS_OFF; //parsing keys
//EEPROM DATA
int eep_blinkinterval = 300; // default Delay For Nomal Blinking
byte eep_blinkintervalAddress = 1; // Address Off Interval Holder
///</blinkers>
//Serial
String   input = "";
bool commandFromUI = false;///  any blink command comes from UI (?)
/*
String SerialInputCommands[] ={"off","on","headlight:on","headlight:off","leftfrontblink","leftbackblink","rightfrontblink",
"rightbackblink","brake","lefthorn","righthorn","smalllight","DoSirenLight","TurnOffSiren","multiblink:on","multiblink:off",
"lturn:blink","rturn:blink","lturnblink:off","rturnblink:off","blinkdanceOn","blinkdanceOff","SetBlinkInterVal:300","engineState","EMERGENCYSHOTDOWN","AllowStart"};

String SerialOUTPUTCommands[] ={"off","on","Engine Is OFF(EOF)","Engine Is ON(EON)","RPM:000,SmallLight,Uplight,DownLight,UPlightBlink","L","l","R","r","M","m"};
*/
ISR(TIMER1_OVF_vect)
    {
    T1OVF_Counter++;///RPM Meter
    }

ISR(TIMER1_CAPT_vect)
    {
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
                Freq = 0;
                } else
                {
                Freq = (Timer1_prescaler_freq / input_TOP_time);
                ///freq means revolution per second(RPS) and RPM Means Revolution per Minute
                RPM = Freq * 60;
                }
                T1OVF_Counter = 0;
                MeasEnd = 0;
        }
    }
void setup() 
{
  analogReference(DEFAULT);
  Serial.begin(115200);

  //Outputs
  pinMode(headlightPin,OUTPUT);
  pinMode(backLeftBlinkPin,OUTPUT);
  pinMode(frontLeftBlinkPin,OUTPUT);
  pinMode(backRightBlinkPin,OUTPUT);
  pinMode(frontRightBlinkPin,OUTPUT);
  pinMode(BrakePin,OUTPUT);
  pinMode(LeftHornPin,OUTPUT);
  pinMode(RightHornPin,OUTPUT);
  pinMode(blueSPin,OUTPUT);
  pinMode(RedSPin,OUTPUT);
  pinMode(AudioSwitcherPin,OUTPUT);
  pinMode(EMERGENCYShutDownPin, OUTPUT);
  pinMode(MotorPin, OUTPUT);
  ///Inputs
  pinMode(LturnINpin, INPUT);
  pinMode(RturnINpin, INPUT);
  pinMode(HEADLightINpin, INPUT);
  pinMode(BrakeINpin, INPUT);
  pinMode(HornINpin, INPUT);
  pinMode(VBattIN, INPUT);
  pinMode(TempSenseIn, INPUT);
  ///Horn Button Listener
  attachInterrupt(digitalPinToInterrupt(HornINpin), checkHornKey, CHANGE);
  //RPM Meter
  TCCR1A = 0;           // Init Timer1A
  TCCR1B = 0;           // Init Timer1B
  TCCR1B |= B11000011;  // (B11000010) Internal Clock, Prescaler = 16, ICU Filter EN, ICU Pin RISING
  TIMERMASK |= B00100001;  // Enable Timer OVF & CAPT Interrupts
  ////SERVO
  IdleServo.attach(MotorPin);
}

void loop() 
{
 
  byte firstcheck = 0;
  uint16_t add = 0;
  firstcheck = EEPROM[add];
  if ( firstcheck != 123 )
  {
    EEPROM.put(add, 123);
    delay(5);
    EEPROM.put(eep_blinkintervalAddress, eep_blinkinterval);
    delay(5);
 }else 
  {
   eep_blinkinterval = EEPROM.get(eep_blinkintervalAddress,eep_blinkinterval);
 }

 ///// RPM Adjusting
 if (RPMadjusted == false) adjustIdleSpeed();

//////brake
      if (digitalRead(BrakeINpin) == HIGH ) 
    {   
    digitalWrite(BrakePin, HIGH);
    digitalWrite(BrakePin,HIGH);
    brakeflag = true;
    }
    else {
      if (brakeflag)
      {
    digitalWrite(BrakePin, LOW);
    digitalWrite(BrakePin,LOW);
      }

    }
    //headlight
  if (digitalRead(HEADLightINpin) == HIGH)
   {
    digitalWrite(headlightPin, HIGH);
    headlightFlag = true;
    
   }else  //headLight Off

    {
      if (headlightFlag == true)
      {
    digitalWrite(headlightPin, LOW);
    Serial.print("\nHEAD off\n");
    headlightFlag = false;
      }


    }
/////left turn
  if (digitalRead(LturnINpin) == HIGH )
  {
      if(!blinkInterval)EEPROM.get(eep_blinkintervalAddress,blinkInterval);
      Rightfrontblinkerstate = false;//turn off
      Rightbackblinkerstate = false;// the others
      if (blinkdance) Serial.print("danceOff");   //UpdateUI  
      blinkdance = false;//off
      multiblink = false;//off
      //

       blinkerstate = true;
    Leftfrontblinkerstate = true;
    Leftbackblinkerstate = true;
     lturnflag = true;

  }  else // left turn OFF


  {
    if (lturnflag == true)
    {
     
        digitalWrite(backLeftBlinkPin,LOW);
      digitalWrite(frontLeftBlinkPin,LOW);
         blinkerstate = false;
      Leftfrontblinkerstate = false;
      Leftbackblinkerstate = false;
      multiblink = false;
      lturnflag = false;
      if (blinkdance) Serial.print("danceOff");   //UpdateUI  
      blinkdance = false;
    }
 
 }
////right turn
  if (digitalRead(RturnINpin) == HIGH)
   {
      if (!blinkInterval)EEPROM.get(eep_blinkintervalAddress,blinkInterval);//interval
      // check for multiblink
      if ((Leftfrontblinkerstate == true) && (Leftbackblinkerstate == true))
      {
          Leftfrontblinkerstate = false;
          Rightfrontblinkerstate = false;
          Rightbackblinkerstate = false;
          Rightfrontblinkerstate = false;
          if (blinkdance) Serial.print("danceOff");   //UpdateUI  
          blinkdance = false;//
          //
          blinkerstate = true;//
          multiblink = true;
      } else
      {
          multiblink = false;      ///the
          digitalWrite(backLeftBlinkPin, LOW);//other
          digitalWrite(frontLeftBlinkPin, LOW);//blinkers
          input = ""; //clear the buffer
          if (blinkdance) Serial.print("danceOff");   //UpdateUI  
          blinkdance = false;//
          blinkerstate = true;//
          Rightbackblinkerstate = true;//same as above 
          Rightfrontblinkerstate = true;//turn off other
          rturnflag = true;
      }
  
   } else // Right Turn off
   {
    if (rturnflag == true)
    {
      digitalWrite(backRightBlinkPin,LOW);
      digitalWrite(frontRightBlinkPin,LOW);
       blinkerstate = false;
       Rightbackblinkerstate = false;
       Rightfrontblinkerstate = false;
       multiblink = false;
       rturnflag = false;
       if (blinkdance) Serial.print("danceOff");   //UpdateUI  
       blinkdance = false;
    }
   }
/////////RPM METER   
  if ((millis() - rpmPrvmillis) >= 1000)
           {  
            rpmPrvmillis =   millis();             
              if(Freq <=0)
              {  
                  RPM = 0;
                  if (engPowerFlag == ENGINE_IS_ON)
                  {
                      engPowerFlag = ENGINE_IS_OFF;
                      Serial.print("EOF");
                  }
              }else 
               {
                  
                  prevMillis = millis();
                  Serial.println("Frequency:");
                  Serial.println(Freq);
                  Serial.println("RPM:");
                  Serial.println((Freq * 60));
                  if (engPowerFlag == ENGINE_IS_OFF)
                  {
                      engPowerFlag = ENGINE_IS_ON;
                      Serial.print("EON");
                  }
              }           
          }
  // read incomming commands
  if (Serial.available())
    {
       input = Serial.readStringUntil('\n');
     
    }
  /// turn on main lights///////
  if (input == "headlight:on")
     {
      digitalWrite(headlightPin,HIGH);
      input ="";

     }//
   //////turn off main lights///////////
   if ((input == "headlight:off"))
       {
        if (headlightFlag ==false)
        {
          digitalWrite(headlightPin,LOW);
          input ="";
        }
       }////////
   // engine State Called.
   if (input == "engineState")
     {
        if (engPowerFlag ==ENGINE_IS_OFF)
        {
          Serial.print("Engine Is Off");
          
        }
        else {
        Serial.print("Engine Is ON");
        }
        input ="";
      }////////   
   // turn on Police Lights
   if (input == "DoSirenLight")
       {
       m_doSiren = true;
       input = "";
       blinkSiren();

       }
   // turn off Police Lights
   if (input == "TurnOffSiren")
       {
       m_doSiren = false;
       input = "";
       }
   //Load speaker,Unload BUZZER
   if (input == "BuzzerToSpeaker")
       {
       toggleSpeakerPin(false);
       }
   //onload loud speaker,Load BUZZER
   if (input == "speakerToBuzzer")
       {
       toggleSpeakerPin(true);
       }
   // Emergency. Called By UI (recieved Remotely)
   if (input == "EMERGENCYSHOTDOWN")
       {
       analogWrite(EMERGENCYShutDownPin, HIGH);
       }
   // Can Start Engine
   if (input == "AllowStart")
       {
       analogWrite(EMERGENCYShutDownPin, LOW);
       }
    //////turn off All Blinkers ///////////
  if ((input == "lturnblink:off") || (input == "rturnblink:off")  || (input == "multiblink:off")  || (input == "blinkdanceOff") )
  {
      if ((rturnflag == false) && (lturnflag == false))
      {// if physical button was not pressed
          Serial.print("off All LEDs");
          Leftfrontblinkerstate = false;
          Rightfrontblinkerstate = false;
          Rightbackblinkerstate = false;
          Rightfrontblinkerstate = false;
          blinkerstate = false;
          multiblink = false;
          digitalWrite(backLeftBlinkPin, LOW);
          digitalWrite(frontLeftBlinkPin, LOW);
          digitalWrite(backRightBlinkPin, LOW);
          digitalWrite(frontRightBlinkPin, LOW);
          input = ""; //clear the buffer
          if (blinkdance) Serial.print("danceOff");   //UpdateUI  
          blinkdance = false;
      }
  }////////

 //////////checks for Left blinker///////////////
  if (input == "lturn:blink" )
  {
   Serial.print("lturn:blink:");

     blinkerstate = true;
    Leftfrontblinkerstate = true;
    Leftbackblinkerstate = true;
    input ="";
    
  }
 ////////////////////////
if(input.startsWith("SetBlinkInterVal:"))
{

  eep_blinkinterval = input.substring(17).toInt();
if (eep_blinkinterval >0) {
 EEPROM.update(eep_blinkintervalAddress, eep_blinkinterval);

}


}
 ///////commands from UI//////////////////
  if (input == "rturn:blink")
   {
       blinkerstate = true;
       Rightbackblinkerstate = true;
       Rightfrontblinkerstate = true;
       input ="";    
   }  
  //multi blink
  if (input == "multiblink:on")
  {   
     blinkerstate = true;
     multiblink = true;
     input ="";
  }

  if (input == "blinkdanceOn")
  {
   blinkerstate = true;
   blinkdance = true;
  }

//call  functions  
     Horn(); // for horn
 Blink();//for blinkers
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
        if (Leftfrontblinkerstate && Leftbackblinkerstate)
        {        
          if ((millis() - prevMillis) >= blinkInterval)
             {                          
              digitalWrite(frontLeftBlinkPin,state);
               digitalWrite(backLeftBlinkPin,state);  
               /// call ui app
               if (state)
               {
                   Serial.print("L");
               } else
               {
                   Serial.print("l");
               }
               state =!state;
              prevMillis = millis();
             }
         }//Left Turn
        //Right Turn
         if (Rightbackblinkerstate && Rightfrontblinkerstate)
         {
                  if (millis() - prevMillis >= blinkInterval)
           {
                   digitalWrite(backRightBlinkPin,state);
                   digitalWrite(frontRightBlinkPin,state);
                  state =!state;
                  if (state)
                  {
                      Serial.print("R");
                  } else
                  {
                      Serial.print("r");
                  }
             prevMillis = millis();
           }
        }//
         //Multiblink
         if (multiblink)
          {
            if (millis() - prevMillis >= (blinkInterval)){
                              digitalWrite(backRightBlinkPin,!state);
                   digitalWrite(frontRightBlinkPin,!state);
                   digitalWrite(frontLeftBlinkPin,!state);
                   digitalWrite(backLeftBlinkPin,!state);
                   digitalWrite(backRightBlinkPin, !state);
                   if (state)//Update UI
                   {
                       Serial.print("M");
                   } else
                   {
                       Serial.print("m");
                   }
             state =!state; 
              prevMillis = millis();
            }
          }
        if (blinkdance)
        {
            Serial.print("dance");   //UpdateUI  
            if (danceMode >= 5 ) danceMode = 1;
          switch (danceMode)
          {
                   case 1:
                   if ((millis() - delayMillis) >= 300)
                   {
                    danceblinkcounter++;
                    switch (danceblinkcounter) 
                    {             
                    case 1:         
                      digitalWrite(frontLeftBlinkPin,HIGH);
                      digitalWrite(backLeftBlinkPin,LOW);
                     break;
                     case 2:
                     digitalWrite(frontLeftBlinkPin,LOW);
                     digitalWrite(frontRightBlinkPin,HIGH);
                     break;
                     case 3:
                     digitalWrite(frontRightBlinkPin,LOW);
                     digitalWrite(backRightBlinkPin,HIGH);
                     break;
                     case 4:
                     digitalWrite(backRightBlinkPin,LOW);
                     digitalWrite(backLeftBlinkPin,HIGH);
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


                     Serial.print("\n case2 \n");
              if (danceTwoFrontCounter <=5 && danceTwoFrontFlag == true)
            {            
              if ((millis() - delayMillis) >= 50)
              {
                digitalWrite(backLeftBlinkPin,LOW);//turn off the 
                digitalWrite(backRightBlinkPin,LOW);//back blinkers
                delayMillis = millis();
                danceTwoFrontState = !danceTwoFrontState;
                digitalWrite(frontLeftBlinkPin,danceTwoFrontState);//toggle the 
                digitalWrite(frontRightBlinkPin,danceTwoFrontState);// front blinkers.
                if (danceTwoFrontState == HIGH)
                {
                  Serial.print("\n 1A \n");
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
               if (millis() - delayMillis >=50)
               {
                digitalWrite(frontLeftBlinkPin,LOW);//turn off the 
                digitalWrite(frontRightBlinkPin,LOW);//front blinkers
                delayMillis = millis();
                danceTwoBackState =!danceTwoBackState;
                digitalWrite(backLeftBlinkPin,danceTwoBackState);
                digitalWrite(backRightBlinkPin,danceTwoBackState);
                 if (danceTwoBackState == HIGH)
                 {
                  Serial.print("\n 1B \n");
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
                     Serial.print("\n case3 \n");
           if (danceTwoFrontCounter <=5 && danceTwoFrontFlag == true)
           {            
              if ((millis() - delayMillis) >= 50)
              {
                digitalWrite(frontRightBlinkPin,LOW);//turn off the 
                digitalWrite(backRightBlinkPin,LOW);//Right blinkers
                delayMillis = millis();
                danceTwoFrontState = !danceTwoFrontState;
                digitalWrite(frontLeftBlinkPin,danceTwoFrontState);//toggle the 
                digitalWrite(backLeftBlinkPin,danceTwoFrontState);// Left blinkers.
                if (danceTwoFrontState == HIGH)
                {
                  Serial.print("\n 1A \n");
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
              if (millis() - delayMillis >=50)
              {
                digitalWrite(frontLeftBlinkPin,LOW);//turn off the 
                digitalWrite(backLeftBlinkPin,LOW);//Left blinkers
                delayMillis = millis();
                danceTwoBackState =!danceTwoBackState;
                digitalWrite(frontRightBlinkPin,danceTwoBackState);
                digitalWrite(backRightBlinkPin,danceTwoBackState);
                 if (danceTwoBackState == HIGH)
                 {
                  Serial.print("\n 1B \n");
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
              if ((millis() - delayMillis) >= 50)
              {
                delayMillis = millis();
                danceTwoFrontState = !danceTwoFrontState;
                digitalWrite(frontLeftBlinkPin,danceTwoFrontState);///Toggle
                digitalWrite(frontRightBlinkPin,danceTwoFrontState);///the
                digitalWrite(backLeftBlinkPin,danceTwoFrontState);///All
                digitalWrite(backRightBlinkPin,danceTwoFrontState);//Blinkers
                if (danceTwoFrontState == HIGH)
                {
                  Serial.print("\n 1A \n");
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
              if (millis() - delayMillis >=50)
              {
                digitalWrite(frontLeftBlinkPin,LOW);//turn off the 
                digitalWrite(backLeftBlinkPin,LOW);//Left blinkers
                digitalWrite(frontRightBlinkPin,LOW);
                digitalWrite(backRightBlinkPin,LOW);
                delayMillis = millis();
                danceTwoBackState =!danceTwoBackState;

                 if (danceTwoBackState == HIGH)
                 {
                  Serial.print("\n mode4 waiting \n");
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
  //Serial.println("horn");
 // hornclicks = 3;
    if (digitalRead(3) == HIGH)
  {
       // Serial.println("horNing..");
      // {
        // case 1:
        if ((hornclicks == 1) && (millis() - buttonPrevMillis) > 300)
        {  
          digitalWrite(RightHornPin,HIGH);
          digitalWrite(LeftHornPin,HIGH);
        }else if ((hornclicks == 2) && (millis() - buttonPrevMillis) > 300)
        {
          
          Serial.print("\n case2 \n");
          if (hornCountA <=5 && horn1Aflag == true)
          {            
              if ((millis() - delayMillis) >= 50)
              {
                digitalWrite(RightHornPin,LOW);
                delayMillis = millis();
                hornStateA = !hornStateA;
                digitalWrite(LeftHornPin,hornStateA);
                if (hornStateA == HIGH)
                {
                  Serial.print("\n 1A \n");
                  hornCountA++;
                  if (hornCountA >=5 )
                  {
                    
                   horn1Aflag = false;
                   horn1Bflag = true;
                  }

                 }
               }
          }else
          {
           if (hornCountB <=5 && horn1Bflag == true )
           {
              if (millis() - delayMillis >=50)
              {
                digitalWrite(LeftHornPin,LOW);
                delayMillis = millis();
                hornStateB =!hornStateB;
                digitalWrite(RightHornPin,hornStateB);
                 if (hornStateB == HIGH)
                 {
                  Serial.print("\n 1B \n");
                  hornCountB++;
                  if (hornCountA >= 5 && hornCountB >=5)
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
        }else if ((hornclicks == 3) && (millis() - buttonPrevMillis) > 300)
        {
        
           Serial.print("\n case3 \n");
          ////mod2
            if (millis() - delayMillis > 100)
             {
              delayMillis = millis();
              if (hornModeTwoState == HIGH)
              {
              digitalWrite(LeftHornPin,LOW);
              digitalWrite(RightHornPin,HIGH);
              hornModeTwoState = false;
              }else
               {
                digitalWrite(RightHornPin,LOW);
                digitalWrite(LeftHornPin, HIGH);
                hornModeTwoState = true;
               }
             }
         //hornclicks = 0;
        }else if ((hornclicks == 4) && (millis() - buttonPrevMillis) > 300)
        {       
           Serial.print("\n case4 \n");
          //Mode 3 Wedding Mode ^_^
          if (millis() - delayMillis >= 100 && hornModThreeState == false && hornModeCStage == 1)
          {
            Serial.println("stage 1");
            hornModeCStage = 2;
           hornModThreeState = true;
           delayMillis = millis();
           digitalWrite(LeftHornPin ,hornModThreeState);
           digitalWrite(RightHornPin, hornModThreeState);
          } else if (millis() - delayMillis >= 50 && hornModThreeState == true && hornModeCStage == 2)
            {
              Serial.println("stage 2");
              hornModeCStage = 3;
              hornModThreeState = false;
              delayMillis = millis();
              digitalWrite(LeftHornPin,hornModThreeState);
              digitalWrite(RightHornPin,hornModThreeState);
            }else if (millis() - delayMillis >=100 && hornModThreeState == false && hornModeCStage == 3)
             {
              Serial.println("stage 3");
              hornModeCStage = 4;
              hornModThreeState = true;
              delayMillis = millis();
              digitalWrite(LeftHornPin,hornModThreeState);
              digitalWrite(RightHornPin,hornModThreeState);              
             } else if (millis() - delayMillis >=50 && hornModThreeState == true && hornModeCStage == 4)
               {
                Serial.println("stage 4");
                hornModeCStage = 5;
                hornModThreeState = false;
                delayMillis = millis();
              digitalWrite(LeftHornPin,hornModThreeState);
              digitalWrite(RightHornPin,hornModThreeState);                
               } else if (millis() - delayMillis >= 200 && hornModThreeState == false && hornModeCStage == 5)
                 {
                  Serial.println("stage 5");
                  hornModeCStage = 6;
                  hornModThreeState = true;
                  delayMillis = millis();
                  digitalWrite(LeftHornPin,hornModThreeState);
                  digitalWrite(RightHornPin,hornModThreeState);
                 } else if (millis() - delayMillis >=200 && hornModThreeState ==true && hornModeCStage == 6)
                   {
                    Serial.println("stage 6");
                    hornModeCStage = 7;
                    hornModThreeState = false;
                    delayMillis = millis();
                     digitalWrite(LeftHornPin,hornModThreeState);
                     digitalWrite(RightHornPin,hornModThreeState);
                   } else if (millis() - delayMillis >=250 && hornModThreeState == false && hornModeCStage == 7)
                     {
                      Serial.println("stage 7");
                      hornModeCStage = 8;
                      hornModThreeState = true;
                      delayMillis = millis();
                      digitalWrite(LeftHornPin,hornModThreeState);
                      digitalWrite(RightHornPin,hornModThreeState);
                     } else if (millis() - delayMillis >= 250 && hornModThreeState == true && hornModeCStage == 8)
                       {
                        Serial.println("stage 8");
                        hornModeCStage = 9;
                        hornModThreeState = false;
                        delayMillis = millis();
                        digitalWrite(LeftHornPin,hornModThreeState);
                        digitalWrite(RightHornPin,hornModThreeState);
                       } else if (millis() - delayMillis >=250 && hornModThreeState ==false && hornModeCStage == 9)
                         {
                          Serial.println("stage 9");
                          hornModeCStage = 10;
                          hornModThreeState = true;
                          delayMillis = millis();
                          digitalWrite(LeftHornPin,hornModThreeState);
                          digitalWrite(RightHornPin,hornModThreeState);
                         }else if (millis() - delayMillis >=250 && hornModThreeState ==true && hornModeCStage == 10)
                         {
                          Serial.println("stage 10");
                          hornModeCStage = 11;
                          hornModThreeState = false;
                          delayMillis = millis();
                          digitalWrite(LeftHornPin,hornModThreeState);
                          digitalWrite(RightHornPin,hornModThreeState);
                         }else if (millis() - delayMillis >=250 && hornModThreeState ==false && hornModeCStage == 11)
                         {
                          Serial.println("stage 11");
                          hornModeCStage = 1;
                         }
         //hornclicks = 0;
        }
  }else
  { 
 //   Serial.println("released");
   digitalWrite(LeftHornPin,LOW);
   digitalWrite(RightHornPin,LOW);
  }
} 
///Horn Button Parser(called By Interrupt)
void checkHornKey()
{

  if((digitalRead(3)) == HIGH)
  {
    //Serial.println("pressing");
   // Serial.print(hornclicks);
      if ((millis() - buttonPrevMillis) > 500)hornclicks = 0; //if no cicks comes after timeout so restart counter .
   if (millis() - lastDebounceTime > debounceDelay)
    {
      lastDebounceTime = millis();
     // Serial.println("intermillis");
      buttunstate = true;
      //cli();
      hornclicks++;
      if (hornclicks > 4 ) hornclicks = 1; // if we was  pressed the button more than 4 times so reset the click counter.
      buttonPrevMillis = millis();
    }
    
  }else buttunstate = false;//this code is Unnecessary. will removed.


 
  //Serial.println("interout");
//Serial.print(hornclicks);
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


                Serial.print("\n mod1 \n");
                if (sirenDanceRedCounter <= 5 && sirenDanceRedFLAG == true)
                    {
                    if ((millis() - sirenDancePrevMillis) >= 50)
                        {
                        digitalWrite(blueSPin, LOW);//turn off the blues

                        sirenDancePrevMillis = millis();
                        sirenDanceREDState = !sirenDanceREDState;
                        digitalWrite(RedSPin, sirenDanceREDState);//toggle the 

                        if (sirenDanceREDState == HIGH)
                            {
                            Serial.print("\n 1A \n");
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
                        if (millis() - sirenDancePrevMillis >= 50)
                            {
                            digitalWrite(RedSPin, LOW);//turn off the reds

                            sirenDancePrevMillis = millis();
                            sirenDanceBLUEState = !sirenDanceBLUEState;
                            digitalWrite(blueSPin, sirenDanceBLUEState);

                            if (sirenDanceBLUEState == HIGH)
                                {
                                Serial.print("\n 1B \n");
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
                Serial.print("\n mod1 \n");
                if (sirenDanceRedCounter <= 5 && sirenDanceRedFLAG == true)
                    {
                    if ((millis() - sirenDancePrevMillis) >= 50)
                        {
                        digitalWrite(blueSPin, LOW);//turn off the blues

                        sirenDancePrevMillis = millis();
                        sirenDanceREDState = !sirenDanceREDState;
                        digitalWrite(RedSPin, sirenDanceREDState);//toggle the 

                        if (sirenDanceREDState == HIGH)
                            {
                            Serial.print("\n 1A \n");
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
                        if (millis() - sirenDancePrevMillis >= 50)
                            {
                            digitalWrite(RedSPin, LOW);//turn off  red and blues
                            digitalWrite(blueSPin, LOW);//
                            sirenDancePrevMillis = millis();
                            //because this code is copy-Pasted from previous case, Below Lines is unnecessary.
                            //they should be Modified!
                            // we should write code for waiting for 1000millisecond 
                           // sirenDanceBLUEState = !sirenDanceBLUEState;
                            //if (sirenDanceBLUEState == HIGH)
                               // {
                            Serial.print("\n 1B \n");
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
    digitalWrite(RedSPin, LOW);
    digitalWrite(blueSPin, LOW);
    }
}


/// <summary>
/// toggles Audio Out Pin bitween Normal Speaker For Playing etc Music and Buzzer For Play Siren Sounds.
/// </summary>
/// <param name="ToBuzzer"></param>
/// <bool ToBuzzer>  determinates  wich one should be actived(buzz Or spk) 
void toggleSpeakerPin(bool ToBuzzer)
    {
    if (ToBuzzer)
        {
        digitalWrite(AudioSwitcherPin, LOW);
        } else
        {
        digitalWrite(AudioSwitcherPin, HIGH);
        }
    }

/// <summary>
/// uint16_t GetEngineTemp
/// Reads Data From Engine's Temperature Sensor and Convert it to celcius.
/// </summary>
/// <returns>Current Engine's Temperature</returns>
uint16_t GetEngineTemp()
{ 
    
    uint16_t adcval = 0;
    adcval = analogRead(TempSenseIn);
    return adcval / 100;
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
 

    if (millis() - idleSpeedPrevMillis >= 1000)
    {
        idleSpeedPrevMillis = millis();


        if (temperature < 5)
        {
            // Ice
            SetIdleRPM(28);
        } else if (temperature >= 5 && temperature < 10)
        {
            // very cold temperature near ice
            SetIdleRPM(25);
        } else if (temperature >= 10 && temperature < 15)
        {
            // very cold temperature
            SetIdleRPM(23);
        } else if (temperature >= 15 && temperature < 20)
        {
            // cold temperature
            SetIdleRPM(20);
        } else if (temperature >= 20 && temperature < 25)
        {
            // cool temperature
            SetIdleRPM(18);
        } else if (temperature >= 25 && temperature < 30)
        {
            //normal temperature
            SetIdleRPM(17);
        } else if (temperature >= 30 && temperature < 35)
        {
            //warm temperature
            SetIdleRPM(16);
        } else if (temperature > 35)
        {
            SetIdleRPM(15);
            RPMadjusted = true;
        }
    }

}

/// <summary>
/// void SetIdleRPM(byte _RPM)
/// Set Current Engine's RPM 
/// </summary>
/// <param name="_RPM"> Specified RPM to Be Set. RPM Will Moltiplied By 1000. Because Variable MEMORY Is too Small!</param>
void SetIdleRPM(byte _RPM)
{
    uint16_t SpecifiedAngle = map(_RPM, 10, 28, 0, 360);
    IdleServo.write(SpecifiedAngle);
}



