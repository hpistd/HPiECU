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
/       Last modification: sat. 1402/08/19 from 15:05 To 16:55(HORN+blinkers...)            *
/                                                                                           *
/TODO: Complete Siren + LED FLASHERs                                                        *
/TODO: Add Mode For HeadLight (blinker etc)                                                 *
/TODO: # Buzzer,Emergency PWR,,SIREN LEDS!,Serial Communic.                                 *
/*******************************************************************************************/


#include <EEPROM.h>





#define headlightPin  2
#define frontLblinkPin  4
#define backLblinkPin  5
#define frontRblinkPin  6
#define backRblinkPin  7
#define LhornPin  8
#define RhornPin  9
#define RedSPin  10
#define blueSPin  11
#define BrakePin  12
#define SirenPin  A0 /// Used For Swith Between Normal HIFI Speaker And Piezo Buzzer For Siren. 
#define EMERGENCYShutDownPin A4 /// For Remote ShutDown

// Input Keys
#define LturnINpin A2
#define RturnINpin A3
#define HEADLightINpin A5
#define BrakeINpin 13
#define HornINpin 3
#define SignalInputPin A1//for RPM Meter

///////////RPM Meter
int X;
int Y;
float Time;
float frequency;
int RPM = 0;
unsigned long freqPrvmillis = 0;
///////////
///HORN
unsigned short debounceDelay = 200;
unsigned long delayMillis = 0,hornPrevMillis = 0, buttonPrevMillis = 0, lastDebounceTime = 0;
short hornclicks = 0;
bool buttunstate= false;
//HORN modes
short modeCcounter = 0;
short hornCountA=0,hornCountB=0;
bool horn1Aflag = true, horn1Bflag = false;
bool hornStateA = false,hornStateB = false;
bool _hornmodetwostate = true;
//short modThreedelays = 50; //delay between each cycle of horning in wedding mode
bool modThreestate = false; // current state of patern. below line is our patern 
short modeCstage  =1;
//->A{on(50)->off(30)} 
//-> B{on(50)->off(30)} 
//-> C{on(80)->off(100)} 
//-> D{on(100)->off(80)} 
//-> E{on(100)->off(100)} 
//bebebbbbbeebbbbbeeebbbbbeeeeeb...


///////////
/////<blinkers>
unsigned int bInterval = 250; // normal blinkers on/of delay in ms.
unsigned long  prevMillis = 1000,currentMillis = 0; //for millis();. it used instead of old depricated delay() .
///MODE2
bool danceTwoFrontFlag = true,danceTwoBackFlag = false,danceTwoFrontState = false,danceTwoBackState = false; // states
short danceTwoFrontCounter = 0,danceTwoBackCounter = 0; // blink counters
///
 int dancerandom =1;// random mode changing NOTE : it should to be Modified To run one After One!.
short dancecounter =0;// how many times current mode repeated?.
short stagecounter = 0;// current stage play counter.

String   input = "";
bool state = false;
bool blinkerstate = false;
bool multiblink = false;
bool lfrontblinkerstate = false;
bool rfrontblinkerstate = false;
bool lbackblinkerstate = false;
bool rbackblinkerstate = false;
bool blinkdance = false;
//flags
#define ENGINE_IS_OFF false //
#define ENGINE_IS_ON true//
/// input keys Flag
bool headlightFlag = false, lturnflag = false, rturnflag = false, brakeflag = false, engPowerFlag = ENGINE_IS_OFF; //parsing keys
//EEPROM DATA
int eep_blinkinterval = 300; // default Delay For Nomal Blinking
int eep_blinkintervalAddress = 1; // Address Off Interval Holder
///</blinkers>
/*
String SerialInputCommands[] ={"off","on","headlight:on","headlight:off","leftfrontblink","leftbackblink","rightfrontblink",
"rightbackblink","brake","lefthorn","righthorn","smalllight","multiblink:on:","multiblink:off",
"lturn:blink:","rturn:blink:","lturnblink:off","rturnblink:off","blinkdanceOn","blinkdanceOff","SetBlinkInterVal:300","engineState"};

String SerialOUTPUTCommands[] ={"off","on","Engine Is OFF","Engine Is ON","RPM:000"};
*/
void setup() 
{
  analogReference(DEFAULT);
  Serial.begin(115200);
  //Outputs
  pinMode(headlightPin,OUTPUT);
  pinMode(backLblinkPin,OUTPUT);
  pinMode(frontLblinkPin,OUTPUT);
  pinMode(backRblinkPin,OUTPUT);
  pinMode(frontRblinkPin,OUTPUT);
  pinMode(BrakePin,OUTPUT);
  pinMode(LhornPin,OUTPUT);
  pinMode(RhornPin,OUTPUT);
  pinMode(blueSPin,OUTPUT);
  pinMode(RedSPin,OUTPUT);
  pinMode(SirenPin,OUTPUT);
  pinMode(EMERGENCYShutDownPin, OUTPUT);
  ///Inputs
  pinMode(LturnINpin, INPUT);
  pinMode(RturnINpin, INPUT);
  pinMode(HEADLightINpin, INPUT);
  pinMode(BrakeINpin, INPUT);
  pinMode(HornINpin, INPUT);
  pinMode(SignalInputPin, INPUT);
  ///Horn Button Listener
  attachInterrupt(digitalPinToInterrupt(HornINpin), checkHornKey, CHANGE);
}

void loop() 
{
  currentMillis = millis();
  byte firstcheck = 0;
  int add = 0;
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
    
   }

  if (digitalRead(HEADLightINpin) == LOW ) 
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

    EEPROM.get(eep_blinkintervalAddress,bInterval);
  

       blinkerstate = true;
    lfrontblinkerstate = true;
    lbackblinkerstate = true;
     lturnflag = true;

  }
  if (digitalRead(LturnINpin) == LOW)
  {
    if (lturnflag == true)
    {
     
        digitalWrite(backLblinkPin,LOW);
      digitalWrite(frontLblinkPin,LOW);
         blinkerstate = false;
      lfrontblinkerstate = false;
      lbackblinkerstate = false;
      lturnflag = false;
    }
 
 }
////right turn
  if (digitalRead(RturnINpin) == HIGH)
   {
    EEPROM.get(eep_blinkintervalAddress,bInterval);
  
           blinkerstate = true;
       rbackblinkerstate = true;
       rfrontblinkerstate = true;
       rturnflag = true;
  
   }
   if (digitalRead(RturnINpin) == LOW ) 
   {
    if (rturnflag == true)
    {
      digitalWrite(backRblinkPin,LOW);
      digitalWrite(frontRblinkPin,LOW);
       blinkerstate = false;
       rbackblinkerstate = false;
       rfrontblinkerstate = false;
       rturnflag = false;
    }
   }
/////////RPM METER   
  if ((currentMillis - freqPrvmillis) >= 1000)
           {  
          

            freqPrvmillis =   currentMillis;             
            // X=pulseIn(SignalInputPin,HIGH);
            // Y=pulseIn(SignalInputPin,LOW);
             Time = X+Y;
             frequency=1000000/Time;
              if(Time <=0)
              {                
                 if (engPowerFlag == ENGINE_IS_ON) engPowerFlag = ENGINE_IS_OFF;   
              }
              else 
               {
                  RPM = frequency * 60 ;
                Serial.print("\n freq:");
                Serial.print(frequency);
                Serial.print("\n");
                Serial.print("\nRPM:");
                Serial.print(RPM);
                Serial.print("\n");
                if (engPowerFlag == ENGINE_IS_OFF) engPowerFlag = ENGINE_IS_ON;       
              }           
          }
  if (Serial.available())
    {
       input = Serial.readStringUntil('\n');
     /// turn on main lights///////
    }
  if (input == "headlight:on")
     {
      digitalWrite(headlightPin,HIGH);
      input ="";

     }//////////
   //////turn off main lights///////////
   if ((input == "headlight:off"))
       {
        if (headlightFlag ==false)
        {
          digitalWrite(headlightPin,LOW);
          input ="";
        }

       }////////
   if ((input == "engineState"))
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
    //////turn off All Blinkers ///////////
  if ((input == "lturnblink:off") || (input == "rturnblink:off")  || (input == "multiblink:off")  || (input == "blinkdanceOff") )
  {
    Serial.print("off All LEDs");
  lfrontblinkerstate = false;
  rfrontblinkerstate = false;
  rbackblinkerstate = false;
  rfrontblinkerstate = false;
  blinkerstate = false;
  multiblink = false;
  digitalWrite(backLblinkPin,LOW);
  digitalWrite(frontLblinkPin,LOW);
  digitalWrite(backRblinkPin,LOW);
  digitalWrite(frontRblinkPin,LOW);
  input =""; //clear the buffer
  }////////

 //////////checks for Left blinker///////////////
  if (input.startsWith("lturn:blink:"))
  {
   bInterval = input.substring(12).toInt();
   Serial.print("lturn:blink:");
  

   if (bInterval > 0)
     {
      Serial.print(bInterval);

     blinkerstate = true;
    lfrontblinkerstate = true;
    lbackblinkerstate = true;
    //Serial.print("true");
    input ="";
    }
  }
 ////////////////////////
if(input.startsWith("SetBlinkInterVal:"))
{

  eep_blinkinterval = input.substring(17).toInt();
if (eep_blinkinterval >0) {
 EEPROM.update(eep_blinkintervalAddress, eep_blinkinterval);

}


}
 ///////Checks For Right Blinkers//////////////////
  if (input.startsWith("rturn:blink:"))
   {
     //Serial.print("rturn:blink:");
      bInterval = input.substring(12).toInt();
     if (bInterval > 0)
     {
       // Serial.print(bInterval);
       blinkerstate = true;
       rbackblinkerstate = true;
       rfrontblinkerstate = true;
       input ="";
     }
   }  

  if (input.startsWith("multiblink:on:"))
  {
      bInterval = input.substring(14).toInt();
      if (bInterval > 0)
    {
     blinkerstate = true;
     multiblink = true;
     input ="";
    }
  }

  if (input == "blinkdanceOn")
  {
   //Serial.print("dance");
   blinkerstate = true;
   blinkdance = true;
  }

 ///toggle the blinkers  
     Horn(); 
 Blink();
}//loop

void Blink(void)
{
  if (blinkerstate)
      {
        if (lfrontblinkerstate && lbackblinkerstate)
        {        
          if ((currentMillis - prevMillis) >= bInterval)
             {                          
              digitalWrite(frontLblinkPin,state);
               digitalWrite(backLblinkPin,state);       
               state =!state;
              // Serial.print("left");
              prevMillis = currentMillis;
             }
         }
         if (rbackblinkerstate && rfrontblinkerstate)
         {
                  if (currentMillis - prevMillis >= bInterval)
           {
                   digitalWrite(backRblinkPin,state);
                   digitalWrite(frontRblinkPin,state);
                  state =!state;
                  
             prevMillis = currentMillis;
           }
        }
         if (multiblink)
          {
            if (currentMillis - prevMillis >= (bInterval)){
                              digitalWrite(backRblinkPin,!state);
                   digitalWrite(frontRblinkPin,!state);
                   digitalWrite(frontLblinkPin,!state);
                   digitalWrite(backLblinkPin,!state);
                   digitalWrite(backRblinkPin, !state);
             state =!state; 
              prevMillis = currentMillis;
            }
          }
        if (blinkdance)
        {
            if (dancerandom >= 5 ) dancerandom = 1;
          switch (dancerandom)
          {
                   case 1:
                   if ((currentMillis - delayMillis) >= 300)
                   {
                    dancecounter++;
                    switch (dancecounter) 
                    {
                     case 1:
                      digitalWrite(frontLblinkPin,HIGH);
                      digitalWrite(backLblinkPin,LOW);
                     break;
                     case 2:
                     digitalWrite(frontLblinkPin,LOW);
                     digitalWrite(frontRblinkPin,HIGH);
                     break;
                     case 3:
                     digitalWrite(frontRblinkPin,LOW);
                     digitalWrite(backRblinkPin,HIGH);
                     break;
                     case 4:
                     digitalWrite(backRblinkPin,LOW);
                     digitalWrite(backLblinkPin,HIGH);
                     dancecounter = 0;
                     stagecounter++;
                     if (stagecounter >=5)
                     {
                       dancerandom++;
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
              if ((currentMillis - delayMillis) >= 50)
              {
                digitalWrite(backLblinkPin,LOW);//turn off the 
                digitalWrite(backRblinkPin,LOW);//back blinkers
                delayMillis = currentMillis;
                danceTwoFrontState = !danceTwoFrontState;
                digitalWrite(frontLblinkPin,danceTwoFrontState);//toggle the 
                digitalWrite(frontRblinkPin,danceTwoFrontState);// front blinkers.
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
               if (currentMillis - delayMillis >=50)
               {
                digitalWrite(frontLblinkPin,LOW);//turn off the 
                digitalWrite(frontRblinkPin,LOW);//front blinkers
                delayMillis = currentMillis;
                danceTwoBackState =!danceTwoBackState;
                digitalWrite(backLblinkPin,danceTwoBackState);
                digitalWrite(backRblinkPin,danceTwoBackState);
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
                     dancerandom++;
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
              if ((currentMillis - delayMillis) >= 50)
              {
                digitalWrite(frontRblinkPin,LOW);//turn off the 
                digitalWrite(backRblinkPin,LOW);//Right blinkers
                delayMillis = currentMillis;
                danceTwoFrontState = !danceTwoFrontState;
                digitalWrite(frontLblinkPin,danceTwoFrontState);//toggle the 
                digitalWrite(backLblinkPin,danceTwoFrontState);// Left blinkers.
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
              if (currentMillis - delayMillis >=50)
              {
                digitalWrite(frontLblinkPin,LOW);//turn off the 
                digitalWrite(backLblinkPin,LOW);//Left blinkers
                delayMillis = currentMillis;
                danceTwoBackState =!danceTwoBackState;
                digitalWrite(frontRblinkPin,danceTwoBackState);
                digitalWrite(backRblinkPin,danceTwoBackState);
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
                     dancerandom++;
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
              if ((currentMillis - delayMillis) >= 50)
              {
                delayMillis = currentMillis;
                danceTwoFrontState = !danceTwoFrontState;
                digitalWrite(frontLblinkPin,danceTwoFrontState);///Toggle
                digitalWrite(frontRblinkPin,danceTwoFrontState);///the
                digitalWrite(backLblinkPin,danceTwoFrontState);///All
                digitalWrite(backRblinkPin,danceTwoFrontState);//Blinkers
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
              if (currentMillis - delayMillis >=50)
              {
                digitalWrite(frontLblinkPin,LOW);//turn off the 
                digitalWrite(backLblinkPin,LOW);//Left blinkers
                digitalWrite(frontRblinkPin,LOW);
                digitalWrite(backRblinkPin,LOW);
                delayMillis = currentMillis;
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
                     dancerandom = 1;
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
// Horn function
//     it Has Several Modes 
// Changes Per click Under 2seconds
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
        if ((hornclicks == 1) && (currentMillis - buttonPrevMillis) > 300)
        {  
          digitalWrite(RhornPin,HIGH);
          digitalWrite(LhornPin,HIGH);
        }else if ((hornclicks == 2) && (currentMillis - buttonPrevMillis) > 300)
        {
          
          Serial.print("\n case1 \n");
          if (hornCountA <=5 && horn1Aflag == true)
          {            
              if ((currentMillis - delayMillis) >= 50)
              {
                digitalWrite(RhornPin,LOW);
                delayMillis = currentMillis;
                hornStateA = !hornStateA;
                digitalWrite(LhornPin,hornStateA);
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
              if (currentMillis - delayMillis >=50)
              {
                digitalWrite(LhornPin,LOW);
                delayMillis = currentMillis;
                hornStateB =!hornStateB;
                digitalWrite(RhornPin,hornStateB);
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
        }else if ((hornclicks == 3) && (currentMillis - buttonPrevMillis) > 300)
        {
        
           Serial.print("\n case2 \n");
          ////mod2
            if (currentMillis - delayMillis > 100)
             {
              delayMillis = currentMillis;
              if (_hornmodetwostate == HIGH)
              {
              digitalWrite(LhornPin,LOW);
              digitalWrite(RhornPin,HIGH);
              _hornmodetwostate = false;
              }else
               {
                digitalWrite(RhornPin,LOW);
                digitalWrite(LhornPin, HIGH);
                _hornmodetwostate = true;
               }
             }
         //hornclicks = 0;
        }else if ((hornclicks == 4) && (currentMillis - buttonPrevMillis) > 300)
        {       
           //Serial.print("\n case3 \n");
          //Mode 3 Wedding Mode ^_^
          if (currentMillis - delayMillis >= 100 && modThreestate == false && modeCstage == 1)
          {
            Serial.println("stage 1");
            modeCstage = 2;
           modThreestate = true;
           delayMillis = currentMillis;
           digitalWrite(LhornPin ,modThreestate);
           digitalWrite(RhornPin, modThreestate);
          } else if (currentMillis - delayMillis >= 50 && modThreestate == true && modeCstage == 2)
            {
              Serial.println("stage 2");
              modeCstage = 3;
              modThreestate = false;
              delayMillis = currentMillis;
              digitalWrite(LhornPin,modThreestate);
              digitalWrite(RhornPin,modThreestate);
            }else if (currentMillis - delayMillis >=100 && modThreestate == false && modeCstage == 3)
             {
              Serial.println("stage 3");
              modeCstage = 4;
              modThreestate = true;
              delayMillis = currentMillis;
              digitalWrite(LhornPin,modThreestate);
              digitalWrite(RhornPin,modThreestate);              
             } else if (currentMillis - delayMillis >=50 && modThreestate == true && modeCstage == 4)
               {
                Serial.println("stage 4");
                modeCstage = 5;
                modThreestate = false;
                delayMillis = currentMillis;
              digitalWrite(LhornPin,modThreestate);
              digitalWrite(RhornPin,modThreestate);                
               } else if (currentMillis - delayMillis >= 200 && modThreestate == false && modeCstage == 5)
                 {
                  Serial.println("stage 5");
                  modeCstage = 6;
                  modThreestate = true;
                  delayMillis = currentMillis;
                  digitalWrite(LhornPin,modThreestate);
                  digitalWrite(RhornPin,modThreestate);
                 } else if (currentMillis - delayMillis >=200 && modThreestate ==true && modeCstage == 6)
                   {
                    Serial.println("stage 6");
                    modeCstage = 7;
                    modThreestate = false;
                    delayMillis = currentMillis;
                     digitalWrite(LhornPin,modThreestate);
                     digitalWrite(RhornPin,modThreestate);
                   } else if (currentMillis - delayMillis >=250 && modThreestate == false && modeCstage == 7)
                     {
                      Serial.println("stage 7");
                      modeCstage = 8;
                      modThreestate = true;
                      delayMillis = currentMillis;
                      digitalWrite(LhornPin,modThreestate);
                      digitalWrite(RhornPin,modThreestate);
                     } else if (currentMillis - delayMillis >= 250 && modThreestate == true && modeCstage == 8)
                       {
                        Serial.println("stage 8");
                        modeCstage = 9;
                        modThreestate = false;
                        delayMillis = currentMillis;
                        digitalWrite(LhornPin,modThreestate);
                        digitalWrite(RhornPin,modThreestate);
                       } else if (currentMillis - delayMillis >=250 && modThreestate ==false && modeCstage == 9)
                         {
                          Serial.println("stage 9");
                          modeCstage = 10;
                          modThreestate = true;
                          delayMillis = currentMillis;
                          digitalWrite(LhornPin,modThreestate);
                          digitalWrite(RhornPin,modThreestate);
                         }else if (currentMillis - delayMillis >=250 && modThreestate ==true && modeCstage == 10)
                         {
                          Serial.println("stage 10");
                          modeCstage = 11;
                          modThreestate = false;
                          delayMillis = currentMillis;
                          digitalWrite(LhornPin,modThreestate);
                          digitalWrite(RhornPin,modThreestate);
                         }else if (currentMillis - delayMillis >=250 && modThreestate ==false && modeCstage == 11)
                         {
                          Serial.println("stage 11");
                          modeCstage = 1;
                         }
         //hornclicks = 0;
        }
  }else
  { 
 //   Serial.println("released");
   digitalWrite(LhornPin,LOW);
   digitalWrite(RhornPin,LOW);
  }
} 
///Horn Button Parser(called By Interrupt)
void checkHornKey()
{
  currentMillis = millis();
  if((digitalRead(3)) == HIGH)
  {
    //Serial.println("pressing");
   // Serial.print(hornclicks);
      if ((currentMillis - buttonPrevMillis) > 500)hornclicks = 0; //if no cicks comes after timeout so restart counter .
   if (currentMillis - lastDebounceTime > debounceDelay)
    {
      lastDebounceTime = currentMillis;
     // Serial.println("intermillis");
      buttunstate = true;
      //cli();
      hornclicks++;
      if (hornclicks > 4 ) hornclicks = 1; // if we was  pressed the button more than 4 times so reset the click counter.
      buttonPrevMillis = currentMillis;
    }
    
  }else buttunstate = false;//this code is Unnecessary. will removed.


 
  //Serial.println("interout");
//Serial.print(hornclicks);
}







