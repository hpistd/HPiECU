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
/       Last modification: fri. 1402/07/14 from 14:00 To 16:35 (RPM...)                     *
/                                                                                           *
/TODO: Complete Horn Func                                                                   *
/TODO: Add Mode For HeadLight (blinker etc)                                                 *
/TODO: # Buzzer,Emergency PWR,HORN,Serial Communic.                                         *
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

/////<blinkers>
int bInterval = 250; // normal blinkers on/of delay in ms.
unsigned long  prevMillis = 1000,currentMillis = 0; //for millis();. it used instead of old depricated delay() .
#define DANCE_DELAY 100 // delay between on/off blinkers in ms.
#define MOD1_REPEATE_TIMES 18 ////how many times current mode should be repeated.
#define MOD2_REPEATE_TIMES 3///// how many times current mode should be repeated.
#define MOD3_REPEAT_TIMES 3 //how many times current mode should be repeated.
#define BACK_LR_INTERVAL 300  // delay between off/on blinkers in ms.
#define FRONT_LR_INTERVAL 300 // delay between off/on blinkers in ms.
#define FB_L_INTERVAL 300 // delay between off/on blinkers (in mode 3) in ms.
#define FB_R_INTERVAL 300 // delay between off/on blinkers (in mode 3) in ms.
#define MOD4_INTERVAL 300 //delay between off/on blinkers (in mode 3) in ms.
#define MOD4_REPEAT_TIMES 3 //how many times current mode should be repeated.
#define MOD4_DELAY 30 // delay between of/on on every cycle
#define MOD3_INTERVAL 300 // delay between move to (etc from left blinkers to right and reverse)
#define MOD3_DELAY 30 // delay between of/on on every cycle
#define MOD2_DELAY 30 // delay between of/on on every cycle
#define MOD1_DELAY 30 // delay between of/on on every cycle


 int dancerandom =1;// random mode changing NOTE : it should to be Modified To run one After One!.
short dancecounter =0;// how many times current mode repeated?.
short mod3counter = 0;
short mod4counter = 0;
short mod2counter = 0;
short i = 0;//For loop in MOD4
bool dummy = false; // for end

enum dancenum
{
  NONE,
  F_L_ON,//Front Left Blinker
  F_L_OFF,//Front Left Blinker is off
  FB_L_ON,//Front And Back Left Blinkers
  F_R_ON,//Front Right Blinker
  FB_R_ON,//Front And Back right Blinkers
  B_L_ON,// Back Left Blinker
  F_LR_ON,//Front Left And Right Blinkers
  B_R_ON,// Back Right Blinker
  B_LR_ON //Back Left And Right Blinkers

};

dancenum dancestate = F_L_ON;

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
void setup() {
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
  /////
  //clear EEPROM data
  /*
 for (int i=0; i > EEPROM.length() - 1; i++) {

 EEPROM.write(i, 0);
 delay(5);
 }
 Serial.print("deleted");
 */
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

  }
  else 
  {
  eep_blinkinterval = EEPROM.get(eep_blinkintervalAddress,eep_blinkinterval);

  }

//tone(9, 166);
///
  if (digitalRead(HornINpin) == HIGH ) 
    {    
     digitalWrite(LhornPin, HIGH);
     digitalWrite(RhornPin,HIGH);
    }
    else {
    digitalWrite(LhornPin, LOW);
    digitalWrite(RhornPin,LOW);   
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
             X=pulseIn(SignalInputPin,HIGH);
             Y=pulseIn(SignalInputPin,LOW);
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

                      firstMode:///****************************
                       /// Below Line was declared wrongly and only for test!.  Must Modified
                      if(dancestate !=F_L_ON && dancestate !=F_R_ON && dancestate !=B_L_ON && dancestate !=B_R_ON && dancestate !=F_L_OFF ) dancestate = F_L_ON;
                      if ((currentMillis - prevMillis) >= DANCE_DELAY)
                     {
                       Serial.print("\n first mode \n");
                       prevMillis = currentMillis;

                       switch (dancestate)
                       {
                       case F_L_ON:
                                 digitalWrite(frontLblinkPin,HIGH);
                       dancestate = F_L_OFF;
                       break;

                       case F_L_OFF:
                          digitalWrite(frontLblinkPin,LOW);
                       digitalWrite(frontRblinkPin,HIGH);
                       dancestate = F_R_ON;//front left bliker is off so turn on next
                        break;

                       case F_R_ON:
                            digitalWrite(frontRblinkPin,LOW);
                       digitalWrite(backRblinkPin,HIGH);
                       dancestate = B_R_ON;
                       break;

                       case B_R_ON:
                       digitalWrite(backRblinkPin,LOW);
                       digitalWrite(backLblinkPin,HIGH);
                       dancestate = B_L_ON;
                       break;

                       case B_L_ON:
                       digitalWrite(backLblinkPin,LOW);
                       dancestate = F_L_ON;
                       break;
                       }              
                     
                       dancecounter++;
                      }
                     
                      if (dancecounter < MOD1_REPEATE_TIMES)
                        {
                          goto end;
                        }
                        else
                        {
                          dancecounter = 0;
                          dancerandom++;
                         break;
                        }

             case 2: /// turn on Front Blinkers Rapidly etc 3 times,then turn on back blinkers rapidly too.
                      // this task wil repeated for etc 3 times
                        if (dancestate != B_LR_ON  && dancestate != F_LR_ON )
                       {
                         dancestate = F_LR_ON;
                         }

                       secondMode:
                         if ( (currentMillis - prevMillis) >= FRONT_LR_INTERVAL )
                       {
                          prevMillis = currentMillis;
                             Serial.print("\n second mode \n");
                            for (i=0; i<10; i++) 
                            {
                                  delay(MOD2_DELAY);
                             digitalWrite(frontLblinkPin,state);
                             digitalWrite(frontRblinkPin,state);
                             digitalWrite(backLblinkPin,LOW);
                             digitalWrite(backRblinkPin,LOW);
                             state =!state; 
                             delay(MOD2_DELAY);
                            }

                            for (i=0; i<10; i++) 
                            {
                                  delay(MOD2_DELAY);
                            digitalWrite(frontLblinkPin,LOW);
                            digitalWrite(frontRblinkPin,LOW);
                            digitalWrite(backLblinkPin,state);
                            digitalWrite(backRblinkPin,state);
                            state =!state;
                            delay(MOD2_DELAY);
                            }
                            mod2counter++;  
                           //}
                       }
                            if (mod2counter < MOD2_REPEATE_TIMES)
                             {
                               goto end;
                             }
                              mod2counter = 0;
                              dancerandom++;
                              break;
             case 3:
                     ////blinks front and back Left blinkers etc 3 times then turns right blinkers...
                    if ((dancestate != FB_L_ON) && (dancestate != FB_R_ON))
                    { 
                     dancestate = FB_L_ON;//initial value
                    }
                     mod3top:

                     if ((currentMillis - prevMillis) >= BACK_LR_INTERVAL )
                      {
                          prevMillis = currentMillis;

                         switch (dancestate)
                        {
                         case FB_L_ON:            
                            state = true;
                            for (i=0; i<10; i++) 
                            {
                            delay(MOD3_DELAY);
                          digitalWrite(frontRblinkPin,LOW);
                         digitalWrite(backRblinkPin,LOW);
                         digitalWrite(frontLblinkPin,state);
                         digitalWrite(backLblinkPin,state);
                         state =!state;
                         delay(MOD3_DELAY);
                        }
                         mod3counter = 0;
                          dancestate = FB_R_ON;
                          break;

                         case FB_R_ON:
                        state = true;
                         for (i=0; i<10; i++) 
                          {
                            delay(MOD3_DELAY);
                             digitalWrite(frontLblinkPin,LOW);
                              digitalWrite(backLblinkPin,LOW);
                             digitalWrite(frontRblinkPin,state);
                             digitalWrite(backRblinkPin,state);
                             state =!state;
                             delay(MOD3_DELAY);
                          }
                         dancestate = FB_L_ON;
                         break;
                        }//switch
                        dancecounter++;
                     }//if
              
                        if (dancecounter < MOD2_REPEATE_TIMES)
                        {
                          goto end;
                        }
                        else
                        {
                          dancecounter = 0;
                            dancerandom++;
                          break;
                        }  

             case 4:////////Blink All 4 Blinkers Rapidly
                  state = true;
                    mod4top:                       
                    if (currentMillis - prevMillis >= MOD4_INTERVAL)
                    {
                      prevMillis = currentMillis;
                     Serial.print("\n fourth mode \n");
                    
                       for (i=0; i<10; i++) {                
                       delay(MOD4_DELAY);
                      digitalWrite(frontLblinkPin,state);
                      digitalWrite(frontRblinkPin,state);
                      digitalWrite(backLblinkPin,state);
                      digitalWrite(backRblinkPin,state);
                      state =!state;
                      delay(MOD4_DELAY);
                       }
                       delay(MOD4_INTERVAL);
                       i=0;
                      dancecounter++;
                    } 
                     if (dancecounter < MOD4_REPEAT_TIMES)
                        {
                          goto end;
                        }
                        else
                        {
                          dancecounter = 0;
                            dancerandom++;
                          break;
                        }
                  }  

                 end:              
                dummy = false;
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




}









