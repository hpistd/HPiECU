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
/       Second modification: fri. 1402/06/24 from 13:00 To 18:35 (blinkers)                 *
/                                                                                           *
/TODO: Complete Blinkers and test them for working.create horn, engine rpm meter....        *
/TODO: test mod1&2 & fix Swtich() in dance                                                  *
/                                                                                           *
/*******************************************************************************************/

const int headlightPin = 2;
const int TailLightPin = 3;
const int frontLblinkPin = 4;
const int backLblinkPin = 5;
const int frontRblinkPin = 6;
const int backRblinkPin = 7;
const int LhornPin = 8;
const int RhornPin = 9;
const int RedSPin = 10;
const int blueSPin = 11;
const int SirenPin = A0;
const int BrakePin = 12;
const int SmallLightPin = 13;

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
///</blinkers>

 //String input = "";

String SerialInputCommands[] ={"off","on","headlight:on","headlight:off","leftfrontblink","leftbackblink","rightfrontblink",

"rightbackblink","brake","lefthorn","righthorn","smalllight","multiblink:on:","multiblink:off",
"lturn:blink:","rturn:blink:","lturnblink:off","rturnblink:off","blinkdanceOn","blinkdanceOff"};


void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);
  pinMode(headlightPin,OUTPUT);
  pinMode(TailLightPin,OUTPUT);
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

  
//Serial.print("Hi! \n");
//digitalWrite(frontLblinkPin,HIGH);
}

void loop() 
{
     currentMillis = millis();
  // put your main code here, to run repeatedly:

  if (Serial.available())
    {
       input = Serial.readStringUntil('\n');

 ///// turn on main lights///////
    }
   // Serial.print(input);

 if (input == "headlight:on")
     {
      // Serial.print(input);
 digitalWrite(headlightPin,HIGH);
 digitalWrite(TailLightPin,HIGH);
 //input = "";
     }//////////

 //////turn off main lights///////////
  if (input == "headlight:off")
       {
  digitalWrite(headlightPin,LOW);
 digitalWrite(TailLightPin,LOW);


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
 //Serial.print(bInterval);

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


    ////////////

}//loop



void Blink(void)
{

  if (blinkerstate)

       {
    //       currentMillis = millis();

    //blink();
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
                    
                  // Serial.print("right");
                   digitalWrite(backRblinkPin,state);
                   digitalWrite(frontRblinkPin,state);
                  state =!state;
                  
             prevMillis = currentMillis;
           }


        }
         if (multiblink)
          {
            if (currentMillis - prevMillis >= (bInterval)){
                //  Serial.print("multi");
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
      //     Serial.print("\n in dance \n");
          /* if (dancerandom == 0)*/
             if (dancerandom >= 5 ) dancerandom = 1;
         // dancerandom = 1;
       //     Serial.print("\n random value: " + String(dancerandom) + " \n");

               
            switch (dancerandom)
                  {
                   //         Serial.print("\n switch (dancerandom)  \n");

             case 1:

                      firstMode:///****************************
                      // turn all blinkers one after one

                        /// Below Line was declared wrongly and only for test!.  Must Modified
                      if(dancestate !=F_L_ON && dancestate !=F_R_ON && dancestate !=B_L_ON && dancestate !=B_R_ON && dancestate !=F_L_OFF ) dancestate = F_L_ON;


                      if ((currentMillis - prevMillis) >= DANCE_DELAY)
                     {
                      Serial.print("\n first mode \n");

                      prevMillis = currentMillis;

                       switch (dancestate)
                       {
                       case F_L_ON:
                       // Serial.print("\n fl \n");
                                 digitalWrite(frontLblinkPin,HIGH);
                       dancestate = F_L_OFF;

                       break;

                       case F_L_OFF:
                        //Serial.print("\n fR \n");
                          digitalWrite(frontLblinkPin,LOW);
                       digitalWrite(frontRblinkPin,HIGH);
                       // prevMillis = currentMillis;
                       dancestate = F_R_ON;//front left bliker is off so turn on next

                        break;

                       case F_R_ON:
                        //Serial.print("\n bR \n");
                            digitalWrite(frontRblinkPin,LOW);
                       digitalWrite(backRblinkPin,HIGH);
                       //  prevMillis = currentMillis;
                       dancestate = B_R_ON;

                       break;
                       // case F_R_OFF:
                       //  break;
                       case B_R_ON:
                        //Serial.print("\n bL \n");
                       digitalWrite(backRblinkPin,LOW);
                       digitalWrite(backLblinkPin,HIGH);
                        // prevMillis = currentMillis;
                       dancestate = B_L_ON;
                       break;
                        // case B_R_OFF:
                       //  break;
                       case B_L_ON:
                        //Serial.print("\n BLoff \n");
                       digitalWrite(backLblinkPin,LOW);
                      // digitalWrite(frontLblinkPin,HIGH);
                        // prevMillis = currentMillis;
                       dancestate = F_L_ON;

                       break;

                       

                        // case B_L_OFF:

                       //  break;
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

                        

                        ////////////////////case1


             case 2: /// turn on Front Blinkers Rapidly etc 3 times,then turn on back blinkers rapidly too.
                      // this task wil repeated for etc 3 times
                       

                        if (dancestate != B_LR_ON  && dancestate != F_LR_ON )
                       {
                         dancestate = F_LR_ON;
                         }
                     
                       secondMode:
                       

                         // switch (dancestate)
                        //   {
   
                         //  case F_LR_ON:
                      //   Serial.print("\n A \n");
                         if ( (currentMillis - prevMillis) >= FRONT_LR_INTERVAL )
                       {
                          prevMillis = currentMillis;
                             Serial.print("\n second mode \n");

                            for (i=0; i<10; i++) 
                            {
                           //    Serial.print("\n AA \n");

                                  delay(MOD2_DELAY);
                             digitalWrite(frontLblinkPin,state);
                             digitalWrite(frontRblinkPin,state);
                             digitalWrite(backLblinkPin,LOW);
                             digitalWrite(backRblinkPin,LOW);
                             state =!state; 
                             delay(MOD2_DELAY);
                         //    Serial.print("\n AAA \n");
                            }
                            

                            for (i=0; i<10; i++) 
                            {

                          //     Serial.print("\n BB \n");

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
                             //  Serial.print("\n gs \n");
                             }

                           
                             // Serial.print("\n br \n");
                            
                              mod2counter = 0;
                              dancerandom++;
                              break;
                            

                          
                          

                    //  }
                      // break;/////////////////case2

             case 3:
                     ////blinks front and back Left blinkers etc 3 times then turns right blinkers...

                     
                    if ((dancestate != FB_L_ON) && (dancestate != FB_R_ON))
                    { 
                     dancestate = FB_L_ON;//initial value
                    }
                   //  mod3counter=0;
                     mod3top:
                    
                     
                     
                     if ((currentMillis - prevMillis) >= BACK_LR_INTERVAL )
                      {
                          prevMillis = currentMillis;
                         Serial.print("\n third mode \n");
                         switch (dancestate)
                        {

                         case FB_L_ON:
                           // mod3counter++;
                            state = true;

                            for (i=0; i<10; i++) 
                            {
                            
                            delay(MOD3_DELAY);
                         // Serial.print("\n third AAAA \n");

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
                       //   }
                      

                           // break;

                         case FB_R_ON:
                         
                       //   mod3counter++;
                        state = true;

                         for (i=0; i<10; i++) 
                          {
                            delay(MOD3_DELAY);
                             //

                             digitalWrite(frontLblinkPin,LOW);
                              digitalWrite(backLblinkPin,LOW);
                             digitalWrite(frontRblinkPin,state);
                             digitalWrite(backRblinkPin,state);
                             state =!state;
                             delay(MOD3_DELAY);
                          }
                          

                         dancestate = FB_L_ON;

                         break;
                         // }

                        }//switch

                        dancecounter++;
                     }//if
                     
                        if (dancecounter < MOD2_REPEATE_TIMES)
                        {
                          
                        //  Serial.print("\n go top \n");
                          goto end;
                        }

                        else
                        {
                          dancecounter = 0;
                            dancerandom++;
                          break;
                        }
                    
                         
                       //////////////////////MODE 3
                      
             case 4:////////Blink All 4 Blinkers Rapidly
                  state = true;
                    mod4top:

                       

                    if (currentMillis - prevMillis >= MOD4_INTERVAL)
                    {
                      prevMillis = currentMillis;

                     Serial.print("\n fourth mode \n");
                    
                       for (i=0; i<10; i++) {
                     
                    
                       delay(MOD4_DELAY);
                     //    Serial.print("\n fourth run \n");
                      digitalWrite(frontLblinkPin,state);
                      digitalWrite(frontRblinkPin,state);
                      digitalWrite(backLblinkPin,state);
                      digitalWrite(backRblinkPin,state);
                      state =!state;
                      delay(MOD4_DELAY);
                      // i++;
                       }
                       delay(MOD4_INTERVAL);
                       i=0;
                    //  mod4counter++;
                     // if(mod4counter < MOD4_REPEAT_TIMES) goto mod4top;     
                     // mod4counter = 0;
                      dancecounter++;
                    } 
                     if (dancecounter < MOD4_REPEAT_TIMES)
                        {
                          goto end;
                            //   Serial.print("\n dancecount" + String(dancecounter)  +" \n");

                        }

                        else
                        {
                          dancecounter = 0;
                            dancerandom++;
                          break;
                        }


                    

                     //       break;
                        //  case 5:
                         //  break;


 
                  }  

                 end:              
                dummy = false;
             }




//currentMillis = millis();
//Serial.print("\n prev :  " + String(prevMillis) + " \n");
//Serial.print("\n current: " + String(currentMillis) + " \n");

      
//Serial.print("\n end \n");
       }




}//blink
