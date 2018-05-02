#include <FuzzyRule.h>
#include <FuzzyComposition.h>
#include <Fuzzy.h>
#include <FuzzyRuleConsequent.h>
#include <FuzzyOutput.h>
#include <FuzzyInput.h>
#include <FuzzyIO.h>
#include <FuzzySet.h>
#include <FuzzyRuleAntecedent.h>

double I_ref = 0;                               //reference current [A] initially 0
double I_meas;                                 //measured current [A]
double eI[2] = {0,0};                          //error vector [A]
double deI;                                    //error change eI2-eI1
double D,ds;                                   //duty cycle
double dd;                                     //duty cycle increment
int FB = 30;                                   //Fuzzy Logic Error Band (after which hyst. control takes over)[A]
int Ib = 1;                                    //Hysterisis Current band I_ref +- Ib [A]
bool S,R;                                      //Flip-flop parameters
double V_DG, V_out;                            //Measured input voltage and measured output voltage [V]
double K = 0.01;                               //droop coeficient initially set to 0.01
double dK = 2e-3;                              //droop coeficient incriment
double Kp = 0.001;                             //Proportional coeff for the calculation of dK
double P;                                      //Power [W]
double V_ref = 400;                            //No load voltage [V]
double I_PV,V_PV;                              //I_PV V_PV
double d_I_PV, d_P_PV;
double ipv[2] = {0,0};                         //PV current vector
double ppv[2] = {0,0};                         //PV power vector
bool A = true;                                //Startup Mode
bool EN = false;                             //Enable MPPT
Fuzzy* CC = new Fuzzy();                      //Create new instance of the fuzzy controller called CC (current controller)

  //Deffine membership functions for IE
  FuzzySet* NE = new FuzzySet(-100,-100,-30,-10); //Negative Current Error
  FuzzySet* ZE = new FuzzySet(-20,0,0,20);        //Zero Current Error
  FuzzySet* PE = new FuzzySet(10,30,100,100);     //Positive Current Error

  //Define membership functions for dIE
  FuzzySet* NDE = new FuzzySet(-10,-10,-3,-1);    //Negative Current Derivative Error
  FuzzySet* ZDE = new FuzzySet(-2,0,0,2);         //Zero Current Derivative Error
  FuzzySet* PDE = new FuzzySet(1,3,10,10);        //Positive Current Derivative Error

  //Define membership functions for output d
  FuzzySet* No = new FuzzySet(-1,-1,-0.6,-0.2);   //Negative output 
  FuzzySet* Zo = new FuzzySet(-0.2,0,0,0.2);      //Zero output
  FuzzySet* Po = new FuzzySet(0.2,0.6,1,1);       //Positive output
  
void setup() {
  
  //Serial.begin(9600);
    analogReadResolution(12);                     //4096
    pinMode(9, OUTPUT); // Sets the pin as output
    pinMode(10, OUTPUT); // Sets the pin as output
    

  //***Define Fuzzy Inputs***
  
  //Current Error I_ref-I_meas
  FuzzyInput* IE = new FuzzyInput(1); // id=1

  //Add membership functions to Input
  IE->addFuzzySet(NE);
  IE->addFuzzySet(ZE);
  IE->addFuzzySet(PE);
 
  CC->addFuzzyInput(IE);                          //Add IE as an input for CC
  
  //Current Error Derivative IEt2-IEt1
  FuzzyInput* dIE = new FuzzyInput(2);
 
  //Add membership functions to input
  dIE->addFuzzySet(NDE);
  dIE->addFuzzySet(ZDE);
  dIE->addFuzzySet(PDE);

  CC->addFuzzyInput(dIE);                       //Add dIE as input for CC
//**End of Fuzzy Input Definition**

//**Fuzzy Output Definition**

  FuzzyOutput* d = new FuzzyOutput(1);            //Fuzzy Outout named d with id=1 d is the value by which the duty cycle has to be changed D=D+d where D is the duty cycle

  //Add membership functions to output
  d->addFuzzySet(No);                 
  d->addFuzzySet(Zo);
  d->addFuzzySet(Po);

  CC->addFuzzyOutput(d);                        //Add d as an Output for CC
//**End of Fuzzy Output Definition**

//**Definition of Fuzzy Rules**

  FuzzyRuleAntecedent* ifNN = new FuzzyRuleAntecedent();    //if IE negative and dIE negative
  ifNN->joinWithAND(NE,NDE);                                //link corresponding membership functions to Antecedent using AND 
 
  FuzzyRuleAntecedent* ifNZ = new FuzzyRuleAntecedent();    //if IE negative and dIE zero
  ifNZ->joinWithAND(NE,ZDE);                                //link with and
  
  FuzzyRuleAntecedent* ifNP = new FuzzyRuleAntecedent();    //if IE neagtive and dIE positive
  ifNP->joinWithAND(NE,PDE);                                //link

  FuzzyRuleAntecedent* ifZN = new FuzzyRuleAntecedent();    //if IE zero and dIE negative
  ifZN->joinWithAND(ZE,NDE);                                //link

  FuzzyRuleAntecedent* ifZZ = new FuzzyRuleAntecedent();    //if IE zero and dIE zero
  ifZZ->joinWithAND(ZE,ZDE);                                //link

  FuzzyRuleAntecedent* ifZP = new FuzzyRuleAntecedent();    //if IE zero and dIE positive
  ifZP->joinWithAND(ZE,PDE);                                //link

  FuzzyRuleAntecedent* ifPN = new FuzzyRuleAntecedent();    //if IE positive and dIE negative
  ifPN->joinWithAND(PE,NDE);                                //link

  FuzzyRuleAntecedent* ifPZ = new FuzzyRuleAntecedent();    //if IE positive and dIE zero
  ifPZ->joinWithAND(PE,ZDE);                                //link

  FuzzyRuleAntecedent* ifPP = new FuzzyRuleAntecedent();    //if IE positive and dIE negative
  ifPP->joinWithAND(PE,PDE);  
                                

  FuzzyRuleConsequent* thenO_N = new FuzzyRuleConsequent();  //then d negative
  thenO_N->addOutput(No);                                   //link corresponding output membership function to Consequent
  
  FuzzyRuleConsequent* thenO_Z = new FuzzyRuleConsequent(); //then d zero
  thenO_Z->addOutput(Zo);                                   //link
  
  FuzzyRuleConsequent* thenO_P = new FuzzyRuleConsequent(); //then d positive
  thenO_P->addOutput(Zo);                                   //link
  
  //Build Rules by combining anticidents and consequents
  
  FuzzyRule* fuzzyRule01 = new FuzzyRule(1,ifNN,thenO_N); //fuzzy rule named "fuzzyRule01" consistind of antecedent ifNN and consequent thenO_N with id=1
  FuzzyRule* fuzzyRule02 = new FuzzyRule(2,ifNZ,thenO_N);
  FuzzyRule* fuzzyRule03 = new FuzzyRule(3,ifNP,thenO_Z);
  FuzzyRule* fuzzyRule04 = new FuzzyRule(4,ifZN,thenO_N);
  FuzzyRule* fuzzyRule05 = new FuzzyRule(5,ifZZ,thenO_Z);
  FuzzyRule* fuzzyRule06 = new FuzzyRule(6,ifZP,thenO_P);
  FuzzyRule* fuzzyRule07 = new FuzzyRule(7,ifPN,thenO_Z);
  FuzzyRule* fuzzyRule08 = new FuzzyRule(8,ifPZ,thenO_P);
  FuzzyRule* fuzzyRule09 = new FuzzyRule(9,ifPP,thenO_P);
  
  CC->addFuzzyRule(fuzzyRule01);                        //Add rules to CC
  CC->addFuzzyRule(fuzzyRule02);                        //Add rules to CC
  CC->addFuzzyRule(fuzzyRule03);                        //Add rules to CC
  CC->addFuzzyRule(fuzzyRule04);                        //Add rules to CC
  CC->addFuzzyRule(fuzzyRule05);                        //Add rules to CC
  CC->addFuzzyRule(fuzzyRule06);                        //Add rules to CC
  CC->addFuzzyRule(fuzzyRule07);                        //Add rules to CC
  CC->addFuzzyRule(fuzzyRule08);                        //Add rules to CC
  CC->addFuzzyRule(fuzzyRule09);                        //Add rules to CC

  //**End of Fuzzy Logic Controller definition**//

}

void loop() {
//***************K_Droop calculation****************
I_meas = analogRead(A0);
I_meas = map(I_meas,0,4096,0,100);

V_DG = analogRead(A1);
V_DG = map(V_DG,0,4096,0.1,100);              //maping starts from 0.1 to avoid deviding by 0 later on

V_out = analogRead(A2);
V_out = map(V_out,0,4096,0,450);

I_PV = analogRead(A4);
I_PV = map(I_PV,0,4096,0,100);

V_PV = analogRead(A5);
V_PV = map(V_PV,0,4096,0,50);
//P = analogRead(A3);
//P = map(P,0,1023,0,2000);
//*****MPPT***************//
ipv[1] = ipv[2];
ipv[2] = I_PV;

ppv[1] = ppv[2];
ppv[2] = I_PV*V_PV;

d_I_PV = ipv[2]-ipv[1];
d_P_PV = ppv[2]-ppv[1];


//***************A*******************//
if( V_out>=V_ref-5){               //if V_out > 395V it means the system is no longer starting up and MPPT can commence
  A = false;
}
if(V_out<100){
  A = true;
}
//*******************************//
if(abs(I_ref-I_meas)<=5){
  EN = true;
}
else{
  EN = false;
}

//***MPPT adaptive Droop*******/
if(A||EN){
K = kMPPT(d_I_PV,d_P_PV,K,dK);
}

if(A && K>=0.05){
  K = 0.05;
}
//***********End of K_Droop calculation************//


//**************Calculation of I_ref***************
I_ref = (V_ref-V_out)*K*V_out/V_DG;

if(A && I_ref>=5){
  I_ref = 5;
}
if(!A && I_ref>=I_PV+5){
  I_ref = I_PV+5;
}
//************End of I_ref calculation***********//
//************Current Controller*****************

  eI[1] = eI[2];
  eI[2] = I_ref-I_meas; 
  D = curcont(eI[1],eI[2],I_ref,I_meas,FB,Ib,D);
  
   ds = map(D,0,1,0,255);
  analogWrite(9,ds);
  //Serial.println(dd);
  //delay(5000);

}

/******************Function Definition**********************/
double kMPPT( double dI_PV, double dP_PV, double Kd, double dK){    //calculates Kdroop for MPPT traking dI_PV = T2PV-I1PV[A] dP_PV = P2_PV-P1_PV[W] Kd: old droop coefficient dK incrementation value for Kdroop
  double K;   //Droop coefficient to be returned
  int dPdI;
  dPdI = signn(dI_PV)*signn(dP_PV);
  K = Kd+dPdI*dK;
  return K;
}

int signn(double num){
    if (num < 0) return -1;
    if (num == 0) return 0;
    return 1;
  }


double curcont(double eI1, double eI2, double Ir, double Im, int FLB, int di, double Do){    //eI1 previous current error eI2: current error, Ir: reference current
      double DeI = eI2-eI1;                                                                  //Im: measured current, FLB fuzzy logic bound, di Hysterises band, Do: duty cycle
      double dD;
      double Dc;                                  //New duty cycle
      bool S,R;
    if (eI2>FLB){                                 //Current error lies within FL error band then F-controller is used
      CC->setInput(1,eI2);                        //Enter current error to fuzzy controller
      CC->setInput(1,DeI);                        //Enter error change to fuzzy
      CC->fuzzify();
      dD = CC->defuzzify(1);                      //Fuzzy controller outputs increment/decrement that needs to be added to the Duty Cycle
      Dc = Do+dD;                                 //Update value of duty cycle D=Dold + d
    }
    else{                                        //Current error is within range of hysteretic controller
      S = Im<=Ir-di;                            //S-R flip flop
      R = Im>=Ir+di;
      if(S) Dc=1;
      if(R) Dc=0;
    }
    return Dc;
}

