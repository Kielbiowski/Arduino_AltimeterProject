/*
Niniejszy kod prezentuje oprogramowanie wysokościomierza z dnia obrony pracy inżynierskiej.
Mam pełną świadomość jakości tego kodu, dlatego pracuję nad wersją zrefactorowaną zgodnie z moim obecnym 
stanem wiedzy oraz umiejętności.
Szymon Kiełbiowski
*/

#include <Dps310.h>
#include <SimpleKalmanFilter.h>
#include <Encoder.h>
#include <LedControl.h>

#define CLK 2 //Numer Pinu CLK enkodera
#define DT 4 //Numer Pinu DT enkodera

Encoder ECR(DT,CLK);//Deklaracja enkodera

int mod1=0;//Deklaracja zmiennej pomocniczej trybu temp/vario
int mod2=0;//Deklaracja zmiennej pomocniczej trybu altimetera
byte menu=0;//Deklaracja zmiennej pomocniczej menu
long encoder=0;//Deklaracja zmiennej pomocniczej obrotu enkodera
unsigned long timer=0;//Zmienna pomocnicza czasu wcisniecia przycisku

unsigned long int pressure=1013;//Deklaracja zmiennej ciśnienia odniesienia
float gndpres=101300;//Deklaracja zmiennej ciśnienia odniesienia dla QFE 
int16_t ret;

float mtemperature;//Deklaracja zmiennej temperatury pobranej z czujnika
float mpressure;//Deklaracja zmiennej ciśnienia pobranej z czujnika
unsigned long int mtimer;//Deklaracja zmiennej czasu podczas pierwszego pomiaru

float haltitude;//Wysokość w trybie H
float galtitude;//Wysokość w trybie G
float faltitude;//Wysokość w trybie F

float vtemperature;//Deklaracja zmiennej temperatury na potrzeby wariometru
float vpressure;//Deklaracja zmiennej ciśnienia na potrzeby wariometru
float valtitude;//Deklaracja zmiennej wysokości na potrzeby wariometru
float vario;
float kalmanvario;
unsigned long int vtimer;//Deklaracja zmiennej czasu na potrzeby wariometru

long int var;//zmienne na potrzeby wyswietlaczy
int ones;
int tens;
int hundreds;
int thousends;
int tenthousends;
int hundredthousends;
bool minus;

Dps310 Dps310PressureSensor = Dps310();

SimpleKalmanFilter varioKalman(50,50,0.005);

LedControl lc=LedControl(11,13,8,2); //Deklaracja wyswietlacza LedControl(dataPin, clockPin, csPin, numDevices)

void setup() 
{
  Dps310PressureSensor.begin(Wire);
  ret = Dps310PressureSensor.startMeasureBothCont(0, 7, 0, 7);
  Serial.begin(9600); // Włączenie komunikacji Serial dla testowania
  
  ECR.write(0);//Ustawienie początkowej "pozycji" enkodera
  attachInterrupt(1, PUSH, LOW);//Deklaracja drugiego pinu interrupt jako przycisku enkodera

  for(int a=0; a<lc.getDeviceCount(); a++)
    {
      lc.shutdown(a,false);//Wybudzenie wyświetlacza a-tego
      lc.setIntensity(a,10);//Ustawienie jasności wyświetlacza a-tego
      lc.clearDisplay(a);//"Wyczyszczenie" wyswietlacza a-tego
    }      
}

void loop() 
{
    
    uint8_t pressureCount = 20;
    float xpressure[pressureCount];
    uint8_t temperatureCount = 20;
    float xtemperature[temperatureCount];
  
    ret = Dps310PressureSensor.getContResults(xtemperature, temperatureCount, xpressure, pressureCount);
    mtimer=millis();
    mtemperature = xtemperature[0];
    mpressure = xpressure[0];
      
  float ftemperature=mtemperature*9/5+32;//obliczenie temperatury w stopniach farenheita
  galtitude=(pow((gndpres/mpressure),(1/5.257))-1)*(mtemperature+273.15)/0.0065;//obliczenie wysokości nad poziom GND
  galtitude=galtitude*3.28084;//Zamiana jednostek z metrow na stopy
  haltitude=(pow((pressure*100/mpressure),(1/5.257))-1)*(mtemperature+273.15)/0.0065;//obliczenie wysokości nad poziom odniesienia w metrach
  faltitude=haltitude*3.28084;//Zamiana jednostek z metrow na stopy
  
  switch (menu)
  {
    case 0://zmiana ciśnienia odniesienia dla trybów H,F
    if(encoder<ECR.read()/4)
    {
      pressure++;
      //wyświetlenie nowego, zmienionego enkoderem cisnienia
      var=pressure;
      ones=var%10;
      var=var/10;
      tens=var%10;
      var=var/10;
      hundreds=var%10;
      var=var/10;
      thousends = var;
      lc.setChar(1,1,'H',0);
      lc.setChar(1,2,'P',0);
      lc.setChar(1,3,'A',0);
      lc.setDigit(1,0,(byte)ones,0);
      lc.setDigit(0,7,(byte)tens,0);
      lc.setDigit(0,6,(byte)hundreds,0);
      if(thousends==0)lc.setChar(0,5,' ',0);
      else lc.setDigit(0,5,(byte)thousends,0);
      delay(500);
      lc.clearDisplay(0);
      lc.clearDisplay(1);
    }
    if(encoder>ECR.read()/4)
    {
      pressure--;
      //wyświetlenie nowego, zmienionego enkoderem cisnienia
      var=pressure;
      ones=var%10;
      var=var/10;
      tens=var%10;
      var=var/10;
      hundreds=var%10;
      var=var/10;
      thousends = var;
      lc.setChar(1,1,'H',0);
      lc.setChar(1,2,'P',0);
      lc.setChar(1,3,'A',0);
      lc.setDigit(1,0,(byte)ones,0);
      lc.setDigit(0,7,(byte)tens,0);
      lc.setDigit(0,6,(byte)hundreds,0);
      if(thousends==0)lc.setChar(0,5,' ',0);
      else lc.setDigit(0,5,(byte)thousends,0);
      delay(500);
      lc.clearDisplay(0);
      lc.clearDisplay(1);
    }
    break;
    
    case 1://zmiana trybu wyświetlacza TEMP/VARIO
    if(encoder<ECR.read()/4)
    {
      mod1++;
      if(mod1>2)
      {
      mod1=0;
      }
    }
    if(encoder>ECR.read()/4)
    {
      mod1--;
      if(mod1<0)
      {
       mod1=2;
      }
    }
    break;
    
    case 2://zmiana trybu wyświetlacza ALTIMETER
    if(encoder<ECR.read()/4)
    {
      mod2++;
      if(mod2>2)
      {
        mod2=0;
      }
    }
    if(encoder>ECR.read()/4)
    {
      mod2--;
      if(mod2<0)
      {
        mod2=2;
      }
    }
    break;
  }
  encoder=ECR.read()/4;
  
  switch (mod1)
  {
    case 0:
    if (mtemperature<0)
        {
          lc.setChar(0,0,'-',0);
          mtemperature=mtemperature*-1;
        }
        else lc.setChar(0,0,' ',0);
        var=mtemperature*100;
        ones=var%10;
        var=var/10;
        tens=var%10;
        var=var/10;
        hundreds=var%10;
        var=var/10;
        thousends = var;
        if (thousends==0)lc.setChar(0,1,' ',0);
        else lc.setDigit(0,1,(byte)thousends,0);
        lc.setDigit(0,2,(byte)hundreds,1);
        lc.setDigit(0,3,(byte)tens,0);
        lc.setRow(0,4,B01001110);//wyswietlenie "C"
        
      switch (mod2)
        {
        case 0:
        if(galtitude<0)
        {
          lc.setChar(0,5,'-',0);
          galtitude=galtitude*-1;
          minus=1;
        }
        var=galtitude*100;
        ones=var%10;
        var=var/10;
        tens=var%10;
        var=var/10;
        hundreds=var%10;
        var=var/10;
        thousends=var%10;
        var=var/10;
        tenthousends=var%10;
        var=var/10;
        hundredthousends = var;
        if(hundredthousends==0)lc.setChar(0,5,' ',0);
        else lc.setDigit(0,5,(byte)hundredthousends,0);
        if(tenthousends==0&&hundredthousends==0)lc.setChar(0,6,' ',0);
        else lc.setDigit(0,6,(byte)tenthousends,0);
        if(thousends==0&&tenthousends==0&&hundredthousends==0)lc.setChar(0,7,' ',0);
        else lc.setDigit(0,7,(byte)thousends,0);
        lc.setDigit(1,0,(byte)hundreds,1);
        lc.setDigit(1,1,(byte)tens,0);
        lc.setDigit(1,2,(byte)ones,0);
        lc.setRow(1,3,B01011110);//wyswietlenie "G"
        break;
        
        case 1:
        var=haltitude*100;
        ones=var%10;
        var=var/10;
        tens=var%10;
        var=var/10;
        hundreds=var%10;
        var=var/10;
        thousends=var%10;
        var=var/10;
        tenthousends=var%10;
        var=var/10;
        hundredthousends = var;
        if(hundredthousends==0)lc.setChar(0,5,' ',0);
        else lc.setDigit(0,5,(byte)hundredthousends,0);
        if(hundredthousends==0&&tenthousends==0)lc.setChar(0,6,' ',0);
        else lc.setDigit(0,6,(byte)tenthousends,0);
        if(hundredthousends==0&&tenthousends==0&&thousends==0)lc.setChar(0,7,' ',0);
        else lc.setDigit(0,7,(byte)thousends,0);
        lc.setDigit(1,0,(byte)hundreds,1);
        lc.setDigit(1,1,(byte)tens,0);
        lc.setDigit(1,2,(byte)ones,0);
        lc.setChar(1,3,'H',0);//wyswietlenie "H"
        break;
        
        case 2:
        var=faltitude*100;
        ones=var%10;
        var=var/10;
        tens=var%10;
        var=var/10;
        hundreds=var%10;
        var=var/10;
        thousends=var%10;
        var=var/10;
        tenthousends=var%10;
        var=var/10;
        hundredthousends = var;
        if(hundredthousends==0)lc.setChar(0,5,' ',0);
        else lc.setDigit(0,5,(byte)hundredthousends,0);
        if(hundredthousends==0&&tenthousends==0)lc.setChar(0,6,' ',0);
        else lc.setDigit(0,6,(byte)tenthousends,0);
        if(hundredthousends==0&&tenthousends==0&&thousends==0)lc.setChar(0,7,' ',0);
        else lc.setDigit(0,7,(byte)thousends,0);
        lc.setDigit(1,0,(byte)hundreds,1);
        lc.setDigit(1,1,(byte)tens,0);
        lc.setDigit(1,2,(byte)ones,0);
        lc.setChar(1,3,'F',0);//wyswietlenie "F"
        break;
        }
    break;
    case 1:
    if(ftemperature<0)
        {
          ftemperature=ftemperature*-1;
          lc.setChar(0,0,'-',0);
        }
        var=ftemperature*10;
        ones=var%10;
        var=var/10;
        tens=var%10;
        var=var/10;
        hundreds=var%10;
        var=var/10;
        thousends = var;
        if(thousends==0)lc.setChar(0,0,' ',0);
        else lc.setDigit(0,0,(byte)thousends,0);
        if(thousends==0&&hundreds==0) lc.setChar(0,1,' ',0);
        else lc.setDigit(0,1,(byte)hundreds,0);
        lc.setDigit(0,2,(byte)tens,1);
        lc.setDigit(0,3,(byte)ones,0);
        lc.setChar(0,4,'F',0);//wyswietlenie "F"
      switch (mod2)
        {
        case 0:
        if(galtitude<0)
        {
          lc.setChar(0,5,'-',0);
          galtitude=galtitude*-1;
          minus=1;
        }
        var=galtitude*100;
        ones=var%10;
        var=var/10;
        tens=var%10;
        var=var/10;
        hundreds=var%10;
        var=var/10;
        thousends=var%10;
        var=var/10;
        tenthousends=var%10;
        var=var/10;
        hundredthousends = var;
        if(hundredthousends==0)lc.setChar(0,5,' ',0);
        else lc.setDigit(0,5,(byte)hundredthousends,0);
        if(tenthousends==0&&hundredthousends==0)lc.setChar(0,6,' ',0);
        else lc.setDigit(0,6,(byte)tenthousends,0);
        if(thousends==0&&tenthousends==0&&hundredthousends==0)lc.setChar(0,7,' ',0);
        else lc.setDigit(0,7,(byte)thousends,0);
        lc.setDigit(1,0,(byte)hundreds,1);
        lc.setDigit(1,1,(byte)tens,0);
        lc.setDigit(1,2,(byte)ones,0);
        lc.setRow(1,3,B01011110);//wyswietlenie "G"
        break;
        
        case 1:
        var=haltitude*100;
        ones=var%10;
        var=var/10;
        tens=var%10;
        var=var/10;
        hundreds=var%10;
        var=var/10;
        thousends=var%10;
        var=var/10;
        tenthousends=var%10;
        var=var/10;
        hundredthousends = var;
        if(hundredthousends==0)lc.setChar(0,5,' ',0);
        else lc.setDigit(0,5,(byte)hundredthousends,0);
        if(hundredthousends==0&&tenthousends==0)lc.setChar(0,6,' ',0);
        else lc.setDigit(0,6,(byte)tenthousends,0);
        if(hundredthousends==0&&tenthousends==0&&thousends==0)lc.setChar(0,7,' ',0);
        else lc.setDigit(0,7,(byte)thousends,0);
        lc.setDigit(1,0,(byte)hundreds,1);
        lc.setDigit(1,1,(byte)tens,0);
        lc.setDigit(1,2,(byte)ones,0);
        lc.setChar(1,3,'H',0);//wyswietlenie "H"
        break;
        
        case 2:
        var=faltitude*100;
        ones=var%10;
        var=var/10;
        tens=var%10;
        var=var/10;
        hundreds=var%10;
        var=var/10;
        thousends=var%10;
        var=var/10;
        tenthousends=var%10;
        var=var/10;
        hundredthousends = var;
        if(hundredthousends==0)lc.setChar(0,5,' ',0);
        else lc.setDigit(0,5,(byte)hundredthousends,0);
        if(hundredthousends==0&&tenthousends==0)lc.setChar(0,6,' ',0);
        else lc.setDigit(0,6,(byte)tenthousends,0);
        if(hundredthousends==0&&tenthousends==0&&thousends==0)lc.setChar(0,7,' ',0);
        else lc.setDigit(0,7,(byte)thousends,0);
        lc.setDigit(1,0,(byte)hundreds,1);
        lc.setDigit(1,1,(byte)tens,0);
        lc.setDigit(1,2,(byte)ones,0);
        lc.setChar(1,3,'F',0);//wyswietlenie "F"
        break;
        }
    break;
    case 2:
    delay(100);
    ret = Dps310PressureSensor.getContResults(xtemperature, temperatureCount, xpressure, pressureCount);
        vtimer=millis()-mtimer;
        vtemperature = xtemperature[0];
        vpressure = xpressure[0];
        valtitude=(pow((pressure*100/vpressure),(1/5.257))-1)*(vtemperature+273.15)/0.0065;//obliczenie wysokości nad poziom odniesienia w metrach
        valtitude=valtitude*3.28084;//Zamiana jednostek z metrow na stopy
        
        vario=(valtitude-faltitude)/vtimer*60000;
        Serial.println(vario);
        kalmanvario=varioKalman.updateEstimate(vario);
        if (kalmanvario>0)
        tone(10,pow(kalmanvario+100,1.2),50);
        if(kalmanvario<0)
        {
          lc.setChar(0,0,'-',0);
          kalmanvario=kalmanvario*(-1);
        }
        else lc.setChar(0,0,' ',0);
        var=kalmanvario;
        ones=var%10;
        var=var/10;
        tens=var%10;
        var=var/10;
        hundreds=var;
        lc.setDigit(0,1,hundreds,0);
        lc.setDigit(0,2,tens,0);
        lc.setDigit(0,3,ones,0);
        lc.setRow(0,4,B00111110);//wyswietlenie "V"
      switch (mod2)
        {
          
        case 0:
        if(galtitude<0)
        {
          lc.setChar(0,5,'-',0);
          galtitude=galtitude*-1;
          minus=1;
        }
        var=galtitude*100;
        ones=var%10;
        var=var/10;
        tens=var%10;
        var=var/10;
        hundreds=var%10;
        var=var/10;
        thousends=var%10;
        var=var/10;
        tenthousends=var%10;
        var=var/10;
        hundredthousends = var;
        if(hundredthousends==0)lc.setChar(0,5,' ',0);
        else lc.setDigit(0,5,(byte)hundredthousends,0);
        if(tenthousends==0&&hundredthousends==0)lc.setChar(0,6,' ',0);
        else lc.setDigit(0,6,(byte)tenthousends,0);
        if(thousends==0&&tenthousends==0&&hundredthousends==0)lc.setChar(0,7,' ',0);
        else lc.setDigit(0,7,(byte)thousends,0);
        lc.setDigit(1,0,(byte)hundreds,1);
        lc.setDigit(1,1,(byte)tens,0);
        lc.setDigit(1,2,(byte)ones,0);
        lc.setRow(1,3,B01011110);//wyswietlenie "G"
        break;
        
        case 1:
        var=haltitude*100;
        ones=var%10;
        var=var/10;
        tens=var%10;
        var=var/10;
        hundreds=var%10;
        var=var/10;
        thousends=var%10;
        var=var/10;
        tenthousends=var%10;
        var=var/10;
        hundredthousends = var;
        if(hundredthousends==0)lc.setChar(0,5,' ',0);
        else lc.setDigit(0,5,(byte)hundredthousends,0);
        if(hundredthousends==0&&tenthousends==0)lc.setChar(0,6,' ',0);
        else lc.setDigit(0,6,(byte)tenthousends,0);
        if(hundredthousends==0&&tenthousends==0&&thousends==0)lc.setChar(0,7,' ',0);
        else lc.setDigit(0,7,(byte)thousends,0);
        lc.setDigit(1,0,(byte)hundreds,1);
        lc.setDigit(1,1,(byte)tens,0);
        lc.setDigit(1,2,(byte)ones,0);
        lc.setChar(1,3,'H',0);//wyswietlenie "H"
        break;
        
        case 2:
        var=faltitude*100;
        ones=var%10;
        var=var/10;
        tens=var%10;
        var=var/10;
        hundreds=var%10;
        var=var/10;
        thousends=var%10;
        var=var/10;
        tenthousends=var%10;
        var=var/10;
        hundredthousends = var;
        if(hundredthousends==0)lc.setChar(0,5,' ',0);
        else lc.setDigit(0,5,(byte)hundredthousends,0);
        if(hundredthousends==0&&tenthousends==0)lc.setChar(0,6,' ',0);
        else lc.setDigit(0,6,(byte)tenthousends,0);
        if(hundredthousends==0&&tenthousends==0&&thousends==0)lc.setChar(0,7,' ',0);
        else lc.setDigit(0,7,(byte)thousends,0);
        lc.setDigit(1,0,(byte)hundreds,1);
        lc.setDigit(1,1,(byte)tens,0);
        lc.setDigit(1,2,(byte)ones,0);
        lc.setChar(1,3,'F',0);//wyswietlenie "F"
        break;
        }
    break;
  } 
}
void PUSH()
{
  timer=0;
  while(digitalRead(3)==LOW)
  {
    delayMicroseconds(15000);
    timer+=15;
  }
  if(timer>=2000)
  {
  gndpres=mpressure;
  }
  else
   {
    menu++;
    if (menu>2)
    {menu=0;}
   }
  
}