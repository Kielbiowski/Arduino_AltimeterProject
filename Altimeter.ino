/*
Niniejszy kod prezentuje oprogramowanie wysokościomierza z dnia obrony pracy inżynierskiej.
Mam pełną świadomość jakości tego kodu, dlatego jeśli nadal biorą Państwo pod uwagę moją kandydaturę
na stanowisko inżyniera oprogramowania, prześlę wersję zrefactorowaną zgodnie z moim obecnym 
stanem wiedzy oraz umiejętności, tym samym ułatwiając Państwu decyzję.
Szymon Kiełbiowski
*/

#include <Dps310.h>
#include <SimpleKalmanFilter.h>
#include <Encoder.h>
#include <LedControl.h>

#define CLK 2 //Numer Pinu CLK enkodera
#define DT 4 //Numer Pinu DT enkodera
Encoder ECR(DT,CLK);//Deklaracja enkodera

int mod1=0;
int mod2=0;
byte menu=0;

long encoder=0;

int standardPressure=1013;
float groundPressure=101300;


//Configuration of temperature/pressure sensor object
Dps310 Dps310PressureSensor = Dps310();
//Declaration of variables regarding temperature/pressure measurements
float temperature;
float pressure;
uint8_t oversampling = 7;
int16_t ret;

unsigned long timestamp = micros();

//Configuration of Kalman filter object
SimpleKalmanFilter varioKalman(50,50,0.005);

//Configuration of led display object
LedControl lc=LedControl(11,13,8,2);

void setup() 
{
  //DPS310 config: 0-measure rate, 7-oversampling rate
  Dps310PressureSensor.begin(Wire);
  int16_t ret = Dps310PressureSensor.startMeasureBothCont(0,7,0,7); 

  // Serial config for testing purposes
  Serial.begin(9600); 

  //Encoder config
  ECR.write(0);
  attachInterrupt(1, onButtonPress, LOW);

  //Display config
  for(int a=0; a<lc.getDeviceCount(); a++){
      lc.shutdown(a,false);
      lc.setIntensity(a,10);
      lc.clearDisplay(a);
    }      
}

void loop() 
{
  //Temperature and pressure measurements, timestamp record
  uint8_t temperatureCount = 20;
  float temperatureArray[temperatureCount];
  uint8_t pressureCount = 20;
  float pressureArray[pressureCount];
  ret = Dps310PressureSensor.getContResults(temperatureArray, temperatureCount, pressureArray, pressureCount);
  temperature = temperatureArray[0];
  pressure = pressureArray[0];
  timestamp=micros();
  
  switch (menu)//Menu that enables user to choose what value to change/adjust.
  {
    case 0://Adjusting base pressure for altimeters
      if(encoder<ECR.read()/4){
        standardPressure++;
        displayPressure(standardPressure);
      }
      if(encoder>ECR.read()/4){
        standardPressure--;
        displayPressure(standardPressure);
      }
    break;
    
    case 1://Changing modes of left display (temp/vario)
      if(encoder<ECR.read()/4){
        mod1++;
        if(mod1>2) mod1=0;
      }
      if(encoder>ECR.read()/4){
        mod1--;
        if(mod1<0) mod1=2;
      }
    break;
    
    case 2://Changing modes of right display (altimeters)
      if(encoder<ECR.read()/4){
        mod2++;
        if(mod2>2) mod2=0;
      }
      if(encoder>ECR.read()/4){
        mod2--;
        if(mod2<0) mod2=2;
      }
    break;
    }
    
    encoder=ECR.read()/4;

  switch (mod1) //Menu of modes controlled via encoder
  {
    case 0: celsiusTemp();
    switch (mod2){
        case 0: groundAltitude(); break;
        case 1: meterAltitude(); break;
        case 2: feetAltitude(); break;
        }
    break;
    case 1: farenheitTemp();
      switch (mod2){
        case 0: groundAltitude(); break;
        case 1: meterAltitude(); break;
        case 2: feetAltitude(); break;
        }
    break;
    case 2: feetVario(temperatureArray, temperatureCount, pressureArray, pressureCount);
      switch (mod2){
        case 0: groundAltitude(); break;
        case 1: meterAltitude(); break;
        case 2: feetAltitude(); break;
        }
    break;
  } 
}

void groundAltitude() {
  displayRight(calculateAltitude('G'),'G');
}

void meterAltitude() {
  displayRight(calculateAltitude('M'),'H');
}
  
void feetAltitude() {
  displayRight(calculateAltitude('F'),'F');
}

void celsiusTemp(){
  displayLeft(temperature,'C');
}

void farenheitTemp(){
  displayLeft(temperature*9/5+32,'F');//Celcius to Farenheit conversion
}

void feetVario(float temperatureArray[],uint8_t temperatureCount, float pressureArray[], uint8_t pressureCount){
  //Slight delay to maximize effectiveness of variometer
  //delay(100);
  
  //New temperature and pressure measurements for variometer mode
  ret = Dps310PressureSensor.getContResults(temperatureArray, temperatureCount, pressureArray, pressureCount);
  float varioTemperature = temperatureArray[0];
  float varioPressure = pressureArray[0];
  
  //Timestamp calculation for vertical velocity calculations
  unsigned long timestampDelta = micros()- timestamp + 1;
  timestamp = timestampDelta;

  //Variometer calculation, 60000000 for time conversion from microseconds to minutes
  float vario = ((pow((standardPressure*100.0/varioPressure),(1.0/5.257))-1.0)*(varioTemperature+273.15)*3.28084/0.0065 - calculateAltitude('F'))/timestampDelta*60000000;

  Serial.println();
  Serial.println(vario);
  float kalmanvario=varioKalman.updateEstimate(vario);
  if (kalmanvario>0)
  tone(10,pow(kalmanvario+100,1.2),50);
  displayLeft(kalmanvario,'V');
}

float calculateAltitude(char calcUnit){
  float result = (pow((standardPressure*100.0/pressure),(1.0/5.257))-1.0)*(temperature+273.15)/0.0065;//altitude in meters
  if(calcUnit=='M')return result;
  else if(calcUnit=='G')return result == (pow((groundPressure*100.0/pressure),(1.0/5.257))-1.0)*(temperature+273.15)*3.28084/0.0065;//altitude above ground in feet
  return result*3.28084;//altitude in feet
}

void displayLeft(float value, char mode){
  //Range check, display error
  int toDecimal = digitsToDecimalPoint(value);
  if (toDecimal>3) displayErrorLeft();
  else{
    //Sign check, display minus or blank  
    if(value<0){
      lc.setChar(0,0,'-',0);
      value*=-1;
    } else lc.setChar(0,0,' ',0);
  
    int* digits = valueToDigits(value);
    
    //Display data
    int i = 1;
    if(toDecimal==0){
        lc.setDigit(0,1,(byte)0,1);
        i++;
      }
    for (; i<=3; i++){
      if (i==toDecimal) lc.setDigit(0,i,(byte)digits[i-1],1);
      else lc.setDigit(0,i,(byte)digits[i-1],0);
    }
  }
  
  //Display mode
  switch(mode){
    case 'C': lc.setRow(0,4,B01001110); break;//Displaying "C" 
    case 'F': lc.setChar(0,4,'F',0); break;//Displaying "F"
    case 'V': lc.setRow(0,4,B00111110); break;//Displaying "V"
  } 
}

void displayErrorLeft(){
  for (int i=0;i<=3;i++){
    lc.setChar(0,i,'E',0);
  }
}

void displayRight(float value, char mode){
  //Range check, display error
  int toDecimal = digitsToDecimalPoint(value);
  if (toDecimal>5) displayErrorRight();
  else{
    //Sign check, display minus or blank  
    if(value<0){
      lc.setChar(0,5,'-',0);
      value*=-1;
    } else lc.setChar(0,5,' ',0);
  
    int* digits = valueToDigits(value);
    
    //Display data
    int i = 6;
    if(toDecimal==0){
        lc.setDigit(0,6,(byte)0,1);
        i++;
      }
    for (; i<=10; i++){
      if(i<=7){
        if (i==toDecimal+5) lc.setDigit(0,i,(byte)digits[i-6],1);
        else lc.setDigit(0,i,(byte)digits[i-6],0);
      }else{
      if (i==toDecimal+5) lc.setDigit(1,i-8,(byte)digits[i-6],1);
      else lc.setDigit(1,i-8,(byte)digits[i-6],0);
      }
    }
  }
  
  //Display mode
  switch(mode){
    case 'G': lc.setRow(1,3,B01011110); break;//Displaying "G"
    case 'H': lc.setChar(1,3,'H',0); break;//Displaying "H"
    case 'F': lc.setChar(1,3,'F',0); break;//Displaying "F"
  } 
}

void displayErrorRight(){
  for (int i=5;i<=7;i++){
    lc.setChar(0,i,'E',0);
  }
  for (int i=0;i<=2;i++){
    lc.setChar(1,i,'E',0);
  }
}

void displayPressure(int pressure){
    int* digits = valueToDigits(pressure);
     
    //Display data
    for (int i = 5; i<=7; i++) lc.setDigit(0,i,(byte)digits[i-5],0);
    lc.setDigit(1,0,(byte)digits[3],0);
    
    //Displaying HPA
    lc.setChar(1,1,'H',0);
    lc.setChar(1,2,'P',0);
    lc.setChar(1,3,'A',0);
    
    //Displays refreshing
    delay(500);
    lc.clearDisplay(0);
    lc.clearDisplay(1);
}

int digitsToDecimalPoint(float value){
  int counter = 0;
  while (value>=1){
    value/=10;
    counter++;
  }
  return counter;
}

int* valueToDigits(float floatValue){
  //Data prep
  int toDecimal = digitsToDecimalPoint(floatValue);
  long int intValue  = floatValue*10000;
  //+2 to size as value is multiplied by 10000
  int sizeOfValue = toDecimal+4;
  static int digits [10];

  //Conversion to digits
  for (int i=sizeOfValue-1;i>=0;i--){
    digits[i]=intValue%10;
    intValue/=10;
  }
  return digits;
}

void onButtonPress()
{
  unsigned long buttonPressTimestamp = micros();
  while(digitalRead(3)==LOW) delayMicroseconds(10000);
  
  //2 second button press sets groundPressure
  if(micros()-buttonPressTimestamp>=2000000) groundPressure=pressure;
  else{
    menu++;
    if(menu>2) menu=0;
  }
  
}
