#include <OneWire.h>
#include <EEPROM.h>
#define EEPROM_write(address, p) {int i = 0; byte *pp = (byte*)&(p);for(; i < sizeof(p); i++) EEPROM.write(address+i, pp[i]);}
#define EEPROM_read(address, p)  {int i = 0; byte *pp = (byte*)&(p);for(; i < sizeof(p); i++) pp[i]=EEPROM.read(address+i);}

#define ReceivedBufferLength 20
char receivedBuffer[ReceivedBufferLength+1];   // store the serial command
byte receivedBufferIndex = 0;

#define ecSensorPin  A1  //EC Meter analog output,pin on analog 1
#define ds18b20Pin  2    //DS18B20 signal, pin on digital 2

#define SCOUNT  100           // sum of sample point
int analogBuffer[SCOUNT];    //store the analog value read from ADC
int analogBufferIndex = 0;

#define compensationFactorAddress 8    //the address of the factor stored in the EEPROM
float compensationFactor;

#define VREF 5000  //for arduino uno, the ADC reference is the power(AVCC), that is 5000mV

boolean enterCalibrationFlag = 0;
float temperature,ECvalue,ECvalueRaw;

OneWire ds(ds18b20Pin);

void setup()
{
    Serial.begin(115200);
    readCharacteristicValues(); //read the compensationFactor
}

void loop()
{
  
  if(serialDataAvailable() > 0)
  {
      byte modeIndex = uartParse();
      ecCalibration(modeIndex);    // If the correct calibration command is received, the calibration function should be called.
  }

   static unsigned long analogSampleTimepoint = millis();
   if(millis()-analogSampleTimepoint > 30U) //every 30ms,read the analog value from the ADC
   {
     analogSampleTimepoint = millis();
     analogBuffer[analogBufferIndex] = analogRead(ecSensorPin);    //read the analog value and store into the buffer,every 40ms
     analogBufferIndex++;
     if(analogBufferIndex == SCOUNT) 
         analogBufferIndex = 0;
   }
   
   static unsigned long tempSampleTimepoint = millis();
   if(millis()-tempSampleTimepoint > 850U)  // every 1.7s, read the temperature from DS18B20
   {
      tempSampleTimepoint=millis();
      temperature = readTemperature();  // read the current temperature from the  DS18B20
   }
   
   static unsigned long printTimepoint = millis();
   if(millis()-printTimepoint > 1000U)
   {
      printTimepoint = millis();
      float AnalogAverage = getMedianNum(analogBuffer,SCOUNT);   // read the stable value by the median filtering algorithm
      float averageVoltage=AnalogAverage*(float)VREF/1024.0;
      if(temperature == -1000)
      {
          temperature = 25.0;      //when no temperature sensor ,temperature should be 25^C default
          Serial.print(temperature,1); 
          Serial.print(F("^C(default)    EC:"));
      }else{
          Serial.print(temperature,1);    //current temperature
          Serial.print(F("^C             EC:"));
      }
     float TempCoefficient=1.0+0.0185*(temperature-25.0);    //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.0185*(fTP-25.0));
     float CoefficientVolatge=(float)averageVoltage/TempCoefficient;   
     if(CoefficientVolatge<150)Serial.println(F("No solution!"));   //25^C 1413us/cm<-->about 216mv  if the voltage(compensate)<150,that is <1ms/cm,out of the range
     else if(CoefficientVolatge>3300)Serial.println(F("Out of the range!"));  //>20ms/cm,out of the range
     else{ 
      if(CoefficientVolatge<=448)ECvalue=6.84*CoefficientVolatge-64.32;   //1ms/cm<EC<=3ms/cm
      else if(CoefficientVolatge<=1457)ECvalue=6.98*CoefficientVolatge-127;  //3ms/cm<EC<=10ms/cm
      else ECvalue=5.3*CoefficientVolatge+2278;                           //10ms/cm<EC<20ms/cm
      ECvalueRaw = ECvalue/1000.0;
      ECvalue = ECvalue/compensationFactor/1000.0;    //after compensation,convert us/cm to ms/cm
      Serial.print(ECvalue,2);     //two decimal
      Serial.print(F("ms/cm"));
       if(enterCalibrationFlag)             // in calibration mode, print the voltage to user, to watch the stability of voltage
       {  
          Serial.print(F("            Factor:"));
          Serial.print(compensationFactor);   
       }
     Serial.println();
     }          
   }   
}

boolean serialDataAvailable(void)
{
  char receivedChar;
  static unsigned long receivedTimeOut = millis();
  while (Serial.available()>0) 
  {   
    if (millis() - receivedTimeOut > 500U) 
    {
      receivedBufferIndex = 0;
      memset(receivedBuffer,0,(ReceivedBufferLength+1));
    }
    receivedTimeOut = millis();
    receivedChar = Serial.read();
    if (receivedChar == '\n' || receivedBufferIndex==ReceivedBufferLength){
		receivedBufferIndex = 0;
		strupr(receivedBuffer);
		return true;
    }else{
      receivedBuffer[receivedBufferIndex] = receivedChar;
      receivedBufferIndex++;
    }
  }
  return false;
}

byte uartParse()
{
  byte modeIndex = 0;
  if(strstr(receivedBuffer, "CALIBRATION") != NULL) 
      modeIndex = 1;
  else if(strstr(receivedBuffer, "EXIT") != NULL) 
      modeIndex = 3;
  else if(strstr(receivedBuffer, "CONFIRM") != NULL)   
      modeIndex = 2;
  return modeIndex;
}

void ecCalibration(byte mode)
{
    char *receivedBufferPtr;
    static boolean ecCalibrationFinish = 0;
    float factorTemp;
    switch(mode)
    {
      case 0:
      if(enterCalibrationFlag)
         Serial.println(F("Command Error"));
      break;
      
      case 1:
      enterCalibrationFlag = 1;
      ecCalibrationFinish = 0;
      Serial.println();
      Serial.println(F(">>>Enter Calibration Mode<<<"));
      Serial.println(F(">>>Please put the probe into the 12.88ms/cm buffer solution<<<"));
      Serial.println();
      break;
     
     case 2:
      if(enterCalibrationFlag)
      {
          factorTemp = ECvalueRaw / 12.88;
          if((factorTemp>0.85) && (factorTemp<1.15))
          {
              Serial.println();
              Serial.println(F(">>>Confrim Successful<<<"));
              Serial.println();
              compensationFactor =  factorTemp;
              ecCalibrationFinish = 1;
          }
          else{
            Serial.println();
            Serial.println(F(">>>Confirm Failed,Try Again<<<"));
            Serial.println();
            ecCalibrationFinish = 0;
          }        
      }
      break;

        case 3:
        if(enterCalibrationFlag)
        {
            Serial.println();
            if(ecCalibrationFinish)
            {
               EEPROM_write(compensationFactorAddress, compensationFactor);
               Serial.print(F(">>>Calibration Successful"));
            }
            else Serial.print(F(">>>Calibration Failed"));       
            Serial.println(F(",Exit Calibration Mode<<<"));
            Serial.println();
            ecCalibrationFinish = 0;
            enterCalibrationFlag = 0;
        }
        break;
    }
}

int getMedianNum(int bArray[], int iFilterLen) 
{
      int bTab[iFilterLen];
      for (byte i = 0; i<iFilterLen; i++)
      {
	  bTab[i] = bArray[i];
      }
      int i, j, bTemp;
      for (j = 0; j < iFilterLen - 1; j++) 
      {
	  for (i = 0; i < iFilterLen - j - 1; i++) 
          {
	    if (bTab[i] > bTab[i + 1]) 
            {
		bTemp = bTab[i];
	        bTab[i] = bTab[i + 1];
		bTab[i + 1] = bTemp;
	     }
	  }
      }
      if ((iFilterLen & 1) > 0)
	bTemp = bTab[(iFilterLen - 1) / 2];
      else
	bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
      return bTemp;
}

void readCharacteristicValues()
{
    EEPROM_read(compensationFactorAddress, compensationFactor);  
    if(EEPROM.read(compensationFactorAddress)==0xFF && EEPROM.read(compensationFactorAddress+1)==0xFF && EEPROM.read(compensationFactorAddress+2)==0xFF && EEPROM.read(compensationFactorAddress+3)==0xFF)
    {
      compensationFactor = 1.0;   // If the EEPROM is new, the compensationFactorAddress is 1.0(default).
      EEPROM_write(compensationFactorAddress, compensationFactor);
    }
}

//returns the temperature from one DS18B20 in DEG Celsius
float readTemperature()
{
  static byte data[12],addr[8];
  static float TemperatureSum = 25;
  static boolean ch = 0;
  if(!ch){
          if ( !ds.search(addr)) {
             // Serial.println("no more sensors on chain, reset search!");
              ds.reset_search();
              return -1000;
          }      
          if ( OneWire::crc8( addr, 7) != addr[7]) {
            //  Serial.println("CRC is not valid!");
              return -1000;
          }        
          if ( addr[0] != 0x10 && addr[0] != 0x28) {
            //  Serial.print("Device is not recognized!");
              return -1000;
          }      
          ds.reset();
          ds.select(addr);
          ds.write(0x44,1); // start conversion, with parasite power on at the end
  }else{  
          byte present = ds.reset();
          ds.select(addr);    
          ds.write(0xBE); // Read Scratchpad            
          for (int i = 0; i < 9; i++) { // we need 9 bytes
            data[i] = ds.read();
          }         
          ds.reset_search();           
          byte MSB = data[1];
          byte LSB = data[0];        
          float tempRead = ((MSB << 8) | LSB); //using two's compliment
          TemperatureSum = tempRead / 16;
    }
          ch = !ch;
          return TemperatureSum;  
}
