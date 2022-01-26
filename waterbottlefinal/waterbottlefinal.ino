//library I used:
#include <SPI.h>
#include <LiquidCrystal.h>
#include <Servo.h>

//Constants won't change:
int weightSensor = A0;

//Variables:
  int sensorValue = 0;
  int sensorMin = 1023;        // minimum sensor value
  int sensorMax = 0;           // maximum sensor value
  int cupMinWeight = 760;      //the net weight of the cup
  float weight = 0;
  int goal = 3;
  int pos = 90;    // variable to store the servo position
  
//time-sensitive variable:
  int totalDrinkVolume = 0;
  unsigned long nowTime = 0;
  int WaterLevel = 0;
  unsigned long startTime = 0;
  int reminderTime = 10000;
  int cupCounter = 0; 

//variables for smoothing:
  const int numReadings = 100;
  int readings[numReadings];      // the readings from the analog input
  int readIndex = 0;              // the index of the current reading
  int total = 0;                  // the running total
  int average = 0;                // the average


LiquidCrystal lcd(9);//for lcd screen
Servo myservo;  // create servo object to control a servo
//image for progress bar
//reference: https://www.instructables.com/Simple-Progress-Bar-for-Arduino-and-LCD/
byte zero[] = {B00000,B00000,B00000,B00000,B00000,B00000,B00000,B00000};
byte one[] = {B10000,B10000,B10000,B10000,B10000,B10000,B10000,B10000};
byte two[] = {B11000,B11000,B11000,B11000,B11000,B11000,B11000,B11000};
byte three[] = {B11100,B11100,B11100,B11100,B11100,B11100,B11100,B11100};
byte four[] = {B11110,B11110,B11110,B11110,B11110,B11110,B11110,B11110};
byte five[] = {B11111,B11111,B11111,B11111,B11111,B11111,B11111,B11111};

void setup() {
    lcd.begin(16, 2);
    lcd.createChar(0, zero);
    lcd.createChar(1, one);
    lcd.createChar(2, two);
    lcd.createChar(3, three);
    lcd.createChar(4, four);
    lcd.createChar(5, five);
    
    Serial.begin(9600);
    myservo.attach(10);
    
    //initial calibration
    calibration();
    delay(6000);
    bool G_checked = 0;
    while (G_checked != 1) {
      sensorValue = analogRead (weightSensor);
      Serial.println(sensorValue);
      String sentence1 = "Put your";
      String sentence2 = "empty cup on";
      printScreen (sentence1, sentence2);
      delay(1000);
      if (sensorValue>10) {
        G_checked = 1;
      }
   }
   delay(3000);
}

void loop() {
  int raw = analogRead (weightSensor);
  Serial.println("raw reading" + String (raw));
  int averageweight = getweight();
  if (averageweight < 330) {
      getcupweight();
      delay (1000);
  }else if (averageweight > cupMinWeight) {
    String sentence1 = "Welcome to your";
    String sentence2 = "First cup";
    printScreen (sentence1, sentence2);
    delay(2000);
    dailyTask();
    delay (2000);
  }
}

void dailyTask () {//need to drink to 3L to finish daily task
  delay (2000); //wait for cup and water to balance
  WaterLevel = getweight();
  startTime = millis ();

  while (1) {
    sensorValue = getweight();
    nowTime = millis ();
    //the person take up the cup to drink:
    if (sensorValue ==0){ //the cup is up
      while (getweight() ==0) { //cup is not yet returned
        continue;
      }
      delay (3000); //cup is back, wait for water to balance
      int newWaterLevel = getweight(); //get the water volume
      nowTime = millis(); //update the time

      if (newWaterLevel <= WaterLevel) { //drink something from the cup
        totalDrinkVolume += WaterLevel - newWaterLevel;
      } else if (newWaterLevel > WaterLevel) { //new level larger than previous level, indicate the user drink and go to refill the cup
        totalDrinkVolume += WaterLevel;
        cupCounter += 1;
        Serial.println ("cup" + String (cupCounter));
        delay (3000);
      }

      //after drinking, get new level, update the time
      WaterLevel = getweight();
      startTime = nowTime;
    }
    
//    Serial.println (nowTime);
//    Serial.println (startTime);

    if (cupCounter >= 0 && cupCounter <=goal) {
       updateProgressBar(cupCounter);
       //have not been drinking for xxx mins, define at the top for different cases
          if (nowTime - startTime > reminderTime) {//reminder
            servoRotation();
            delay (2000);
            startTime = millis (); //update reminder time
          }     
    }
    else if (cupCounter > goal) {
      String sentence1 = "Yeah!!!";
      String sentence2 = "Finished!!";
      printScreen (sentence1, sentence2);
       myservo.write(pos); 
 
    }
  }
}


int getweight() {

 //smoothing process
    for (int i = 0; i < 100; i++){
    total = total - readings[readIndex];
    float r = analogRead (weightSensor);
    readings[readIndex] = r;
    total = total + readings[readIndex];
    readIndex = readIndex + 1;
    
      if (readIndex >= numReadings) {
        // ...wrap around to the beginning:
        readIndex = 0;
      }
  }
      average = total / numReadings;
//      return average;

  int fsrReading;     // the analog reading from the FSR resistor divider
  int fsrVoltage;     // the analog reading converted to voltage
  float fsrResistance;  // The voltage converted to resistance
  float fsrConductance; 
  float fsrForce;       // Finally, the resistance converted to force
//
  fsrReading = analogRead (weightSensor);
//  Serial.print("Analog reading = ");
//  Serial.println(fsrReading);
// 
//  // analog voltage reading ranges from about 0 to 1023 which maps to 0V to 5V (= 5000mV)
//  fsrVoltage = map(fsrReading, 0, 1023, 0, 5000);
//  Serial.print("Voltage reading in mV = ");
//  Serial.println(fsrVoltage);  
//
//  //reference: https://lastminuteengineers.com/fsr-arduino-tutorial/
  if (fsrVoltage == 0) {
//    Serial.println("No pressure");  
  } else {
    // The voltage = Vcc * R / (R + FSR) where R = 10K and Vcc = 5V
    // so FSR = ((Vcc - V) * R) / V        
    fsrResistance = 5000 - fsrVoltage;     // fsrVoltage is in millivolts so 5V = 5000mV
    fsrResistance *= 10000;                // 10K resistor
    fsrResistance /= fsrVoltage;
    fsrConductance = 1000000;           // we measure in micromhos so 
    fsrConductance /= fsrResistance;
    }
 
    // Use the two FSR guide graphs to approximate the force
    if (fsrConductance <= 1000) {
      fsrForce = fsrConductance / 50;
//      Serial.print("Force in Newtons: ");
//      Serial.println(fsrForce);   
    } else {
      fsrForce = fsrConductance - 1000;
      fsrForce /= 30;
//      Serial.print("Force in Newtons: ");
//      Serial.println(fsrForce);         
    }
//    Serial.println("avg " +String(fsrReading));
    if (fsrReading<700) {
      weight = 0;
    }else{
      weight = fsrReading*1;
    }
    Serial.println("weight " + String (weight)) ; 
    return weight;

  }

int getcupweight (){
    sensorValue = getweight();
    String sentence3 = "Calibrating...";
    String sentence4 = "Let's start!";
    printScreen (sentence3, sentence4);
    delay(2000);
}

void calibration () {//function to calibrate at the begining
  Serial.println("cali start");
  while (millis() < 2000) {
    sensorValue = analogRead(weightSensor);

    // record the maximum sensor value
    if (sensorValue > sensorMax) {
      sensorMax = sensorValue;
    }

    // record the minimum sensor value
    if (sensorValue < sensorMin) {
      sensorMin = sensorValue;
    }
    lcd.setCursor(0, 0);
    lcd.print("Welcome to");
    lcd.setCursor(0, 1);
    lcd.print("your day!!");
  }
}

void updateProgressBar(int cupCounter) {
       lcd.clear();
       lcd.setCursor(0, 0);
       lcd.print("Your progress");
       int i = 0 ;
       while (i<cupCounter*4){
       lcd.setCursor(i, 1);
       lcd.write(5);
       i+=1;
       }
     
}

void servoRotation (){
  for (pos = 0; pos <= 180; pos += 90) { 
    myservo.write(pos);              
    delay(200);                       
  }
  for (pos = 180; pos >= 0; pos -= 90) {
    myservo.write(pos);             
    delay(200);                       
  }
}
 
void printScreen (String sentence1, String sentence2) {//function to print lcd
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(sentence1);
  lcd.setCursor(0, 1);
  lcd.print(sentence2);
}
