#include <evolver_si.h>
#include "Tlc5940_SAMD.h"

// Input Variables Used
String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;

String data = "stir";
String end_mark = "end";
int num_vials = 16;
evolver_si in("zv"," !", num_vials);

int Input[16];

void setup()
{

  Tlc.init(4095); // initialise TLC5940 and set all channels off
  Serial1.begin(9600);
  SerialUSB.begin(9600);
  // reserve 200 bytes for the inputString:
  inputString.reserve(1000);
  while (!Serial1);
  
}


void loop() {
  serialEvent();
  if (stringComplete) {
  
    in.analyzeAndCheck(inputString);
    
    if(in.addressFound){
      SerialUSB.println("Found");
      update_values();
//      digitalWrite(12, HIGH);
//      delay(100);
//      Serial1.println(data + "OK" + end_mark);
//      digitalWrite(12, LOW);
      
      
    } else {
      SerialUSB.println("Address Not Found");
    }
    inputString = "";
    stringComplete = false;
    in.addressFound = false;
  }
  exec_stir();
}

void update_values() {
  for (int i = 0; i < num_vials; i++) {
    Input[i] =  in.input_array[i].toInt();
  }
}

void exec_stir()
{
  for (int i = 0; i < num_vials; i++) {
    if (Input[i]  != 0) {
      Tlc.set(i, 0);
      //Serial.print("Code: " + String(in.output_array[i]) + " ");
    }
  }
  //Serial.println();
  
    while(Tlc.update());
    delay(12);
    
   // 10 settings for the stir rate
   for (int n = 0; n < 98; n++) { 
    for (int i = 0; i < num_vials; i++) {

      if (Input[i] == n) {
        Tlc.set(i, 4095);
      }
    }
    while(Tlc.update());
    delay(1);
   }

   delay(70);//250
}

void serialEvent() {
    while (Serial1.available()) {
      char inChar = (char)Serial1.read();
      inputString += inChar;
      if (inChar == '!') {
        stringComplete = true;
      }
    }
  
}
