///// eVOLVER FLUIDIC CONTROL VERSION 4
///// Written by Brandon Wong and Chris Mancuso
///// This version of the code has built in turbidostat and chemostat functionalities
///// The queue from V2 is added to the code in V3 to prevent the need for queues in the
///// Raspberry Pi. Thus a clear command is also integrated into the code for clearing of the queue
///// Turbidostat functionality stops whatever chemostat command was running and turns on pumps ON for a set amount of time.
///// Chemostat functionality turns pumps on at a specific interval of time.

//// Sample Commands sent over RS485 (Serial1):
//// Turn on pumps 1,2,3,5,6,7 (in turbidostat mode): "stt,1110111,10,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, !"
//// Run chemostat mode (will clear tstat queue): "stc,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,500, !"
//// Force queue to clear: "stq,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, !"
//// Force pumps to stop: "sto,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, !"

#include <evolver_si.h>
#include <SimpleTimer.h>
#include "Tlc5940_SAMD.h"
#include <QueueList.h>

// Setup Variables
int num_pumps = 48;
String fluid_mode = "";
String inputString = "";         // a string to hold incoming data
String binary_string = "";
boolean stringComplete = false;  // whether the string is complete
boolean busy = true;
boolean status_ready = true;

// Chemostat Variables
//initialize command storage arrays for commands currently being run and pump state variable
int period[16] = {10000000,10000000,10000000,10000000,10000000,10000000,10000000,10000000,10000000,10000000,10000000,10000000,10000000,10000000,10000000,10000000}; // initialize with 10000 second event period
int duration[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // initialize with 0 second pump duration (don't pump)
int efflux[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // initialize with 0 second pump duration (don't pump)
// initialize speed of pumps
int speedset[2] = {4095,0}; //fully off, max speed
//initialize event time storage variable for each vial, since unsigned long can only run up to 49 days
unsigned long eventtime[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // initialize time of last event as 0 for each vial

// String Address Analyze
evolver_si in("st"," !",18);
evolver_si fromqueue("st"," !",18);
SimpleTimer timer;
QueueList <String> InputQueue;

void setup() {

  pinMode(12, OUTPUT);
  digitalWrite(12, LOW);
  
  // initialize serial:
  Serial1.begin(9600);
  SerialUSB.begin(9600);
  // reserve 200 bytes for the inputString:
  inputString.reserve(2000);
  Tlc.init(4095);
  while (!Serial1);
}



void loop() {
  unsigned long current = millis(); // set current time

  // Runs chemostat timer logic if in "cstat" mode; skips if in "tstat" mode
  if (fluid_mode == "c"){
    chemoRun(current);
  }

  // Read Serial Inputs and checks if it is the right addresses
  serialEvent();
  if (stringComplete) {
    SerialUSB.println(inputString);
    in.analyzeAndCheck(inputString);
    
    // If the address matches, then insert into queue
    if (in.addressFound){
      SerialUSB.println("Input Address Found");
      fluid_mode = in.input_array[0]; // Obtains Fluid Mode ("c" or "t")
      
      /////////////// RUNS CHEMOSTAT MODE /////////////////////////
      if (fluid_mode == "c"){
        clearQueue();
        chemoRead(current);
      }

      /////////////// ENDS CHEMOSTAT MODE /////////////////////////


      /////////////// RUNS TURBIDOSTAT MODE /////////////////////////
      if (fluid_mode == "t"){
        SerialUSB.println("Inserting into queue");
        InputQueue.push(inputString);        
      }
      /////////////// ENDS TURBIDOSTAT MODE /////////////////////////    

      /////////////// CLEARS QUEUE /////////////////////////
      if (fluid_mode == "q"){
        clearQueue();
      }
      /////////////// ENDS CLEAR QUEUE /////////////////////////

      /////////////// PUMP OFF COMMAND /////////////////////////
      if (fluid_mode == "o"){
        allPumpsOFF(current);
        SerialUSB.println("Pumps off!");
      }
      /////////////// ENDS PUMP OFF COMMANDS /////////////////////////
      
    }
    // clear the string:
    inputString = "";
    stringComplete = false;
    in.addressFound = false;
  }  

 if (status_ready){
  if (!InputQueue.isEmpty()){
    String queuestring = InputQueue.pop();
    SerialUSB.println(queuestring);
    fromqueue.analyzeAndCheck(queuestring);
    if (fromqueue.addressFound){

      // Obtains Input String
      allPumpsOFF(current);
      binary_string = fromqueue.input_array[1];
      float duration_tstat_seconds = fromqueue.input_array[2].toFloat();
      long duration_tstat = (1000 * duration_tstat_seconds);
      SerialUSB.println(binary_string);
      SerialUSB.println(duration_tstat);
      tstatStart(binary_string, duration_tstat);
      
    }  
      fromqueue.addressFound = false;
      queuestring = "";
    }
  }   

  timer.run();
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

void chemoRead (unsigned long current) {
  int bolus = in.input_array[17].toInt(); //bolus duration, in milliseconds
  SerialUSB.print("Bolus Size:");
  SerialUSB.println(bolus);
  
  for (int i = 0; i < 16; i++) {
    SerialUSB.print("VIAL "); 
    SerialUSB.print(i); 
    SerialUSB.print(": "); 
    int per_setting = in.input_array[i+1].toInt(); //collect period setting for this vial from string array
    //SerialUSB.println(per_setting);
    if(per_setting==0){
      //check to see if chemostat mode is already off for this vial 
      if(duration[i]!= 0){ //if not already off, then turn off everything
        SerialUSB.println("TURNING OFF");
        // set pump duration to zero
        duration[i] = 0;
        // set efflux to zero
        efflux[i] = 0;
        // set period to 10000 seconds
        period[i] = 10000000;
        // reset event time to stop whatever vial is currently doing
        eventtime[i] = current;
      }
      else {
        SerialUSB.println("ALREADY OFF!");
      }
      // if already off, do nothing
    } 
    else if(period[i] == per_setting){ // if period setting has not changed
      SerialUSB.println("STAY THE SAME");
      // do nothing, setting has not changed
    } 
    else { //update with new settings
      // set pump duration to zero
      duration[i] = bolus;
      // set efflux to 1000 milliseconds
      efflux[i] = 1000;
      // set period appropriately
      period[i] = per_setting;
      // reset event time to stop whatever vial is currently doing
      eventtime[i] = current;
      SerialUSB.print("CHANGING VALUE TO ");
      SerialUSB.println(period[i]);
    }
  }
}

void chemoRun (unsigned long current) {
  // compare current time to figure out elapsed time for each pump
  for (int x = 0; x < 16; x++) {
    if (current - eventtime[x] >= period[x]){ 
      //if period has elapsed, reset event time, turn on pumps for dilution
      eventtime[x] = current; // save event time
      Tlc.set(x,speedset[1]); //influx on
      Tlc.set(x+16,speedset[1]); //efflux on
    }
    else if (current - eventtime[x] >= duration[x]){
      // if influx is finished, but period between pumps hasn't elapsed, turn off influx
      Tlc.set(x,speedset[0]); //influx off
      if (current - eventtime[x] >= efflux[x]){
        //if efflux is finished, but period between pumps hasn't elapsed, turn off efflux
        Tlc.set(x+16,speedset[0]); //efflux off
      }
    }
  }
  Tlc.update(); //update pumps 
}

void allPumpsOFF(unsigned long current) {

  for (int i = 0; i < 16; i++) {
    // set pump duration to zero
    duration[i] = 0;
    // set efflux to zero
    efflux[i] = 0;
    // set period to 10000 seconds
    period[i] = 10000000;
    // reset event time to stop whatever vial is currently doing
    eventtime[i] = current;
    // if already off, do nothing
  } 
    
  for (int i = 0; i < num_pumps; i++){
    Tlc.set(i,4095);
  }
  Tlc.update(); 
}

void tstatStart (String binary_string, long duration_tstat) {
  
  // Sets all pumps to ON or OFF based on 2nd CSV from inputString
  for (int i = 0; i <= binary_string.length(); i++){
    SerialUSB.println(i);
    if (binary_string[binary_string.length()-i-1] == '0'){
       SerialUSB.println("Pump: " + String(i+1) + " OFF!");
       Tlc.set(i, 4095);
    } else if (binary_string[binary_string.length()-i-1] == '1'){
      SerialUSB.println("Pump: " + String(i+1) + " ON!");
      Tlc.set(i,0);
    }
  }
  Tlc.update();
  delay(5);

  // Start Timer
  SerialUSB.println();
  SerialUSB.print("Start (s): ");
  SerialUSB.println(millis() / 1000);
  timer.setTimeout(duration_tstat, tstatEnd);

  status_ready = false;
}


void tstatEnd() {
    SerialUSB.print("End (s): ");
    SerialUSB.println(millis() / 1000);
    for (int i = 0; i < num_pumps; i++) {
      Tlc.set(i, 4095);
    }
    Tlc.update();
    status_ready = true;
}

void clearQueue(){
  while (!InputQueue.isEmpty ()){
    InputQueue.pop ();
  }
  SerialUSB.println("Queue cleared!");
}
  
