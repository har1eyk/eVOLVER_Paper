#include <evolver_si.h>
#include "Tlc5940.h"
#include <Stepper.h>
#include <avr/pgmspace.h>
#include <MemoryFree.h>

//Define Parameters for Fluidics Setup
int stepsPerRevolution = 200; 
int num_pumps = 48;
Stepper myStepper(stepsPerRevolution, 2, 4, 6, 7);

// intialize variables for generating valve config
int controlNum = 48;
String binaryString= "";
String valvePath= "";
int airSource = 15;
int h2oSource = 14;
int bleachSource = 13;
int etohSource = 12;
char buffer[100];
int max_stepsize = 5000;

//Initialze Serial Communication Variables
String inputString = "";         // a string to hold incoming data
String zero_buffer = "";
String binary_string = "";
boolean stringComplete = false;  // whether the string is complete
boolean busy = true;
boolean status_ready = true;
evolver_si in("st"," !",10);
evolver_si check("re"," !",0);

//////////////// PASTE GLOBAL VARIABLES FROM LOOKUP PYTHON SCRIPT ///////////////////
const char bridge_0[] PROGMEM =  "000000000000000000000000000000000000000010000000";
const char* const bridge[] PROGMEM = {bridge_0};

const char effluxgate_0[] PROGMEM =  "000000000000000000000000000000000000000100000000";
const char* const effluxgate[] PROGMEM = {effluxgate_0};

const char rev_filter_0[] PROGMEM =  "000000000000000000000100000000000000000000000000";
const char* const rev_filter[] PROGMEM = {rev_filter_0};

const char up_bypass_0[] PROGMEM =  "000000000000000000000001000000000000000000000000";
const char* const up_bypass[] PROGMEM = {up_bypass_0};

const char allmuxdemux_0[] PROGMEM =  "000000000000000000000000001111000111111010111111";
const char* const allmuxdemux[] PROGMEM = {allmuxdemux_0};

const char syringe_0[] PROGMEM =  "000000000000000000000000000000001000000000000000";
const char* const syringe[] PROGMEM = {syringe_0};

const char mediagate_0[] PROGMEM =  "000000000000000000000000000000010000000000000000";
const char* const mediagate[] PROGMEM = {mediagate_0};

const char media_0[] PROGMEM =  "000000000000010101010000000000000000000000000000";
const char media_1[] PROGMEM =  "000000000000100101010000000000000000000000000000";
const char media_2[] PROGMEM =  "000000000000011001010000000000000000000000000000";
const char media_3[] PROGMEM =  "000000000000101001010000000000000000000000000000";
const char media_4[] PROGMEM =  "000000000000010110010000000000000000000000000000";
const char media_5[] PROGMEM =  "000000000000100110010000000000000000000000000000";
const char media_6[] PROGMEM =  "000000000000011010010000000000000000000000000000";
const char media_7[] PROGMEM =  "000000000000101010010000000000000000000000000000";
const char media_8[] PROGMEM =  "000000000000010101100000000000000000000000000000";
const char media_9[] PROGMEM =  "000000000000100101100000000000000000000000000000";
const char media_10[] PROGMEM =  "000000000000011001100000000000000000000000000000";
const char media_11[] PROGMEM =  "000000000000101001100000000000000000000000000000";
const char media_12[] PROGMEM =  "000000000000010110100000000000000000000000000000";
const char media_13[] PROGMEM =  "000000000000100110100000000000000000000000000000";
const char media_14[] PROGMEM =  "000000000000011010100000000000000000000000000000";
const char media_15[] PROGMEM =  "000000000000101010100000000000000000000000000000";
const char* const media[] PROGMEM = {media_0,media_1,media_2,media_3,media_4,media_5,media_6,media_7,media_8,media_9,media_10,media_11,media_12,media_13,media_14,media_15};

const char mux_0[] PROGMEM =  "000000000000000000000000000100000010101000000000";
const char mux_1[] PROGMEM =  "000000000000000000000000000100000010110000000000";
const char mux_2[] PROGMEM =  "000000000000000000000000000100000011001000000000";
const char mux_3[] PROGMEM =  "000000000000000000000000000100000011010000000000";
const char mux_4[] PROGMEM =  "000000000000000000000000000100000100101000000000";
const char mux_5[] PROGMEM =  "000000000000000000000000000100000100110000000000";
const char mux_6[] PROGMEM =  "000000000000000000000000000100000101001000000000";
const char mux_7[] PROGMEM =  "000000000000000000000000000100000101010000000000";
const char mux_8[] PROGMEM =  "000000000000000000000000001000000010101000000000";
const char mux_9[] PROGMEM =  "000000000000000000000000001000000010110000000000";
const char mux_10[] PROGMEM =  "000000000000000000000000001000000011001000000000";
const char mux_11[] PROGMEM =  "000000000000000000000000001000000011010000000000";
const char mux_12[] PROGMEM =  "000000000000000000000000001000000100101000000000";
const char mux_13[] PROGMEM =  "000000000000000000000000001000000100110000000000";
const char mux_14[] PROGMEM =  "000000000000000000000000001000000101001000000000";
const char mux_15[] PROGMEM =  "000000000000000000000000001000000101010000000000";
const char* const mux[] PROGMEM = {mux_0,mux_1,mux_2,mux_3,mux_4,mux_5,mux_6,mux_7,mux_8,mux_9,mux_10,mux_11,mux_12,mux_13,mux_14,mux_15};

const char peri2_0[] PROGMEM =  "000000000010000000000000000000000000000000000000";
const char* const peri2[] PROGMEM = {peri2_0};

const char peri1_0[] PROGMEM =  "000000000001000000000000000000000000000000000000";
const char* const peri1[] PROGMEM = {peri1_0};

const char down_path_0[] PROGMEM =  "000000000000000000000000010000000000000000000000";
const char* const down_path[] PROGMEM = {down_path_0};

const char down_bypass_0[] PROGMEM =  "000000000000000000000010000000000000000000000000";
const char* const down_bypass[] PROGMEM = {down_bypass_0};

const char up_path_0[] PROGMEM =  "000000000000000000000000000000100000000000000000";
const char* const up_path[] PROGMEM = {up_path_0};

const char for_filter_0[] PROGMEM =  "000000000000000000001000000000000000000000000000";
const char* const for_filter[] PROGMEM = {for_filter_0};

const char wastegate_0[] PROGMEM =  "000000000000000000000000100000000000000000000000";
const char* const wastegate[] PROGMEM = {wastegate_0};

const char demux_0[] PROGMEM =  "000000000000000000000000000001000000000000010101";
const char demux_1[] PROGMEM =  "000000000000000000000000000001000000000000100101";
const char demux_2[] PROGMEM =  "000000000000000000000000000001000000000000011001";
const char demux_3[] PROGMEM =  "000000000000000000000000000001000000000000101001";
const char demux_4[] PROGMEM =  "000000000000000000000000000001000000000000010110";
const char demux_5[] PROGMEM =  "000000000000000000000000000001000000000000100110";
const char demux_6[] PROGMEM =  "000000000000000000000000000001000000000000011010";
const char demux_7[] PROGMEM =  "000000000000000000000000000001000000000000101010";
const char demux_8[] PROGMEM =  "000000000000000000000000000010000000000000010101";
const char demux_9[] PROGMEM =  "000000000000000000000000000010000000000000100101";
const char demux_10[] PROGMEM =  "000000000000000000000000000010000000000000011001";
const char demux_11[] PROGMEM =  "000000000000000000000000000010000000000000101001";
const char demux_12[] PROGMEM =  "000000000000000000000000000010000000000000010110";
const char demux_13[] PROGMEM =  "000000000000000000000000000010000000000000100110";
const char demux_14[] PROGMEM =  "000000000000000000000000000010000000000000011010";
const char demux_15[] PROGMEM =  "000000000000000000000000000010000000000000101010";
const char* const demux[] PROGMEM = {demux_0,demux_1,demux_2,demux_3,demux_4,demux_5,demux_6,demux_7,demux_8,demux_9,demux_10,demux_11,demux_12,demux_13,demux_14,demux_15};

const char influx_0[] PROGMEM =  "000000000000000000000000000000000000000001000000";
const char* const influx[] PROGMEM = {influx_0};

////////////////////////// END PASTING FROM LOOKUP PYTHON SCRIPT/////////////////////////////

void setup() {
  Tlc.init(4095); // initialize TLC5940
  Serial.begin(9600);  // initialize serial
  binaryString.reserve(controlNum);
  valvePath.reserve(controlNum);
  inputString.reserve(2000);  // reserve 200 bytes for the inputString:
  myStepper.setSpeed(150);  // Set speed for syringe pump
}

void loop() {
  serialEvent(); // Check for serial input

  // if serial input ends with correct ending (' !'), then run code in if statement
  if (stringComplete) {
    //Serial.println(inputString);
    check.analyzeAndCheck(inputString); //ready status check
    in.analyzeAndCheck(inputString); // start analyzing string check

    // if inputString matches status check address
    if(check.addressFound){
      busy = true;
      delay(100);
      // responds if is ready for next command, if ready, Beaglebone will send in check command
      if (status_ready){
        Serial.println("datareadyend");
      } else {
        Serial.println("databusyend");
      }   
    }

    // if inputString matches status check address
    if (in.addressFound){
      busy = false;
      //Serial.println("not busy now!");
    }

    // if input command properly received and is not busy,
    if (in.addressFound && !busy && status_ready){
      float start_time = millis();
      
      ///////////// ADD CODE HERE FOR ABSTRACTION OF FLUIDIC COMMANDS //////////////////
      if (in.input_array[0] == "dilute"){
        //Serial.println("Diluting Now!");
        //dilute(0, 1000, 1000, 3, 3, 3);
        dilute(in.input_array[1].toInt(), in.input_array[2].toInt(), in.input_array[3].toInt(), 
               in.input_array[4].toInt(), in.input_array[5].toInt(), in.input_array[6].toInt());
      }
      if (in.input_array[0] == "dilutemix"){
        //Serial.println("Diluting Now!");
        //dilute(0, 1000, 1000, 3, 3, 3);
        dilute_mix(in.input_array[1].toInt(), in.input_array[2].toInt(), in.input_array[3].toInt(), 
               in.input_array[4].toInt(), in.input_array[5].toInt(), in.input_array[6].toInt(),
               in.input_array[7].toInt(), in.input_array[8].toInt());
      }
      if (in.input_array[0] == "v2v"){
        //Serial.println("Vial to Vial!");
        //vial2vial(1, 1000, 1000, 1, 15, 1000, 3, 3, 1000);
        vial2vial(in.input_array[1].toInt(), in.input_array[2].toInt(), in.input_array[3].toInt(), 
               in.input_array[4].toInt(), in.input_array[5].toInt(), in.input_array[6].toInt(),
               in.input_array[7].toInt(), in.input_array[8].toInt());
      }
      if (in.input_array[0] == "v2v_2x"){
        //Serial.println("Vial to Vial!");
        //vial2vial(1, 1000, 1000, 1, 15, 1000, 3, 3, 1000);
        vial2vial_2x(in.input_array[1].toInt(), in.input_array[2].toInt(), in.input_array[3].toInt(), 
               in.input_array[4].toInt(), in.input_array[5].toInt(), in.input_array[6].toInt(),
               in.input_array[7].toInt(), in.input_array[8].toInt(),in.input_array[9].toInt());
      }
      if (in.input_array[0] == "cleanSyringe"){
        //Serial.println("Clean Syringe!");
        cleanSyringe(in.input_array[1].toInt(), in.input_array[2].toInt());
      }

      if (in.input_array[0] == "flush"){
        //Serial.println("Flush through all lines via Bridge!");
        flushLines(in.input_array[1].toInt(),in.input_array[2].toInt());
      }

      //// Scripts for setup
      
      if (in.input_array[0] == "cleanDev"){
        //Serial.println("Clean Device!");
        //cleanDevice(7000, 1000, 3)
        cleanDevice(in.input_array[1].toInt(), in.input_array[2].toInt(), in.input_array[3].toInt());
      }

      if (in.input_array[0] == "cleanML"){
        //Serial.println("Clean Lines!");
        //sterilzeLines(5);
        sterilzeMediaLines(in.input_array[1].toInt());
      }

      if (in.input_array[0] == "cleanVL"){
        //Serial.println("Clean Vial Lines!");
        //sterilzeLines(5);
        sterilzeVialLines(in.input_array[1].toInt());
      }

      

     if (in.input_array[0] == "washLines"){ 
        //Serial.println("Wash Lines of Excess EtOH when cleaning!");
        //washLines(5);
        washLines(in.input_array[1].toInt(), in.input_array[2].toInt(), in.input_array[3].toInt());
      }


     if (in.input_array[0] == "washHM"){ 
        //Serial.println("Wash Lines of Excess EtOH when cleaning!");
        washHalfmux();
      }
      /////////////////////////////////////////////////////////////////////////////

      float elapsed_time = (millis() - start_time)/1000;
      //Serial.print("Elapsed Time:");
      //Serial.println(elapsed_time);
      
      // clear the string
      busy = true;
      serialFlush();
    }

    inputString = "";
    zero_buffer = "";
    stringComplete = false;
    in.addressFound = false;
    check.addressFound = false;
  }
}

////////////////////////// CLEANING FUNCTIONS /////////////////////////////
void washLines(int media_amount, int prime_amount, float flush_time){
  for (int i = 0; i < 16; i = i + 1) {
    flushToVial(h2oSource, i, flush_time);
    flushToVial(airSource, i, flush_time);
  }
}

void flushLines(int media_num, float peri_time){
  for (int i = 0; i < 16; i = i + 1) {
    flush_singlemuxdemux(media_num, i, peri_time);
  }
}

void sterilzeMediaLines(float peri_time){
  for (int i = 0; i < 16; i = i + 1) {
    flush_singlemuxdemux(i, i, peri_time);
  }
}

void sterilzeVialLines(float peri_time){
  for (int i = 0; i < 16; i = i + 1) {
    influx_clean (i, peri_time);
    vial2waste (i, peri_time);
  }
}

void washHalfmux (){
    //Flush MUX/ DEMUX
  flush_halfmux(etohSource,5);
  flush_halfmux(bleachSource,5);
  flush_halfmux(h2oSource,20);
  flush_halfmux(airSource,10);
}

void cleanDevice(int wash_amount, int prime_amount, float peri_time){
  draw_syringe_media(etohSource, wash_amount, prime_amount);
  syringe2waste(wash_amount, prime_amount);
  flush_upBypass(etohSource, peri_time);
  flush_downBypass(etohSource, peri_time);
  flush_upBypass(etohSource, peri_time);
  flushLines(etohSource, peri_time);

  draw_syringe_media(h2oSource, wash_amount, prime_amount);
  syringe2waste(wash_amount, prime_amount);
  draw_syringe_media(h2oSource, wash_amount, prime_amount);
  syringe2waste(wash_amount, prime_amount);
  
  flush_upBypass(h2oSource, peri_time);
  flush_downBypass(h2oSource, peri_time);
  flush_upBypass(h2oSource, peri_time);
  flushLines(etohSource, peri_time);

  flush_upBypass(airSource, peri_time);
  flush_downBypass(airSource, peri_time);
  flush_upBypass(airSource, peri_time);
  flushLines(etohSource, peri_time);
}

void cleanSyringe(int wash_amount, int prime_amount){
  draw_syringe_media(h2oSource, wash_amount, prime_amount);
  syringe2waste(wash_amount, prime_amount);
  draw_syringe_media(h2oSource, wash_amount, prime_amount);
  syringe2waste(wash_amount, prime_amount);
}

//////// START WRITING FUNCTIONS FOR FLUID ABSTRACTIONS (Level 3 Abstractions///////////////


void vial2vial(int media_num, int media_amount, int prime_amount, int vial_out, int vial_in, 
               int efflux_offset, float peri_time,int clean_offset){
  draw_syringe_media(h2oSource, max_stepsize, 0);
  syringe2waste(max_stepsize, 0);
  efflux (vial_out,peri_time,flush_time);
  efflux (vial_in,peri_time,flush_time);
  
  draw_sample(media_num, media_amount, prime_amount, vial_out, efflux_offset);
  syringe2vial_efflux(media_amount, prime_amount+efflux_offset, vial_in);
  vial2syringe(media_amount, vial_out);
  syringe2vial_efflux(media_amount, prime_amount+efflux_offset , vial_in);
  flushToVial_noefflux(airSource, vial_in, 3);
  vial2waste (vial_in,peri_time);
  vial2waste (vial_out,peri_time);
  //Wash with EtOH
  syringe_EtOH_clean(media_amount+clean_offset);
  syringe2waste(media_amount+clean_offset, 1);
  //Wash with H20 twice
  draw_syringe_media(h2oSource, media_amount+clean_offset, 1);
  syringe2waste(media_amount+clean_offset, 1);
  draw_syringe_media(h2oSource, media_amount+clean_offset, 1);
  syringe2waste(media_amount+clean_offset, 1);
  //Flush MUX/ DEMUX
  flush_halfmux(etohSource,5);
  flush_halfmux(bleachSource,5);
  flush_halfmux(h2oSource,10);
  flush_halfmux(airSource,10);
}

void vial2vial_2x(int media_num, int media_amount, int prime_amount, int vial_out, int vial_in1, 
               int vial_in2, int efflux_offset, float peri_time,int clean_offset){
  
  draw_syringe_media(h2oSource, max_stepsize, 0);
  syringe2waste(max_stepsize, 0);
  efflux (vial_out,peri_time,flush_time);
  efflux (vial_in1,peri_time,flush_time);
  efflux (vial_in2,peri_time,flush_time);
  
  draw_sample(media_num, media_amount, prime_amount, vial_out, efflux_offset);
  syringe2vial_efflux(media_amount, prime_amount+efflux_offset, vial_in1);
  vial2syringe(media_amount, vial_out);
  syringe2vial_efflux(media_amount, prime_amount+efflux_offset , vial_in1);
  flushToVial_noefflux(airSource, vial_in1, 3);
  vial2waste (vial_in1,peri_time);
  vial2waste (vial_out,peri_time);

  draw_sample(media_num, media_amount, prime_amount, vial_out, efflux_offset);
  syringe2vial_efflux(media_amount, prime_amount, vial_in2);
  vial2syringe(media_amount, vial_out);
  syringe2vial_efflux(media_amount, prime_amount, vial_in2);
  flushToVial_noefflux(airSource, vial_in2, 3);
  vial2waste (vial_in2,peri_time);
  vial2waste (vial_out,peri_time);
  
  //Wash with EtOH
  syringe_EtOH_clean(media_amount+clean_offset);
  syringe2waste(media_amount+clean_offset, 1);
  //Wash with H20 twice
  draw_syringe_media(h2oSource, media_amount+clean_offset, 1);
  syringe2waste(media_amount+clean_offset, 1);
  draw_syringe_media(h2oSource, media_amount+clean_offset, 1);
  syringe2waste(media_amount+clean_offset, 1);
  //Flush MUX/ DEMUX
  flush_halfmux(etohSource,5);
  flush_halfmux(bleachSource,5);
  flush_halfmux(h2oSource,10);
  flush_halfmux(airSource,10);
}


void dilute(int media_num, int media_amount, int prime_amount, int vial, float peri_time, float flush_time){
  int times_repeat = floor(media_amount/max_stepsize);
  int remainder = media_amount - (times_repeat * max_stepsize);

  draw_syringe_media(h2oSource, max_stepsize, 0);
  syringe2waste(max_stepsize, 0);
  efflux (vial,peri_time,flush_time);
  
  for (int i = 0; i < times_repeat; i++){
    draw_syringe_media(media_num, max_stepsize, prime_amount);
    syringe2vial_efflux(max_stepsize,prime_amount,vial);
  }
  if (remainder > 0){
    draw_syringe_media(media_num, remainder, prime_amount);
    syringe2vial_efflux(remainder,prime_amount,vial);  
  }

  flushToVial(airSource, vial, flush_time);
  efflux (vial,peri_time,flush_time);
}

//stdilutemix,0,5000,1,5000,0,0,3,5,0, !
void dilute_mix(int media1_num, int media1_amount, int media2_num, int media2_amount,int prime_amount, int vial, float peri_time, float flush_time){
  int media_amount = media1_amount + media2_amount;
  int times_repeat = floor(media_amount/max_stepsize);
  int remainder = media_amount - (times_repeat * max_stepsize);
  int media1_main = max_stepsize * ((float) media1_amount/ (float) media_amount);
  int media2_main = max_stepsize * ((float) media2_amount/ (float) media_amount);
  int media1_remainder = remainder * ((float) media1_amount/ (float) media_amount);
  int media2_remainder = remainder * ( (float) media2_amount/ (float) media_amount);
  draw_syringe_media(h2oSource, max_stepsize, 0);
  syringe2waste(max_stepsize, 0);
  efflux (vial,peri_time,flush_time);

  for (int i = 0; i < times_repeat; i++){
    draw_syringe_media(media1_num, media1_main, prime_amount);
    media2syringe(media2_num, media2_main);
    syringe2vial_efflux(max_stepsize,prime_amount,vial);
  }
  if (remainder > 0){
    draw_syringe_media(media1_num, media1_remainder, prime_amount);
    media2syringe(media2_num, media2_remainder);
    syringe2vial_efflux(remainder,prime_amount,vial);
  }
  flushToVial(airSource, vial, flush_time);
  efflux (vial,peri_time,flush_time);
}
//////////////////////////////END LEVEL 3 FLUID ABSTRACTIONS/////////////////////////////


//////////////////////////////LEVEL 2 FLUID ABSTRACTIONS/////////////////////////////

void draw_syringe_media(int media_num, int media_amount, int prime_amount){
  airPrime(prime_amount);
  media2syringe(media_num,media_amount);
}

void draw_sample(int media_num, int media_amount, int prime_amount, int vial, int efflux_offset){
  airPrime(prime_amount);
  media2syringe(media_num,media_amount);
  syringe2vial(media_amount,prime_amount,vial);
  flushToVial(airSource, vial, 3);
  airPrime(prime_amount);
  vial2syringe(media_amount, vial);
}

// Suck out of efflux and then clean with bleach, h20 and air.
void efflux (int vial, int peri_time, float flush_time){
  vial2waste (vial,peri_time);
  //flush_singlemuxdemux(bleachSource, vial, flush_time);
  flush_singlemuxdemux(h2oSource, vial, flush_time);
  flush_singlemuxdemux(airSource, vial, flush_time);
}

///////// END WRITING FUNCTIONS FOR LEVEL 2 FLUID ABSTRACTS /////////////////////


////////// Basic Fluidic Tasks (Level 1 Abstractions)////////////////////

void syringe_EtOH_clean(int media_amount){
  resetbinaryString();
  strcpy_P(buffer, (char*)pgm_read_word(&(syringe[0])));
  combineStrings(binaryString,buffer);
  strcpy_P(buffer, (char*)pgm_read_word(&(rev_filter[0])));
  combineStrings(binaryString,buffer);
  openSolenoids();
  moveStepper(media_amount);
  closeSolenoids();
}

void syringe2vial_efflux(int media_amount, int prime_amount, int vial){
  resetbinaryString();
  strcpy_P(buffer, (char*)pgm_read_word(&(syringe[0])));
  combineStrings(binaryString,buffer);
  strcpy_P(buffer, (char*)pgm_read_word(&(up_path[0])));
  combineStrings(binaryString,buffer);
  strcpy_P(buffer, (char*)pgm_read_word(&(demux[vial])));
  combineStrings(binaryString,buffer); //0-15
  strcpy_P(buffer, (char*)pgm_read_word(&(influx[0])));
  combineStrings(binaryString,buffer);
  strcpy_P(buffer, (char*)pgm_read_word(&(effluxgate[0])));
  combineStrings(binaryString,buffer);
  strcpy_P(buffer, (char*)pgm_read_word(&(mux[vial])));
  combineStrings(binaryString,buffer); //0-15
  strcpy_P(buffer, (char*)pgm_read_word(&(down_path[0])));
  combineStrings(binaryString,buffer);
  strcpy_P(buffer, (char*)pgm_read_word(&(wastegate[0])));
  combineStrings(binaryString,buffer);
  strcpy_P(buffer, (char*)pgm_read_word(&(peri1[0])));
  combineStrings(binaryString,buffer);
  openSolenoids();
  moveStepper(-media_amount-prime_amount);
  closeSolenoids();
}

void influx_clean (int vial, float peri_time){
  resetbinaryString();
  strcpy_P(buffer, (char*)pgm_read_word(&(influx[0])));
  combineStrings(binaryString,buffer);
  strcpy_P(buffer, (char*)pgm_read_word(&(bridge[0])));
  combineStrings(binaryString,buffer);
  strcpy_P(buffer, (char*)pgm_read_word(&(mux[vial])));
  combineStrings(binaryString,buffer); //0-15
  strcpy_P(buffer, (char*)pgm_read_word(&(down_path[0])));
  combineStrings(binaryString,buffer);
  strcpy_P(buffer, (char*)pgm_read_word(&(wastegate[0])));
  combineStrings(binaryString,buffer);
  strcpy_P(buffer, (char*)pgm_read_word(&(peri1[0])));
  combineStrings(binaryString,buffer);
  openSolenoids();
  delay(peri_time*1000);
  closeSolenoids();
}

void vial2waste (int vial, float peri_time) {
  resetbinaryString();
  strcpy_P(buffer, (char*)pgm_read_word(&(effluxgate[0])));
  combineStrings(binaryString,buffer);
  strcpy_P(buffer, (char*)pgm_read_word(&(mux[vial])));
  combineStrings(binaryString,buffer); //0-15
  strcpy_P(buffer, (char*)pgm_read_word(&(down_path[0])));
  combineStrings(binaryString,buffer);
  strcpy_P(buffer, (char*)pgm_read_word(&(wastegate[0])));
  combineStrings(binaryString,buffer);
  strcpy_P(buffer, (char*)pgm_read_word(&(peri1[0])));
  combineStrings(binaryString,buffer);
  openSolenoids();
  delay(peri_time*1000);
  closeSolenoids();
}

// Flush media through one lane of mux demux device into waste
void flush_halfmux(int media_num, float peri_time){
  //For 1st half of device
  resetbinaryString();
  strcpy_P(buffer, (char*)pgm_read_word(&(mediagate[0])));
  combineStrings(binaryString,buffer);
  strcpy_P(buffer, (char*)pgm_read_word(&(media[media_num])));
  combineStrings(binaryString,buffer); //0-15
  strcpy_P(buffer, (char*)pgm_read_word(&(up_path[0])));
  combineStrings(binaryString,buffer);
  for (int i = 0; i < 8; i = i + 1) {
    strcpy_P(buffer, (char*)pgm_read_word(&(demux[i])));
    combineStrings(binaryString,buffer); //0-15
    strcpy_P(buffer, (char*)pgm_read_word(&(bridge[0])));
    combineStrings(binaryString,buffer);
    strcpy_P(buffer, (char*)pgm_read_word(&(mux[i])));
    combineStrings(binaryString,buffer); //0-15
  }
  strcpy_P(buffer, (char*)pgm_read_word(&(down_path[0])));
  combineStrings(binaryString,buffer);
  strcpy_P(buffer, (char*)pgm_read_word(&(wastegate[0])));
  combineStrings(binaryString,buffer);
  strcpy_P(buffer, (char*)pgm_read_word(&(peri1[0])));
  combineStrings(binaryString,buffer);
  openSolenoids();
  delay(peri_time*1000);
  closeSolenoids();

  //for 2nd half of device
  resetbinaryString();
  strcpy_P(buffer, (char*)pgm_read_word(&(mediagate[0])));
  combineStrings(binaryString,buffer);
  strcpy_P(buffer, (char*)pgm_read_word(&(media[media_num])));
  combineStrings(binaryString,buffer); //0-15
  strcpy_P(buffer, (char*)pgm_read_word(&(up_path[0])));
  combineStrings(binaryString,buffer);
  for (int i = 8; i < 16; i = i + 1) {
    strcpy_P(buffer, (char*)pgm_read_word(&(demux[i])));
    combineStrings(binaryString,buffer); //0-15
    strcpy_P(buffer, (char*)pgm_read_word(&(bridge[0])));
    combineStrings(binaryString,buffer);
    strcpy_P(buffer, (char*)pgm_read_word(&(mux[i])));
    combineStrings(binaryString,buffer); //0-15
  }
  strcpy_P(buffer, (char*)pgm_read_word(&(down_path[0])));
  combineStrings(binaryString,buffer);
  strcpy_P(buffer, (char*)pgm_read_word(&(wastegate[0])));
  combineStrings(binaryString,buffer);
  strcpy_P(buffer, (char*)pgm_read_word(&(peri1[0])));
  combineStrings(binaryString,buffer);
  openSolenoids();
  delay(peri_time*1000);
  closeSolenoids();
}

// Flush media through one lane of mux demux device into waste
void flush_singlemuxdemux(int media_num, int vial, float peri_time){
  resetbinaryString();
  strcpy_P(buffer, (char*)pgm_read_word(&(mediagate[0])));
  combineStrings(binaryString,buffer);
  strcpy_P(buffer, (char*)pgm_read_word(&(media[media_num])));
  combineStrings(binaryString,buffer); //0-15
  strcpy_P(buffer, (char*)pgm_read_word(&(up_path[0])));
  combineStrings(binaryString,buffer);
  strcpy_P(buffer, (char*)pgm_read_word(&(demux[vial])));
  combineStrings(binaryString,buffer); //0-15
  strcpy_P(buffer, (char*)pgm_read_word(&(bridge[0])));
  combineStrings(binaryString,buffer);
  strcpy_P(buffer, (char*)pgm_read_word(&(mux[vial])));
  combineStrings(binaryString,buffer); //0-15
  strcpy_P(buffer, (char*)pgm_read_word(&(down_path[0])));
  combineStrings(binaryString,buffer);
  strcpy_P(buffer, (char*)pgm_read_word(&(wastegate[0])));
  combineStrings(binaryString,buffer);
  strcpy_P(buffer, (char*)pgm_read_word(&(peri1[0])));
  combineStrings(binaryString,buffer);
  openSolenoids();
  delay(peri_time*1000);
  closeSolenoids();
}

// Flush media through the upstream bypass valve to waste
void flush_upBypass(int media_num, float peri_time){
  resetbinaryString();
  strcpy_P(buffer, (char*)pgm_read_word(&(mediagate[0])));
  combineStrings(binaryString,buffer);
  strcpy_P(buffer, (char*)pgm_read_word(&(media[media_num])));
  combineStrings(binaryString,buffer); //0-15
  strcpy_P(buffer, (char*)pgm_read_word(&(up_bypass[0])));
  combineStrings(binaryString,buffer);
  strcpy_P(buffer, (char*)pgm_read_word(&(wastegate[0])));
  combineStrings(binaryString,buffer);
  strcpy_P(buffer, (char*)pgm_read_word(&(peri1[0])));
  combineStrings(binaryString,buffer);
  openSolenoids();
  delay(peri_time*1000);
  closeSolenoids();
}

// Flush media through the downstream bypass valve to waste
void flush_downBypass(int media_num,float peri_time){
  resetbinaryString();
  strcpy_P(buffer, (char*)pgm_read_word(&(mediagate[0])));
  combineStrings(binaryString,buffer);
  strcpy_P(buffer, (char*)pgm_read_word(&(media[media_num])));
  combineStrings(binaryString,buffer); //0-15
  strcpy_P(buffer, (char*)pgm_read_word(&(up_path[0])));
  combineStrings(binaryString,buffer);
  strcpy_P(buffer, (char*)pgm_read_word(&(down_bypass[0])));
  combineStrings(binaryString,buffer);
  strcpy_P(buffer, (char*)pgm_read_word(&(down_path[0])));
  combineStrings(binaryString,buffer);
  strcpy_P(buffer, (char*)pgm_read_word(&(wastegate[0])));
  combineStrings(binaryString,buffer);
  strcpy_P(buffer, (char*)pgm_read_word(&(peri1[0])));
  combineStrings(binaryString,buffer);
  openSolenoids();
  delay(peri_time*1000);
  closeSolenoids();
}

void flushToVial(int media_num, int vial, float flush_time){
  resetbinaryString();
  strcpy_P(buffer, (char*)pgm_read_word(&(media[media_num])));
  combineStrings(binaryString,buffer);
  strcpy_P(buffer, (char*)pgm_read_word(&(mediagate[0])));
  combineStrings(binaryString,buffer);
  strcpy_P(buffer, (char*)pgm_read_word(&(up_path[0])));
  combineStrings(binaryString,buffer);
  strcpy_P(buffer, (char*)pgm_read_word(&(demux[vial])));
  combineStrings(binaryString,buffer); //0-15
  strcpy_P(buffer, (char*)pgm_read_word(&(influx[0])));
  combineStrings(binaryString,buffer);
  strcpy_P(buffer, (char*)pgm_read_word(&(effluxgate[0])));
  combineStrings(binaryString,buffer);
  strcpy_P(buffer, (char*)pgm_read_word(&(mux[vial])));
  combineStrings(binaryString,buffer); //0-15
  strcpy_P(buffer, (char*)pgm_read_word(&(down_path[0])));
  combineStrings(binaryString,buffer);
  strcpy_P(buffer, (char*)pgm_read_word(&(wastegate[0])));
  combineStrings(binaryString,buffer);
  strcpy_P(buffer, (char*)pgm_read_word(&(peri1[0])));
  combineStrings(binaryString,buffer);
  openSolenoids();
  delay(flush_time*1000);
  closeSolenoids();
}

void flushToVial_noefflux(int media_num, int vial, float flush_time){
  resetbinaryString();
  strcpy_P(buffer, (char*)pgm_read_word(&(media[media_num])));
  combineStrings(binaryString,buffer);
  strcpy_P(buffer, (char*)pgm_read_word(&(mediagate[0])));
  combineStrings(binaryString,buffer);
  strcpy_P(buffer, (char*)pgm_read_word(&(up_path[0])));
  combineStrings(binaryString,buffer);
  strcpy_P(buffer, (char*)pgm_read_word(&(demux[vial])));
  combineStrings(binaryString,buffer); //0-15
  strcpy_P(buffer, (char*)pgm_read_word(&(influx[0])));
  combineStrings(binaryString,buffer);
  openSolenoids();
  delay(flush_time*1000);
  closeSolenoids();
}


// Dispense syringe straight to vial
void syringe2vial(int media_amount, int prime_amount, int vial){
  resetbinaryString();
  strcpy_P(buffer, (char*)pgm_read_word(&(syringe[0])));
  combineStrings(binaryString,buffer);
  strcpy_P(buffer, (char*)pgm_read_word(&(up_path[0])));
  combineStrings(binaryString,buffer);
  strcpy_P(buffer, (char*)pgm_read_word(&(demux[vial])));
  combineStrings(binaryString,buffer); //0-15
  strcpy_P(buffer, (char*)pgm_read_word(&(influx[0])));
  combineStrings(binaryString,buffer);
  openSolenoids();
  moveStepper(-media_amount-prime_amount);
  closeSolenoids();
}

//Dispense syringe content straight to waste
void syringe2waste(int media_amount, int prime_amount){
  resetbinaryString();
  strcpy_P(buffer, (char*)pgm_read_word(&(syringe[0])));
  combineStrings(binaryString,buffer);
  strcpy_P(buffer, (char*)pgm_read_word(&(up_path[0])));
  combineStrings(binaryString,buffer);
  strcpy_P(buffer, (char*)pgm_read_word(&(down_bypass[0])));
  combineStrings(binaryString,buffer);
  strcpy_P(buffer, (char*)pgm_read_word(&(for_filter[0])));
  combineStrings(binaryString,buffer);
  openSolenoids();
  moveStepper(-media_amount-prime_amount);
  closeSolenoids();
}

//Dispense syringe content to clear filter (flush out cells) into waste
void syringe2waste_clearfilter(int media_amount, int prime_amount){
  resetbinaryString();
  strcpy_P(buffer, (char*)pgm_read_word(&(syringe[0])));
  combineStrings(binaryString,buffer);
  strcpy_P(buffer, (char*)pgm_read_word(&(down_path[0])));
  combineStrings(binaryString,buffer);
  strcpy_P(buffer, (char*)pgm_read_word(&(rev_filter[0])));
  combineStrings(binaryString,buffer);
  strcpy_P(buffer, (char*)pgm_read_word(&(down_bypass[0])));
  combineStrings(binaryString,buffer);
  strcpy_P(buffer, (char*)pgm_read_word(&(wastegate[0])));
  combineStrings(binaryString,buffer);
  strcpy_P(buffer, (char*)pgm_read_word(&(peri1[0])));
  combineStrings(binaryString,buffer);
  openSolenoids();
  moveStepper(-media_amount-prime_amount);
  closeSolenoids();
}


// Draw from sample directly to filter
void vial2syringe(int media_amount,int vial){
  resetbinaryString();
  strcpy_P(buffer, (char*)pgm_read_word(&(syringe[0])));
  combineStrings(binaryString,buffer);
  strcpy_P(buffer, (char*)pgm_read_word(&(effluxgate[0])));
  combineStrings(binaryString,buffer);
  strcpy_P(buffer, (char*)pgm_read_word(&(down_bypass[0])));
  combineStrings(binaryString,buffer);
  strcpy_P(buffer, (char*)pgm_read_word(&(up_path[0])));
  combineStrings(binaryString,buffer);
  strcpy_P(buffer, (char*)pgm_read_word(&(mux[vial])));
  combineStrings(binaryString,buffer); //0-15
  openSolenoids();
  moveStepper(media_amount);
  closeSolenoids();
}

//Draw media to Syringe
void media2syringe(int media_num, int media_amount){
  resetbinaryString();
  strcpy_P(buffer, (char*)pgm_read_word(&(media[media_num])));
  combineStrings(binaryString,buffer);
  strcpy_P(buffer, (char*)pgm_read_word(&(syringe[0])));
  combineStrings(binaryString,buffer);
  strcpy_P(buffer, (char*)pgm_read_word(&(mediagate[0])));
  combineStrings(binaryString,buffer);
  openSolenoids();
  moveStepper(media_amount);
  closeSolenoids();
}

//Prime Syringe with Air
void airPrime(int prime_amount){
  if (prime_amount > 0){
    resetbinaryString();
    strcpy_P(buffer, (char*)pgm_read_word(&(media[airSource])));
    combineStrings(binaryString,buffer);
    strcpy_P(buffer, (char*)pgm_read_word(&(syringe[0])));
    combineStrings(binaryString,buffer);
    strcpy_P(buffer, (char*)pgm_read_word(&(mediagate[0])));
    combineStrings(binaryString,buffer);
    openSolenoids();
    moveStepper(prime_amount);
    closeSolenoids();
  }
}

void openSolenoids(){
  //Serial.println(binaryString);
  //Serial.println("Solenoids Open!");
  for (int i = 0; i <= binaryString.length(); i++){
    if (binaryString[num_pumps - i -1] == '0'){
       Tlc.set(i, 4095);
    } else if (binaryString[num_pumps - i -1] == '1'){
      Tlc.set(i,0);
    }
  }
  Tlc.update();
  //Serial.println(freeMemory());
  delay(1000);
}

void closeSolenoids(){
  //Serial.println("Solenoids Closing!");
  int closeFirst[] = {7,9,35,36};
  
  for (int i = 0; i < 3; i++){
    //Serial.println(closeFirst[i]);
    Tlc.set(closeFirst[i]-1,4095);
  }

  Tlc.update();
  delay(300);
  
  for (int i = 0; i < controlNum; i++){
    Tlc.set(i,4095);
  }
  Tlc.update();
  delay(300);
  
  }


void moveStepper(int steps){
  //Serial.println("Stepper Moving!");
  myStepper.step(steps);
  delay(5000);
  digitalWrite(2,LOW);digitalWrite(4,LOW);digitalWrite(6,LOW);digitalWrite(7,LOW); //turn off stepper
  //Serial.println("Stepper OFF!");
}
//////////////////////// END LEVEL 1 ABSTRACTIONS /////////////////////////////////


/// Function compares binaryString to another char array and saves into "binaryString" variable. Strings must be 48 char.
void combineStrings(String stringA, char *stringB) {
  char char_stringA[controlNum+1];
//  Serial.println("Start");
//  Serial.println(stringA);
//  Serial.println(stringB);
//  Serial.println("End");
  stringA.toCharArray(char_stringA, controlNum+1);
  binaryString = "";
  for (int j = 0; j <= controlNum; j++){
      if ((char_stringA[j] == '1') || (stringB[j] =='1')){
        binaryString += '1';
      } else{
        binaryString += '0';
      }
  }
}
void resetbinaryString(){
  binaryString = "";
  binaryString = "000000000000000000000000000000000000000000000000";
  memset(buffer, 0, 100);
}

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '!') {
      stringComplete = true;
    }
  }
}


void serialFlush(){
  while(Serial.available() > 0) {
    char t = Serial.read();
  }
}   
