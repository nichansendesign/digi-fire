/*
Candle Sensor Campfire - AnalogReadSerial
Nic Hansen - 20161107
with much [appreciated] copy&paste from others' examples.
...
Branched from "Record-Replay_20161106c_processing-4ch.ino"
*/

//............................................................
//=======================VARIABLES============================
//============================================================

// input variables
const int ldrs = 4;  // number of sensors attached, for calculating arrays.
const int ldr[ldrs] = {A0, A1, A2, A3};  // light dependent resistors (ldr).
const int switch1 = 47;  // switch (use internal pullup).
const int pots = 3;  // number of pots for controlling.
const int pot[pots] = {A4, A5, A6};  // pots for controlling.
byte inByte;  // for receiving serial markers.

// output & tuning variables
const int outputs = 4;  // number of output channels to processing. (was 15).
const int led[outputs] = {3, 5, 6, 9};  // LED outs with PWM for brightness control. (was {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 44, 45, 46})
const int smoothing = 2;  // number of rows for arrays (for old vals and for averaging). (was 4).
int inVal[ldrs][smoothing];  // array to store&communicate LED outputs.
const int delayHistory = 2;  // history cycles to skip for for each light delay level. (was 3).
const int history = (delayHistory * smoothing);  // nuber of rows for arrays, for output delay.
int storageVal[outputs][history];  // array to store&tune.
int outVal [outputs];  // array for communicating tuned outputs.
int inputLow = 0;  // for tuning input off (0-1023) (higher is more sensitive). (was 230).
int inputHigh = 1023;  // for tuning input full (0-1023) (lower is more sensitive). (was 350).
int storageLow = 0;  // for storing input off.
int storageHigh = 1023;  // for storing input full (cap at 512).
int outputLow = 0;  // for tuning output off (0-255). (was 120).
int outputHigh = 1023;  // for tuning full (0-255). (was 255).

// record/replay components
const int lengthSample = 50;  // number of samples to record for replay. (was 680).
int countSample = 0;  // keep track of current sample in record/replay.
int storeSample[ldrs][lengthSample];  // array to store recorded samples for replay.
bool recordingMode = true;  // true=recording; false=replay (ie.not recording).
bool recordingComplete = false;  // have we finished one cycle of recording?

// administrative variables
int ldrRead[ldrs];
bool switchOff1 = false;  // is the switch on? (for onboard pullup resistor, low(0) is on)
int delayAnalog = 2;  // millis between each analog read (multiplied by 5+!).
int delayPrint = 1000;  // millis between printing to serial.
int i = 0;  // for counting in arrays and stuff.
int j = 0;  // for counting in arrays and stuff.
int sum = 0;  // for averaging arrays and stuff.
unsigned long currentMillis = 0;
unsigned long lastMillis = 0;
unsigned long lastMillisPrint = 0;
unsigned long timeLoop = 0;

//............................................................
//............................................................
//=========================SETUP==============================
//============================================================
// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
    // establishContact with Processing over serial
  while (Serial.available() <= 0) {
    Serial.println("A");   // send a capital A
    delay(500);
  }  // close "while(Serial.available...)"
  // input pins:
  pinMode(switch1, INPUT_PULLUP);  // internal Pullup resistor, LOW (0) will indicate closed circuit.
  for (i = 0; i < ldrs; i++) {
    pinMode(ldr[i], INPUT);  // turn on input pin for each sensor.
  }  // close "for(i=0...)"
  // output pins:
  for (i = 0; i < outputs; i++) {
    pinMode(led[i], OUTPUT);  // turn on output pins.
  }  // close "for(i=0...)"
  // start clocks
  currentMillis = millis();
  lastMillis = millis();
  lastMillisPrint = millis();
}  // close setup
//............................................................
//............................................................
//=========================LOOP===============================
//============================================================
// the loop routine runs over and over again forever:
void loop() {
  timekeeping();  // update counters.
  readInputs();  // check controls.
  modeSelect();  // determine what mode has been selected.
  if (recordingMode == true) {
    readLDR();  // read LDR (light sensor) inputs and store them.
  }  // close "if(recordingMode...)"
  else {
    playLDR();  // retrieve LDR (light sensor) values that were stored:
  }  // close "else."
  records();  // update records with previous vals.
  equations();  // convert to meaningful units.
  // printReadings(); // serial print the converted input values.
  light(); // actuate lights.
  communicateSerial();  // send array to "lights" on computer in Processing.
}  // close loop
//............................................................
//............................................................
//=======================FUNCTIONS============================
//============================================================
//...................timekeeping function.....................
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void timekeeping() {
  lastMillis = currentMillis;
  currentMillis = millis();
  timeLoop = currentMillis - lastMillis;
  if (countSample >= lengthSample) {  // if we're at sample max.
    countSample = 0;  // ...reset counter.
    recordingComplete = true;  // mark the recording as complete.
  }  // close "if(countSample...)"
}  // close timekeeping function.
//............................................................
//.....................readInputs function....................
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void readInputs() {  // check controls.
  switchOff1 = digitalRead(switch1);  // check switch1.
  delay(delayAnalog);  //give a moment to stabilize voltage.
//  outputHigh = map(analogRead(pot[0]), 0, 1023, 0, 255);
//  delay(delayAnalog);  //give a moment to stabilize voltage.
//  inputLow = map(analogRead(pot[1]), 0, 1023, 30, 255);
//  delay(delayAnalog);  //give a moment to stabilize voltage.
//  inputHigh = map(analogRead(pot[2]), 0, 1023,   0, 255);
}  // close "readInputs" function.
//............................................................
//.....................modeSelect function....................
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void modeSelect() {  // determine what mode has been selected.
  if (switchOff1 == false) {  // if the switch is on...
    if (recordingComplete == true) {  // ...and if recording is full...
      recordingMode = false;  // ...stop recording (and just play back).
    }  // close "if(recordingComplete...)"
  }  // close "if(recordingMode...)"
  if (switchOff1 == true) {  // if switch is off...
    recordingMode = true;  // ...keep reading the sensors.
  }  // close "if(switchOff1=true...)"
}  // close "modeSelect" function.
//............................................................
//.....................records function.......................
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void records() {
  for (i = 0 ; i < outputs ; i++) {  // shift values down...
    for (j = 0 ; j < (smoothing - 1) ; j++) {  // ...for averaging (smoothing).
      inVal[i][(smoothing - 1 - j)] = inVal[i][(smoothing - 2 - j)];
    }  // close "for(j...)"
    for (j = 0 ; j < (history - 1) ; j++) {  // ...for delaying.
      storageVal[i][(history - 1 - j)] = storageVal[i][(history - 2 - j)];
    }  // close "for(j...)"
  }  // close "for(i...)"
}  // close records function.
//............................................................
//......................readLDRfunction.......................
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void readLDR() {  // read LDR (light sensor) inputs and store them:
  for (i = 0; i < ldrs; i++) {
    delay(delayAnalog);
    ldrRead[i] = analogRead(ldr[i]);
    storeSample[i][countSample] = ldrRead[i];
  }  // close "for(i=0...)"
  countSample++;
}  // close function readLDR
//............................................................
//......................playLDRfunction.......................
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void playLDR() {  // retrieve LDR (light sensor) values that were stored:
  for (i = 0; i < ldrs; i++) {
    delay(delayAnalog);
    ldrRead[i] = storeSample[i][countSample];
  }  // close "for(i=0...)"
  countSample++;
}  // close function readLDR
//............................................................
//...................equations function.......................
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void equations() {  // convert input values to meaningful units, add delays.
  // map inputs to storage range.
  for (i = 0; i < ldrs; i++) {
    inVal[i][0] = map(ldrRead[i], inputLow, inputHigh, storageLow, storageHigh);  // tune on-off for lights. (was 230, 350, 120, 255).
    inVal[i][0] = constrain(inVal[i][0], storageLow, storageHigh);  // constrain to valid range, just in case outlier.
  }  // close "for(int i...)"
  // do smoothing and sensitivity settings.
  for (i = 0; i < outputs; i++) {
    if (i < ldrs) {  // set first 4 as average of prev (smoothing).
      sum = 0;
      for (j = 1; j < smoothing; j++) {
        sum = sum + inVal[i][j];
      }  // close "for(j=0...)"
      storageVal[i][0] = sum / smoothing; // smooth by averaging recent readings.
    }  // close "if(i<ldrs...)"
    else {
      storageVal[i][0] = storageVal[i - ldrs][delayHistory - 1];
    }  // close else.
    outVal[i] = map(storageVal[i][0], storageLow, storageHigh, outputLow, outputHigh);  // tune according to settings.
  }  // close "for(i...)"
}  // end function equations.
//............................................................
//.................printReadings function.....................
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void printReadings() {  // print the converted input values.
  if (currentMillis > (lastMillisPrint + delayPrint)) {
    Serial.println("");
    Serial.println("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
    Serial.print("cycle time 0.");
    Serial.print(timeLoop);
    Serial.println(" seconds");
    Serial.println("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
    Serial.println("__inVals__");
    Serial.println("");
    for (i = 0; i < outputs; i++) {
      Serial.print("storageVal[");
      Serial.print(i);
      Serial.print("][0] = ");
      Serial.println(storageVal[i][0]);
      Serial.println("");
    }  // close "for(int i=0...)"
    Serial.println("");
    lastMillisPrint = currentMillis ;
  }  // end if(currentMillisPrint...).
}  // end function printReadings.
//............................................................
//......................light function........................
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void light() {  // actuate lighs according to [ldr] light sensors.
  for (i = 0; i < outputs; i++) {
    analogWrite(led[i], outVal[i]);  //
  }  // close "for(int i=0...)"
}  // end function lights.
//............................................................
//.............communicateSerial function.....................
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void communicateSerial() {
  // if we get a valid byte, read analog ins:
  if (Serial.available() > 0) {
    // get incoming byte:
    inByte = Serial.read();
    if (inByte == 'A') {
      // send sensor values:
      for ( i = 0; i < outputs; i++) {
        Serial.print(outVal[i]);
        Serial.print('\t');  // separator.
      }  // close "for(i=0...)
      Serial.println("");
      delay(1);  // short delay for stability.
    }  // close "if(inByte==A...)"
  }  // close "if(Serial.available...)"
}  // close communicateSerial function.
//............................................................
