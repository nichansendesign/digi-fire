/*
Candle Sensor Campfire - AnalogReadSerial
Nic Hansen - 20160314

branch from "Playback-15ch_20161204a_working-swell.ino," "Record-Replay_20161101c_working.ino" & "...1121d_speed.ino"
...just for playback
illuminates hardware LEDs.

make recording arrays with arduino "Serial-4ch_Processing_20161204a_working-recorder.ino"...
...going into processing "Record_Replay_multiply_4ch_20161115a.pde"
*/

//............................................................
//=======================VARIABLES============================
//============================================================

// input variables
const int sources = 4;  // number of sensors attached, for calculating arrays.
const int ldr[sources] = {A0, A1, A2, A3};  // light dependent resistors (ldr).
const int switch1 = 47;  // switch (use internal pullup).
const int pot1 = A7;  // for input speed.
int potPos1 = 0;  // for storing position.

// output & tuning variables
const int outputs = 15;  // number of output channels to processing. (was 15).
const int led[outputs] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 44, 45, 46};  // LED outs with PWM for brightness control. (was {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 44, 45, 46} ).
const byte smoothing = 4;  // [][][] number of rows for arrays (for old vals and for averaging). (2 is good; when allowing smoothingChoice, shoot higher for flex.)
byte smoothingChoice = smoothing;  // for adjusting smoothing in realtime, mapped within "smoothing" in readInputs() to not break inVal[][].
int inVal[smoothing][outputs];  // array to store&communicate LED outputs.
const int delayHistory = 4;  // [][][] history cycles to skip for for each light delay level. (4 is good)
const int history = (delayHistory * smoothing);  // nuber of rows for arrays, for output delay.
int storageVal[history][outputs];  // array to store&tune.
int outVal[outputs];  // array for communicating tuned outputs.
int inputLow = 30;  // [][][] for tuning input off (0-1023) (higher is more sensitive). (was 230).
int inputHigh = 150;  // [][][] for tuning input full (0-1023) (lower is more sensitive). (was 350).
int storageLow = 0;  // [][][] for storing input off.
int storageHigh = 1023;  // [][][] for storing input full (cap at 512).
int outputLow = 2;  // [][][] for tuning output off (0-255). (was 120).
int outputHigh = 255;  // [][][] for tuning full (0-255). (was 255).
int countSkip = 1;  // [][][] playback fastforward (must be more than 0).

// record/replay components
const int lengthSample = 89;  // number of samples to record for replay.
const int sampleDivs = 8;  // [][][] number of segments for reporting position within lengthSampe (one more than qty indicators).
const int splitSample = (lengthSample / sampleDivs);  // for signaling rough position.
const int reportingPins[(sampleDivs - 1)] = {14, 15, 16, 17, 18, 19, 20};  // for reporting position within lengthSample.
int countSample = 0;  // keep track of current sample in record/replay.
int storeSample[lengthSample][sources] = {
  {32, 32, 24, 50},
  {39, 59, 65, 89},
  {68, 108, 136, 122},
  {105, 135, 150, 128},
  {133, 128, 141, 109},
  {119, 109, 128, 96},
  {101, 105, 135, 98},
  {105, 105, 149, 104},
  {116, 121, 147, 114},
  {127, 149, 114, 140},
  {112, 139, 86, 118},
  {144, 147, 123, 117},
  {135, 128, 155, 122},
  {110, 118, 104, 115},
  {91, 119, 87, 119},
  {81, 100, 67, 99},
  {87, 112, 76, 112},
  {91, 114, 85, 119},
  {118, 131, 100, 124},
  {134, 149, 143, 150},
  {113, 111, 80, 89},
  {63, 53, 50, 60},
  {52, 45, 53, 61},
  {55, 57, 89, 79},
  {98, 101, 128, 96},
  {65, 60, 63, 73},
  {48, 44, 41, 61},
  {50, 60, 43, 59},
  {39, 39, 26, 41},
  {44, 62, 43, 74},
  {50, 63, 42, 66},
  {40, 49, 36, 66},
  {48, 79, 53, 85},
  {85, 116, 85, 88},
  {150, 112, 116, 78},
  {87, 89, 110, 89},
  {85, 107, 109, 114},
  {93, 117, 114, 114},
  {131, 120, 119, 106},
  {63, 58, 52, 51},
  {54, 46, 36, 41},
  {47, 62, 43, 71},
  {86, 111, 70, 87},
  {55, 52, 33, 47},
  {42, 41, 36, 55},
  {54, 75, 75, 91},
  {69, 95, 91, 101},
  {58, 63, 64, 72},
  {42, 40, 40, 57},
  {40, 44, 41, 54},
  {32, 32, 31, 44},
  {31, 30, 33, 48},
  {38, 57, 45, 68},
  {50, 64, 36, 55},
  {39, 49, 27, 46},
  {39, 50, 28, 48},
  {59, 85, 67, 87},
  {85, 117, 102, 122},
  {73, 98, 84, 96},
  {82, 86, 96, 76},
  {64, 47, 47, 32},
  {41, 30, 29, 24},
  {34, 28, 33, 43},
  {38, 37, 60, 56},
  {57, 59, 106, 68},
  {49, 43, 55, 38},
  {34, 30, 44, 36},
  {50, 48, 56, 37},
  {29, 21, 23, 16},
  {21, 14, 15, 11},
  {18, 12, 12, 13},
  {23, 32, 41, 40},
  {43, 56, 70, 56},
  {46, 47, 73, 72},
  {33, 32, 43, 65},
  {40, 56, 87, 81},
  {47, 61, 83, 84},
  {54, 75, 101, 87},
  {59, 77, 108, 84},
  {64, 78, 102, 76},
  {58, 78, 85, 77},
  {58, 76, 71, 68},
  {49, 65, 56, 61},
  {44, 65, 54, 70},
  {56, 85, 94, 81},
  {68, 83, 85, 68},
  {69, 71, 77, 58},
  {48, 53, 48, 44},
  {33, 33, 30, 30}
};  // array to store recorded samples for replay.
bool recordingMode = false;  // true=recording; false=replay (ie.not recording).
bool recordingComplete = false;  // have we finished one cycle of recording?

// administrative variables
int breathSeconds = 12;  // [][][] seconds for full breathing cycle.
int breathLevel = 50;  // for fading breathing, gets changed in breathing().
int breathDirection = -1;  // for monitoring direction of breathing, gets changed in breathing().
int breathIncrement = 1;  // steps for each loop, gets changed in breathing().
int hertz = 40;  // [][][] for speed of flashing. (delta/sleep = <4; theta/hypno = 4-7; alpha/calm; 8-12; beta/thinking = 13-38; gamma/mental = 40-100; SensoryMotorRhythm/catSeizure = 14)
int delayAdvance = ( 1000 / hertz );  // millis delay between oscillations to achieve desired hertz.
int ldrRead[sources];
bool switchOff1 = false;  // is the switch on? (for onboard pullup resistor, low(0) is on)
int delayAnalog = 1;  // millis between each analog read (multiplied by 5+!).
int delayPrint = 1000;  // millis between printing to serial.
int i = 0;  // for counting in arrays and stuff.
int j = 0;  // for counting in arrays and stuff.
int sum = 0;  // for averaging arrays and stuff.
unsigned long currentMillis = 0;
unsigned long lastMillis = 0;
unsigned long lastMillisBreath = 0;
unsigned long lastMillisAdvance = 0;
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
  // input pins:
  pinMode(switch1, INPUT_PULLUP);  // internal Pullup resistor, LOW (0) will indicate closed circuit.
  for (i = 0; i < sources; i++) {
    pinMode(ldr[i], INPUT);  // turn on input pin for each sensor.
  }  // close "for(i=0...)"
  // output pins:
  for (i = 0; i < outputs; i++) {
    pinMode(led[i], OUTPUT);  // turn on output pins.
  }  // close "for(i=0...)"
  for (i = 0; i < sampleDivs; i++) {
    pinMode(reportingPins[i], OUTPUT);  // turn on reportingPins.
  }  // close "for(i=0...)"
  pinMode(pot1, INPUT);  // turn on pot.
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
  //positionReport();  // report playback position within lengthSample.
  //modeSelect();  // determine what mode has been selected.
  if (recordingMode == true) {
    readSources();  // read LDR (light sensor) inputs and store them.
  }  // close "if(recordingMode...)"
  else {
    playSources();  // retrieve LDR (light sensor) values that were stored:
  }  // close "else."
  records();  // update records with previous vals.
  equations();  // convert to meaningful units.

  if (switchOff1 == false) {  // if switch is on...
    light(); // ...actuate lights normally...
  }  // close "if(switchOff1...)"
  else {  // otherwise...
    breathing();  // do breathing function.
  }  // close else.
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
  potPos1 = analogRead(pot1);  // check pot1.
  delay(1);
  if (potPos1 > 400) {  // 1f slow...
    smoothingChoice = constrain(map(potPos1, 400, 1023, smoothing, 2), 2, smoothing);  // set smooth by pot.
    delayAdvance = constrain(map(potPos1, 400, 1023, 0, 0), 0, 0);  // set delay by pot.
    countSkip = constrain(map(potPos1, 900, 1023, 1, 4), 1, 4);  // set fast-forward by pot.
    outputLow = constrain(map(potPos1, 400, 1023, 5, 1), 1, 5);  // ...set lows by pot.
    outputHigh = constrain(map(potPos1, 400, 1023, 20, 5), 5, 20);  // set highs by pot.
  }  // close "if(potPos1>...)"
  else {  // if fast...
    smoothingChoice = constrain(map(potPos1, 0, 400, 2, smoothing), 2, smoothing);  // set smooth by pot.
    delayAdvance = 0;  // no delay in this range.
    countSkip = constrain(map(potPos1, 0, 333, 2, 1), 1, 2);  // set fast-forward by pot.
    outputLow = constrain(map(potPos1, 0, 400, 30, 5), 5, 30);  // set lows by pot.
    outputHigh = constrain(map(potPos1, 200, 400, 255, 20), 20, 255);  // set highs by pot.
  }  // close else.
  Serial.println("");
  Serial.println("potPos1/delayAdvance/timeLoop");
  Serial.print(potPos1);
  Serial.print(" / ");
  Serial.print(delayAdvance);
  Serial.print(" / ");
  Serial.println(timeLoop);
  Serial.println("smoothingChoice/countSkip/outputLow/outputHigh");
  Serial.print(smoothingChoice);
  Serial.print(" / ");
  Serial.print(countSkip);
  Serial.print(" / ");
  Serial.print(outputLow);
  Serial.print(" / ");
  Serial.println(outputHigh);
  Serial.print("countSample: ");
  Serial.println(countSample);
  Serial.println("");
}  // close "readInputs" function.
//............................................................
//.................positionReport function....................
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void positionReport() {  // report playback position within lengthSample.
  if (countSample <= (splitSample * 1)) {
    digitalWrite(reportingPins[0], HIGH);  // turn on pin "1."
    digitalWrite(reportingPins[1], LOW);  // turn off.
    digitalWrite(reportingPins[2], LOW);  // turn off.
    digitalWrite(reportingPins[3], LOW);  // turn off.
    digitalWrite(reportingPins[4], LOW);  // turn off.
    digitalWrite(reportingPins[5], LOW);  // turn off.
    digitalWrite(reportingPins[6], LOW);  // turn off.
  }  // close "if(...splitSample*1))"
  if (countSample > (splitSample * 1) && countSample <= (splitSample * 2)) {
    digitalWrite(reportingPins[0], HIGH);  // turn on.
    digitalWrite(reportingPins[1], HIGH);  // turn on.
  }  // close "if(...splitSample*2))"
  if (countSample > (splitSample * 2) && countSample <= (splitSample * 3)) {
    digitalWrite(reportingPins[1], HIGH);  // turn on.
    digitalWrite(reportingPins[2], HIGH);  // turn on.
    digitalWrite(reportingPins[0], LOW);  // turn off.
  }  // close "if(...splitSample*3))"
  if (countSample > (splitSample * 3) && countSample <= (splitSample * 4)) {
    digitalWrite(reportingPins[2], HIGH);  // turn on.
    digitalWrite(reportingPins[3], HIGH);  // turn on.
    digitalWrite(reportingPins[1], LOW);  // turn off.
  }  // close "if(...splitSample*4))"
  if (countSample > (splitSample * 4) && countSample <= (splitSample * 5)) {
    digitalWrite(reportingPins[3], HIGH);  // turn on.
    digitalWrite(reportingPins[4], HIGH);  // turn on.
    digitalWrite(reportingPins[2], LOW);  // turn off.
  }  // close "if(...splitSample*5))"
  if (countSample > (splitSample * 5) && countSample <= (splitSample * 6)) {
    digitalWrite(reportingPins[4], HIGH);  // turn on.
    digitalWrite(reportingPins[5], HIGH);  // turn on.
    digitalWrite(reportingPins[3], LOW);  // turn off.
  }  // close "if(...splitSample*6))"
  if (countSample > (splitSample * 6) && countSample <= (splitSample * 7)) {
    digitalWrite(reportingPins[5], HIGH);  // turn on.
    digitalWrite(reportingPins[6], HIGH);  // turn on.
    digitalWrite(reportingPins[4], LOW);  // turn off.
  }  // close "if(...splitSample*7))"
  if (countSample > (splitSample * 7) && countSample <= lengthSample) {
    digitalWrite(reportingPins[6], HIGH);  // turn on.
    digitalWrite(reportingPins[5], LOW);  // turn off.
  }  // close "if(...<= lengthSample))"
}  // close positionReport function.
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
//......................readSourcesfunction.......................
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void readSources() {  // read LDR (light sensor) inputs and store them:
  for (i = 0; i < sources; i++) {
    delay(delayAnalog);
    ldrRead[i] = analogRead(ldr[i]);  // read each sensor.
    storeSample[countSample][i] = ldrRead[i];  // store it just in case.
  }  // close "for(i=0...)"
  //Serial.println("readSources");
}  // close function readSources
//............................................................
//......................playSourcesfunction.......................
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void playSources() {  // retrieve LDR (light sensor) values that were stored:
  if (currentMillis > (lastMillisAdvance + delayAdvance)) {  // if timer for advancing has run out...
    for (i = 0; i < sources; i++) {  // ...go through all sources...
      ldrRead[i] = storeSample[countSample][i];  // ...and read new samples.
      //    Serial.println(ldrRead[i]);
    }  // close "for(i=0...)"
    lastMillisAdvance = currentMillis;  // reset the timer for advancing.
    countSample += countSkip;  // advance counter.
  }  // close "if(currentMillis>...)"
  //Serial.println("LDRplay");
}  // close function readSources
//............................................................
//.....................records function.......................
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void records() {
  for (j = (smoothing - 1) ; j > 0 ; j--) {  // for each row...
    for (i = 0 ; i < outputs ; i++) {  // ...go column by column...
      inVal[j][i] = inVal[(j - 1)][i];  // ...& shift vaues down (starting from bottom). for averaging/smoothing.
    }  // close "for(i...)"
  }  // close "for(j...)"
  for (j = (history - 1) ; j > 0 ; j--) {  // for each row...
    for (i = 0 ; i < outputs ; i++) {  // ...go column by column...
      storageVal[j][i] = storageVal[j - 1][i];  // ...& shift values down (starting from bottom). for delay effects.
    }  // close "for(i...)"
  }  // close "for(j...)"
  //Serial.println("records");
}  // close records function.
//............................................................
//...................equations function....,..................
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void equations() {  // convert input values to meaningful units, add delays.
  // map inputs to storage range.
  for (i = 0; i < sources; i++) {
    inVal[0][i] = constrain(map(ldrRead[i], inputLow, inputHigh, storageLow, storageHigh), storageLow, storageHigh);  // tune on-off for lights. (was 230, 350, 120, 255).
  }  // close "for(int i...)"
  // do smoothing and sensitivity settings.
  for (i = 0; i < outputs; i++) {
    if (i < sources) {  // set first 4 as average of prev (smoothing).
      sum = 0;
      for (j = 0; j < smoothingChoice; j++) {
        sum += inVal[j][i];
      }  // close "for(j=0...)"
      storageVal[0][i] = sum / smoothingChoice; // smooth by averaging recent readings.
    }  // close "if(i<sources...)"
    else {  // for any extra/multiplied channels...
      storageVal[0][i] = storageVal[delayHistory - 1][i - sources];  // ...look at history of prev channels.
    }  // close else.
    outVal[i] = map(storageVal[0][i], storageLow, storageHigh, outputLow, outputHigh);  // tune according to settings.
  }  // close "for(i...)"
  //Serial.println("equations");
}  // end equations function.
//............................................................
//....................breathing function......................
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void breathing() {  // do breathing when "asleep"
  for (i = 0; i < outputs; i++) {  // ... go through outputs...
    analogWrite(led[i], breathLevel);  // ...to update all the levels.
  }  // close "for(int i=0...)"
  breathIncrement = ( 88 / (((breathSeconds / 2) * 1000) / timeLoop )) * breathDirection;  //  (hi-low increments) / (number of timeLoops in half-breath time).
  breathLevel += breathIncrement;  // add increment (in whichever direction).
  if ( breathLevel <= (abs(breathIncrement) * 2) || breathLevel >= (88 - (abs(breathIncrement) * 2)) )  {  // if reaching the level limit...
    breathDirection = -breathDirection;  // switch direction.
  }  // close "if(breathLevel...)"
}  // end breathing function.
//............................................................
//......................light function........................
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void light() {  // actuate lighs.
  for (i = 0; i < outputs; i++) {
    analogWrite(led[i], outVal[i]);  // turn on the pin.
    Serial.println(outVal[i]);  // say the intensity.
  }  // close "for(int i=0...)"
  //  Serial.println(countSample);
  //  Serial.println("light");
}  // end function lights.
//............................................................

