/* 
 branch 20161113c for animated visual display by
 receiving 4 channels from arduino sensors (maybe more) and 
 storing them to .txt file with ability for
 playback without arduino link, plus some
 signal smoothing and channel multiplication (by delay).
 
 somehow works with arduino "Serial-4ch_Processing_20161204a_working-recorder.ino"
 */

//............................................................
//=======================VARIABLES============================
//============================================================
//.............serial communications with arduino.............
PrintWriter recFile;  // get ready to record a file.
import processing.serial.*;  // import the serial library.
Serial myPort;  // set the serial port.
int inByte; // for receiving over serial.
int arduinoChannels = 4;  // how many channels are incoming from arduino.
int[] inputVals = new int[arduinoChannels];  // where we'll store what we receive.
boolean firstContactDone = false;   // whether we've heard from the microcontroller.
boolean waitForSerial = true;  // is serial waiting for next reading.

//...................control variables........................
boolean recordInput = true;   // [][][] to remember if we're recording inputs (in "serialEvent()").
boolean recordOutput = false;  // [][][] to remember if we're recording outputs (in "records()").
//....................tuning variables........................
int inputMultiplier = 1;      // [][][] how many times to multiply the inputs.
int smoothing = 3;            // [][][] number of rows for arrays (for smoothing and delay). (was 4).
int delayHistory = 3;         // [][][] history cycles to skip for for each light delay level. (was 3).
int inputTuneLow = 0;         // [][][] for tuning input off (0-255).
int inputTuneHigh = 1023;      // [][][] for tuning input full (0-255).
int storageTuneLow = 0;       // [][][] for storing input off (0-1023).
int storageTuneHigh = 1023;   // [][][] for storing input full (0-1023).
int outputTuneLow = 0;        // [][][] for tuning output off (0-1023).
int outputTuneHigh = 1023;     // [][][] for tuning full (0-1023).
//....................drawing variables.......................
int bgColor = 128;   // [][][] background color.
int divisionsX = 9;  // [][][] number of elements per X axis.
int size1;           // for sizing the elements (in setup() according to draw size).

//...............vars and arrays for equations................
int channelCount = 0;  // for keeping track which channel we're on.
int outputChannels = (arduinoChannels * inputMultiplier);  // number of outputs channels (columns for arrays).
int history = (inputMultiplier * smoothing * delayHistory);  // nuber of rows for arrays, for output delay.
int[][] storageVals = new int[outputChannels][history];  // array for doing signal tuning.
int[] outputVals = new int[outputChannels];  // array for communicating tuned arduinoChannels.
//............................................................
//............................................................
//=========================SETUP==============================
//============================================================
void setup() {
  recFile = createWriter("recFile_.txt");  // make/link-to a file for when we want to write.
  size(600, 600);  // screen size. (was 1400x850).
  background(bgColor);  // background color (0=black).
  size1 = (height/divisionsX);  // how many divisionsX/elements.
  // Print a list of the serial ports for debugging purposes
  // if using Processing 2.1 or later, use Serial.printArray()
  //  println(Serial.list());  // only use this for checking which port for "myport."
  // I know that the first port in the serial list on my mac
  // is always my  FTDI adaptor, so I open Serial.list()[0].
  // On Windows machines, this generally opens COM1.
  // Open whatever port is the one you're using.
  String portName = Serial.list()[2];  // for my computer, port 2 works.
  myPort = new Serial(this, portName, 9600);
  myPort.bufferUntil('\n');
}  // close setup.
//............................................................
//............................................................
//========================draw LOOP===========================
//============================================================
void draw() {
  if (waitForSerial == false) {  // if the next reading is here...
    storeInputs();  // store in an array for processing, map to nominal range.
    effects();  // multiply, delay, & tune signals for smoothing and effects.
    records();  // update storage locations before next cycle.
  }  // close "if(newReadings...)"

  background(bgColor);                                                           // clear the screen.
  channelCount = 0;                                                              // reset channel counter.
  for (int y = ( size1 / 2 ); y < width; y += size1) {                           // go through all the colums...
    for (int x = ( size1 / 2 ); x < width; x += size1) {                         // ...and all the rows...
      if (channelCount > (outputChannels - 1)) {                                 // ...by cycling through all the output channels.
        channelCount = 0;
      }  // close "if(i>(outputChannels...)"
      noStroke();                                                                // choose stroke.
      fill(50);   // choose fill color.
      rectMode(CENTER);                                                          // reference rectangles to their centers.
      rect(x, y, (size1*(inputVals[channelCount]/30)), (size1*(inputVals[channelCount]/outputTuneHigh)));        // draw the rectange.
      channelCount++;                                                            // advance the outputChannel counter.
    }  // close "for(x...)"
  }  // close "for(y...)"
}  // close draw loop
//............................................................
//............................................................
//====================serialEvent LOOP========================
//============================================================
void serialEvent(Serial myPort) {
  while (myPort.available () > 0) {
    if (firstContactDone == false) {
      delay(100);  // wait for reply.
      inByte = myPort.read();  // read a byte from the serial port:
      // if this is the first byte received, and it's an A,
      // clear the serial buffer and note that you've
      // had first contact from the microcontroller.
      // Otherwise, add the incoming byte to the array:
      if (inByte == 'A') {
        myPort.clear();          // clear the serial port buffer
        firstContactDone = true;     // you've had first contact from the microcontroller
        myPort.write('A');       // ask for more
        delay(1000);  // wait for reply.
        myPort.clear();          // clear the serial port buffer
      }  // close "if(inByte...)"
    }  // close "if(firstContactDone...)"
    else {
      waitForSerial = true;  // let other functions know that next reading hasn't come.
      myPort.write('A');       // ask for more
      myPort.bufferUntil('\n');
      String inBuffer = myPort.readStringUntil('\n');  // read the response.
      if (inBuffer != null) {  // if we've received something...
        String[] serialStringTemp = splitTokens(inBuffer, "\t");  // store, separated by tab(?).
        if (recordInput == true) {  // if we're supposed to be recording...
          recFile.print("{"); // ...add entry to file.
        }  // close "if(recordInput == true)"
        for (int i = 0; i < arduinoChannels; i++) {
          serialStringTemp[i] = trim(serialStringTemp[i]);  // ...then trim it...
          float serialParsed = Integer.parseInt(serialStringTemp[i]);  // ... convert to integer. (was "float...").
          int intermediateInt = int(serialParsed);  // convert from a float to an int (for saving space?).
          inputVals[i] = intermediateInt;  // save to outputVals array for output.
          storageVals[i][0] = inputVals[i];
          if (recordInput == true) {  // if we're supposed to be recording...
            if (i < (arduinoChannels - 1)) {  // if this isn't last number of line...
              recFile.print(inputVals[i] + ", "); // ...add entry to file (with comma).
            }  // close "if(i<arduinoChannels-1...)"
            else {  // if it's the last number of line...
              recFile.print(inputVals[i] + "},"); // ...add entry to file (with bracket). (for reading array).
            }  
          }  // close "if(recordInput == true)"
        }  // close "for(i...)"
        waitForSerial = false;  // ...signal to other functions
      }  // close "if(inBuffer...)"
      else {  // ... if we didn't received anything...
        waitForSerial = true;  // ...signal to other functions.
      }  // close "else..."
      if (recordInput == true) {  // if we're supposed to be recording...
        recFile.println(); // ...add line (close out this sample).
      }  // close "if(recordInput == true)"
      myPort.clear();          // clear the serial port buffer
    }  // close else.
  }  // close "while(myPort.available...)"
}  // close serialEvent.
//............................................................
//............................................................
//=====================keyPressed LOOP========================
//============================================================
void keyPressed() {
  if (key == 'r') {  // if someone presses "r" (ie. "record")...
    recordInput = true;  // ...activate record mode.
  }  // close "if(key=r...)"
  if (key == 'p') {  // if someone presses "p" (ie. "playback")...
    recordInput = false;  // ...de-activate record mode.
  }  // close "if(key=p...)"
  if (key == 's') {  // if someone presses "s" (ie. "save")...
    recFile.flush(); 
    recFile.close(); // ...write file
  }  // close "if(key=r...)"
}  // close keyPressed.
//............................................................
//............................................................
//...................storeInputs function.......................
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void storeInputs() {  // store in an array for processing, map to nominal range.
  // map inputs to storage range.
  for (int i = 0; i < arduinoChannels; i++) {  // just for the inputs from arduino...
    storageVals[i][0] = inputVals[i];
    //    storageVals[i][0] = constrain(inputVals[i], storageTuneLow, storageTuneHigh);  // save to storageVals array for tuning.
  }  // close "for(int i...)"
}  // end function storeInputs.
//............................................................
//.....................effects function.......................
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void effects() {  // multiply, delay, & tune signals for smoothing and effects.
  for (int i = 0; i < outputChannels; i++) {  // for however many outputs we have... 
    if (i < arduinoChannels) {  // ...set first 4 as average of prev (smoothing).
      int sum = 0;  // starting from zero...
      for (int j = 1; j < smoothing; j++) {  // ...look at previous values down the chart...
        sum += storageVals[i][j];  // ...and tally them up...
      }  // close "for(j=0...)"
      storageVals[i][0] = sum / smoothing; // ...then store the average.
    }  // close "if(i<arduinoChannels...)"
    else {
      storageVals[i][0] = storageVals[i - arduinoChannels][delayHistory - 1];  // for any multiplied channels, use history from an input.
    }  // close "else..."
    float intermediateFloat = map(storageVals[i][0], storageTuneLow, storageTuneHigh, outputTuneLow, outputTuneHigh);  // tune output values.
    outputVals[i] = int(intermediateFloat);  // convert from a float to an int (for saving space?) and save to outputVals array for output.
  }  // close "for(i...)"
}  // end function effects.
//............................................................
//.....................records function.......................
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void records() {  // update storage locations before next cycle.
  for (int i = 0; i < outputChannels; i++) {  // for each column...
    if (recordOutput == true) {  // if we're supposed to be recording...
      recFile.print(storageVals[i][0] + "*" + '\t');  // ...add entry to file.
    }  // close "if(recordInput == true)"
    for (int j = 0; j < (history - 1); j++) {  // ...go down the rows...
      storageVals[i][(history - 1 - j)] = storageVals[i][(history - 2 - j)];  // ...shift values down a row to open top row.
    }  // close "for(j...)"
  }  // close "for(i...)"
  if (recordOutput == true) {  // if we're supposed to be recording...
    recFile.println(); // ...add line (close out this sample).
  }  // close "if(recordInput == true)"
}  // close records function.
//............................................................