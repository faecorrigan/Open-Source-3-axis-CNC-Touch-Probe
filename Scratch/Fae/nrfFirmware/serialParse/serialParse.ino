#include <Arduino.h>
#include <bluefruit.h>

// Example 5 - Receive with start- and end-markers combined with parsing

const byte blueart_numChars = 32;
char blueart_receivedChars[blueart_numChars];
char blueart_tempChars[blueart_numChars];        // temporary array for use when parsing

      // variables to hold the parsed data
char blueart_returnCommand[blueart_numChars] = {0};
char blueart_returnVariable[blueart_numChars] = {0};
char blueart_returnValue[blueart_numChars] = {0};

boolean blueart_newData = false;

//============

void setup() {
    Serial.begin(115200);
    while(!Serial) delay(10); //remove for battery only operation
    Serial.println("This demo expects 3 pieces of data - text, an integer and a floating point value");
    Serial.println("Enter data in this style <HelloWorld, 12, 24.7>  ");
    Serial.println();
}

//============

void loop() {
    recvWithStartEndMarkers();
    if (blueart_newData == true) {
        strcpy(blueart_tempChars, blueart_receivedChars);
            // this temporary copy is necessary to protect the original data
            //   because strtok() used in blueart_parseData() replaces the commas with \0
        blueart_parseData();
        blueart_parseInput();
        blueart_newData = false;
    }
}

//============

void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;

  while (Serial.available() > 0 && blueart_newData == false) {
    rc = Serial.read();

    
    if (rc != endMarker) {
      blueart_receivedChars[ndx] = rc;
      ndx++;
      if (ndx >= blueart_numChars) {
          ndx = blueart_numChars - 1;
      }
    }
    else {
      blueart_receivedChars[ndx] = '\0'; // terminate the string
      recvInProgress = false;
      ndx = 0;
      blueart_newData = true;
    }
      

      
  }
}

//============

void blueart_parseData() {      // split the data into its parts

    char * strtokIndx; // this is used by strtok() as an index

    strtokIndx = strtok(blueart_tempChars," ");      // get the first part - the string
    strcpy(blueart_returnCommand, strtokIndx); // copy it to blueart_returnCommand

    strtokIndx = strtok(NULL," ");      // get the first part - the string
    strcpy(blueart_returnVariable, strtokIndx); // copy it to blueart_returnVariable

    strtokIndx = strtok(NULL," ");      // get the first part - the string
    strcpy(blueart_returnValue, strtokIndx); // copy it to blueart_returnValue
 
    //strtokIndx = strtok(NULL, " "); // this continues where the previous call left off
    //integerFromPC = atoi(strtokIndx);     // convert this part to an integer

    //strtokIndx = strtok(NULL, " ");
    //floatFromPC = atof(strtokIndx);     // convert this part to a float

}

//============

void blueart_print()
{

}

void blueart_get()
{
  if (strcmp(blueart_returnVariable, "uninitialized") == 0)
  {
    Serial.println("variable: uninitialized = ");
    
  } else
  if (strcmp(blueart_returnVariable, "polling_rate") == 0)
  {
    Serial.println("variable: polling_rate = ");
    
  } else
  if (strcmp(blueart_returnVariable, "debounceDelay") == 0)
  {
    Serial.println("variable: debounceDelay = ");
    
  } else
  if (strcmp(blueart_returnVariable, "buttonLongPressLength") == 0)
  {
    Serial.println("variable: buttonLongPressLength = ");
    
  } else
  if (strcmp(blueart_returnVariable, "buttonDoublePressTime") == 0)
  {
    Serial.println("variable: buttonDoublePressTime = ");
    
  } else
  if (strcmp(blueart_returnVariable, "buttonHeartbeatUnpressed") == 0)
  {
    Serial.println("variable: buttonHeartbeatUnpressed = ");
    
  } else
  if (strcmp(blueart_returnVariable, "pairingDelay") == 0)
  {
    Serial.println("variable: pairingDelay = ");
    
  } else
  if (strcmp(blueart_returnVariable, "laserDelay") == 0)
  {
    Serial.println("variable: laserDelay = ");
    
  } else
  if (strcmp(blueart_returnVariable, "idle_Delay") == 0)
  {
    Serial.println("variable: idle_Delay = ");
    
  } else
  if (strcmp(blueart_returnVariable, "sleeping_delay") == 0)
  {
    Serial.println("variable: sleeping_delay = ");
    
  } else
  if (strcmp(blueart_returnVariable, "idle_heartbeat_delay") == 0)
  {
    Serial.println("variable: idle_heartbeat_delay = ");
    
  } else
  if (strcmp(blueart_returnVariable, "pan") == 0)
  {
    Serial.println("variable: pan = ");
    /
  } else
  if (strcmp(blueart_returnVariable, "destination") == 0)
  {
    Serial.println("variable: destination = ");
  } else
  if (strcmp(blueart_returnVariable, "source") == 0)
  {
    Serial.println("variable: source = ");
  } else
  if (strcmp(blueart_returnVariable, "ack_interval") == 0)
  {
    Serial.println("variable: ack_interval = ");
    
  } else
  if (strcmp(blueart_returnVariable, "channel") == 0)
  {
    Serial.println("variable: channel = ");
  } else
  if (strcmp(blueart_returnVariable, "channel_rx") == 0)
  {
    Serial.println("variable: channel_rx = ");
  } else
  {
    Serial.println("error: variable does not exist");
  }
}

void blueart_set()
{
  if (strcmp(blueart_returnVariable, "uninitialized") == 0)
  {
    Serial.println("variable: uninitialized = ");
    //set variable
  } else
  if (strcmp(blueart_returnVariable, "polling_rate") == 0)
  {
    Serial.println("variable: polling_rate = ");
    //set variable
  } else
  if (strcmp(blueart_returnVariable, "debounceDelay") == 0)
  {
    Serial.println("variable: debounceDelay = ");
    //set variable
  } else
  if (strcmp(blueart_returnVariable, "buttonLongPressLength") == 0)
  {
    Serial.println("variable: buttonLongPressLength = ");
    //set variable
  } else
  if (strcmp(blueart_returnVariable, "buttonDoublePressTime") == 0)
  {
    Serial.println("variable: buttonDoublePressTime = ");
    //set variable
  } else
  if (strcmp(blueart_returnVariable, "buttonHeartbeatUnpressed") == 0)
  {
    Serial.println("variable: buttonHeartbeatUnpressed = ");
    //set variable
  } else
  if (strcmp(blueart_returnVariable, "pairingDelay") == 0)
  {
    Serial.println("variable: pairingDelay = ");
    //set variable
  } else
  if (strcmp(blueart_returnVariable, "laserDelay") == 0)
  {
    Serial.println("variable: laserDelay = ");
    //set variable
  } else
  if (strcmp(blueart_returnVariable, "idle_Delay") == 0)
  {
    Serial.println("variable: idle_Delay = ");
    //set variable
  } else
  if (strcmp(blueart_returnVariable, "sleeping_delay") == 0)
  {
    Serial.println("variable: sleeping_delay = ");
    //set variable
  } else
  if (strcmp(blueart_returnVariable, "idle_heartbeat_delay") == 0)
  {
    Serial.println("variable: idle_heartbeat_delay = ");
    //set variable
  } else
  if (strcmp(blueart_returnVariable, "pan") == 0)
  {
    Serial.println("variable: pan = ");
    //set variable
  } else
  if (strcmp(blueart_returnVariable, "destination") == 0)
  {
    Serial.println("variable: destination = ");
    //set variable
  } else
  if (strcmp(blueart_returnVariable, "source") == 0)
  {
    Serial.println("variable: source = ");
    //set variable
  } else
  if (strcmp(blueart_returnVariable, "ack_interval") == 0)
  {
    Serial.println("variable: ack_interval = ");
    //set variable
  } else
  if (strcmp(blueart_returnVariable, "channel") == 0)
  {
    Serial.println("variable: channel = ");
    //set variable
  } else
  if (strcmp(blueart_returnVariable, "channel_rx") == 0)
  {
    Serial.println("variable: channel_rx = ");
    //set variable
  } else
  {
    Serial.print("error: variable ");
    Serial.print(blueart_returnVariable);
    Serial.println( "does not exist");
  }
}

void blueart_parseInput() {
  if (strcmp(blueart_returnCommand, "exit") == 0)
  {
    Serial.println("recieved command: exit");
    //shut down blueart 
    //switch to probe mode
  } else
  if (strcmp(blueart_returnCommand, "get") == 0)
  {
    blueart_get();
  } else
  if (strcmp(blueart_returnCommand, "set") == 0)
  {
    Serial.println("recieved command: set");
    blueart_set();
  } else
  if (strcmp(blueart_returnCommand, "read") == 0)
  {
    Serial.println("recieved command: read");
  } else
  if (strcmp(blueart_returnCommand, "save") == 0)
  {
    Serial.println("recieved command: save");
  } else
  if (strcmp(blueart_returnCommand, "test") == 0)
  {
    Serial.println("recieved command: test");
  } else
  {
    Serial.println("error: improper command");
  }

}