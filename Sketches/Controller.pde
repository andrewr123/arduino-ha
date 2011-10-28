

// Home automation controller

#include <SPI.h>
#include <SdFat.h>
#include <SdFatUtil.h>
#include <Ethernet.h>
#include <Time.h>
#include <Udp.h>
#include <OneWire.h>
#include <DallasTemperature.h>

/*********** DATA LOGGING/DEBUG *************/

#define DEBUGON 01
#define DEBUGHEARTBEAT 1
#define DEBUGREADINGS 1
#define DEBUGEVAL 1
#define DEBUGACTIONS 1
//#if DEBUGON
  byte logIP[4];                           // Populated on first read - client IP
  char logBuffer[UDP_TX_PACKET_MAX_SIZE];  // 64 bytes
  const byte loggerCheckFreq = 10;            // How often to check if logger is running (secs * heartBeatSecs)  
  boolean debugH = false;
  boolean debugR = false;
  boolean debugE = false;
  boolean debugA = false;
//#endif

/************ ETHERNET STUFF ************/
const byte maxArduinos = 8;
byte mac[maxArduinos][6]; // = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
byte ip[maxArduinos][4]; // = { 192, 168, 7, 177 };
byte arduinoMe;
Server WebServer(80);               // Socket for HTTP comms
UdpClass UdpNTP;                    // Socket for getting the time
UdpClass UdpLog;                    // Socket for sending error messages, receiving commands and sending log messages
UdpClass UdpArd;                    // Socket for comms with other arduinos
unsigned int UdpNTPPort = 8888;      // port for NTP time comms
unsigned int UdpLogPort = 8889;      // port for logging comms
unsigned int UdpArdPort = 8890;      
#define const_HTTP_BUFSIZ 100
#define const_Token_Bufsiz 20

/************ SDCARD STUFF ************/
Sd2Card card;
SdVolume volume;
SdFile root;

// store error strings in flash to save RAM
#define error(s) error_P(PSTR(s))
#define const_SDCard_BUFSIZ 100

void error_P(const char* str) {
  PgmPrint("error: ");
  SerialPrintln_P(str);
  if (card.errorCode()) {
    PgmPrint("SD error: ");
    Serial.print(card.errorCode(), HEX);
    Serial.print(',');
    Serial.println(card.errorData(), HEX);
  }
  while(1);
}

/************ NTP TIME STUFF ************/

#define CURRENT_YEAR 2011            // used for sense test
#define EXPIRY_YEAR 2050              // Used for sense test
#define const_NTPRefreshInterval 30      // Frequence (secs) of polling timeserver
const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message
byte packetBuffer[ NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets
byte timeServer[4]; // = {64,90,182,55}, or { 192, 43, 244, 18}; time.nist.gov

/************** ONE WIRE STUFF ****************/
#define TEMPERATURE_PRECISION 10
#define MAXTEMPDEVICES 1                // Only one temp device per pin at present
#define const_TempRefreshInterval 10

const byte maxTempSensors = 10;
OneWire oneWire[maxTempSensors];
DallasTemperature tempSensor[maxTempSensors] = {  DallasTemperature(&oneWire[0]), 
                                                  DallasTemperature(&oneWire[1]),
                                                  DallasTemperature(&oneWire[2]),
                                                  DallasTemperature(&oneWire[3]),
                                                  DallasTemperature(&oneWire[4]),
                                                  DallasTemperature(&oneWire[5]),
                                                  DallasTemperature(&oneWire[6]),
                                                  DallasTemperature(&oneWire[7]),
                                                  DallasTemperature(&oneWire[8]),
                                                  DallasTemperature(&oneWire[9])
                                                };
DeviceAddress tempDeviceAddress[maxTempSensors]; // Hold device addresses to speed things up - only one device per pin


/**************** DEVICE STUFF **************/

const byte maxFreqs = 8;
const byte stackSize = 8;
const int maxReadings = stackSize * 80;            // Max zones (xH and xL) = 16 * 2 + max windows = 32 + max doors = 16
const byte maxDevices = 128;                       // 127 is limit (0 is reserved as null)
const byte maxVars = 64;                           // 127 is limit
const byte maxEvals = 64;                          // 256 is limit if argArray remains byte array
const byte maxElems = 32;
const byte maxSensorTypes = 16;
const byte maxArgs = 64;     

int numDevices = 0;                            // Set when config file read
int numVars = 0;                              // Set on initialisation
int numEvals = 0;                                  // Set when config file read
int numArgs = 0;

const byte readFlag = 0x01;
const byte writeFlag = 0x02;
const byte pinNoOp = 127;
byte p_freqIdx[maxDevices];                  // Indexes into deviceMap for polling @ different frequencies
byte p_freqMarker[maxFreqs + 1];                        // Markers dividing up freqIdx into 8 polling segments

int frequency[maxFreqs];                                  // Seconds between each scan, indexed by frequency


const byte valRef = 0xf0;
const byte valRegion = 0x00;  
const byte valZone = 0x10;
const byte valLocation = 0x20;
const byte valSensor = 0x30;
const byte valType = 0x40;
const byte valArduino = 0x50 | 1;
const byte valPin = 0x60 | 1;
const byte valCascade =0x70 | 1;
const byte valHandler = 0x80 | 1;
const byte valPollFreq = 0x90 | 1;
const byte valStatus = 0xa0 | 2;
const byte valStackMode = 0xb0 | 2;
const byte valTOSIdx = 0xc0 | 2;
const byte valStack = 0xd0 | 2;                    // Stack of bits, or index to stack of values
const byte valCurr = 0xe0 | 2;              // GET or PUT value to top of history stack; value is either a bit and stored in Stack (StackMode = 0) or an int stored in readingHistory
const byte valPrev = 0xf0 | 2;                    // GET previous reading
const byte valAvg = 0x14 | 2;                    // GET average of stack values
const byte valMax = 0x24 | 2;                    // GET max of stack values
const byte valMin = 0x34 | 2;                    // GET min of stack values
const byte valROC = 0x44 | 2;                    // GET rate of change of stack values
const byte valDay = 0x54 | 2;                    
const byte valHour = 0x64 | 2;                    
const byte valMinute = 0x74 | 2;      

const byte valStatusStable = 0x00;
const byte valStatusPending = 0x01;             // Used to indicate sensor reading initiated and to be read on next heartbeat (used for slow devices such as temperature)
const byte valStatusTarget = 0x02;              // Used to indicate target physical state of device, typcially requiring pin to be set
const byte valStatusUnset = 0x03;
const byte valSlowCapture = 3;               // Indicates temp sensor read triggered previously, so collect reading this time

unsigned int (*getSensorReading[maxSensorTypes])(unsigned int pin, unsigned int handler);        // Array of function pointers to sensor readers, indexed by sensorType


/****************** DECISION STUFF ***************/


const byte valA = 0x01;
const byte valB = 0x02;
const byte valLen = 0x03;
const byte valPtr = 0x04;
const byte valDest = 0x05;
const byte valCalc = 0x06;
const byte valExp = 0x07;
const byte valTurnOff = 0x08;
const byte valListExp = 0x09;

const byte valCalcListE = 0x00;        // List of pointers to evals
const byte valCalcListM = 0x01;        // List of pointers to devices/variables
const byte valCalcNOT = 0x02;
const byte valCalcCURR = 0x03;
const byte valCalcPREV = 0x04;
const byte valCalcAvg = 0x05;
const byte valCalcMax = 0x06;
const byte valCalcMin = 0x07;
const byte valCalcROfC = 0x08;
const byte valCalcYear = 0x09;
const byte valCalcMonth = 0x0a;
const byte valCalcDay = 0x0b;
const byte valCalcHour = 0x0c;
const byte valCalcMinute = 0x0d;

// Exp values doubled-up in some cases for use if CalcList
const byte valExpEQ = 0x00;
const byte valExpNEQ = 0x01;
const byte valExpGT = 0x02;
const byte valExpAvg = 0x02;    // List version
const byte valExpLT = 0x03;
const byte valExpMax = 0x03;    // List version
const byte valExpADD = 0x04;
const byte valExpSUB = 0x05;
const byte valExpMin = 0x05;    // List version
const byte valExpMULT = 0x06;
const byte valExpDIV = 0x07;
const byte valExpAND = 0x08;
const byte valExpOR = 0x09;
const byte valExpBTW = 0x0a;
const byte valExpNotBTW = 0x0b;
                            

/**************** OTHER STUFF *****************/
uint32_t timestarted;                // For debugging file serve times
const char charTokenStart = '<';
const char charTokenEnd = '>';
const byte heartBeatSecs = 1;
unsigned int heartBeat = 0;
boolean debug = false;
const byte ledPin = 13;
int ledState = 0;      // 0 = Off, 1 = On
const byte arduinoVoltage = 5;
const int analogRange = 1024;
const byte valOff = 0;
const byte valOn = 1;
const byte maskLSB = 0x01;
const byte mask8BitMSB = 0x80;
const unsigned int mask16BitMSB = (0x80 * 256) + 0x00;
const byte offsetDay = 11;
const byte offsetHour = 6;






void setup() {
  Serial.begin(9600);
 
  PgmPrint("Free RAM: ");
  Serial.println(FreeRam());
  
  // Initialize the SD card, initialise a FAT volume and open the root
  if (!card.init(SPI_FULL_SPEED,4)) error("card.init failed!"); 
  if (!volume.init(&card)) error("vol.init failed!"); 
  if (!root.openRoot(&volume)) error("openRoot failed"); 
  
  // Read in config file; establishes the identity, mac address & IP address of this Arduino, plus device IDs and pins, decision logic and variables initialisation
  getConfig();
  
  // Start up Ethernet for Web & Udp - uses up all 4 sockets on Ethernet shield
  Ethernet.begin(mac[arduinoMe], ip[arduinoMe]);    
  UdpNTP.begin(UdpNTPPort);    
  UdpLog.begin(UdpLogPort);     
  UdpArd.begin(UdpArdPort);
  
  // Set up pins & device handlers, incl Time, and start UDP for logging
  initialiseDevices();
  
  // Start up server
  WebServer.begin();  
  
  Serial.println("Startup complete");

}

void getConfig() {            // Read in config.jso file; also read by javascript in web client    
  SdFile configFile;
  
  if (configFile.open(root, "config.jso", O_READ)) {
    loadIdentity (&configFile);          // Get information about the servers and identity of this arduino    
    loadDevices (&configFile);           // Load physical device attributes    
    loadVariables (&configFile);         // Load internal variables    
    loadFreqs (&configFile);             // Get list of scanning frequencies and build optimised route for device scanning    
    loadEvals (&configFile);             // Load evaluations    
    configFile.close();
  }
  else {
    Serial.println("No config file found");
  }
}


void loop() {

  // Regular heartbeat actions
  if ( (millis() % (heartBeatSecs * 1000)) == 0 ) {
      
    heartBeat += heartBeatSecs;    // Eventual overflow @ 32k not material
    
    #if DEBUGON
      // If debug compiled, then periodically see if logging needs activating and if so what type
      if ((heartBeat % (loggerCheckFreq * heartBeatSecs) == 0)) if (UdpLog.available()) { 
        UdpLog.readPacket((byte*)logBuffer, 1, logIP, &UdpLogPort); 
        switch (logBuffer[0]) {
          case 'H':
          case 'h':    debugH = true; break;
          case 'R':
          case 'r':    debugR = true; break;
          case 'E':
          case 'e':    debugE = true; break;
          case 'A':
          case 'a':    debugA = true; break;
          case 'N':
          case 'n':    debugH = debugR = debugE = debugA = false; UdpLogPort = 0;
        }
      }
    #endif
  
    #if DEBUGHEARTBEAT
      if (debugH) { sprintf (logBuffer, "Heartbeat = %d\n", heartBeat); sendLog(logBuffer); }
    #endif
 
    // Get the latest sensor information
    checkSensors();
    
    
    // Decide what to do and if needed do it
    if (makeDecisions()) takeAction();
  }

  // See if any HTTP dialogue  
  if (Client client = WebServer.available()) processHTTP(client);
} 



void checkSensors() {          // Get latest readings for all devices due this heartbeat; optimised to avoid trawling through all devices.  Repeat for slow-read devices
  int i, start, startnext, freqCode;
  char response[10];
  unsigned long startMS = millis();
  unsigned int prevHeartBeat = heartBeat - heartBeatSecs;
  
  for (freqCode = 0; freqCode < maxFreqs; freqCode++) {
    if ( (heartBeat % frequency[freqCode]) == 0) {            // If due now
      start = p_freqMarker[freqCode];            // Get start and end markers for this frequency
      startnext = p_freqMarker[freqCode + 1];
      for (i = start; i < startnext; i++) {
        deviceGet(p_freqIdx[i], false);  // Read all devices at this polling frequency (or trigger read for slow sensors)
//        if (onWatchList(p_freqIdx[i])) { Serial.println("In checkSensors - watch "); Serial.println(p_freqIdx[i]); }
      }
    }
    if ( (prevHeartBeat % frequency[freqCode]) == 0) {    // Check for any slow sensor reads triggered previously that have now arrived (eg temperature)
      start = p_freqMarker[freqCode];            // Get start and end markers for this frequency
      startnext = p_freqMarker[freqCode + 1];
      for (i = start; i < startnext; i++) if (mapGet(p_freqIdx[i], valStatus) == valStatusPending)  deviceGet(p_freqIdx[i], true);   // Get delayed reading
    }
  }
  #if DEBUGHEARTBEAT
    if (debugH) { sprintf (logBuffer, "Sensors checked in %dms\n", millis() - startMS); sendLog(logBuffer); }
  #endif
}


boolean makeDecisions() {        // Perform evalations and set target values if needed; flag if action needed
  byte evalIdx, evalDest;
  unsigned int result;
  boolean actOn, actionNeeded = false;
  unsigned long startMS = millis();
    
  for (evalIdx = 0; evalIdx < numEvals; evalIdx++) {
    if (evalDest = evalGet(evalIdx, valDest)) {            // deviceIdx == 0 is noop (eval used in arg list only)
    
      result = evalRun(evalIdx, &actOn);
    
      if (actOn) {                            // Store result of evaluation and if different to existing set flag to indicate action needed (unless pin = noOp)
        if (evalGet(evalIdx, valTurnOff)) result = valOff;
        
        do {
          mapPut (evalDest, valCurr, result);
          if (mapGet(evalDest, valPin) != pinNoOp && mapGet(evalDest, valPrev) != result) {
            mapPut (evalDest, valStatus, valStatusTarget);
            actionNeeded = true;
          }
          else mapPut(evalDest, valStatus, valStatusStable);
          
          #if DEBUGEVAL
            if (debugE) {
              sendLog("Dest = ");
              if (evalGet(evalIdx, valTurnOff)) sendLog("~");
              printRef(evalDest);      
              sprintf(logBuffer, ", existing = %d, new = %d, pin = %d, status = %d\n", mapGet(evalDest, valCurr), result, mapGet(evalDest, valPin), mapGet(evalDest, valStatus));
              sendLog(logBuffer);
              if (mapGet(evalDest, valCascade)) sendLog("--> Cascade ");
            }
          #endif
        } while (mapGet (evalDest++, valCascade));        // Cascade result if needed
      }
    }
  }

  #if DEBUGHEARTBEAT
    if (debugH) { sprintf (logBuffer, "Decisions made in %dms\n", millis() - startMS); sendLog(logBuffer); }
  #endif

  return actionNeeded;
}    


void takeAction() {      // Issue device instructions based on devices with target values (status == valStatusTarget)
  byte deviceIdx;
  unsigned long startMS = millis(); 
  
  for (deviceIdx = 1; deviceIdx < numDevices; deviceIdx++) {      // Skip over NULL device
    if (mapGet(deviceIdx, valStatus) == valStatusTarget) {
      
      #if DEBUGACTIONS
        if (debugA) {
          sprintf(logBuffer, "Set pin %d (", mapGet(deviceIdx, valPin));
          sendLog(logBuffer);
          printRef(deviceIdx);
          sprintf(logBuffer, ") to %d\n", mapGet(deviceIdx, valCurr));
          sendLog(logBuffer);
        }
      #endif
      
      if (mapGet(deviceIdx, valSensor)) sendLog("Trying to set a sensor\n");
      else {
        if (mapGet(deviceIdx, valCurr) > 1) sendLog("Target neither 0 nor 1\n");
        digitalWrite (mapGet(deviceIdx, valPin), mapGet(deviceIdx,valCurr));
        mapPut(deviceIdx, valStatus, valStatusStable);
      }
    }
  } 

  #if DEBUGHEARTBEAT
    if (debugH) { sprintf (logBuffer, "Actions taken in %dms\n", millis() - startMS); sendLog(logBuffer); }
  #endif
}


void processHTTP(Client client)  {
  char clientline[const_HTTP_BUFSIZ];
  char URLline[const_HTTP_BUFSIZ];
  int index = 0;
  char *dataStart;
  char mode = ' ';    // ' ' = normal, 'G' = GET, 'P' = waiting for blank line in POST, 'D' = waiting for data line in POST, 'X' = request complete
  long int contLen;
  unsigned long startMS = millis(); 
  
  while (client.connected() && mode != 'X') {
    if (client.available()) { 
      char c = client.read();
              
      switch (c) {
        case '\n':
          clientline[index] = 0;  
          
          if (index==0) {      // Blank line indicates end of request, unless it was a POST (in which case next line contains data, so keep alive)
            switch (mode) {
              case 'G':        // Was a GET; data is in the saved URLline
                if (dataStart = strstr(URLline, "ajax!")) { handleAjaxGet (client, dataStart + 6, dataStart[5]); }      // Ajax Get: 'R'eading, 'T'ime, or 'P'ut
                else if ((dataStart = strstr(URLline,"?")) != 0) { handleHTTPCmd(client,dataStart+1); }  // Was a GET after a Form submit;  handle the submitted text
                else { handleHTTPGet(client, URLline); }                 // Normal GET
                mode = 'X';
                break;
              case 'P':                                         // Finished POST headers, expect data in next line
                mode = 'D';
                break;
              default:                           // Shouldn't happen?
                Serial.println("No mode set");              
                client.println("HTTP/1.1 200 OK");
                client.println();
                stopClient(client);
                mode = 'X';
            }
          }
          else {                                      // Not a blank line, check what's on it
            if (strstr(clientline,"GET /") != 0) {              
              mode = 'G';                            
              strncpy (URLline,clientline,const_HTTP_BUFSIZ);    // Save line for later
              echoLine(clientline); 
            }
            else if (strstr(clientline,"POST /") != 0) {
              mode = 'P';                            // Got a POST; set mode & remember URL
              strncpy (URLline,clientline,const_HTTP_BUFSIZ);
              echoLine(clientline); 
            }
            else if (mode =='P' && (dataStart = strstr(clientline,"Content-Length:")) != 0) {    // Got the data length for a POST; remember it
              dataStart += 16;      // Step over "Content-Length: " to start of numbers
              contLen = atoi(dataStart);        // Get the content length
              if (contLen > const_HTTP_BUFSIZ) contLen = const_HTTP_BUFSIZ;        // Can't cope with huge POSTs
            }
            
            index = 0;
          }
          break;
        case '\r':    // Ignore CR
          break;
        default:      // Add a character to the input buffer; if too large just truncate (enough info in first const_HTTP_BUFSIZ chars)
          clientline[index] = c;
          if (index < (const_HTTP_BUFSIZ-1)) index++;

          if (mode == 'D' && index >= contLen) {    // If waiting for POST data, then process when got it all and refresh page
            handleHTTPCmd(client,clientline);
            strncpy (clientline,URLline,const_HTTP_BUFSIZ);    // Contains "POST /"
            strncpy (clientline+1,"GET",3);                    // Replace with "GET"
            handleHTTPGet(client,clientline+1);                    // ... and issue refresh, starting at 'G'
            stopClient(client);
            mode = 'X';
          }
      }    // Switch (c)
    }    // Client.available
  }    // Client.connected
  
  #if DEBUGHEARTBEAT
    if (debugH) { sprintf (logBuffer, "Web checked in %dms\n", millis() - startMS); sendLog(logBuffer); }
  #endif
}

// **************** Read physical device - support to CHECK SENSORS ***************************

void deviceGet(int deviceIdx, boolean captureRead) {              // Read physical device and store in device map; captureRead set true if this is a follow-up for slow sensors
  unsigned int reading;
  
  if (mapGet(deviceIdx,valArduino) != arduinoMe) {
    sendLog("Get from other arduino");        // Code to be written
    return;
  }
  
  if (mapGet(deviceIdx, valSensor)) {
    reading = getSensorReading[mapGet(deviceIdx, valType)]
                    (mapGet(deviceIdx, valPin), 
                     (captureRead) ? valSlowCapture : mapGet(deviceIdx, valHandler));            // Invoke appropriate handler (slowCapture == true reads temp) and tell it where and how to read
                     
    if ( isSlowSensor (deviceIdx) && captureRead == false) {      // Test if temp sensor and whether to use reading or wait until next time
      mapPut (deviceIdx, valStatus, valStatusPending);  // Temp sensor and waiting - just put status 
    }
    else {                                              // Either not a temp sensor, or have already triggered a read and now need to store captured read
      mapPut(deviceIdx, valCurr, reading);
      mapPut(deviceIdx, valStatus, valStatusStable);          
      #if DEBUGREADINGS
        if (debugR) { printRef(deviceIdx); sprintf (logBuffer, " pin = %d reading = %d\n", mapGet(deviceIdx, valPin), reading); sendLog(logBuffer); } 
      #endif
    }

  }
  else { sprintf(logBuffer, "Not sensor %d\n", deviceIdx); sendLog(logBuffer); }
}

/********************* PERFORM EVALUATION - support to MAKE DECISIONS *********************/

unsigned int evalRun (byte evalIdx, boolean *actOn) {
  byte evalCalc, evalExp, tempA, listElem;
  unsigned int result, evalA, evalB, tempResult;
  #if DEBUGEVAL
    char textString[10];
  #endif
  
  evalCalc = evalGet(evalIdx, valCalc);
  evalExp = evalGet(evalIdx, valExp);
  
  // Calc determines how to interpret high word
  if ((evalCalc == valCalcListE) || (evalCalc == valCalcListM)) {      // High word is a len/index pair into argArray, which holds list of eval or map pointers
    unsigned int argsLen = evalGet(evalIdx, valLen);
    unsigned int argsPtr = evalGet(evalIdx, valPtr);
    boolean noBreak = true, evals = (evalCalc == valCalcListE);
    
    #if DEBUGEVAL
      if (debugE) {
        getCalcChar (evalCalc, textString);
        sprintf(logBuffer, "\nEvalIdx = %d %s %c args = %d\n--> Start", evalIdx, textString, getExpListChar (evalExp), argsLen);
        sendLog(logBuffer);
      }
    #endif
      
    result = (evals) ? evalRun(argGet(argsPtr), NULL) : mapGet(argGet(argsPtr), valCurr);
    
    switch (evalExp) {
      case valExpAvg:  
        for (int i = 1; i < argsLen; i++) result += (evals) ? evalRun(argGet(argsPtr + i), NULL) : mapGet(argGet(argsPtr + i), valCurr);
        result = result / argsLen;
        break;
      case valExpMax:
        for (int i = 1; i < argsLen; i++) if (tempResult = (evals) ? evalRun(argGet(argsPtr + i), NULL) : mapGet(argGet(argsPtr + i), valCurr) > result) result = tempResult;
        break;        
      case valExpADD:
        for (int i = 1; i < argsLen; i++) result += (evals) ? evalRun(argGet(argsPtr + i), NULL) : mapGet(argGet(argsPtr + i), valCurr);
        break;
      case valExpMin:
        for (int i = 1; i < argsLen; i++) if (tempResult = (evals) ? evalRun(argGet(argsPtr + i), NULL) : mapGet(argGet(argsPtr + i), valCurr) < result) result = tempResult;
        break;        
      case valExpMULT:
        for (int i = 1; i < argsLen; i++) result *= (evals) ? evalRun(argGet(argsPtr + i), NULL) : mapGet(argGet(argsPtr + i), valCurr);
        break;
      case valExpAND:   for (int i = 1; i < argsLen && result != 0; i++) { result = (evals) ? evalRun(argGet(argsPtr + i), NULL) : mapGet(argGet(argsPtr + i), valCurr); } break;           // For AND, quit on a false
      case valExpOR:    for (int i = 1; i < argsLen && result == 0; i++) { result = (evals) ? evalRun(argGet(argsPtr + i), NULL) : mapGet(argGet(argsPtr + i), valCurr); } break;           // For OR, quit on a true
      default:          Serial.print("Bad Exp");
    }
    
    *actOn = (evalExp == valExpAND || evalExp == valExpOR) ? result : true;
    
    #if DEBUGEVAL
      if (debugE) {
        sprintf(logBuffer, "\n--> End %s Result = %d ", textString, result);
        sendLog(logBuffer);
        if (result == 0) sendLog("\n");
      }
    #endif
  }
  else {              // High word holds A and B values
    // Arg A is either index to multiple variants of device or variable readings, or variants on current time.
    switch (evalCalc) {        // "&&", "||", "!", "CURR", "PREV", "AV", "MX", "MN", "ROC", "YR", "MTH", "DAY", "HR", "MIN"
      case valCalcNOT:    evalA = !mapGet(tempA = evalGet (evalIdx, valA), valCurr); break;
      case valCalcCURR:   evalA = mapGet(tempA = evalGet (evalIdx, valA), valCurr); break;
      case valCalcPREV:   evalA = mapGet(tempA = evalGet (evalIdx, valA), valPrev); break;
      case valCalcAvg:    evalA = mapGet(tempA = evalGet (evalIdx, valA), valAvg); break;
      case valCalcMax:    evalA = mapGet(tempA = evalGet (evalIdx, valA), valMax); break;
      case valCalcMin:    evalA = mapGet(tempA = evalGet (evalIdx, valA), valMin); break;
      case valCalcROfC:   evalA = mapGet(tempA = evalGet (evalIdx, valA), valROC); break;
      case valCalcYear:   evalA = year(); break;            // 4 digit year
      case valCalcMonth:  evalA = month(); break;          // Jan = 1
      case valCalcDay:    evalA = day(); break;         
      case valCalcHour:   evalA = hour(); break;
      case valCalcMinute: evalA = minute(); break;
      default: Serial.print("Unrecognised calc mode, idx = "); Serial.println(evalIdx, HEX);
    }
   
    // Arg B is index to current value of either device or variable
    evalB = mapGet(evalGet(evalIdx, valB), valCurr);   

    // Got arguments, now do evaluation -  "=", "!", ">", "<", "+", "-", "*", "/", '&', '|', '[', ']'
    switch (evalExp) {    
      case valExpEQ:     (*actOn) = result = evalA == evalB; break;
      case valExpNEQ:    (*actOn) = result = evalA != evalB; break;
      case valExpGT:     (*actOn) = result = evalA > evalB; break;
      case valExpLT:     (*actOn) = result = evalA < evalB; break;
      case valExpADD:    result = evalA + evalB; (*actOn) = true; break;
      case valExpSUB:    result = evalA - evalB; (*actOn) = true; break;
      case valExpMULT:   result = evalA * evalB; (*actOn) = true; break;
      case valExpDIV:    result = evalA / evalB; (*actOn) = true; break;
      case valExpAND:    (*actOn) = result = evalA && evalB; break;
      case valExpOR:     (*actOn) = result = evalA || evalB; break;
      case valExpBTW: 
      case valExpNotBTW:   {
        unsigned int start, end, dhmValA = evalA, dhmValB = evalB, dhmDay = dhmGet(dhmValA, valDay), timeNow = dhmMake(5,22,30);
        switch (dhmDay) {
          case 0:        start = 1; end = 7; break;        // Daily
          case 1:                                          // Sun
          case 2:                                          // Mon
          case 3:                                          // Tue
          case 4:                                          // Wed
          case 5:                                          // Thu
          case 6:                                          // Fri
          case 7:        start = end = dhmDay; break;      // Sat
          case 8:        start = 2; end = 5; break;        // Mon-Thu
          case 9:        start = 2; end = 6; break;        // Mon-Fri
          case 10:       start = 7; end = 0; break;        // Weekend
          default:       Serial.println("Bad dhmDay");
        }
        for (int i = start; i <= ((end) ? end : 7); i++) {
          dhmPut (&dhmValA, valDay, i);
          dhmPut (&dhmValB, valDay, i);
          if (result = (timeNow >= dhmValA && timeNow <= dhmValB)) break;
        }
        if (end == 0 && result == 0) {                          // If Sun of weekend and dhmNow not on Sat
          dhmPut (&dhmValA, valDay, 1);
          dhmPut (&dhmValB, valDay, 1);
          result = timeNow >= dhmValA && timeNow <= dhmValB;
        }
        }
        (*actOn) = (evalExp == valExpNotBTW) ? result = (result == 0) : result;    // Swap logic if not between  
        break;
      default:  Serial.print ("Bad eval"); Serial.println(evalExp, HEX);
    }
    
    #if DEBUGEVAL
      if (debugE) {
        getCalcChar (evalCalc, textString);
        sprintf(logBuffer, "\nEvalIdx = %d IF (%s ", evalIdx, textString);
        sendLog(logBuffer);
        if (evalGet(evalIdx, valCalc) > valCalcROfC) sprintf(logBuffer,"%d %c",evalA, getExpChar(evalGet(evalIdx, valExp))); 
        else { printRef(tempA); sprintf(logBuffer, " [%d] %c ", evalA, getExpChar(evalGet(evalIdx, valExp))); }
        sendLog(logBuffer);
        printRef(evalGet(evalIdx, valB)); 
        sprintf(logBuffer, " [ %d]) Result = %d ", evalB, result);
        sendLog(logBuffer);
      }
    #endif
  }    
  return result;
} 



// ***************************** WEB MANAGEMENT ********************************

void handleAjaxGet(Client client, char* actionline, char type){        // Used to process Ajax GET; actionline points to first char after 'R', 'T' or 'P' - gets overwritten
  char responseText[100];
  char element[const_Token_Bufsiz] = "";
  char *newVal;
  const char str1[] = "{\"id\": \"";
  const char str2[] = "\", \"reading\": \"";
  const char str3[] = "\", \"status\": \"";
  const char str4[] = "\"}";
  byte deviceIdx, devStatus, timeHr, timeMin;
  unsigned int devReading;
  unsigned long timestarted = millis();
  
  *strstr(actionline," HTTP") = 0;                      // Terminate string before " HTTP/1.1"
  if ((newVal = strstr(actionline,"=")) != 0) {          // See if there is an assignment (type = 'P')
    devReading = atoi(newVal+1);                          // Save the new value
    *newVal = 0;                                        // And set new termination point
  }
  strncpy (element, actionline, const_Token_Bufsiz);    // Start of Actionline contains an element id, shared with browser
  
  switch (type) {      
    case 'H':                                            // Heartbeat required
      sprintf (responseText, "%s%s%s%u%s%d%s", str1, element, str2, heartBeat, str3, valStatusStable, str4);
      break;    
    case 'R':                                            // Reading required
      deviceIdx = getDeviceIdx(element);                    // Get index of element in device or variable array
      devReading = mapGet(deviceIdx, valCurr);
      devStatus = mapGet(deviceIdx, valStatus);
      sprintf (responseText, "%s%s%s%d%s%d%s", str1, element, str2, devReading, str3, devStatus, str4);
      break;
    case 'T':                                            // Time required in format hh:mm
      if (timeStatus() == timeSet) { devReading = (weekday() * pow(2, offsetDay)) + (hour() * pow(2, offsetHour)) + minute(); devStatus = valStatusStable; }
      else { timeHr = millis()/1000/SECS_PER_HOUR; timeMin = millis()/1000/SECS_PER_MIN; devStatus = valStatusUnset; }
      sprintf (responseText, "%s%s%s%d%s%d%s", str1, element, str2, devReading, str3, devStatus, str4);
      break;
    case 'P':                                          // Put required
      deviceIdx = getDeviceIdx(element);                    // Get index of element in device or variable array
      mapPut(deviceIdx, valCurr, devReading);
      mapPut(deviceIdx, valStatus, devStatus = valStatusStable);
      sprintf (responseText, "%s%s%s%d%s%d%s", str1, element, str2, devReading, str3, devStatus, str4);
      break;
    default:      Serial.println("Unrecognised ajax GET");
  }

  client.println("HTTP/1.1 200 OK");
  client.print("Server: Arduino/");
  client.println(arduinoMe);
  client.println("Content-Type: text");
  client.print("Content-Length: ");
  client.println(strlen(responseText));    
  client.println();
  client.print(responseText); 
}


void handleHTTPGet(Client client, char* clientline) { 
  SdFile file_a;
  SdFile file_b;
  SdFile *p_parent;
  SdFile *p_child;
  SdFile *p_temp;
  char homePage[] = "index.htm"; 
  char *filename;
  char *subDirMark;
  char lcExtn[3];
  
  timestarted = millis();            // Start the clock running
  
  // Initialise pointers to SdFile objects and set parent to root;  
  p_parent = &file_a;
  p_child = &file_b;
  *p_parent = root;
  
  // Housekeeping to standardise processing
  if (strstr(clientline,"GET / ")) memcpy(clientline+5,homePage,strlen(homePage));  // If no file specified, then serve home page
  strstr(clientline," HTTP/")[0] = 0;      // Place terminator after path/filename  
  filename = clientline + 5;            // Step over the "GET /"
  
  // Traverse path
  while ((subDirMark=strstr(filename,"/")) != 0) { 
    subDirMark[0] = 0;
    if ((*p_child).open(p_parent,filename,O_READ)) {
      
      // Close old parent and establish new
      (*p_parent).close();      
      p_temp = p_parent;        
      p_parent = p_child;        // New (open) parent
      p_child = p_temp;          // Closed SdFile object
      
      filename = subDirMark+1;
    }
    else reply404(client);
  }

  // Path traversed; now deal with file
  if ((*p_child).open(p_parent, filename, O_READ)) {
      memcpy(lcExtn,strstr(filename,".")+1,3);
      stolower(lcExtn); 
      serveHTTPFile(client,p_child,lcExtn);
      (*p_child).close();
  }
  else {
    Serial.println("no file found");
    reply404(client);
  }
  (*p_parent).close();

}






void handleHTTPCmd(Client client, char* actionline){        // Used to process POST and GET /? strings
  echoLine(actionline);
  if (strstr(actionline,"=On")) {
    digitalWrite (ledPin,HIGH);
    Serial.println("Switching heating on");
    ledState = 1;
  }
  else if (strstr(actionline,"=Off")) {
    digitalWrite (ledPin,LOW);
    Serial.println("Switching heating off");
    ledState = 0;
  } 
}


void serveHTTPFile(Client client, SdFile *p_file, char * extn) {
  int16_t byteCnt;
  uint32_t fileSize;
  byte readBuffer[const_SDCard_BUFSIZ];
  char mode;          // 's' = serve unchanged; 't' process token
  char token[const_Token_Bufsiz];  
  char tokResponse[10]; 
  int token_idx, firstValid;
      
  client.println("HTTP/1.1 200 OK");
  
  client.print("Server: Arduino/");
  client.println(arduinoMe);
  
  client.print("Content-Type: ");
  if (strstr(extn, "htm") != 0)         client.println("text/html");
  else if (strstr(extn, "css") != 0)    client.println("text/css");
  else if (strstr(extn, "jpg") != 0)    client.println("image/jpeg");
  else if (strstr(extn, "png") != 0)    client.println("image/png");
  else if (strstr(extn, "gif") != 0)    client.println("image/gif");
  else if (strstr(extn, "pdf") != 0)    client.println("image/pdf");
  else if (strstr(extn, "ico") != 0)    client.println("image/x-icon");
  else if (strstr(extn, "xml") != 0)    client.println("application/xml");
  else if (strstr(extn, "jso") != 0)    client.println("application/json");
  else if (strstr(extn, "js") != 0)     client.println("application/javascript");
  else                                    client.println("text");
  
  fileSize = (*p_file).fileSize();
  client.print("Content-Length: ");
  client.println(fileSize);
//  if (strstr(extn, "jso") != 0) client.println(fileSize*1.1+100); else client.println(fileSize);    // If JSON file, may have token substitution, so add a bit to length
  client.println();

/*
  if (strstr(extn, "jso") != 0) {    // Check for tokens in JSON file and process appropriately
    mode = 's';      
    memset(token,0,const_Token_Bufsiz);
    token_idx = 0;
    
    while ((byteCnt=((*p_file).read(readBuffer,const_SDCard_BUFSIZ)))>0) {
      firstValid = 0;
      for (int i=0;i<byteCnt;i++) {
        switch (mode) {
          case 's':                                // File serve mode
            if (readBuffer[i] == charTokenStart) {            // Token starts with next char, so set mode and flush buffer
                mode = 't';   
                client.write(readBuffer+firstValid,i-firstValid);
            }
            else if (i==(byteCnt-1)) {
              client.write(readBuffer+firstValid,byteCnt-firstValid);  // If end of input then flush buffer, otherwise just go to next char
            }
            break;             
          case 't':                                    // Token processing mode
            if (readBuffer[i] == charTokenEnd) {        // End marker; process token then revert back to file serve mode
              memset(tokResponse,0,10);
              processToken(token,tokResponse);
              client.write(tokResponse);
              memset(token,0,const_Token_Bufsiz);
              token_idx = 0;
              mode = 's';
              firstValid = i+1;
            }
            else {          // Add token to buffer
              token[token_idx] = readBuffer[i];
              if (token_idx < const_Token_Bufsiz - 1) token_idx++;
            }
            break;
          default:
            Serial.println("Unrecognised mode");
        }
      }
    }
  }
  else {
    */
    while ((byteCnt=((*p_file).read(readBuffer,const_SDCard_BUFSIZ)))>0) client.write(readBuffer, byteCnt);    // Not serving .jso file; no pre-processing required
 // }
}

void processToken(char *token,char *strResponse) {

  char strRed[] = "red";
  char strAmber[] ="amber";
  char strGreen[]="green";
  char strLightOn[] = "lighton";
  char strLightOff[] = "X";
  
  switch (token[0]) {
    case 'T':          // Get time  
      if (timeStatus() == timeSet) sprintf(strResponse,"%d:%02d:%02d",hour(), minute(), second());
      else sprintf(strResponse,"%d:%02d:%02d",millis()/1000/SECS_PER_HOUR,millis()/1000/SECS_PER_MIN,millis()/1000);
      break;
    case 'R':          // Get sensor reading
      sprintf(strResponse,"%s",strLightOff);
      break;
    case 'S':          // Get sensor state
 //     printTemp(tempC[0],strResponse);
      break;
    case 'C':          // Set class
      sprintf(strResponse,"%s",strAmber);
      break;
    default:
      strResponse[0] = token[0];
  }
  
}



/********************* GET CONFIG helper functions *******************/

void loadIdentity (SdFile *configFile) {      // Get identity of arduinos and set identity of this one
  char element[const_Token_Bufsiz];
  byte arduino;

  if (!getNextElement(configFile, "servers:", element)) Serial.println("Config error: no servers");   // ':' in tag indicates just position to the tag

  while (getNextElement(configFile, "arduino", element) && (arduino = atoi(element) - 1) < maxArduinos) {
    
    getNextElement(configFile, "ip:", element);      // Get IP address of arduino
    for (int i = 0; i < 4; i++) { getNextElement(configFile, "element", element); ip[arduino][i] =  atoi(element); } 
    
    getNextElement(configFile, "mac:", element);      // Get mac address
    for (int i=0; i<6; i++) { getNextElement(configFile, "element", element); mac[arduino][i] = strtol(element, NULL, 16); }
  }
  
  getNextElement(configFile, "me", element);
  arduinoMe = atoi(element) - 1;
     
  if (!getNextElement(configFile, "timeserver:", element)) Serial.println("Config error: no timeserver");
  for (int i=0; i<4; i++) { getNextElement(configFile, "element", element); timeServer[i] = atoi(element); }
}


void loadDevices (SdFile *configFile) {      // Load up devices
  char element[const_Token_Bufsiz];
  char element2[const_Token_Bufsiz];
  const unsigned int indSensorStackMode = (0xFF * 256) + B11001100;      // Flag for each sensorType - 1 - device gives readings; 0 - device is on/off   
  
  if (!getNextElement(configFile, "devices:", element)) Serial.println("Config error: no devices");
  
  mapPut(0, valRef, 0);        // NULL device; shouldn't be referenced

  int deviceIdx = 1;
  int readingIdx = 0;
    
  while (getNextElement(configFile, "arduino", element) && deviceIdx < maxDevices) {
    
    byte arduino = atoi(element) - 1;
    
    // Ignore if not for this arduino
    if (arduino == arduinoMe) {
      getNextElement(configFile, "id", element);
      strncpy (element2,element, const_Token_Bufsiz);
      mapPut(deviceIdx, valRef, convertRefToBit(element));

      mapPut(deviceIdx, valArduino, arduino);
     
      getNextElement(configFile, "pin", element);
      mapPut(deviceIdx, valPin, atoi(element));
      
      getNextElement(configFile, "cascade", element);
      mapPut(deviceIdx, valCascade, element[0] == 'Y' ? 1 : 0);
      
      getNextElement(configFile, "handler", element);
      mapPut(deviceIdx, valHandler, atoi(element) - 1);
              
      getNextElement(configFile, "freq", element);
      mapPut(deviceIdx, valPollFreq, atoi(element) - 1);
      
      mapPut(deviceIdx, valStatus, valStatusUnset);
      
      // Set pointers & clear reading history 
      unsigned int stackMode = mapGet(deviceIdx, valSensor) ? ((indSensorStackMode & (1 << mapGet(deviceIdx, valType))) != 0) : 0;
      mapPut (deviceIdx, valStackMode, stackMode);
      if (stackMode) {                                      // Longer readings, main array holds index into separate array
        mapPut(deviceIdx, valStack, readingIdx);
        mapPut(deviceIdx, valTOSIdx, 0);
        for (int j = 0; j < stackSize; j++) stackPush(deviceIdx, 0);    // Write stackSize times to clear stack
        if (++readingIdx * stackSize >= maxReadings) { Serial.println ("Hist OF"); readingIdx--; break;}
      }
      else mapPut(deviceIdx, valStack, 0);            // On/off history held as bitmap in main array
      
      if (++deviceIdx >= maxDevices) { Serial.println ("Dev OF"); deviceIdx--; break;}
      
    }
  }
  numDevices = deviceIdx;
}

void loadVariables (SdFile *configFile) {      // Load up variables
  char element[const_Token_Bufsiz];
  char element2[const_Token_Bufsiz];      
  
  if (!getNextElement(configFile, "variables:", element)) Serial.println("Config error: no variables");
  
  int varIdx = 0;
    
  while (getNextElement(configFile, "arduino", element) && varIdx < maxVars) {
   
    byte arduino = atoi(element) - 1;
         
    // Ignore if not for this arduino
    if (arduino == arduinoMe) {
      getNextElement(configFile, "id", element);
      if ( element[0] = 'V' && varIdx == atoi(element + 2) ) {
        char *colonPosn;
        getNextElement(configFile, "val", element);
        if ( colonPosn = (char*)memchr(element, ':', const_Token_Bufsiz) ) {      // Got a time field in d:mm:ss format
          unsigned int dhmVal = 0;
          dhmPut (&dhmVal, valDay, atoi(element));
          dhmPut (&dhmVal, valHour, atoi(colonPosn + 1));
          colonPosn = (char*)memchr(colonPosn + 1, ':', const_Token_Bufsiz);
          dhmPut (&dhmVal, valMinute, atoi(colonPosn + 1));
          mapPut(varIdx | mask8BitMSB, valCurr, dhmVal);
        }
        else mapPut(varIdx | mask8BitMSB, valCurr, atoi(element));
      }
      else { Serial.print("Var out of seq: "); Serial.println(element); }

      if (++varIdx >= maxVars) { Serial.println ("Vars OF"); varIdx--; break; }
    }
  }
  numVars = varIdx; 
}

void loadEvals (SdFile *configFile) {      // Load up evaluations
  char element[const_Token_Bufsiz];
  byte elemIdx;
  
  if (!getNextElement(configFile, "evals:", element)) Serial.println("Config error: no evaluations");
  
  int evalIdx = 0;
  int argsIdx = 0;
    
  while (getNextElement(configFile, "arduino", element) && evalIdx < maxEvals) {
    
    byte arduino = atoi(element) - 1;
    
    // Ignore if not for this arduino
    if (arduino == arduinoMe) {
      int calcIdx;
      getNextElement(configFile, "seq", element);
      if ((atoi(element) - 1) != evalIdx) Serial.println("Eval out of seq "); 
      
      getNextElement(configFile, "calc", element); 
      evalPut (evalIdx, valCalc, calcIdx = getCalcIdx (element));
 
      switch (calcIdx) {
        case valCalcListE: 
        case valCalcListM:                // Is a list (of evaluations or map elements)   
          evalPut(evalIdx, valPtr, argsIdx);             // Save pointer to start of evaluations
            
          // Load up list 
          elemIdx = 0;
          if (!getNextElement(configFile, "list:", element)) Serial.println("No elems");
          while (getNextElement(configFile, "elem", element) && elemIdx < maxElems) {
            argPut(argsIdx, (calcIdx == valCalcListE) ? atoi(element) - 1 : getDeviceIdx(element));
            if (++argsIdx >= maxArgs) { Serial.println("Args OF"); argsIdx --; } else elemIdx++;
          }
          evalPut(evalIdx, valLen, elemIdx);      // Save number of elements
          break;         
        case valCalcCURR:
        case valCalcPREV:
        case valCalcAvg:
        case valCalcMax:
        case valCalcMin:
        case valCalcROfC:            // Arg A is device or variable ref in these calc types
          getNextElement(configFile, "arga", element);
          evalPut(evalIdx, valA, getDeviceIdx(element) );
          // No break - drop through to pick up Arg B
        case valCalcYear:        // Arg A not used for remaining args - is current year/month/day/hr/minute instead
        case valCalcMonth:
        case valCalcDay:
        case valCalcHour:
        case valCalcMinute:                 
          // Arg B is device or variable ref
          getNextElement(configFile, "argb", element);
          evalPut(evalIdx, valB, getDeviceIdx(element));      
          break;        
        default:
          Serial.println ("Invalid calc type"); 
      }
  
      // Get expression
      getNextElement(configFile, "exp", element);
      evalPut (evalIdx, valExp, (calcIdx == valCalcListE || calcIdx == valCalcListM) ? getExpListIdx(element) : getExpIdx (element));

      // Get destination
      getNextElement(configFile, "dest", element);
      if (element[0] == 'X') evalPut(evalIdx, valDest, 0);            // NULL dest - eval not to be used independently - only as part of arg list
      else {
        boolean turnOff = element[0] == '~';       // An '~' indicates set dest to Off, rather than to result of expression
        evalPut(evalIdx, valDest, getDeviceIdx((turnOff) ? element + 1 : element ));
        evalPut(evalIdx, valTurnOff, turnOff);
      }

      if (++evalIdx >= maxEvals) evalIdx--;      
    }
  }

  numEvals = evalIdx;
  numArgs = argsIdx;
}

void loadFreqs (SdFile *configFile) {      // Load up frequencies & optimised scanning route
  char element[const_Token_Bufsiz];

  // Get list of scanning frequencies
  if (!getNextElement(configFile, "frequencies:", element)) Serial.println ("Config error: no frequencies");
  for (int i=0; i < maxFreqs; i++) {
    getNextElement(configFile, "seconds", element);
    frequency[i] = atoi(element);
  }
  
  // Prepare optimised route into deviceMap, ordered by frequency
  int latest = 0;
  for (int i=0; i < maxFreqs + 1; i++) p_freqMarker[i] = 0;

  for (int freqCode = 0; freqCode < maxFreqs; freqCode++) {
    for (int deviceIdx = 1; deviceIdx < numDevices; deviceIdx++) {
      if (mapGet(deviceIdx, valPollFreq) == freqCode &&
          mapGet(deviceIdx, valSensor) &&
          mapGet(deviceIdx, valArduino) == arduinoMe) p_freqIdx[latest++] = deviceIdx;
    }
    p_freqMarker[freqCode + 1] = latest;
  }
}
  
boolean getNextElement( SdFile *p_file, char *p_tag, char *p_element) {   // Helper function for getConfig routines; assumes valid JSON file
  
  static byte readBuffer[const_SDCard_BUFSIZ];      // Input buffer from file
  static unsigned int byteCnt, buffer_idx;          // Number of bytes read in (normally buffer length, but not for final read), and index into current char
  char mode = 's';          //  's' = scan for tag; 't' = process tag, 'r' = scan for element, 'e' = process element, '}' = found end struct, check for end of array; ']' = found end of array - quit; 'x' = got result
  const byte tagBuffSiz = 30;
  char tagBuff[tagBuffSiz];          // Holds the tag as read - to be tested against p_tag
  boolean justTag;           // If true then return after positioning to the tag; ie, don't read the tag element
  static byte arrayDepth;    // Current depth of array levels
  static unsigned int scope[10];      // 0 if tag not found, else simplistic hash value of first tag in array
  byte element_idx, tag_idx;
  
  // If first time in, then initialise buffer
  if (((*p_file).curPosition()) == 0) {
    byteCnt=((*p_file).read(readBuffer,const_SDCard_BUFSIZ));   
    buffer_idx = 0;
    arrayDepth = 0;
    scope[0] = 0;
  }
  
  justTag = strstr(p_tag,":") != 0;          // ':' indicates position just to the tag, not the element
 
  while (byteCnt > 0 && mode != 'x' && mode != ']') {
    byte input = readBuffer[buffer_idx];

    switch (mode) {
      case 's':                                // Scanning mode
        switch (input) {
          case '"':    mode = 't'; memset(tagBuff, 0, tagBuffSiz); tag_idx = 0; break;  // Got the start of a tag; clear buffer and position to start
          case '[':    scope[++arrayDepth] = 0; break;    // Clear hash at start of array
          case '}':    mode = '}';                        // Found end of struct; check if another one to come (next char ','), or end of array (']')
        }
        break;      
      case 't':                                    // Tag processing mode
        if (input == '"') {        // End marker; check if correct tag
          if (strncmp(tagBuff, p_tag, strlen(tagBuff)) == 0) mode = (justTag) ? 'x' : 'r';       // Tag matches; either finish or start looking for element
          else mode = 's';                                // Wrong tag, keep scanning
        }
        else tagBuff[tag_idx++] = input;  // Add char to tag
        break;
      case 'r':                                // Scanning for element
        if (input == '"') {
          if (scope[arrayDepth] == 0) for (int i = 0; i < strlen(tagBuff); i++) scope[arrayDepth] = (scope[arrayDepth] << 1) + (byte)tagBuff[i];     // Simplistic hash of found tag (previous mode = 't')
          mode = 'e';  // Value starts with next char
          memset(p_element, 0, const_Token_Bufsiz);    // Clear buffer in readiness
          element_idx = 0;
        }
        break;
      case 'e':                                // Element processing
        if (input == '"') mode = 'x'; else p_element[element_idx++] = input;   // Either got to end of element and terminate, or add char to element
        break;
      case '}':                                  // Checking what comes after end struct
        if (input == ',') mode = 's';    // Another struct to come, resume scanning mode
        if (input == ']') {             // End of array; break out if same tag previously found at this level, otherwise resume search
          unsigned int newScope = 0;
          for (int i = 0; i < (strlen(p_tag) - ((justTag) ? 1 : 0)); i++) newScope = (newScope << 1) + (byte)p_tag[i];     // Hash of current search
          mode = (newScope == scope[arrayDepth--]) ? ']' : 's';         // Compare to previous search; ']' terminates WHILE, 's' resumes search     
        }
        break;
    }   // switch (mode)

    if (++buffer_idx >= byteCnt) { byteCnt=((*p_file).read(readBuffer, const_SDCard_BUFSIZ)); buffer_idx = 0;  
    }  // If got to end of buffer, then read another one
  }    // while
  
  return mode == 'x';
}

// ******** Device map helper functions ******************


unsigned int mapGet(byte deviceIdx, byte type) { unsigned int result = mapAccess (deviceIdx, type, NULL, readFlag); return result; }
void mapPut(byte deviceIdx, byte type, unsigned int value) { mapAccess (deviceIdx, type, value, writeFlag); }
  
// deviceMap ref ([0]) gives unique ref for device.  Decode format is an.nn.aaa, coded as follows (constants outside mapAccess to allow wider use)
const unsigned int maskRef = (0xff * 256) + 0xff;                // Zero is NULL reference used to indicate evaluation not to write
const unsigned int maskRegion = (B11100000 * 256) + 0x00;        // Up to 8 regions
const unsigned int maskZone = (B00011100 * 256) + 0x00;      // Up to 8 zones per region.  NB: certain zones need rationalising
const unsigned int maskLocation = (B00000011 * 256) + B11100000;  // Up to 32 locations per zone (0 reserved for whole zone).    NB: certain locations need rationalising
const unsigned int maskSensor = B00010000;                               // 1 = sensor, 0 = actor
const unsigned int maskDeviceType = 0x0f;                                // Up to 16 sensors & 16 actors
const byte offsetRegion = 13;
const byte offsetZone = 10;
const byte offsetLocation = 5;
const byte offsetSensor = 4;

unsigned int mapAccess(byte deviceIdx, byte type, unsigned int value, int flag) {            // Get appropriate value out of deviceMap or varReading
  static unsigned int deviceMap[3][maxDevices];      // [0] = coded device ref; [1] = device handler; [2] = state  //; [3] = history
  static unsigned int varReading[maxVars];           // Same format as reading, but for internal variables (indexed by 7 bit variable number - maskVar)
  
  const byte deviceRefIdx = 0x00;
  const byte deviceHandlerIdx = 0x01;
  const byte deviceStateIdx = 0x02;
  const byte maskDeviceMapIdx = 0x03;            // This &'ed with Type gives deviceRefIdx, deviceHandlerIdx, deviceStateIdx
  
  // deviceMap handler ([1]) tells how to access the physical device.  Decode as follows:
  const unsigned int maskArduino = (B11100000 * 256) + 0x00;      // 3 bits, up to 8 Arduinos
  const unsigned int maskPin = (B00011111 * 256) + B11000000;    // 7 bits, up to 128 pins/virtual pins.  Pin 127 is NOP
  const unsigned int maskCascade = B00100000;                    // 1 = cascade this reading to the next deviceIdx; 0 = no cascade
  const unsigned int maskHandler = B00011000;                    // 2 bits, up to 4 handler types per device type
  const unsigned int maskFreq = B00000111;                    // 3 bits, up to 8 action frequencies
  const byte offsetArduino = 13;
  const byte offsetPin = 6;
  const byte offsetCascade = 5;
  const byte offsetHandler = 3;
  
  // deviceMap state([2]) holds the current status of the device.  Decode as follows:
  const unsigned int maskStatus = (B11000000 * 256) + 0x00;            // 0 = stable; 1 = sensor reading pending; 2 = target; 3 = unset
  const unsigned int maskStackMode = (B00100000 * 256) + 0x00;            // How to interpret Stack.  0 = bitmap, 1 = index
  const unsigned int maskTOSIdx = (B00011100 * 256) + 0x00;            // 3 bits = 8 values, only applicable for StackMode = 1; gives offset on Stack * stackSize to latest reading in readingHistory
  const unsigned int maskStack = 0xFF;                               // Stackmode = 0 - 8 bits of On/Off history (MSB = latest) for on/off device types (xTo, xF, xM, xP, P1, P2, P, p, D, L, R)
                                                                    // Stackmode = 1 - 0-127 * stackSize as an index into readingHistory, with TOSIdx giving offset to current top of stack                                                                      
  
  const byte offsetStatus = 14;
  const byte offsetStackMode = 13;
  const byte offsetTOSIdx = 10; 
  
  byte deviceMapIdx = type & maskDeviceMapIdx;
  unsigned int mask;
  int offset, i, temp1, temp2;
    
  if (deviceIdx & mask8BitMSB) {        // Variable
    if ((deviceIdx &= ~mask8BitMSB) >= maxVars) Serial.println("Var out of bounds");    
    switch (type) {
      case valRegion:     return 'V';
      case valArduino:    return arduinoMe;
      case valPin:        return pinNoOp;
      case valStatus:     return valStatusStable;
      case valCascade:    return false;
      case valPrev:
      case valAvg:  
      case valMax:
      case valMin:
      case valROC:
      case valCurr:    if (flag == readFlag) return varReading [deviceIdx]; else { varReading [deviceIdx] = value; break; }
      default:
        Serial.println("Unexpected var type");
        return 0;
    }
  }
  else {
    if (deviceIdx >= maxDevices) { Serial.print("Device "); Serial.print(deviceIdx); Serial.println(" out of bounds"); }
    switch (type) {
      case valRef:        mask = maskRef; offset = 0; break;
      case valRegion:     mask = maskRegion; offset = offsetRegion; break;
      case valZone:       mask = maskZone; offset = offsetZone; break; 
      case valLocation:   mask = maskLocation; offset = offsetLocation; break;
      case valSensor:     mask = maskSensor; offset = offsetSensor; break;
      case valType:       mask = maskDeviceType; offset = 0; break;
      case valArduino:    mask = maskArduino; offset = offsetArduino; break;
      case valPin:        mask = maskPin; offset = offsetPin; break;
      case valCascade:    mask = maskCascade; offset = offsetCascade; break;
      case valHandler:    mask = maskHandler; offset = offsetHandler; break;
      case valPollFreq:   mask = maskFreq; offset = 0; break;
      case valStatus:     mask = maskStatus; offset = offsetStatus; break;
      case valStackMode:  mask = maskStackMode; offset = offsetStackMode; break;
      case valTOSIdx:     mask = maskTOSIdx; offset = offsetTOSIdx; break;
      case valStack:      mask = maskStack; offset = 0; break;
      case valCurr:    // GET - return latest reading.  PUT - Store reading on stack
        if (flag == readFlag) return stackGet (deviceIdx, 0); else { stackPush (deviceIdx, value); return NULL; }
      case valPrev: 
        return stackGet (deviceIdx, 1);      // Only GET - return previous reading
      case valAvg:  
        temp1 = 0;
        for (i = 0; i < 8; i++) temp1 += stackGet (deviceIdx, i);
        return temp1/8;
      case valMax:
        temp1 = stackGet (deviceIdx, 0);
        for (i = 1; i < 8; i++) if (temp2 = stackGet (deviceIdx, i) > temp1) temp1 = temp2;
        return temp1;
      case valMin:
        temp1 = stackGet (deviceIdx, 0);
        for (i = 1; i < 8; i++) if (temp2 = stackGet (deviceIdx, i) < temp1) temp1 = temp2;
        return temp1;
      case valROC:
        temp1 = 0;
        for (i = 1; i < 8; i++) temp1 += (stackGet (deviceIdx, i - 1) - (temp2 = stackGet (deviceIdx, i))) * 100 / temp2;
        return temp1/8/100;
      default: Serial.println("Unknown access type");
    }
    
    if (flag == readFlag) return (deviceMap[deviceMapIdx][deviceIdx] & mask ) >> offset;
    else {
      deviceMap[deviceMapIdx][deviceIdx] &= ~mask;                          // Clear
      deviceMap[deviceMapIdx][deviceIdx] |= ((value << offset) & mask);                // Store 
    }
  }
}

/*************** Device reference and array utilities *********************/


const byte valToInt = 0;
const byte valToChar = 1;
const byte valTestTemp = 2;
const byte valTestSlow = 3;

// Device Ref helper functions

unsigned int convertRefToBit (char *device) { return convertRef (device, NULL, valToInt); }  // convert device chars in form an.nn.aaa to bitstring
void convertRefToChar (unsigned int bitstring, char *device) { convertRef(device, bitstring, valToChar); }  // Convert bitstring device to chars and return in form an.nn.aaa
boolean isTempSensor (byte deviceIdx) { return convertRef (NULL, mapGet (deviceIdx, valRef), valTestTemp); }
boolean isSlowSensor (byte deviceIdx) { return convertRef (NULL, mapGet (deviceIdx, valRef), valTestSlow); }

unsigned int convertRef(char *device, unsigned int bitstring, byte type) {
  const byte numRegionCodes = 7;
  const byte numSensorTypes = 16;
  const byte numActorTypes = 8;
  const char regionCodes[numRegionCodes] = { 'G', 'D', 'S', 'E', 'K', 'B' };          // Gt Hall, Dining, Study, External, Kitchen, Basement.  (X1.00.Z translates to 0x00 and is a NULL device)
  const char sensorTypes[numSensorTypes] = { 'T', 'F', 'H', 'L', 'M', 'P', 'b', 'o', '1', '2', '3', '4', '5', '6', '7', '8' };  // Touch, fire, heat, light level, motion, presence, ibutton, open, open 1-8
  const char actorTypes[numActorTypes] = { 'Z', 'p', 'D', 'L', 'R', 'P', '1', '2' };    // dummy to avoid zero ref (G1.00.p), 5a power, door lock (was 'B'), lamp, relay, power, P1, P2
  const char charSensor = 'x';
  const byte openIdx = 7;      // All others after this are Open sensors
  const byte powerIdx = 5;      // All others after this are Power outlets
  const byte tempIdx = 2;
  byte bitCode;
  int i;
  
  switch (type) {
    case valToInt:
      bitstring = 0;
      if (device[0] == 'X') Serial.println("Null char");
      else {
        // Convert region ref.  Lots of casts to avoid "error: invalid operands of types 'unsigned int' and 'void*' to binary 'operator|'"
        bitstring |= ( (unsigned int) memchr(regionCodes, device[0], numRegionCodes) - (unsigned int) regionCodes ) << offsetRegion;    // 3 bit << 13 (MSB)
        
        // Convert zone number (reduce by 1) - max 8
        bitstring |= (atoi(device+1) - 1) << offsetZone;    // 3 bits << 10
        
        // Convert location - max 31 (0 reserved for whole zone).
        bitstring |= atoi(device+3) << offsetLocation;  // 5 bits << 5
        
        // Test if sensor
        if (device[6] == charSensor) {
          bitstring |= maskSensor;
          bitstring |= (unsigned int) memchr(sensorTypes, device[(device[8] == '\0') ? 7 : 8], numSensorTypes) - (unsigned int) sensorTypes;      // Caters for xo & xo1-8
        }
        else bitstring |= (unsigned int) memchr(actorTypes, device[(device[7] == '\0') ? 6 : 7], numActorTypes) - (unsigned int) actorTypes;      // Caters for P1 & P2  
      }
      
      return bitstring;
    case valToChar:
      if (bitstring) {
        // Get region ref
        device[0] = regionCodes[(bitstring & maskRegion) >> offsetRegion];      // 3-bit region is index into code string
        
        // Get zone number
        device[1] = ((bitstring & maskZone) >> offsetZone) + 0x31;        // ASCII '0' is 0x31
        
        device[2] = '.';
        
        // Get location
        bitCode = (bitstring & maskLocation) >> offsetLocation;
        device[3] = (bitCode) / 10 + 0x30;
        device[4] = (bitCode) % 10 + 0x30;
        
        device[5] = '.';
        
        // Get device type; terminate string with null
        i = bitstring & maskDeviceType;
        if (bitstring & maskSensor) {
          device[6] = charSensor; 
          if (i > openIdx) { device[7] = sensorTypes[openIdx]; device[8] = sensorTypes[i]; device[9] = '\0'; }
          else { device[7] = sensorTypes[i]; device[8] = '\0'; }
        }
        else {
          if (i > powerIdx) { device[6] = actorTypes[powerIdx]; device[7] = actorTypes[i]; device[8] = '\0'; }
          else { device[6] = actorTypes[i]; device[7] = '\0'; }
        }
      }
      else { device[0] = 'X'; device[1] = '\0'; Serial.println("Null bits"); }        // NULL device
      break;
    case valTestTemp:
      // Check if this is a temperature sensor
      return (bitstring & maskSensor) && ((bitstring & maskDeviceType) == tempIdx);
      break;
    case valTestSlow:      
      // Check if this is a slow sensor - exactly the same as temp test for the moment
      return (bitstring & maskSensor) && ((bitstring & maskDeviceType) == tempIdx);
      break;
  }
}


byte getDeviceIdx (char *deviceRefChar) {      // Returns variable (MSB = 1) or device (MSB = 0) index  
  if (deviceRefChar[0] == 'V') {
    int varNum = atoi(deviceRefChar+2);
    if (varNum > numVars) Serial.println("Vars out of range");
    return mask8BitMSB | varNum;
  }
  else {
    byte deviceIdx;
    unsigned int deviceRefBits = convertRefToBit(deviceRefChar);
    for (deviceIdx = 1; deviceIdx < numDevices; deviceIdx++) if(mapGet(deviceIdx, valRef) == deviceRefBits) return deviceIdx;
    Serial.println("Device not found");
  }
}




// Reading helper functions

unsigned int stackGet (byte deviceIdx, unsigned int element) { 
  unsigned int result = stackAccess (deviceIdx, element, NULL, readFlag); 
  return result;
}
  
  /*
  if (onWatchList(deviceIdx) && element == 0) { 
    Serial.print(deviceIdx, HEX);
    printRef(deviceIdx);
    Serial.print(" element = ");
    Serial.print(element, HEX);
    Serial.print(" result = 0x");
    Serial.print(result, HEX);
    if (result > 32) {
      Serial.print(" (");
      Serial.print(result, DEC);
      Serial.print(")");
    }
    Serial.print(" B");
    Serial.println(result, BIN);
  }
  */
  
  
  



void stackPush (byte deviceIdx, unsigned int value) { 
  stackAccess (deviceIdx, 0, value, writeFlag); 
}
  /*
  if (onWatchList(deviceIdx)) {
    Serial.print("In stackPush - heartbeat ");
    Serial.print(heartBeat);
    Serial.print(" watch ");
    Serial.print(deviceIdx);
    Serial.print(" value = 0x");
    Serial.print(value, HEX);
    if (value > 32) {
      Serial.print(" (");
      Serial.print(value, DEC);
      Serial.print(")");
    }
    Serial.print(" B");
    Serial.println(value, BIN);
  }
  */
  

unsigned int stackAccess (byte deviceIdx, unsigned int element, unsigned int value, byte flag) {          // Get element off or add element to stack (0 = TOS). NB: stacksize implied here as 8
  unsigned int result;
  static unsigned int readingHistory [maxReadings];          // Only used if StackMode == 1. Interpretation varies dependent on deviceType:
                                                    // For xH is temperature(C) x 10 (+/-), so 25.3 = 253 (negative values cast to unsigned)
                                                    // For xo/xL (open/light) is measured voltage of sensor x 100, so 5v = 500
                                                    // For xb is ID of button
                                                    // For time holds day:hour:minute psuedo codes see dhmAccess, with MSG == 1


  byte stack = mapGet(deviceIdx, valStack);    // Holds 8 on/off readings if StackMode == 0, else pointer into readingHistory
  
  if (mapGet(deviceIdx, valStackMode)) {
    unsigned int index = stack * stackSize;
    unsigned int offset = (mapGet(deviceIdx, valTOSIdx) + element) % stackSize;
    
    if (flag == readFlag) result = readingHistory [index + offset];
    else {
       unsigned int newOffset = (offset % stackSize != 0) ? offset - 1 : offset + 7;    // If not at top of stack (non-zero) then decrement, else rotate round    
       readingHistory [index + newOffset] = value;      // Store reading (overwrites oldest)  
       mapPut (deviceIdx, valTOSIdx, newOffset);
    }
  }
  else if (flag == readFlag) result = ((stack << element) >> 7) & maskLSB; else mapPut (deviceIdx, valStack, (stack >> 1) | ((value & 1) << 7));

  return result;
}



// ******** Evaluation array read/write ******************

unsigned int evalGet (byte evalIdx, byte type) { return evalAccess (evalIdx, type, NULL, readFlag); }
void evalPut (byte evalIdx, byte type, unsigned int value) { evalAccess (evalIdx, type, value, writeFlag); }

unsigned int evalAccess(byte evalIdx, byte type, unsigned int value, int flag) {            // Get and Put values from evalArray
  static unsigned long evalArray[maxEvals];                 // Union of bytes containing a, b, dest, exp; or len, ptr, dest, exp (for lists).  Choice determined by calcType
  static unsigned int turnOffArray[maxEvals/16];            // Bit array indicating how to handle result of evaluation - 1 = if result == TRUE then write FALSE to destination
                                                     // Ensure long constants are explicitly typed - http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1260807970
  const unsigned long maskA = 0xff000000UL;          // A = first argument, 8 bit index into deviceMap (MSB = 0), or variable (MSB = 1)
  const unsigned long maskB = 0x00ff0000UL;          // B = second argument, 8 bit index into deviceMap (MSB = 0), or variable (MSB = 1)
  const unsigned long maskLen = 0x7c000000UL;        // 5 bits -> num args in expression (max = 32)
  const unsigned long maskPtr = 0x03ff0000UL;        // 10 bits -> index into argArray (max = 1024)
  const unsigned long maskDest = 0x0000ff00UL;       // Where to put the result - index into deviceMap (MSB = 0), or variable (MSB = 1)
  const unsigned long maskCalc = 0x000000f0UL;       // 4 bit = 16 values - interpretation of arguments
  const unsigned long maskExp = 0x0000000fUL;        // 4 bit = 16 values - expression to evaluate
  const byte offsetA = 24;
  const byte offsetB = 16;
  const byte offsetLen = 26;
  const byte offsetPtr = 16;
  const byte offsetDest = 8;
  const byte offsetCalc = 4;
  const byte offsetExp = 0;

  unsigned long mask;
  unsigned int offset;
  
  if (evalIdx > maxEvals) Serial.println("Evals out of bounds");
  
  switch (type) {
    case valA:       mask = maskA; offset = offsetA; break;
    case valB:       mask = maskB; offset = offsetB; break; 
    case valLen:     mask = maskLen; offset = offsetLen; break;
    case valTurnOff: mask = 1 << (evalIdx % 16); offset = evalIdx % 16; break;
    case valPtr:     mask = maskPtr; offset = offsetPtr; break;    
    case valDest:    mask = maskDest; offset = offsetDest; break; 
    case valCalc:    mask = maskCalc; offset = offsetCalc; break;
    case valExp:     mask = maskExp; offset = offsetExp; break;  
    default: Serial.println("Unknown type (expUpdate)");
  }
  
  if (flag == readFlag) return (type == valTurnOff) ? (mask & turnOffArray[evalIdx / 16]) >> offset : (unsigned int) ((evalArray[evalIdx] & mask ) >> offset); 
  else if (type == valTurnOff) {
    turnOffArray[evalIdx / 16] &= ~mask;          // Clear bit
    turnOffArray[evalIdx / 16] |= (value << offset) & mask;        // Set bit
  }
  else {  
    evalArray[evalIdx] &= ~mask;                                                    // Clear
    evalArray[evalIdx] |= ((unsigned long)(value) << offset) & mask;                // Store 
  }
}



// ******** valCalc and valExp helper functions ******************

byte getCalcIdx(char *calcTypeChar) { return valTranslate (calcTypeChar, NULL, valCalc, valToInt); }
void getCalcChar(byte calcTypeIdx, char *calcTypeChar) { valTranslate (calcTypeChar, calcTypeIdx, valCalc, valToChar); }
byte getExpIdx(char *expTypeChar) { return valTranslate (expTypeChar, NULL, valExp, valToInt); }
char getExpChar(byte expTypeIdx) { return (char) valTranslate (NULL, expTypeIdx, valExp, valToChar); }
byte getExpListIdx(char *expTypeChar) { return valTranslate (expTypeChar, NULL, valListExp, valToInt); }
char getExpListChar(byte expTypeIdx) { return (char) valTranslate (NULL, expTypeIdx, valListExp, valToChar); }

unsigned int valTranslate(char *valTypeChar, unsigned int valTypeIdx, byte valConst, byte convType) {  
  const byte numCalcs = 14;
  const char *calcTypes[numCalcs] = { "ListE", "ListM", "!", "CURR", "PREV", "Avg", "Max", "Min", "ROfC", "Year", "Month", "Day", "Hour", "Minute"};  
          // List of evals; list of devices/variables; A qualifiers: not, current/latest, previous, average, max, min, rate of change; A replacements: now(year, month, day, hour, minute).  B is always current value (except in lists)
  const byte numExps = 12;
  const char expTypes[numExps] = { '=', '!', '>', '<', '+', '-', '*', '/', '&', '|', '[', ']' }; 
          // ==, !=, >, <, +, -, *, /, &&, ||, current time between/not between A & B interpreted as DHM (all result in TRUE or FALSE, except arithmetic operators which yield int result)
  const byte numListExps = 10;
  const char listExpTypes[numListExps] = { '=', '!', 'A', 'x', '+', 'n', '*', NULL , '&', '|' }; 
          // ==, !=, Avg, Max, +, Min, *, noop , &&, ||

  switch (valConst) {
    case valCalc:
      switch (convType) {
        case valToChar:  if (valTypeIdx > numCalcs) Serial.println("Calc idx OF"); else strcpy (valTypeChar, calcTypes[valTypeIdx]);   
        case valToInt:   for (int j=0; j < numCalcs; j++) if (strstr(calcTypes[j], valTypeChar)) return j; Serial.println("Calc NF"); break;
        default: Serial.println("Bad C");
      }
      break;
    case valExp:
      switch (convType) {
        case valToChar:  if (valTypeIdx > numExps) Serial.println("Exp idx OF"); else return (unsigned int) expTypes[valTypeIdx];   
        case valToInt:   for (int j=0; j < numExps; j++) if (expTypes[j] == valTypeChar[0]) return j; Serial.println("Exp NF"); break;
        default: Serial.println("Bad E");
      }
      break;
    case valListExp:
      switch (convType) {
        case valToChar:  if (valTypeIdx > numListExps) Serial.println("Exp list idx OF"); else return (unsigned int) listExpTypes[valTypeIdx];   
        case valToInt:   for (int j=0; j < numListExps; j++) if (listExpTypes[j] == valTypeChar[0]) return j; Serial.println("List Exp NF"); break;
        default: Serial.println("Bad LE");
      }
      break;
    default: Serial.println("Unknown valConst");
  }
}


// ******** Arguments array read/write ******************

byte argGet(unsigned int argIdx) { return argAccess (argIdx, NULL, readFlag); }
void argPut(unsigned int argIdx, byte value) { argAccess (argIdx, value, writeFlag); }

unsigned int argAccess(unsigned int argIdx, byte value, int flag) {            // Get and Put values from argArray
  static byte argArray[maxArgs];                            // Ordered array of arguments to be evaluated for && or ||; args are indexes into evalArray 
                                                            // maskPtr indexes the start argument; maskLen is the number of arguments
  if (argIdx > maxArgs) Serial.println("Args out of bounds");
  
  if (flag == readFlag) return argArray[argIdx];
  else argArray[argIdx] = value;
}

  

// ******** DHM read/write ******************

unsigned int dhmGet(unsigned int dhmVal, byte type) { return dhmAccess (&dhmVal, type, NULL, readFlag); }
void dhmPut(unsigned int *dhmVal, byte type, unsigned int value) { dhmAccess (dhmVal, type, value, writeFlag); }
unsigned int dhmNow() {
  unsigned int result = 0;
  dhmPut (&result, valDay, weekday());
  dhmPut (&result, valHour, hour());
  dhmPut (&result, valMinute, minute());
  return result;
}
unsigned int dhmMake(unsigned int dhmDay, unsigned int dhmHour, unsigned int dhmMinute) {
  unsigned int result = 0;
  dhmPut(&result, valDay, dhmDay);
  dhmPut(&result, valHour, dhmHour);
  dhmPut(&result, valMinute, dhmMinute);
  return result;
}

unsigned int dhmAccess(unsigned int *dhmVal, byte type, unsigned int value, int flag) {            // Get and Put values from evalArray
  unsigned int mask, offset;
  const unsigned int maskDay = B1111 << offsetDay;     // 4 bits: 0 = everyday, 1-7 are days of week (1 = Sun), 8 = Mon-Thu, 9 = Mon-Fri, 10 = Sat/Sun
  // (B01111000 * 256) + 0x00;
  const unsigned int maskHour = B11111 << offsetHour;       // 5 bits: 0-23 are hours
  // (B00000111 * 256) + B11000000;  
  const unsigned int maskMin = B111111;                            // 6 bits: 0-59 are minutes
  
  switch (type) {
    case valDay:   mask = maskDay; offset = offsetDay; if (value > 10) Serial.println("Day > 10"); break;
    case valHour:  mask = maskHour; offset = offsetHour; if (value > 23) Serial.println("Hour > 23"); break; 
    case valMinute:   mask = maskMin; offset = 0; if (value > 59) Serial.println("Mins > 59"); break;
    default: Serial.print("Unknown type (DHM) = "); Serial.println(type, HEX);
  }
  
  if (flag == readFlag) return (unsigned int) (((*dhmVal) & mask ) >> offset);
  else {
    (*dhmVal) &= ~mask;                                   // Clear
    (*dhmVal) |= (value << offset) & mask;                // Store 
  }
}


void initialiseDevices() {   
  byte deviceIdx, deviceType;
  int tempSensorIdx = 0;
  int numTempSensors; // Number of temperature devices found
  
  // Set up appropriate pins for each device, based on device type (assuming a valid pin)
  for (deviceIdx = 1; deviceIdx < numDevices; deviceIdx++) {
    if (mapGet(deviceIdx, valPin) != pinNoOp) {
      if (mapGet(deviceIdx, valSensor)) {      // { 'T', 'F', 'H', 'L', 'M', 'P', 'b', 'o', '1', '2', '3', '4', '5', '6', '7', '8' }
        if (isTempSensor (deviceIdx) ) {
          oneWire[tempSensorIdx].setPin(mapGet(deviceIdx,valPin));            // Set pin using extra library method
          mapPut(deviceIdx, valPin, tempSensorIdx);                           // Replace pin with ref to relevant temp object (which has the pin)      
   
          tempSensor[tempSensorIdx].begin();      // Initialise One-Wire and set resolution of each device
          numTempSensors = tempSensor[tempSensorIdx].getDeviceCount();  
          if (numTempSensors > MAXTEMPDEVICES) {
            Serial.print("Max temp devices per pin exceeded - found: ");
            Serial.println(numTempSensors);
            numTempSensors = MAXTEMPDEVICES;
          }
          for(int i=0; i < numTempSensors; i++) {
            if(tempSensor[tempSensorIdx].getAddress(tempDeviceAddress[tempSensorIdx], i)) { 
              tempSensor[tempSensorIdx].setResolution(tempDeviceAddress[tempSensorIdx], TEMPERATURE_PRECISION);
            }
            else Serial.print("Unable to get address");
          } 
          tempSensor[tempSensorIdx].setWaitForConversion(false);      // Allows async operation to speed things up
          
          if (tempSensorIdx < maxTempSensors) tempSensorIdx++; else Serial.println("Temp sensor limit reached");
        }
        else pinMode(mapGet(deviceIdx, valPin), INPUT);      // All other sensors are inputs (digital or analogue)
      }
      else pinMode(mapGet(deviceIdx, valPin),OUTPUT);       // { 'Z', 'p', 'D', 'L', 'R', 'P', '1', '2' }
    }
  }
      
  // Set pointers to appropriate handlers for each sensor type; sensor type indexes the appropriate handler
  getSensorReading[0] = getSensTouch;
  getSensorReading[1] = getSensFire;
  getSensorReading[2] = getSensTemp;
  getSensorReading[3] = getSensLight;
  getSensorReading[4] = getSensMotion;
  getSensorReading[5] = getSensPresence;
  getSensorReading[6] = getSensIButton;
  for (int i=7; i < maxSensorTypes; i++) getSensorReading[i] = getSensOpen;
   
  // Initialise time  
  setSyncInterval(const_NTPRefreshInterval);
  setSyncProvider(getNtpTime);
  for (int i = 0; i < 5 && timeStatus() != timeSet; i++) delay(500);    // Wait a while to see if time set
 
  if (timeStatus() == timeSet) {                      // Adjust for BST if needed
    time_t startBST_t, endBST_t;
    startBST_t = timeValue(1,0,0,31, 3, year());      // Get 01:00:00 hrs on 31 Mar of current year
    startBST_t = startBST_t - (weekday(startBST_t) - 1) * SECS_PER_DAY;    // Deduct seconds to the last Sunday in March
    endBST_t = timeValue(1,0,0,31, 10, year());      // Repeat for 31 Oct
    endBST_t = endBST_t - (weekday(endBST_t) - 1) * SECS_PER_DAY;  
    setBSTAdjust(startBST_t,endBST_t, 1);        // Set BST adjustment window - 1 hour: extra routine added to Time library
  }
}


unsigned int getSensTouch (unsigned int pin, unsigned int handler) {
  return 1;
}

unsigned int getSensFire (unsigned int pin, unsigned int handler) {
  return (unsigned int) 1;
}

unsigned int getSensTemp (unsigned int pin, unsigned int handler) {    // Returns 1/10ths C, with MSB = 1 if negative.  Called twice in successive heartbeats to trigger and then capture
  unsigned int response;
  float tempC;
 
  switch (handler) {
    case valSlowCapture:               // For a slow sensor (eg temp) signifies the second heartbeat - get the reading triggered by first heartbeat
      tempC = tempSensor[pin].getTempC(tempDeviceAddress[pin]); 
      response = (int) (tempC * 10);
      break;
    default:
      tempSensor[pin].requestTemperatures();                 // Trigger a read.  Pin was set as temperature object idx in initialiseDevices
      response = 0;
  }
  
  return response;
}

unsigned int getSensLight (unsigned int pin, unsigned int handler) {
  long analogValue = analogRead(pin);
  analogValue = analogValue * arduinoVoltage * 100 / analogRange;
  return (unsigned int) analogValue;
}

unsigned int getSensMotion (unsigned int pin, unsigned int handler) {
  return 1;
}

unsigned int getSensPresence (unsigned int pin, unsigned int handler) {
  return (unsigned int) digitalRead(pin) == 0;
}

unsigned int getSensIButton (unsigned int pin, unsigned int handler) {
  return (unsigned int)random(500);
}

unsigned int getSensOpen (unsigned int pin, unsigned int handler) {
  long analogValue = analogRead(pin);
  analogValue = analogValue * arduinoVoltage * 100 / analogRange;
  return (unsigned int) analogValue;
}


void reply404(Client client) {
  client.println("HTTP/1.1 404 Not Found");
  client.println("Content-Type: text/html");
  client.println();
  client.println("<h2>File Not Found!</h2>");
}

void stopClient(Client client) {
  delay(2);
  client.stop();
}


/**
@author: Mathias Van Malderen (tux4life)
*/

void stoupper(char *s) {
    for(; *s; s++)
        if(('a' <= *s) && (*s <= 'z'))
            *s = 'A' + (*s - 'a');
}

void stolower(char *s) {
    for(; *s; s++)
        if(('A' <= *s) && (*s <= 'Z'))
            *s = 'a' + (*s - 'A');
}

/***************** TIME STUFF - FROM TimeNTP *****************/

void digitalClockDisplay(time_t tm){
 // digital clock display of the time
 Serial.print(hour(tm));
 printDigits(minute(tm));
 printDigits(second(tm));
 Serial.print(" ");
 Serial.print(day(tm));
 Serial.print(" ");
 Serial.print(month(tm));
 Serial.print(" ");
 Serial.print(year(tm));
 Serial.println();
}

void printDigits(int digits){
 // utility function for digital clock display: prints preceding colon and leading 0
 Serial.print(":");
 if(digits < 10) Serial.print('0');
 Serial.print(digits);
}


unsigned long getNtpTime() {
  unsigned long epoch = 0;
  static unsigned long prevEpoch = 0;
  static byte numGoes = 0;
  const unsigned long secsSince1970 = (CURRENT_YEAR - 1970) * SECS_PER_YEAR;    // Used as a rough sense test
  const unsigned long secsToExpiry = (EXPIRY_YEAR - 1970) * SECS_PER_YEAR;    // Used as a rough sense test
  const unsigned long seventyYears = 2208988800UL;    // In Arduino Time format time starts on Jan 1 1970. In seconds, that's 2208988800 after 1900
  
  sendNTPpacket(timeServer); // Send request for time
  
  unsigned long startMillis = millis();    
  while( millis() - startMillis < 1000) { // wait up to one second for the response
    // wait to see if a reply is available
    if ( UdpNTP.available() ) {  
      UdpNTP.readPacket(packetBuffer,NTP_PACKET_SIZE);  // read the packet into the buffer

      //the timestamp starts at byte 40 of the received packet and is four bytes, or two words, long. First, esxtract the two words:
      unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
      unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);  
      // combine the four bytes (two words) into a long integer - this is NTP time (seconds since Jan 1 1900):
      unsigned long secsSince1900 = highWord << 16 | lowWord;      
      // now convert NTP time into Arduino time - subtract seventy years:
      epoch = secsSince1900 - seventyYears;
      
      // do some credibility tests
      if (epoch < secsSince1970 || epoch > secsToExpiry) { epoch = 0; sendLog("Wild time"); }   // Basic test - check it's after this software was built and before the end of time
      else if (epoch <= prevEpoch && ++numGoes < 8) {       // Fine-grain test - check it's after the previous time, but only give it 8 goes just in case a previous reading was artifically high
        epoch = prevEpoch + (unsigned long)const_NTPRefreshInterval; 
        sendLog("Time travel"); 
      }  
      else {        // Assume the time is valid
        // Set previous time to now
        prevEpoch = epoch;
        numGoes = 0;
      }
    }
  }
  
  if (epoch == 0) Serial.println("No response from timeserver");
  return epoch;   // return 0 if unable to get the time
}  

// send an NTP request to the time server at the given address
unsigned long sendNTPpacket(byte *address) {
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now send a packet requesting a timestamp:   
  UdpNTP.sendPacket( packetBuffer, NTP_PACKET_SIZE,  address, 123); //NTP requests are to port 123
}
 
 /************** ONE-WIRE CODE ***************/
 
 void refreshTemperatures () {
   /*
  p_tempSensors[0].requestTemperatures(); 
  for(int i=0;i<numTempSensors; i++) {
    if(p_tempSensors[0].getAddress(tempDeviceAddress, i)) tempC[i] = p_tempSensors[0].getTempC(tempDeviceAddress);         	
  }
  */
}

void printTemp(float tempC, char *s) {
    int tempCInt = int(tempC);
    int tempCFrac = int(tempC * 100) - (tempCInt * 100);
  
    sprintf(s,"%d.%02d c",tempCInt,tempCFrac);
}
  
void switchRelay(int ledPin) {
     char reply[30];

     if (ledState == 1) {
          digitalWrite(ledPin,LOW);
          Serial.println("Switching heating off");
          ledState = 0;
        }
        else {
          digitalWrite (ledPin,HIGH);
          Serial.println("Switching heating on");
          ledState = 1;
        }
        return;
}
   
time_t timeValue(int hr,int min,int sec,int dy, int mnth, int yr){
 // Adapted from timeNTP library
 // year can be given as full four digit year or two digts (2010 or 10 for 2010);  
 //it is converted to years since 1970
 
  tmElements_t tm;
 
  if( yr > 99)
      yr = yr - 1970;
  else
      yr += 30;  
  tm.Year = yr;
  tm.Month = mnth;
  tm.Day = dy;
  tm.Hour = hr;
  tm.Minute = min;
  tm.Second = sec;
  return makeTime(tm);
}   

/******************* DEBUG UTILITIES *****************************/

void echoLine(char *clientline) {
//  Serial.print("Client line = ");
//  Serial.println(clientline);
}


boolean onWatchList (byte deviceIdx) {
  return false;
  const byte numWatchListDevs = 2;
  const byte numWatchListVars = 1;
  byte watchListDevs[numWatchListDevs] = { 4, 5 };
  byte watchListVars[numWatchListVars] = { 0 };

  if (deviceIdx & mask8BitMSB) for (int i = 0; i < numWatchListVars; i++) { if (watchListVars[i] == (deviceIdx & ~mask8BitMSB)) return true; }
  else for (int i = 0; i < numWatchListDevs; i++) { if (watchListDevs[i] == deviceIdx) return true; }
  
  return false;
}


void printRef (byte deviceIdx) {
  char element[20];
  if (deviceIdx & mask8BitMSB) {
    element[0] = 'V';
    element[1] = '.';
    element[2] = (deviceIdx & ~mask8BitMSB) / 100 + 0x30;
    element[3] = ((deviceIdx & ~mask8BitMSB) % 100) / 10 + 0x30;
    element[4] = (deviceIdx & ~mask8BitMSB) % 10 + 0x30;    
    element[5] = '\0';
  }
  else convertRefToChar(mapGet(deviceIdx, valRef), element);
  sendLog(element);
}

  void sendLog (char *buffer) { 
    UdpLog.sendPacket (buffer, logIP, UdpLogPort);
  }
