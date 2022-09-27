#include <Arduino.h>
#include <FlexCAN.h>
#include <kinetis_flexcan.h>
#include <DMAChannel.h>
#include <LittleFS.h>
#include <EEPROM.h>
#include <SPIFlash.h>
#include <IntervalTimer.h>



#include <deque>
using std::deque;

// For if/when to print to serial the current deque
bool CandequePrint = true;
bool CANdubugPrint = true;
elapsedMillis dequeprintMillis;
elapsedMillis CANdebugPrintMillis;

// Interupt timer for logging
IntervalTimer logSDInterval;
IntervalTimer readCANInterval;

// "humanReadable" means printing chars not raw data bytes and with delimeters. Should be safe to use with any CAN2 only system but several times less data efficient.
bool humanReadableLogStream = true;
union CAN_ID_Union
{
  uint32_t idInput;
  uint8_t idByteArrayOutput[4];
};


//------------------------------------------------------------------------------


///// CAN /////
// Can_message_t returns 16 bytes from sizeof()
CAN_message_t message;
CAN_message_t rxmsg;
CAN_message_t extended;
bool canReadReturn;

bool NewCommandMessage{false};
bool NewConfigMessage{false};

CAN_filter_t allowExtendedIDFilter;
CAN_filter_t allowStandardIDFilter;

CAN_stats_t Can0stats;

//FlexCan3Controller Can2msgController;
//SerialUSBController SerialUSBdataController;

const int CAN2busSpeed = 500000; // CAN2.0 baudrate - do not set above 500000 for full distance run bunker to pad

std::deque<CAN_message_t> Can2dequeLogAll;
//std::deque<CAN_message_t>::iterator Can2StreamIt;
size_t elementReadoutSize = 16;

void serialPrintCAN_message_t(CAN_message_t msgIn)
{
    Serial.print(msgIn.id);
    Serial.print(" : ");
    for (size_t i = 0; i < msgIn.len; i++)
    {
    Serial.print(msgIn.buf[i]);
    Serial.print(" : ");
    }
  Serial.println("");
  Serial.flush();
}

void serialPrintCAN_message_t(CAN_message_t msgIn, bool printSTDmsg)
{

    CAN_ID_Union idSplit;
    idSplit.idInput = msgIn.id;
    
    // Switch Serial.print to Serial.write for more efficient but less readable data, requires splitting bytes of id field.
    size_t printLen;
    if (humanReadableLogStream)
    {
      Serial.print(msgIn.id);
      Serial.print(":");
      // Print
      if (printSTDmsg)
      {
        printLen = 8;
      }
      else {printLen = msgIn.len;}

        
      for (size_t j = 0; j < msgIn.len; j++)
      {
      //if (j != 0){Serial.print(",");}
      Serial.print(msgIn.buf[j]);
      Serial.print(",");
      }
      // if msgIn.len != printlLen, then print 0 for unused bytes
      for (size_t i = msgIn.len; i < printLen; i++)
      {
      //if (i != 0){Serial.print(",");}
      Serial.print(0);
      Serial.print(",");
      }
      //Serial.println();
    }
    else
    // Format uses standard length printing as extended ID, 8 byte frames but no delimeters and raw data not ASCII chars
    // This does waste some bandwidth, but not much because standardizing number of bytes every frame eliminates fields for ID size and number of data bytes.
    {
      //Serial.print("check ID");
      Serial.write(idSplit.idByteArrayOutput[3]);
      Serial.write(idSplit.idByteArrayOutput[2]);
      Serial.write(idSplit.idByteArrayOutput[1]);
      Serial.write(idSplit.idByteArrayOutput[0]);
      //Serial.print("should have read: ");
      //Serial.print(msgIn.id);
      //Serial.print(" ");
      // Print
      if (printSTDmsg)
      {
        printLen = 8;
      }
      else {printLen = msgIn.len;}

      for (size_t j = 0; j < msgIn.len; j++)
      {
      Serial.write(msgIn.buf[j]);
      }
      // if msgIn.len != printlLen, then print 0 for unused bytes.
      for (size_t i = msgIn.len; i < printLen; i++)
      {
      Serial.write(0);
      }
    }
}

void serialPrintCan2dequeLog(bool removeBuffer)
{
  if (!Can2dequeLogAll.empty())
  {
    // If the arg bool is false, just print the deque without removing from it
    if (!removeBuffer)
    {
    Serial.println("Can2dequeLogAll readout begin: ");
    for (auto canIt : Can2dequeLogAll)
    {
      serialPrintCAN_message_t(canIt, true);
    }
    Serial.println("Can2dequeLogAll readout end: ");
    }
    // If the arg bool is true, write to serial as the logging output stream and remove sent data from deque
    else
    {
      // If deque size small enough to read whole thing
      if (Can2dequeLogAll.size() <= elementReadoutSize)
      {
        for (auto Can2StreamIt = Can2dequeLogAll.begin(); Can2StreamIt != Can2dequeLogAll.end(); Can2StreamIt++)
        {
          serialPrintCAN_message_t(*Can2StreamIt, true);
          //Can2dequeLogAll.erase(Can2StreamIt);
        }
        // Since size is smaller than our element print line, can just clear whole deque at the end?
        Can2dequeLogAll.clear();
      }
      else
      {
        //for (auto Can2StreamIt = Can2dequeLogAll.begin(); std::distance(Can2dequeLogAll.begin(), Can2StreamIt) <= elementReadoutSize; Can2StreamIt++)
        for (auto Can2StreamIt = Can2dequeLogAll.begin(); Can2StreamIt <= (Can2dequeLogAll.begin() + elementReadoutSize + 1); Can2StreamIt++)
        {
          serialPrintCAN_message_t(*Can2StreamIt, true);
        }
        Can2dequeLogAll.erase(Can2dequeLogAll.begin(),Can2dequeLogAll.begin() + elementReadoutSize);
      }
        
      Serial.println("");
      Serial.flush();
    }
  }
}


void testIntFunc()
{
  Serial.print("did Interval run: micros: ");
  Serial.print(micros());
  Serial.println("");
}

void CanIntervalRead()
{
  // For loop pulls the number of CAN frames Can0.available() returns as in the rx ring buffer
  for (size_t i = 0; i < Can0.available(); i++)
  {
  //cli();  // disables interrupts
    // Read message into rxmsg struct
    canReadReturn = Can0.read(rxmsg);
    // if Can0.read returns 0 no message was read, break before writing rxmsg to deque
    if (!canReadReturn)
    {
      break;
    }
    // Add rxmsg to the end of the deque
    Can2dequeLogAll.push_back(rxmsg);
  //sei();  // enables interrupts
  }
  
  // Print the deque if bool is true and polls true vs timer
  if (CandequePrint && (dequeprintMillis >= 500))
  {
  // Print entire deque of Can frames
  //serialPrintCan2dequeLog(false);
  dequeprintMillis = 0;
  }
}

void setup() 
{

  // CAN0 - FlexCAN 2.0 bus
  //Can0.setTxBufferSize(64);
  Can0.setRxBufferSize(5000);
  Can0.begin(CAN2busSpeed);
  allowExtendedIDFilter.flags.extended = 1; // Sets mailboxes to accept extended ID frames
  allowExtendedIDFilter.flags.remote = 0; // Sets mailboxes to ignore RTR frames
  allowStandardIDFilter.flags.extended = 0; // Sets mailboxes to reject extended ID frames
  allowStandardIDFilter.flags.remote = 0; // Sets mailboxes to ignore RTR frames
  
  
  //allMbFilter.id = 0; // Sets mailboxes to accept ID??? frames, I think if you set an ID here is filters that ID out?

  //set all mb filters
  for (uint8_t filterNum = 5; filterNum < 16;filterNum++){
    Can0.setFilter(allowExtendedIDFilter,filterNum); 
  }
  for (uint8_t filterNum = 0; filterNum < 5;filterNum++){
    Can0.setFilter(allowStandardIDFilter,filterNum); 
  }

  // SD logging, perhaps don't run on interval but do it constantly. Maybe I have a deque size trigger and an interval trigger to call it.
  //logSDInterval.begin(testIntFunc, 500000);
  //logSDInterval.priority(124);
  // Can has mailboxes and ring buffer, as long as ring buffer isn't overrun don't need to read it more frequently
  readCANInterval.begin(CanIntervalRead,6400);
  readCANInterval.priority(120);

  Can0.startStats();
}

void loop() 
{
if (CANdebugPrintMillis >= 100 && CANdubugPrint)
{
  Can0stats = Can0.getStats();
  Serial.print("Can0stats.ringRxMax? ");
  Serial.println(Can0stats.ringRxMax);
  Serial.print("Can0stats.ringRxHighWater? ");
  Serial.println(Can0stats.ringRxHighWater);
  for (size_t i = 0; i < 16; i++)
  {
    Serial.print(" mb:  ");
    Serial.print(i);
    Serial.print(" use count:  ");
    Serial.print(Can0stats.mb->refCount);
    Serial.print(" overrun count:  ");
    Serial.println(Can0stats.mb->overrunCount);
  }
}

// 
if (dequeprintMillis >= 100 || Can2dequeLogAll.size() >= (elementReadoutSize + 1))
{
serialPrintCan2dequeLog(true);
dequeprintMillis = 0;
}

/*   // For loop pulls the number of CAN frames Can0.available() returns as in the rx ring buffer
  for (size_t i = 0; i < Can0.available(); i++)
  {
  cli();  // disables interrupts
    // Read message into rxmsg struct
    canReadReturn = Can0.read(rxmsg);
    // if Can0.read returns 0 no message was read, break before writing rxmsg to deque
    if (!canReadReturn)
    {
      break;
    }
    // Add rxmsg to the end of the deque
    Can2dequeLogAll.push_back(rxmsg);
  sei();  // enables interrupts
  }
  
  // Print the deque if bool is true and polls true vs timer
  if (CandequePrint && (dequeprintMillis >= 500))
  {
  // Print entire deque of Can frames
  serialPrintCan2dequeLog();
  dequeprintMillis = 0;
  }
 */
}