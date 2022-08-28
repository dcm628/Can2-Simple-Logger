#include <Arduino.h>
#include <FlexCAN.h>
#include <kinetis_flexcan.h>
#include <SD.h>
#include <LittleFS.h>
#include <EEPROM.h>
#include <SPIFlash.h>
#include <IntervalTimer.h>

#include <deque>
using std::deque;

LittleFS_SPIFlash flash1;

SDClass sd;
File sd_log_file;


///// CAN /////
// Can_message_t returns 16 bytes from sizeof()
CAN_message_t message;
CAN_message_t rxmsg;
CAN_message_t extended;
bool canReadReturn;

bool CANSensorReportConverted = false;
bool NewCommandMessage{false};
bool NewConfigMessage{false};

CAN_filter_t allMbFilter;

//FlexCan3Controller Can2msgController;
//SerialUSBController SerialUSBdataController;

const int CAN2busSpeed = 500000; // CAN2.0 baudrate - do not set above 500000 for full distance run bunker to pad

std::deque<CAN_message_t> Can2dequeLogAll;

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
}

void serialPrintCan2dequeLog()
{
  Serial.println("Can2dequeLogAll readout begin: ");
  for (auto canIt : Can2dequeLogAll)
  {
    serialPrintCAN_message_t(canIt);
  }
  Serial.println("Can2dequeLogAll readout end: ");
}


void setup() 
{

  // CAN0 - FlexCAN 2.0 bus
  Can0.begin(CAN2busSpeed);
  allMbFilter.flags.extended = 1; // Sets mailboxes to accept extended ID frames
  allMbFilter.flags.remote = 0; // Sets mailboxes to ignore RTR frames
  //allMbFilter.id = 0; // Sets mailboxes to accept ID??? frames, I think if you set an ID here is filters that ID out?


}

void loop() 
{
  // For loop pulls the number of CAN frames Can0.available() returns as in the rx ring buffer
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
  
  // Print entire deque of Can frames
  serialPrintCan2dequeLog();

  //Serial.print("sizeof(CAN_message_t)");
  //Serial.print(sizeof(CAN_message_t));
  //Serial.println();
}