#include "../include/CAN.h"


CAN::CAN(CAN_MB mailBox)
{
  m_CAN.enableMBInterrupt((FLEXCAN_MAILBOX) mailBox);

  
  m_CAN.setMB( (FLEXCAN_MAILBOX)CAN::MAIN_BODY,     TX, STD); // Set the mailbox to transmit
  m_CAN.setMB( (FLEXCAN_MAILBOX)CAN::JETSON,        TX, STD); // Set the mailbox to transmit
  m_CAN.setMB( (FLEXCAN_MAILBOX)CAN::SCIENCE_BOARD, TX, STD); // Set the mailbox to transmit
  m_CAN.setMB( (FLEXCAN_MAILBOX)CAN::ARM_BOARD,     TX, STD); // Set the mailbox to transmit

  m_CAN.setMB( (FLEXCAN_MAILBOX)mailBox, RX, STD); // Set the mailbox to receive

  // Start the CAN bus
  m_CAN.begin(); // <- This is needed

  // Set the the configuration of the CAN bus
  CANFD_timings_t config;
  config.clock = CLK_24MHz;
  config.baudrate = 500000;
  config.baudrateFD = 500000;
  config.propdelay = 190;
  config.bus_length = 1;
  config.sample = 70;
  m_CAN.setBaudRate(config); // <- This is needed

  //This is the old way of setting the baud rate for CAN 2.0
  //m_CAN.setBaudRate(500000); 

  // Set the interrupt to call the canSniff function
  m_CAN.onReceive((FLEXCAN_MAILBOX)mailBox, &CAN::CANSniff);
  
}

// Create an object dictionary to store the messages
CAN::ObjectDictionary CAN::m_objectDict;
// Create a message flag map to track new messages
CAN::MessageFlag CAN::m_messageFlag;

// Function to be called when a message is recieved
void CAN::CANSniff(const CANFD_message_t &msg)
{
  Message_ID id = static_cast<Message_ID>(msg.id);

  // Check if the ID exists in the m_objectDict map
  if (m_objectDict.find(id) == m_objectDict.end()) {
    // This is the first time the message with this ID is being received
    m_messageFlag[id] = true;
  }
  // Check if the message has changed and update message flags
  else
  {
    bool newMessage = false;
    const auto &existingMessage = m_objectDict[id];

    for(unsigned int i=0; i<sizeof(msg.buf); i++)
    {
      if(msg.buf[i] != existingMessage.buf[i])
      {
        newMessage = true;
        break;
      }
    }
    m_messageFlag[id] = newMessage;
  }

  m_objectDict[id] = msg;
}

// Send a message to the CAN bus
void CAN::SendMessage( CAN_MB mailBox, Message_ID id, uint8_t message[8])
{
  // Create a message
  CANFD_message_t msg;

  // Set the message ID to 0x123
  msg.id = id;
  
  // Set the message length to 8
  msg.len = 8;

  // Set the message buffer to the m_message buffer
  for(int i = 0; i < 8; i++)
  {
    msg.buf[i] = message[i];
  }

  // Add the message to the object dictionary
  m_objectDict[static_cast<Message_ID>(msg.id)] = msg;

  // Send the message
  m_CAN.write( (FLEXCAN_MAILBOX)mailBox, msg);

}

// Retrieve a message from the object dictionary
CANFD_message_t CAN::GetMessage(Message_ID id)
{
  return m_objectDict[id];
}

bool CAN::NewMessage(Message_ID id)
{
    auto it = m_messageFlag.find(id);
    if (it != m_messageFlag.end()) {
        return it->second;
    } else {
        // No message has been received yet at this ID
        return false;
    }
}
