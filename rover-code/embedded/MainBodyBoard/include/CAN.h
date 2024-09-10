#ifndef CAN_H
#define CAN_H
#include <FlexCAN_T4.h>
#include <unordered_map>

class CAN
{
    public: 
        typedef enum {
            E_STOP = (uint32_t) 0,
            TARGET_VELOCITY = (uint32_t) 1,
            CURRENT_VELOCITY = (uint32_t) 2,
            
        } Message_ID;

        typedef std::unordered_map<Message_ID, CAN_message_t> ObjectDictionary;   
        typedef std::unordered_map<Message_ID, bool> MessageFlag;

        enum CAN_MB{
            JETSON = FLEXCAN_MAILBOX::MB0, 
            MAIN_BODY = FLEXCAN_MAILBOX::MB1, 
            SCIENCE_BOARD = FLEXCAN_MAILBOX::MB2,
            ARM_BOARD = FLEXCAN_MAILBOX::MB3
            };

        CAN();

        CAN(CAN_MB mailBox);

        void sendMessage( CAN_MB mailBox, Message_ID id, uint8_t message[8]);
        CAN_message_t getMessage(Message_ID id);
        bool newMessage(Message_ID id);
        // static bool IsEStop();
        static bool IsEStop(const CAN_message_t &msg);
        void TEST();
    private:
        FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> m_CAN;
        static ObjectDictionary m_objectDict;
        static MessageFlag m_messageFlag;
        static void CANSniff(const CAN_message_t &msg);
        
        // static bool m_isEStop;
        
};
#endif