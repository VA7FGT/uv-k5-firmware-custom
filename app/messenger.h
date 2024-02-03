#ifndef APP_MSG_H
#define APP_MSG_H

#ifdef ENABLE_MESSENGER

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "driver/keyboard.h"

enum {
	NONCE_LENGTH = 13,
	PAYLOAD_LENGTH = 30
};

typedef enum KeyboardType {
	UPPERCASE,
  	LOWERCASE,
  	NUMERIC,
  	END_TYPE_KBRD
} KeyboardType;

extern KeyboardType keyboardType;
extern uint16_t gErrorsDuringMSG;
extern char cMessage[PAYLOAD_LENGTH];
extern char rxMessage[4][PAYLOAD_LENGTH + 2];
extern uint8_t hasNewMessage;
extern uint8_t keyTickCounter;

typedef enum MsgStatus {
    READY,
    SENDING,
    RECEIVING,
} MsgStatus;

typedef enum PacketType {
    MESSAGE_PACKET = 100u,
    ENCRYPTED_MESSAGE_PACKET,
    ACK_PACKET,
    INVALID_PACKET
} PacketType;

// Modem Modulation                             // 2024 kamilsss655
typedef enum ModemModulation {
    MOD_FSK,
    MOD_AFSK,
    MOD_NOAA_SAME
} ModemModulation;

// Modem Baud Rate                             // 2024 kamilsss655
// lower baud provides better reliability in bad conditions
typedef enum ModemBaudRate {
    MOD_BAUD_450,
    MOD_BAUD_550,
    MOD_BAUD_700,
    MOD_BAUD_1200
} ModemBaudRate;

// Data Packet definition                            // 2024 kamilsss655
union DataPacket
{
  struct{
    uint8_t header;
    uint8_t payload[PAYLOAD_LENGTH];
    unsigned char nonce[NONCE_LENGTH];
    // uint8_t signature[SIGNATURE_LENGTH];
  } data;
  // header + payload + nonce = must be an even number
  uint8_t serializedArray[1+PAYLOAD_LENGTH+NONCE_LENGTH];
};

// MessengerConfig                            // 2024 kamilsss655
typedef union {
  struct {
    uint8_t
      receive    :1, // determines whether fsk modem will listen for new messages
      ack        :1, // determines whether the radio will automatically respond to messages with ACK
      encrypt    :1, // determines whether outgoing messages will be encrypted
      unused     :1,
      modulation :2, // determines FSK modulation type
      baud       :2; // determines FSK baud rate
  } data;
  uint8_t __val;
} MessengerConfig;

void MSG_EnableRX(const bool enable);
void MSG_StorePacket(const uint16_t interrupt_bits);
void MSG_Init();
void MSG_ProcessKeys(KEY_Code_t Key, bool bKeyPressed, bool bKeyHeld);
void MSG_SendPacket();
void MSG_FSKSendData();
void MSG_ClearPacketBuffer();
void MSG_SendAck();
void MSG_HandleReceive();
void MSG_Send(const char *cMessage);
void MSG_ConfigureFSK(bool rx);

#endif

#endif
