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


void MSG_EnableRX(const bool enable);
void MSG_StorePacket(const uint16_t interrupt_bits);
void MSG_Init();
void MSG_ProcessKeys(KEY_Code_t Key, bool bKeyPressed, bool bKeyHeld);
void MSG_SendPacket(union DataPacket packet);
void MSG_FSKSendData();

#endif

#endif