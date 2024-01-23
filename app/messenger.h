#ifndef APP_MSG_H
#define APP_MSG_H

#ifdef ENABLE_MESSENGER

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "driver/keyboard.h"

typedef enum KeyboardType {
	UPPERCASE,
  	LOWERCASE,
  	NUMERIC,
  	END_TYPE_KBRD
} KeyboardType;

enum { 
	TX_MSG_LENGTH = 30,
	MSG_HEADER_LENGTH = 20,
	MAX_RX_MSG_LENGTH = TX_MSG_LENGTH + 2
};
//const uint8_t TX_MSG_LENGTH = 30;
//const uint8_t MAX_RX_MSG_LENGTH = TX_MSG_LENGTH + 2;

extern KeyboardType keyboardType;
extern uint16_t gErrorsDuringMSG;
extern char cMessage[TX_MSG_LENGTH];
extern char rxMessage[4][MAX_RX_MSG_LENGTH + 2];
extern uint8_t hasNewMessage;
extern uint8_t keyTickCounter;

typedef enum MsgStatus {
    READY,
    SENDING,
    RECEIVING,
} MsgStatus;

typedef enum PacketType {
    MESSAGE_PACKET,
    ENCRYPTED_MESSAGE_PACKET,
	ACK_PACKET,
	INVALID_PACKET
} PacketType;

enum {
	NONCE_LENGTH = 10,
	PAYLOAD_LENGTH = 19
};

// Data Packet definition                            // 2024 kamilsss655
union DataPacket
{ 
  struct{
	uint8_t header;
    uint8_t ciphertext[PAYLOAD_LENGTH];
    uint8_t nonce[NONCE_LENGTH];
    // uint8_t signature[SIGNATURE_LENGTH];
  } encrypted;
  
  struct{
    uint8_t header;
    char payload[PAYLOAD_LENGTH];
    // uint8_t signature[SIGNATURE_LENGTH];
  } unencrypted;
  // header + payload + nonce
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