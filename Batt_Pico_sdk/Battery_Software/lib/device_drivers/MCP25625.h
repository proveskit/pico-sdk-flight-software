#ifndef MCP25625_H
#define MCP25625_H

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include <stdio.h>
#include <queue>
#include <cstring>
#include <climits>  // Added for UINT_MAX

// MCP25625 Register Addresses
#define MCP_CANCTRL      0x0F
#define MCP_CANSTAT      0x0E
#define MCP_TXB0CTRL     0x30
#define MCP_RXB0CTRL     0x60
#define MCP_CNF1         0x2A
#define MCP_CNF2         0x29
#define MCP_CNF3         0x28
#define MCP_CANINTE      0x2B
#define MCP_CANINTF      0x2C
#define MCP_EFLG         0x2D

// MCP25625 Commands
#define MCP_RESET        0xC0
#define MCP_READ         0x03
#define MCP_WRITE        0x02
#define MCP_RTS          0x80
#define MCP_READ_STATUS  0xA0
#define MCP_RX_STATUS    0xB0
#define MCP_BIT_MODIFY   0x05

// Operation Modes
#define MODE_NORMAL      0x00
#define MODE_SLEEP       0x20
#define MODE_LOOPBACK    0x40
#define MODE_LISTENONLY  0x60
#define MODE_CONFIG      0x80
#define MODE_POWERUP     0xE0

// Updated Bit Timing Configuration for 500kbps with 16MHz crystal
// TQ = 16/(16MHz) = 1us
// Sync Seg = 1TQ
// Prop Seg = 2TQ
// Phase Seg1 = 6TQ
// Phase Seg2 = 7TQ
// Total Bit Time = 16TQ = 2us (500kbps)
#define CNF1_500K        0x03    // BRP = 1, SJW = 1
#define CNF2_500K        0x9A    // BTLMODE = 1, SAM = 0, PHSEG1 = 5, PRSEG = 1
#define CNF3_500K        0x07    // SOF = 0, WAKFIL = 0, PHSEG2 = 6

// Message Types
#define STANDARD_CAN_MSG 0x00
#define EXTENDED_CAN_MSG 0x01

struct CANMessage {
    uint32_t id;      // CAN ID
    uint8_t data[8];  // Data payload
    uint8_t length;   // Length of the data payload
    bool isExtended;  // Extended frame flag
    bool isRTR;       // Remote transmission request flag
};

class MCP25625 {
private:
    spi_inst_t *spi;
    uint cs;
    uint interruptPin;

    std::queue<CANMessage> txQueue; // Transmit queue
    
    // Status flags
    bool initialized;
    uint8_t currentMode;

    void reset();
    void writeRegister(uint8_t address, uint8_t value);
    uint8_t readRegister(uint8_t address);
    void modifyRegister(uint8_t address, uint8_t mask, uint8_t value);
    void setupTransceiver();
    void setMode(uint8_t mode);
    
    // New private methods
    bool configureBuffers();
    bool configureBitTiming(uint8_t cnf1, uint8_t cnf2, uint8_t cnf3);
    bool clearBuffers();
    uint8_t getStatus();
    bool checkError();
    void clearInterrupts();
    bool waitForTxComplete(uint16_t timeout_ms);

public:
    MCP25625(spi_inst_t *spiPort, uint csPin, uint intPin = UINT_MAX);
    bool begin(uint8_t mode = MODE_NORMAL);
    bool sendCANMessage(const CANMessage& message);
    bool receiveCANMessage(CANMessage& message);

    // Mode control methods
    void enableLoopback();
    void disableLoopback();
    void enableSilentMode();
    void disableSilentMode();
    bool setNormalMode();
    bool setConfigMode();
    uint8_t getCurrentMode() { return currentMode; }

    // Queue management methods
    void queueMessage(const CANMessage& message);
    bool transmitQueuedMessages();
    bool receiveAllMessages(std::queue<CANMessage>& outMessages);
    void clearQueues();
    
    // Status and error handling methods
    bool isInitialized() { return initialized; }
    bool hasError();
    uint8_t getErrorFlags();
    void clearErrors();
    
    // Buffer management
    bool isTxBufferFull();
    bool isRxBufferEmpty();
    
    // Interrupt handling
    void enableInterrupts(uint8_t interrupts);
    void disableInterrupts(uint8_t interrupts);
    uint8_t getInterruptStatus();
    
    // Utility methods
    static uint32_t convertToExtendedId(uint16_t standardId);
    static bool isExtendedId(uint32_t id);
};

// Interrupt flag bits
#define MCP_INT_RX0      0x01
#define MCP_INT_RX1      0x02
#define MCP_INT_TX0      0x04
#define MCP_INT_TX1      0x08
#define MCP_INT_TX2      0x10
#define MCP_INT_ERROR    0x20
#define MCP_INT_WAKEUP   0x40
#define MCP_INT_MERRF    0x80

// Error flag bits
#define MCP_EFLG_RX1OVR  0x80
#define MCP_EFLG_RX0OVR  0x40
#define MCP_EFLG_TXBO    0x20
#define MCP_EFLG_TXEP    0x10
#define MCP_EFLG_RXEP    0x08
#define MCP_EFLG_TXWAR   0x04
#define MCP_EFLG_RXWAR   0x02
#define MCP_EFLG_EWARN   0x01

#endif // MCP25625_H