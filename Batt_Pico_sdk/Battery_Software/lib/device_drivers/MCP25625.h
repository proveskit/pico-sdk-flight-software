#ifndef MCP25625_H
#define MCP25625_H

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include <stdio.h>
#include <queue>
#include <cstring>
#include <climits>

// MCP25625 Register Definitions - Complete Set
#define MCP_RXF0SIDH 0x00
#define MCP_RXF0SIDL 0x01
#define MCP_RXF0EID8 0x02
#define MCP_RXF0EID0 0x03
#define MCP_RXF1SIDH 0x04
#define MCP_RXF1SIDL 0x05
#define MCP_RXF1EID8 0x06
#define MCP_RXF1EID0 0x07
#define MCP_CANSTAT  0x0E
#define MCP_CANCTRL  0x0F

// Configuration Registers
#define MCP_CNF1     0x2A
#define MCP_CNF2     0x29
#define MCP_CNF3     0x28
#define MCP_CANINTE  0x2B
#define MCP_CANINTF  0x2C
#define MCP_EFLG     0x2D
#define MCP_TXB0CTRL 0x30
#define MCP_TXB0SIDH 0x31
#define MCP_TXB0SIDL 0x32
#define MCP_TXB0DLC  0x35
#define MCP_TXB0D0   0x36

// Receive Registers
#define MCP_RXB0CTRL 0x60
#define MCP_RXB0SIDH 0x61
#define MCP_RXB0SIDL 0x62
#define MCP_RXB0DLC  0x65
#define MCP_RXB0D0   0x66
#define MCP_RXB1CTRL 0x70
#define MCP_RXB1SIDH 0x71
#define MCP_RXB1SIDL 0x72
#define MCP_RXB1DLC  0x75
#define MCP_RXB1D0   0x76

// Buffer Control Bits
#define MCP_RXB_RXM_STDEXT 0x00
#define MCP_RXB_RXM_STD    0x20
#define MCP_RXB_RXM_EXT    0x40
#define MCP_RXB_BUKT       0x04
#define MCP_TXB_TXREQ      0x08
#define MCP_TXB_TXP3       0x03

// Interrupt Bits
#define MCP_RX0IE    0x01
#define MCP_RX1IE    0x02
#define MCP_TX0IE    0x04
#define MCP_ERRIE    0x20
#define MCP_RX0IF    0x01
#define MCP_RX1IF    0x02

// Commands
#define MCP_RESET       0xC0
#define MCP_READ        0x03
#define MCP_WRITE       0x02
#define MCP_BIT_MODIFY  0x05
#define MCP_RX_STATUS   0xB0

// Error Flags
#define MCP_EFLG_RX1OVR 0x80
#define MCP_EFLG_RX0OVR 0x40
#define MCP_EFLG_TXBO   0x20
#define MCP_EFLG_TXEP   0x10
#define MCP_EFLG_RXEP   0x08
#define MCP_EFLG_TXWAR  0x04
#define MCP_EFLG_RXWAR  0x02
#define MCP_EFLG_EWARN  0x01

// Masks
#define MODE_MASK       0xE0

struct CANMessage {
    uint32_t id;      // CAN ID
    uint8_t data[8];  // Data payload
    uint8_t length;   // Length of the data payload
};

class MCP25625 {
private:
    spi_inst_t *spi;
    uint cs;
    uint interruptPin;
    std::queue<CANMessage> txQueue;

    void reset();
    void writeRegister(uint8_t address, uint8_t value);
    uint8_t readRegister(uint8_t address);
    void modifyRegister(uint8_t address, uint8_t mask, uint8_t value);
    void setMode(uint8_t mode);
    bool readMessageFromBuffer(CANMessage& message, uint8_t buffer);

public:
    MCP25625(spi_inst_t *spiPort, uint csPin);
    bool begin();
    bool sendCANMessage(const CANMessage& message);
    bool receiveCANMessage(CANMessage& message);
    bool isMessageAvailable();
    uint8_t getReceiveStatus();

    void enableLoopback();
    void disableLoopback();
    void enableSilentMode();
    void disableSilentMode();

    void queueMessage(const CANMessage& message);
    bool transmitQueuedMessages();
    bool receiveAllMessages(std::queue<CANMessage>& outMessages);
    
    // Error handling methods
    uint8_t getErrorFlags();
    void clearErrorFlags();
    bool checkError();
};

#endif // MCP25625_H