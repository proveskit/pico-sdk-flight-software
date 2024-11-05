#ifndef MCP25625_H
#define MCP25625_H

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include <stdio.h>
#include <queue>
#include <cstring>
#include <climits>

// MCP25625 Register Definitions
#define MCP_CANCTRL 0x0F
#define MCP_TXB0CTRL 0x30
#define MCP_TXB0SIDH 0x31
#define MCP_TXB0SIDL 0x32
#define MCP_TXB0DLC  0x35
#define MCP_TXB0D0   0x36
#define MCP_RXB0CTRL 0x60
#define MCP_CANINTE  0x2B
#define MCP_CANINTF  0x2C
#define MCP_EFLG     0x2D

// MCP25625 Commands
#define MCP_RESET 0xC0
#define MCP_READ  0x03
#define MCP_WRITE 0x02
#define MCP_BIT_MODIFY 0x05

// Mode Definitions
#define MODE_NORMAL     0x00
#define MODE_SLEEP      0x20
#define MODE_LOOPBACK   0x40
#define MODE_LISTENONLY 0x60
#define MODE_CONFIG     0x80
#define MODE_MASK       0xE0

// Buffer Control Bits
#define MCP_TXB_TXREQ 0x08
#define MCP_TXB_TXP3  0x03
#define MCP_RX0IF     0x01
#define MCP_RX1IF     0x02

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