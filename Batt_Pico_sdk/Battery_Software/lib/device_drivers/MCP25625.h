#ifndef MCP25625_H
#define MCP25625_H

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include <stdio.h>
#include <queue>
#include <cstring>

struct CANMessage {
    uint32_t id;      // CAN ID
    uint8_t data[8];  // Data payload
    uint8_t length;   // Length of the data payload
};

class MCP25625 {
public:
    MCP25625(spi_inst_t *spiPort, uint csPin);
    bool begin();
    bool sendCANMessage(const CANMessage& message);
    bool receiveCANMessage(CANMessage& message);

    void enableLoopback();
    void disableLoopback();
    void enableSilentMode();
    void disableSilentMode();

    void queueMessage(const CANMessage& message);
    bool transmitQueuedMessages();
    bool receiveAllMessages(std::queue<CANMessage>& outMessages);

private:
    spi_inst_t *spi;
    uint cs;
    uint interruptPin;

    std::queue<CANMessage> txQueue; // Transmit queue

    void reset();
    void writeRegister(uint8_t address, uint8_t value);
    uint8_t readRegister(uint8_t address);
    void modifyRegister(uint8_t address, uint8_t mask, uint8_t value);
    void setupTransceiver();
    void setMode(uint8_t mode);
};

#endif // MCP25625_H