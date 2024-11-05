#include "MCP25625.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"

// MCP25625 Registers Definition
#define MCP_RXF0SIDH 0x00
#define MCP_RXF0SIDL 0x01
#define MCP_RXF0EID8 0x02
#define MCP_RXF0EID0 0x03
#define MCP_RXF1SIDH 0x04
#define MCP_RXF1SIDL 0x05
#define MCP_RXF1EID8 0x06
#define MCP_RXF1EID0 0x07
#define MCP_RXF2SIDH 0x08
#define MCP_RXF2SIDL 0x09
#define MCP_RXF2EID8 0x0A
#define MCP_RXF2EID0 0x0B
#define MCP_BFPCTRL  0x0C
#define MCP_TXRTSCTRL 0x0D
#define MCP_CANSTAT  0x0E
#define MCP_CANCTRL  0x0F
#define MCP_RXF3SIDH 0x10
#define MCP_RXF3SIDL 0x11
#define MCP_RXF3EID8 0x12
#define MCP_RXF3EID0 0x13
#define MCP_RXF4SIDH 0x14
#define MCP_RXF4SIDL 0x15
#define MCP_RXF4EID8 0x16
#define MCP_RXF4EID0 0x17
#define MCP_RXF5SIDH 0x18
#define MCP_RXF5SIDL 0x19
#define MCP_RXF5EID8 0x1A
#define MCP_RXF5EID0 0x1B
#define MCP_TEC      0x1C
#define MCP_REC      0x1D
#define MCP_RXM0SIDH 0x20
#define MCP_RXM0SIDL 0x21
#define MCP_RXM0EID8 0x22
#define MCP_RXM0EID0 0x23
#define MCP_RXM1SIDH 0x24
#define MCP_RXM1SIDL 0x25
#define MCP_RXM1EID8 0x26
#define MCP_RXM1EID0 0x27
#define MCP_CNF3     0x28
#define MCP_CNF2     0x29
#define MCP_CNF1     0x2A
#define MCP_CANINTE  0x2B
#define MCP_CANINTF  0x2C
#define MCP_EFLG     0x2D

// Transmit Buffer 0 Registers
#define MCP_TXB0CTRL 0x30
#define MCP_TXB0SIDH 0x31
#define MCP_TXB0SIDL 0x32
#define MCP_TXB0EID8 0x33
#define MCP_TXB0EID0 0x34
#define MCP_TXB0DLC  0x35
#define MCP_TXB0D0   0x36
#define MCP_TXB0D1   0x37
#define MCP_TXB0D2   0x38
#define MCP_TXB0D3   0x39
#define MCP_TXB0D4   0x3A
#define MCP_TXB0D5   0x3B
#define MCP_TXB0D6   0x3C
#define MCP_TXB0D7   0x3D

// Transmit Buffer 1 Registers
#define MCP_TXB1CTRL 0x40
#define MCP_TXB1SIDH 0x41
#define MCP_TXB1SIDL 0x42
#define MCP_TXB1EID8 0x43
#define MCP_TXB1EID0 0x44
#define MCP_TXB1DLC  0x45
#define MCP_TXB1D0   0x46
#define MCP_TXB1D1   0x47
#define MCP_TXB1D2   0x48
#define MCP_TXB1D3   0x49
#define MCP_TXB1D4   0x4A
#define MCP_TXB1D5   0x4B
#define MCP_TXB1D6   0x4C
#define MCP_TXB1D7   0x4D

// Transmit Buffer 2 Registers
#define MCP_TXB2CTRL 0x50
#define MCP_TXB2SIDH 0x51
#define MCP_TXB2SIDL 0x52
#define MCP_TXB2EID8 0x53
#define MCP_TXB2EID0 0x54
#define MCP_TXB2DLC  0x55
#define MCP_TXB2D0   0x56
#define MCP_TXB2D1   0x57
#define MCP_TXB2D2   0x58
#define MCP_TXB2D3   0x59
#define MCP_TXB2D4   0x5A
#define MCP_TXB2D5   0x5B
#define MCP_TXB2D6   0x5C
#define MCP_TXB2D7   0x5D

// Receive Buffer 0 Registers
#define MCP_RXB0CTRL 0x60
#define MCP_RXB0SIDH 0x61
#define MCP_RXB0SIDL 0x62
#define MCP_RXB0EID8 0x63
#define MCP_RXB0EID0 0x64
#define MCP_RXB0DLC  0x65
#define MCP_RXB0D0   0x66
#define MCP_RXB0D1   0x67
#define MCP_RXB0D2   0x68
#define MCP_RXB0D3   0x69
#define MCP_RXB0D4   0x6A
#define MCP_RXB0D5   0x6B
#define MCP_RXB0D6   0x6C
#define MCP_RXB0D7   0x6D

// Receive Buffer 1 Registers
#define MCP_RXB1CTRL 0x70
#define MCP_RXB1SIDH 0x71
#define MCP_RXB1SIDL 0x72
#define MCP_RXB1EID8 0x73
#define MCP_RXB1EID0 0x74
#define MCP_RXB1DLC  0x75
#define MCP_RXB1D0   0x76
#define MCP_RXB1D1   0x77
#define MCP_RXB1D2   0x78
#define MCP_RXB1D3   0x79
#define MCP_RXB1D4   0x7A
#define MCP_RXB1D5   0x7B
#define MCP_RXB1D6   0x7C
#define MCP_RXB1D7   0x7D


// Control Register Bits
#define MCP_RX0IE 0x01 // Receive Buffer 0 Full Interrupt Enable bit (CANINTE register)
#define MCP_RX1IE 0x02 // Receive Buffer 1 Full Interrupt Enable bit (CANINTE register)
#define MCP_TX0IE 0x04 // Transmit Buffer 0 Empty Interrupt Enable bit (CANINTE register)
#define MCP_TX1IE 0x08 // Transmit Buffer 1 Empty Interrupt Enable bit (CANINTE register)
#define MCP_TX2IE 0x10 // Transmit Buffer 2 Empty Interrupt Enable bit (CANINTE register)
#define MCP_ERRIE 0x20 // Error Interrupt Enable bit (CANINTE register)
#define MCP_WAKIE 0x40 // Wake-up Interrupt Enable bit (CANINTE register)
#define MCP_MERRE 0x80 // Message Error Interrupt Enable bit (CANINTE register)

// CANINTF - CAN Interrupt Flag Register Bits
#define MCP_RX0IF 0x01 // Receive Buffer 0 Full Interrupt Flag bit
#define MCP_RX1IF 0x02 // Receive Buffer 1 Full Interrupt Flag bit
#define MCP_TX0IF 0x04 // Transmit Buffer 0 Empty Interrupt Flag bit
#define MCP_TX1IF 0x08 // Transmit Buffer 1 Empty Interrupt Flag bit
#define MCP_TX2IF 0x10 // Transmit Buffer 2 Empty Interrupt Flag bit
#define MCP_ERRIF 0x20 // Error Interrupt Flag bit
#define MCP_WAKIF 0x40 // Wake-up Interrupt Flag bit
#define MCP_MERRF 0x80 // Message Error Interrupt Flag bit

// TXBnCTRL - Transmit Buffer Control Register Bits
#define MCP_TXB_TXREQ 0x08 // Message Transmit Request bit
#define MCP_TXB_TXP0  0x01 // Transmit Buffer Priority Bit 0
#define MCP_TXB_TXP1  0x02 // Transmit Buffer Priority Bit 1
#define MCP_TXB_TXP3  0x03 // Transmit Buffer Priority Highest

// RXBnCTRL - Receive Buffer Control Register Bits
#define MCP_RXB_RXM_STD 0x20 // Receive Buffer Operating mode bits - standard
#define MCP_RXB_RXM_EXT 0x40 // Receive Buffer Operating mode bits - extended
#define MCP_RXB_RXM_STDEXT 0x00 // Receive Buffer Operating mode bits - standard and extended
#define MCP_RXB_RXRTR 0x08 // Receive Remote Transfer Request bit
#define MCP_RXB_BUKT 0x04 // Rollover Enable bit

// CNFx - Configuration Registers for Bit Timing
#define MCP_SJW1 0x00 // Synchronization Jump Width = 1xTQ
#define MCP_BTLMODE 0x80 // PS2 Bit Time Length
#define MCP_SAM 0x40 // Sample Point Configuration

// EFLG - Error Flag Register Bits
#define MCP_EFLG_RX1OVR 0x80 // Receive Buffer 1 Overflow Flag bit
#define MCP_EFLG_RX0OVR 0x40 // Receive Buffer 0 Overflow Flag bit
#define MCP_EFLG_TXBO   0x20 // Bus-Off Error Flag bit
#define MCP_EFLG_TXEP   0x10 // Transmit Error-Passive Flag bit
#define MCP_EFLG_RXEP   0x08 // Receive Error-Passive Flag bit
#define MCP_EFLG_TXWAR  0x04 // Transmit Error Warning Flag bit
#define MCP_EFLG_RXWAR  0x02 // Receive Error Warning Flag bit
#define MCP_EFLG_EWARN  0x01 // Error Warning Flag bit

// Operation Modes
#define MODE_CONFIG    0x80
#define MODE_NORMAL    0x00
#define MODE_SLEEP     0x20
#define MODE_LOOPBACK  0x40
#define MODE_LISTENONLY 0x60
#define MODE_POWERUP    0xE0 // Power up mode
#define MODE_MASK       0xE0 // Mode Mask

// Other Necessary Definitions
#define MCP_RESET 0xC0
#define MCP_READ  0x03
#define MCP_WRITE 0x02
#define MCP_RTS   0x80 // Request to Send
#define MCP_READ_STATUS 0xA0
#define MCP_RX_STATUS 0xB0
#define MCP_BITMOD 0x05

MCP25625::MCP25625(spi_inst_t *spiPort, uint csPin) : spi(spiPort), cs(csPin) {
    gpio_init(cs);
    gpio_set_dir(cs, GPIO_OUT);
    gpio_put(cs, 1); // Deselect the MCP25625
    spi_init(spi, 1000 * 1000 * 10); // Initialize SPI at 10 MHz
    //spi_set_format(spi, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
}

bool MCP25625::begin() {
    reset();
    setMode(MODE_CONFIG); // Start configuration
    // Set bit timing for 500 Kbps with a 16 MHz oscillator
    // Assuming CNF1, CNF2, CNF3 values for 500 Kbps from the table provided
    // These values will need to be adjusted based on actual baudrate and crystal frequency
    writeRegister(MCP_CNF1, 0x00); // Example value, adjust based on actual requirements
    writeRegister(MCP_CNF2, 0xD0); // Example value, adjust based on actual requirements
    writeRegister(MCP_CNF3, 0x82); // Example value, adjust based on actual requirements

    // Enable interrupts for RX0 and RX1 using CANINTE register
    // Here we're enabling interrupts for message reception
    writeRegister(MCP_CANINTE, MCP_RX0IF | MCP_RX1IF);
    setMode(MODE_NORMAL); // Set to normal mode after configuration
    uint8_t canstat = readRegister(MCP_CANSTAT);

    // Check if the device is in normal mode. The mode bits are the two most significant bits of CANSTAT.
    // MODE_NORMAL is defined as 0x00, but because the mode bits are in the most significant bits,
    // we need to shift right to compare.
    if ((canstat & MODE_MASK) >> 5 == MODE_NORMAL) {
        // Successfully communicated with the MCP25625 and verified it is in normal mode
        return true;
    } else {
        // Failed to communicate with the MCP25625 or it is in an unexpected mode
        return false;
    }
}

void MCP25625::reset() {
    gpio_put(cs, 0);
    uint8_t resetCmd = MCP_RESET;
    spi_write_blocking(spi, &resetCmd, 1);
    gpio_put(cs, 1);
    sleep_ms(10);
}

void MCP25625::writeRegister(uint8_t address, uint8_t value) {
    gpio_put(cs, 0);
    uint8_t cmd[3] = {MCP_WRITE, address, value};
    spi_write_blocking(spi, cmd, 3);
    gpio_put(cs, 1);
}

uint8_t MCP25625::readRegister(uint8_t address) {
    gpio_put(cs, 0);
    uint8_t cmd[2] = {MCP_READ, address};
    uint8_t val;
    spi_write_blocking(spi, cmd, 2);
    spi_read_blocking(spi, 0xFF, &val, 1);
    gpio_put(cs, 1);
    return val;
}

void MCP25625::modifyRegister(uint8_t address, uint8_t mask, uint8_t value) {
    gpio_put(cs, 0);
    uint8_t cmd[4] = {MCP_BITMOD, address, mask, value};
    spi_write_blocking(spi, cmd, 4);
    gpio_put(cs, 1);
}

void MCP25625::setMode(uint8_t mode) {
    writeRegister(MCP_CANCTRL, mode);
}

bool MCP25625::sendCANMessage(const CANMessage& message) {
    // Ensure TX buffer is ready
    if (readRegister(MCP_TXB0CTRL) & MCP_TXB_TXREQ) {
        printf("TX buffer is not ready!\n");
        return false; // TX buffer is not ready
    }

    // Set message ID (standard ID)
    writeRegister(MCP_TXB0SIDH, message.id >> 3);
    writeRegister(MCP_TXB0SIDL, message.id << 5);

    // Set data length
    writeRegister(MCP_TXB0DLC, message.length);

    // Load data bytes
    for (uint8_t i = 0; i < message.length; ++i) {
        writeRegister(MCP_TXB0D0 + i, message.data[i]);
    }

    // Request to send
    modifyRegister(MCP_TXB0CTRL, MCP_TXB_TXP3, MCP_TXB_TXP3);
    modifyRegister(MCP_TXB0CTRL, MCP_TXB_TXREQ, MCP_TXB_TXREQ);

    sleep_ms(100);
    bool failed = false;
    int8_t sent = readRegister(MCP_TXB0CTRL);
    if((sent & 0x40) == 0x40){
        printf("Transmission aborted!\n");
        failed = true;
    }
    if((sent & 0x20) == 0x20){
        printf("Message lost arbitration!\n");
        failed = true;
    }
    if((sent & 0x10) == 0x10){
        printf("Transmission error!\n");
        failed = true;
    }
    if((sent & 0x08) == 0x08){
        printf("Message didn't send!\n");
        failed = true;
    }
    if(failed){
        return false;
    }

    return true;
}

bool MCP25625::receiveCANMessage(CANMessage& message) {
    // Check if a message has been received
    if (!(readRegister(MCP_CANINTF) & MCP_RX0IF)) {
        return false; // No message received
    }

    // Read message ID (standard ID)
    uint8_t sidh = readRegister(MCP_RXB0SIDH);
    uint8_t sidl = readRegister(MCP_RXB0SIDH + 1);
    message.id = (uint32_t)(sidh << 3) | (sidl >> 5);

    // Read data length
    message.length = readRegister(MCP_RXB0DLC) & 0x0F;

    // Read data bytes
    for (uint8_t i = 0; i < message.length; ++i) {
        message.data[i] = readRegister(MCP_RXB0D0 + i);
    }

    // Clear the interrupt flag for received message
    modifyRegister(MCP_CANINTF, MCP_RX0IF, 0);

    return true;
}

void MCP25625::enableLoopback() {
    setMode(MODE_LOOPBACK);
}

void MCP25625::disableLoopback() {
    setMode(MODE_NORMAL);
}

void MCP25625::enableSilentMode() {
    setMode(MODE_LISTENONLY);
}

void MCP25625::disableSilentMode() {
    setMode(MODE_NORMAL);
}

void MCP25625::queueMessage(const CANMessage& message) {
    txQueue.push(message); // Add message to the transmit queue
}

bool MCP25625::transmitQueuedMessages() {
    while (!txQueue.empty()) {
        const CANMessage& message = txQueue.front();
        if (!sendCANMessage(message)) {
            return false; // Transmission failed
        }
        txQueue.pop(); // Remove the message from the queue
    }
    return true;
}

bool MCP25625::receiveAllMessages(std::queue<CANMessage>& outMessages) {
    CANMessage message;
    uint8_t data[8], len;
    uint32_t id;
    while (receiveCANMessage(message)) {
        message.id = id;
        message.length = len;
        memcpy(message.data, data, len);
        outMessages.push(message);
    }
    return !outMessages.empty();
}