#include "MCP25625.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"

MCP25625::MCP25625(spi_inst_t *spiPort, uint csPin) : spi(spiPort), cs(csPin) {
    gpio_init(cs);
    gpio_set_dir(cs, GPIO_OUT);
    gpio_put(cs, 1); // Deselect the MCP25625
    spi_init(spi, 1000 * 1000 * 10); // Initialize SPI at 10 MHz
}

bool MCP25625::begin() {
    reset();
    
    // Enter config mode and verify
    setMode(MODE_CONFIG);
    if ((readRegister(MCP_CANSTAT) & MODE_MASK) != MODE_CONFIG) {
        return false;
    }

    // Configure bit timing for 500 Kbps with 16 MHz oscillator
    // TQ = 2/(16MHz) = 0.125us
    // Sync Seg = 1TQ
    // Prop Seg = 2TQ
    // Phase Seg1 = 6TQ
    // Phase Seg2 = 7TQ
    // Total Bit Time = 16TQ = 2us (500kbps)
    writeRegister(MCP_CNF1, 0x03);    // BRP = 1, SJW = 1
    writeRegister(MCP_CNF2, 0x9A);    // BTLMODE = 1, SAM = 0, PHSEG1 = 5, PRSEG = 1
    writeRegister(MCP_CNF3, 0x07);    // SOF = 0, WAKFIL = 0, PHSEG2 = 6

    // Configure receive buffers
    writeRegister(MCP_RXB0CTRL, MCP_RXB_RXM_STDEXT | MCP_RXB_BUKT); // Enable both standard and extended IDs, enable rollover
    writeRegister(MCP_RXB1CTRL, MCP_RXB_RXM_STDEXT); // Enable both standard and extended IDs

    // Clear filters and masks - accept all messages
    for (uint8_t i = MCP_RXF0SIDH; i <= MCP_RXF5EID0; i++) {
        writeRegister(i, 0);
    }
    for (uint8_t i = MCP_RXM0SIDH; i <= MCP_RXM1EID0; i++) {
        writeRegister(i, 0);
    }

    // Enable interrupts
    writeRegister(MCP_CANINTE, MCP_RX0IE | MCP_RX1IE | MCP_ERRIE);
    
    // Clear interrupt flags
    writeRegister(MCP_CANINTF, 0);

    // Enter normal mode
    setMode(MODE_NORMAL);
    
    // Verify normal mode
    uint8_t canstat = readRegister(MCP_CANSTAT);
    return ((canstat & MODE_MASK) == MODE_NORMAL);
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
    uint8_t cmd[4] = {MCP_BIT_MODIFY, address, mask, value};  // Changed from MCP_BITMOD
    spi_write_blocking(spi, cmd, 4);
    gpio_put(cs, 1);
}

void MCP25625::setMode(uint8_t mode) {
    modifyRegister(MCP_CANCTRL, MODE_MASK, mode);
    sleep_ms(10); // Give time for mode change
}

bool MCP25625::sendCANMessage(const CANMessage& message) {
    // Wait for TX buffer to be empty (with timeout)
    uint16_t timeout = 0;
    while ((readRegister(MCP_TXB0CTRL) & MCP_TXB_TXREQ) && (timeout < 1000)) {
        sleep_ms(1);
        timeout++;
    }
    
    if (timeout >= 1000) {
        printf("TX buffer timeout!\n");
        return false;
    }

    // Clear any pending transmission flags
    modifyRegister(MCP_TXB0CTRL, 0x78, 0);

    // Set message ID
    writeRegister(MCP_TXB0SIDH, message.id >> 3);
    writeRegister(MCP_TXB0SIDL, (message.id << 5) & 0xE0);

    // Set data length
    writeRegister(MCP_TXB0DLC, message.length & 0x0F);

    // Write data bytes
    for (uint8_t i = 0; i < message.length && i < 8; i++) {
        writeRegister(MCP_TXB0D0 + i, message.data[i]);
    }

    // Request transmission (highest priority)
    modifyRegister(MCP_TXB0CTRL, MCP_TXB_TXP3 | MCP_TXB_TXREQ, MCP_TXB_TXP3 | MCP_TXB_TXREQ);

    // Wait for transmission to complete or error (with timeout)
    timeout = 0;
    while (timeout < 1000) {
        uint8_t status = readRegister(MCP_TXB0CTRL);
        
        // Check for transmission complete
        if (!(status & MCP_TXB_TXREQ)) {
            return true;  // Success
        }
        
        // Check for errors
        if (status & 0x70) {  // Check error flags
            if (status & 0x40) printf("Transmission aborted!\n");
            if (status & 0x20) printf("Message lost arbitration!\n");
            if (status & 0x10) printf("Transmission error!\n");
            return false;
        }
        
        sleep_ms(1);
        timeout++;
    }

    printf("Transmission timeout!\n");
    return false;
}

bool MCP25625::receiveCANMessage(CANMessage& message) {
    // Check if a message has been received in either buffer
    uint8_t interruptFlags = readRegister(MCP_CANINTF);
    
    // Check RXB0 first (higher priority buffer)
    if (interruptFlags & MCP_RX0IF) {
        return readMessageFromBuffer(message, 0);
    }
    // Then check RXB1
    else if (interruptFlags & MCP_RX1IF) {
        return readMessageFromBuffer(message, 1);
    }
    
    return false; // No message available
}

bool MCP25625::readMessageFromBuffer(CANMessage& message, uint8_t buffer) {
    uint8_t rxBnSIDH, rxBnSIDL;
    uint8_t rxBnDLC;
    
    if (buffer == 0) {
        // Read from RX Buffer 0
        rxBnSIDH = readRegister(MCP_RXB0SIDH);
        rxBnSIDL = readRegister(MCP_RXB0SIDL);
        rxBnDLC = readRegister(MCP_RXB0DLC);
        
        // Read the data
        message.length = rxBnDLC & 0x0F;
        for (uint8_t i = 0; i < message.length && i < 8; i++) {
            message.data[i] = readRegister(MCP_RXB0D0 + i);
        }
        
        // Clear the interrupt flag
        modifyRegister(MCP_CANINTF, MCP_RX0IF, 0);
    }
    else {
        // Read from RX Buffer 1
        rxBnSIDH = readRegister(MCP_RXB1SIDH);
        rxBnSIDL = readRegister(MCP_RXB1SIDL);
        rxBnDLC = readRegister(MCP_RXB1DLC);
        
        // Read the data
        message.length = rxBnDLC & 0x0F;
        for (uint8_t i = 0; i < message.length && i < 8; i++) {
            message.data[i] = readRegister(MCP_RXB1D0 + i);
        }
        
        // Clear the interrupt flag
        modifyRegister(MCP_CANINTF, MCP_RX1IF, 0);
    }
    
    // Construct the message ID
    message.id = ((uint32_t)rxBnSIDH << 3) | (rxBnSIDL >> 5);
    
    return true;
}

bool MCP25625::isMessageAvailable() {
    uint8_t interruptFlags = readRegister(MCP_CANINTF);
    return (interruptFlags & (MCP_RX0IF | MCP_RX1IF)) != 0;
}

uint8_t MCP25625::getReceiveStatus() {
    gpio_put(cs, 0);
    uint8_t cmd = MCP_RX_STATUS;
    uint8_t status;
    spi_write_blocking(spi, &cmd, 1);
    spi_read_blocking(spi, 0xFF, &status, 1);
    gpio_put(cs, 1);
    return status;
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
    txQueue.push(message);
}

bool MCP25625::transmitQueuedMessages() {
    while (!txQueue.empty()) {
        const CANMessage& message = txQueue.front();
        if (!sendCANMessage(message)) {
            return false;
        }
        txQueue.pop();
    }
    return true;
}

bool MCP25625::receiveAllMessages(std::queue<CANMessage>& outMessages) {
    CANMessage message;
    while (receiveCANMessage(message)) {
        outMessages.push(message);
    }
    return !outMessages.empty();
}

uint8_t MCP25625::getErrorFlags() {
    return readRegister(MCP_EFLG);
}

void MCP25625::clearErrorFlags() {
    writeRegister(MCP_EFLG, 0);
}

bool MCP25625::checkError() {
    uint8_t errors = getErrorFlags();
    if (errors) {
        if (errors & MCP_EFLG_RX1OVR) printf("Receive Buffer 1 Overflow\n");
        if (errors & MCP_EFLG_RX0OVR) printf("Receive Buffer 0 Overflow\n");
        if (errors & MCP_EFLG_TXBO) printf("Bus-Off Error\n");
        if (errors & MCP_EFLG_TXEP) printf("Transmit Error-Passive\n");
        if (errors & MCP_EFLG_RXEP) printf("Receive Error-Passive\n");
        if (errors & MCP_EFLG_TXWAR) printf("Transmit Error Warning\n");
        if (errors & MCP_EFLG_RXWAR) printf("Receive Error Warning\n");
        if (errors & MCP_EFLG_EWARN) printf("Error Warning\n");
        return true;
    }
    return false;
}