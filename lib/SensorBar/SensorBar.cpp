#include "SensorBar.h"

SensorBar::SensorBar(PinName sda,
                     PinName scl,
                     float bar_dist,
                     bool run_as_thread) : distAxisToSensor(bar_dist)
                                         , i2c(sda, scl)
                                         , thread(osPriorityAboveNormal2, 4096)
{
    // Store the received parameters into member variables
    deviceAddress = 0x3E<<1;
    pinInterrupt = 255;
    pinOscillator = 255;
    pinReset = 255;
    invertBits = 0;
    barStrobe = 0;

    lastBarRawValue = lastBarPositionValue = 0;

    angle = avgAngle = 0;
    nrOfLedsActive = 0;
    avgFilterAngle.init(AVG_FILTER_ANGLE_N);
    isFirstAvgAngle = true;

    for (int i = 0; i < 8; ++i) {
        avgFilterBits[i].init(AVG_FILTER_BITS_N);
    }

    clearBarStrobe();  // to illuminate all the time
    clearInvertBits(); // to make the bar look for a dark line on a reflective surface

    if (run_as_thread && begin()) {
        thread.start(callback(this, &SensorBar::updateAsThread));
        ticker.attach(callback(this, &SensorBar::sendThreadFlag), std::chrono::microseconds{PERIOD_MUS});
    }
}

SensorBar::~SensorBar()
{
    ticker.detach();
    thread.terminate();
}

//Call .setBarStrobing(); to only illuminate while reading line
void SensorBar::setBarStrobe()
{
    barStrobe = 1; //Do strobe
}

//Call .clearBarStrobing(); to illuminate all the time
void SensorBar::clearBarStrobe()
{
    barStrobe = 0; //Always on
}

// .setInvertBits(); to make the bar functions look for a white line on dark surface
void SensorBar::setInvertBits()
{
    invertBits = 1; //Do strobe
}

// .clearInvertBits(); to make the bar look for a dark line on a reflective surface
void SensorBar::clearInvertBits()
{
    invertBits = 0; //Always on
}

//****************************************************************************//
//
//  Bar functions
//
//****************************************************************************//

uint8_t SensorBar::getRaw() const
{
    return lastBarRawValue;
}

int8_t SensorBar::getBinaryPosition() const
{
    return -lastBarPositionValue;
}

float SensorBar::getAngleRad() const
{
    return angle;
}

float SensorBar::getAvgAngleRad() const
{
    return avgAngle;
}

uint8_t SensorBar::getNrOfLedsActive() const
{
    return nrOfLedsActive;
}

bool SensorBar::isAnyLedActive() const
{
    if(nrOfLedsActive != 0)
        return true;
    return false;
}

float SensorBar::getAvgBit(int bitNumber) const {
    if (bitNumber < 0 || bitNumber >= 8)
        return 0.0f;
    // constrain the value
    const float avgBit = avgFilterBits[bitNumber].read();
    return constrainIntoZeroToOne(avgBit);
}

float SensorBar::getMeanThreeAvgBitsLeft() const {
    // Leftmost 3 bits
    const float avgBits = 1.0f / 3.0f * ( avgFilterBits[0].read()
                                        + avgFilterBits[1].read()
                                        + avgFilterBits[2].read() );
    return constrainIntoZeroToOne(avgBits);
}

float SensorBar::getMeanThreeAvgBitsRight() const {
    // Rightmost 3 bits
    const float avgBits = 1.0f / 3.0f * ( avgFilterBits[5].read()
                                        + avgFilterBits[6].read()
                                        + avgFilterBits[7].read() );
    return constrainIntoZeroToOne(avgBits);
}

float SensorBar::getMeanFourAvgBitsCenter() const {
    // Center 4 bits
    const float avgBits = 1.0f / 3.0f * ( avgFilterBits[2].read()
                                        + 1.0f / 2.0f * ( avgFilterBits[3].read() + avgFilterBits[4].read() )
                                        + avgFilterBits[5].read() );
    return constrainIntoZeroToOne(avgBits);
}

void SensorBar::update()
{
    //Assign values to each bit, -127 to 127, sum, and divide
    int16_t accumulator = 0;
    uint8_t bitsCounted = 0;
    int16_t i;

    //Get the information from the wire, stores in lastBarRawValue
    if( barStrobe == 1 ) {
        writeByte(REG_DATA_B, 0x02); //Turn on IR
        thread_sleep_for(2); // wait_us(2000);
        writeByte(REG_DATA_B, 0x00); //Turn on feedback
    } else {
        writeByte(REG_DATA_B, 0x00); //make sure both IR and indicators are on
    }
    //Operate the I2C machine
    lastBarRawValue = readByte( REG_DATA_A ); //Peel the data off port A

    //Invert the bits if needed
    if( invertBits == 1 ) {
        lastBarRawValue ^= 0xFF;
    }

    //Turn off IR and feedback when done
    if( barStrobe == 1 ) {
        writeByte(REG_DATA_B, 0x03);
    }

    //count bits
    for ( i = 0; i < 8; i++ ) {
        if ( ((lastBarRawValue >> i) & 0x01) == 1 ) {
            bitsCounted++;
        }
    }

    //Find the vector value of each positive bit and sum
    for ( i = 7; i > 3; i-- ) { //iterate negative side bits
        if ( ((lastBarRawValue >> i) & 0x01) == 1 ) {
            accumulator += ((-32 * (i - 3)) + 1);
        }
    }
    for ( i = 0; i < 4; i++ ) { //iterate positive side bits
        if ( ((lastBarRawValue >> i) & 0x01) == 1 ) {
            accumulator += ((32 * (4 - i)) - 1);
        }
    }

    if ( bitsCounted > 0 ) {
        lastBarPositionValue = accumulator / bitsCounted;
    } else {
        lastBarPositionValue = 0;
    }

    //Update average filters
    angle = updateAngleRad();
    nrOfLedsActive = updateNrOfLedsActive();

    if(nrOfLedsActive == 0) {
        if(!isFirstAvgAngle) {
            avgFilterAngle.reset();
            isFirstAvgAngle = true;
        }
    } else {
        if(isFirstAvgAngle) {
            isFirstAvgAngle = false;
            avgFilterAngle.reset(angle);
        }
        avgAngle = avgFilterAngle.apply(angle);
    }

    for (int i = 0; i < 8; ++i) {
        bool bit = (lastBarRawValue >> i) & 0x01;
        avgFilterBits[7 - i].apply(static_cast<float>(bit));
    }
}

//****************************************************************************//
//
//  Utilities
//
//****************************************************************************//

//Run this once during initialization to configure the SX1509 as a sensor bar
//Returns 1 for success
bool SensorBar::begin(void)
{
    bool returnVar = false;

    // Reset the SX1509
    reset();

    // Communication test. We'll read from two registers with different
    // default values to verify communication.
    unsigned int testRegisters = 0;
    testRegisters = readWord(REG_INTERRUPT_MASK_A);   // This should return 0xFF00
    // Then read a uint8_t that should be 0x00
    if (testRegisters == 0xFF00) {
        //Success!  Configure the device.
        writeByte(REG_DIR_A, 0xFF);
        writeByte(REG_DIR_B, 0xFC);
        writeByte(REG_DATA_B, 0x01);

        returnVar = true;
    }

    return returnVar;
}

// Do a software reset
void SensorBar::reset()
{
    // No hardware option, try software reset
    writeByte(REG_RESET, 0x12);
    writeByte(REG_RESET, 0x34);
}

// readByte(uint8_t registerAddress)
//  This function reads a single uint8_t located at the registerAddress register.
//  - deviceAddress should already be set by the constructor.
//  - Return value is the uint8_t read from registerAddress
//
//  Currently returns 0 if communication has timed out
//
uint8_t SensorBar::readByte(uint8_t registerAddress)
{
    uint8_t readValue;
    uint8_t data[2] = {registerAddress, 0};
    i2c.write(deviceAddress, reinterpret_cast<char*>(data), 1);
    i2c.read(deviceAddress, reinterpret_cast<char*>(&readValue), 1);

    return readValue;
}

// readWord(uint8_t registerAddress)
//  This function will read a two-uint8_t word beginning at registerAddress
//  - A 16-bit unsigned int will be returned.
//      - The msb of the return value will contain the value read from registerAddress
//      - The lsb of the return value will contain the value read from registerAddress + 1
unsigned int SensorBar::readWord(uint8_t registerAddress)
{
    unsigned int readValue;
    unsigned int msb, lsb;
    uint8_t data[2] = {registerAddress, 0};
    uint8_t r_data[2];
    i2c.write(deviceAddress, reinterpret_cast<char*>(data), 1);
    i2c.read(deviceAddress, reinterpret_cast<char*>(r_data), 2);
    msb = ((unsigned int)r_data[0] & 0x00FF) << 8;
    lsb = ((unsigned int)r_data[1] & 0x00FF);
    readValue = msb | lsb;

    return readValue;
}

// readBytes(uint8_t firstRegisterAddress, uint8_t * destination, uint8_t length)
//  This function reads a series of uint8_ts incrementing from a given address
//  - firstRegsiterAddress is the first address to be read
//  - destination is an array of uint8_ts where the read values will be stored into
//  - length is the number of uint8_ts to be read
//  - No return value.
void SensorBar::readBytes(uint8_t firstRegisterAddress, uint8_t * destination, uint8_t length)
{
    uint8_t data[2] = {firstRegisterAddress, 0};
    i2c.write(deviceAddress, reinterpret_cast<char*>(data), 1);
    i2c.read(deviceAddress, reinterpret_cast<char*>(destination), length);
}

// writeByte(uint8_t registerAddress, uint8_t writeValue)
//  This function writes a single uint8_t to a single register on the SX509.
//  - writeValue is written to registerAddress
//  - deviceAddres should already be set from the constructor
//  - No return value.
void SensorBar::writeByte(uint8_t registerAddress, uint8_t writeValue)
{
    uint8_t data[2] = {registerAddress, writeValue};
    i2c.write(deviceAddress, reinterpret_cast<char*>(data), 2);
}

// writeWord(uint8_t registerAddress, ungisnged int writeValue)
//  This function writes a two-uint8_t word to registerAddress and registerAddress + 1
//  - the upper uint8_t of writeValue is written to registerAddress
//  - the lower uint8_t of writeValue is written to registerAddress + 1
//  - No return value.
void SensorBar::writeWord(uint8_t registerAddress, unsigned int writeValue)
{
    uint8_t msb, lsb;
    msb = ((writeValue & 0xFF00) >> 8);
    lsb = (writeValue & 0x00FF);
    uint8_t data[3] = {registerAddress, msb, lsb};
    i2c.write(deviceAddress, reinterpret_cast<char*>(data), 3);
}

// writeBytes(uint8_t firstRegisterAddress, uint8_t * writeArray, uint8_t length)
//  This function writes an array of uint8_ts, beggining at a specific adddress
//  - firstRegisterAddress is the initial register to be written.
//  - All writes following will be at incremental register addresses.
//  - writeArray should be an array of uint8_t values to be written.
//  - length should be the number of uint8_ts to be written.
//  - No return value.
void SensorBar::writeBytes(uint8_t firstRegisterAddress, const uint8_t * writeArray, uint8_t length)
{
    uint8_t data[10] = {};
    data[0] = firstRegisterAddress;
    for(int i = 0; i < length; i++) {
        data[1+i] = writeArray[i];
    }
    i2c.write(deviceAddress, reinterpret_cast<char*>(data), length+1);
}

void SensorBar::updateAsThread()
{
    while(true) {
        ThisThread::flags_wait_any(threadFlag);
        update();
    }
}

float SensorBar::updateAngleRad()
{
    int8_t binaryPosition  = getBinaryPosition();
    float position = static_cast<float>(binaryPosition) / 127.0f * 0.0445f; // 0.0445 m is half of sensor length
    return atan2f(position, distAxisToSensor);
}

uint8_t SensorBar::updateNrOfLedsActive()
{
    uint8_t bitsCounted = 0;
    uint8_t i;

    //count bits
    for ( i = 0; i < 8; i++ ) {
        if ( ((lastBarRawValue >> i) & 0x01) == 1 ) {
            bitsCounted++;
        }
    }
    return bitsCounted;
}

float SensorBar::constrainIntoZeroToOne(float val) const
{
    return val < 0.0f ? 0.0f : val > 1.0f ? 1.0f : val;
}

void SensorBar::sendThreadFlag()
{
    thread.flags_set(threadFlag);
}
