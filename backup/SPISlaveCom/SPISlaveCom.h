#ifndef SPISLAVECOM_H
#define SPISLAVECOM_H

#include "mbed.h"
#include <stdint.h>
#include <cstddef>

using namespace std::chrono;

// Protocol constants (integrated from SPIProtocol)
#define SPI_HEADER_MASTER 0x55  // Raspberry Pi header
#define SPI_HEADER_SLAVE  0x45  // Nucleo header
#define SPI_NUM_FLOATS    3     // Number of float values in each message
#define SPI_MSG_SIZE      (1 + SPI_NUM_FLOATS * 4 + 1)  // header + floats + checksum

// Data structure for SPI communication (integrated from SPIProtocol)
class SPIData {
public:
    float data[SPI_NUM_FLOATS];
    uint32_t message_count;
    uint32_t failed_count;
    uint32_t last_delta_time_us;
    
    SPIData() : message_count(0), failed_count(0), last_delta_time_us(0) {
        for (int i = 0; i < SPI_NUM_FLOATS; i++) {
            data[i] = 0.0f;
        }
    }
};

/**
 * @brief Integrated SPI Slave Communication Class
 * 
 * This class provides a thread-safe, polling-based SPI slave communication interface
 * with integrated protocol handling and Python-compatible serial output.
 * 
 * Features:
 * - Background polling thread for non-blocking operation
 * - Thread-safe data access with mutex protection
 * - Integrated protocol constants and checksum handling
 * - Message validation with checksum verification
 * - Communication statistics and timing measurements
 * - Python-compatible serial output format
 * - Automatic message queuing and status feedback
 */
class SPISlaveCom
{
public:
    /**
     * @brief Construct a new SPI Slave Communication object
     * 
     * @param mosi_pin MOSI pin (Master Out, Slave In)
     * @param miso_pin MISO pin (Master In, Slave Out) 
     * @param sck_pin  SCK pin (Serial Clock)
     * @param cs_pin   CS pin (Chip Select)
     */
    SPISlaveCom(PinName mosi_pin, PinName miso_pin, PinName sck_pin, PinName cs_pin);
    
    /**
     * @brief Destroy the SPI Slave Communication object
     */
    virtual ~SPISlaveCom();
    
    /**
     * @brief Check if new setpoints are available
     * 
     * @return true if new setpoints received since last call to getSetpoints()
     * @return false if no new data available
     */
    bool hasNewSetpoints();
    
    /**
     * @brief Get the latest received setpoints
     * 
     * This function returns the most recent setpoints and marks them as read.
     * 
     * @return SPIData containing the latest setpoints and communication statistics
     */
    SPIData getSetpoints();
    
    /**
     * @brief Update the data that will be sent back to the master
     * 
     * Call this function to update robot status/feedback data that will be
     * transmitted in the next SPI exchange.
     * 
     * @param reply_data Robot status data to send back
     */
    void updateReplyData(const SPIData& reply_data);
    
    /**
     * @brief Get communication statistics
     * 
     * @return uint32_t Number of failed communications
     */
    uint32_t getFailedCount() const { return m_received_data.failed_count; }
    
    /**
     * @brief Get message count
     * 
     * @return uint32_t Total number of successfully received messages
     */
    uint32_t getMessageCount() const { return m_received_data.message_count; }
    
    /**
     * @brief Get last communication timing
     * 
     * @return uint32_t Time since last successful communication in microseconds
     */
    uint32_t getLastDeltaTime() const { return m_received_data.last_delta_time_us; }

private:
    // SPI hardware
    SPISlave m_spi;
    DigitalIn m_cs_pin;  // CS pin for polling
    
    // Threading components
    Thread m_thread;
    
    // Communication buffers
    volatile uint8_t m_buffer_rx[SPI_MSG_SIZE];
    volatile uint8_t m_buffer_tx[SPI_MSG_SIZE];
    volatile int m_byte_index;
    volatile bool m_message_started;
    static bool m_timer_started;
    
    // Data structures (protected by mutex)
    SPIData m_received_data;
    SPIData m_reply_data;
    bool m_new_setpoints_available;
    
    // Timing
    Timer m_timer;
    microseconds m_time_previous;
    
    // Thread synchronization
    Mutex m_data_mutex;
    
    // Protocol functions (integrated from SPIProtocol)
    /**
     * @brief Calculate simple checksum (XOR of all bytes)
     */
    static inline uint8_t calculate_checksum(const uint8_t* buffer, size_t length) {
        uint8_t checksum = 0;
        for (size_t i = 0; i < length; i++) {
            checksum ^= buffer[i];
        }
        return checksum;
    }
    
    /**
     * @brief Verify checksum
     */
    static inline bool verify_checksum(const uint8_t* buffer, size_t length, uint8_t expected_checksum) {
        return calculate_checksum(buffer, length) == expected_checksum;
    }
    
    /**
     * @brief Main thread task for polling SPI and processing messages
     * 
     * This function continuously polls the SPI interface and processes
     * complete messages. It runs in a background thread.
     */
    void threadTask();
    
    /**
     * @brief Poll SPI for incoming data
     * 
     * Checks CS pin and processes incoming SPI bytes. Signals the thread
     * when a complete message is received.
     */
    void pollSPI();
    
    /**
     * @brief Process a complete received message
     * 
     * Validates checksum, extracts setpoint data, updates timing,
     * and prepares the next reply message. Includes Python-compatible output.
     */
    void processCompleteMessage();
    
    /**
     * @brief Prepare the reply message with current robot status
     * 
     * Updates the transmission buffer with current reply data and checksum.
     * Must be called from thread context with proper synchronization.
     */
    void prepareReplyMessage();
    
    /**
     * @brief Print communication status in Python-compatible format
     * 
     * Mimics the Python output format for consistency between master and slave.
     */
    void printCommunicationStatus(bool success, uint8_t header_received, uint8_t expected_checksum = 0, uint8_t received_checksum = 0);
};

#endif /* SPISLAVECOM_H */
