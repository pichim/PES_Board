// Threaded SPISlaveCom Implementation - Copy to src/main.cpp for robot control
// This version uses a custom threaded approach compatible with Pi timing
// Better for integrating with robot control systems and other threads

#include "mbed.h"
#include "SPISlaveCom/SPISlaveCom.h"
#include <cstring>

using namespace std::chrono;

// Global communication state
struct ThreadedSPIState {
    SPIData received_data;
    SPIData reply_data;
    bool new_data_available = false;
    Mutex data_mutex;
    
    // SPI buffers
    uint8_t buffer_rx[SPI_MSG_SIZE];
    uint8_t buffer_tx[SPI_MSG_SIZE];
    int byte_index = 0;
    bool message_started = false;
    
    // Timing
    Timer timer;
    microseconds time_previous{0};
    
    // Statistics
    uint32_t message_count = 0;
    uint32_t failed_count = 0;
} spi_state;

// Simple checksum functions
uint8_t calculate_checksum(const uint8_t* buffer, size_t length) {
    uint8_t checksum = 0;
    for (size_t i = 0; i < length; i++) {
        checksum ^= buffer[i];
    }
    return checksum;
}

bool verify_checksum(const uint8_t* buffer, size_t length, uint8_t expected_checksum) {
    return calculate_checksum(buffer, length) == expected_checksum;
}

// SPI communication thread function
void spi_thread_function() {
    // Initialize SPI slave
    SPISlave spi(PC_3, PC_2, PB_10, PB_4);
    spi.format(8, 0);
    
    // Initialize reply data
    spi_state.reply_data.data[0] = 12.34f;
    spi_state.reply_data.data[1] = 56.78f;
    spi_state.reply_data.data[2] = 90.12f;
    
    // Prepare initial reply message
    spi_state.buffer_tx[0] = SPI_HEADER_SLAVE;
    memcpy(&spi_state.buffer_tx[1], spi_state.reply_data.data, SPI_NUM_FLOATS * sizeof(float));
    spi_state.buffer_tx[SPI_MSG_SIZE - 1] = calculate_checksum(spi_state.buffer_tx, SPI_MSG_SIZE - 1);
    spi.reply(spi_state.buffer_tx[0]);
    
    spi_state.timer.start();
    spi_state.time_previous = spi_state.timer.elapsed_time();
    
    printf("SPI Thread: Started real-time communication\r\n");
    
    while (true) {
        if (spi.receive()) {
            uint8_t byte = spi.read();
            
            if (!spi_state.message_started) {
                // Look for header
                if (byte == SPI_HEADER_MASTER) {
                    spi_state.message_started = true;
                    spi_state.buffer_rx[0] = byte;
                    spi_state.byte_index = 1;
                    // Prepare reply for next byte
                    if (spi_state.byte_index < SPI_MSG_SIZE) {
                        spi.reply(spi_state.buffer_tx[spi_state.byte_index]);
                    }
                } else {
                    // Invalid header, ignore and prepare for next potential header
                    spi.reply(spi_state.buffer_tx[0]);
                }
            } else {
                // Collecting message bytes
                spi_state.buffer_rx[spi_state.byte_index] = byte;
                spi_state.byte_index++;
                
                // Prepare reply for NEXT byte
                if (spi_state.byte_index < SPI_MSG_SIZE) {
                    spi.reply(spi_state.buffer_tx[spi_state.byte_index]);
                }
                
                // If complete message received
                if (spi_state.byte_index == SPI_MSG_SIZE) {
                    uint8_t header_received = spi_state.buffer_rx[0];
                    uint8_t received_checksum = spi_state.buffer_rx[SPI_MSG_SIZE - 1];
                    
                    // Verify checksum
                    if (verify_checksum(spi_state.buffer_rx, SPI_MSG_SIZE - 1, received_checksum)) {
                        // Valid message - extract data
                        spi_state.data_mutex.lock();
                        
                        memcpy(spi_state.received_data.data, &spi_state.buffer_rx[1], SPI_NUM_FLOATS * sizeof(float));
                        spi_state.message_count++;
                        spi_state.received_data.message_count = spi_state.message_count;
                        spi_state.received_data.failed_count = spi_state.failed_count;
                        
                        // Measure elapsed time
                        const microseconds time = spi_state.timer.elapsed_time();
                        const uint32_t delta_time = duration_cast<microseconds>(time - spi_state.time_previous).count();
                        spi_state.time_previous = time;
                        spi_state.received_data.last_delta_time_us = delta_time;
                        
                        spi_state.new_data_available = true;
                        
                        // Update reply message with current data
                        spi_state.buffer_tx[0] = SPI_HEADER_SLAVE;
                        memcpy(&spi_state.buffer_tx[1], spi_state.reply_data.data, SPI_NUM_FLOATS * sizeof(float));
                        spi_state.buffer_tx[SPI_MSG_SIZE - 1] = calculate_checksum(spi_state.buffer_tx, SPI_MSG_SIZE - 1);
                        
                        spi_state.data_mutex.unlock();
                        
                        // Print results (Python-compatible format)
                        printf("Message: %lu | Delta Time: %lu us | "
                               "Received: [%.2f, %.2f, %.2f] | "
                               "Header: 0x%02X | Failed: %lu\r\n",
                               spi_state.message_count, delta_time,
                               spi_state.received_data.data[0], spi_state.received_data.data[1], spi_state.received_data.data[2],
                               header_received, spi_state.failed_count);
                    } else {
                        // Invalid checksum
                        spi_state.data_mutex.lock();
                        spi_state.failed_count++;
                        spi_state.data_mutex.unlock();
                        
                        printf("Checksum failed! Expected: 0x%02X, Got: 0x%02X | Failed: %lu\r\n",
                               calculate_checksum(spi_state.buffer_rx, SPI_MSG_SIZE - 1), received_checksum, spi_state.failed_count);
                    }
                    
                    // Reset for next message
                    spi_state.message_started = false;
                    spi_state.byte_index = 0;
                    spi.reply(spi_state.buffer_tx[0]); // preload first byte for next message
                }
            }
        }
    }
}

int main() {
    // Status LED
    DigitalOut led(LED1);
    
    printf("SPI Communication Test Started (Custom Threaded)\r\n");
    printf("Starting SPI communication thread...\r\n");
    
    // Start SPI communication thread
    Thread spi_thread;
    spi_thread.start(spi_thread_function);
    
    printf("Waiting for data from Raspberry Pi...\r\n");
    
    Timer deltaTimer;
    deltaTimer.start();
    
    while (true) {
        // Check for new data from SPI thread
        spi_state.data_mutex.lock();
        bool has_new_data = spi_state.new_data_available;
        SPIData received_data;
        if (has_new_data) {
            received_data = spi_state.received_data;
            spi_state.new_data_available = false;
        }
        spi_state.data_mutex.unlock();
        
        if (has_new_data) {
            // Blink LED to indicate communication
            led = !led;
            
            // Here you can process the received data for robot control
            // Example: robot.setTargetPosition(received_data.data[0], received_data.data[1], received_data.data[2]);
        }
        
        // Update reply data if needed (robot status, sensor readings, etc.)
        // spi_state.data_mutex.lock();
        // spi_state.reply_data.data[0] = robot.getCurrentX();
        // spi_state.reply_data.data[1] = robot.getCurrentY();
        // spi_state.reply_data.data[2] = robot.getCurrentTheta();
        // spi_state.data_mutex.unlock();
        
        // 50Hz main control loop
        ThisThread::sleep_for(20ms);
    }
}
