#include "SPISlaveCom.h"

using namespace std::chrono;

bool SPISlaveCom::m_timer_started = false;

SPISlaveCom::SPISlaveCom(PinName mosi_pin, PinName miso_pin, PinName sck_pin, PinName cs_pin) :
    m_spi(mosi_pin, miso_pin, sck_pin, cs_pin),
    m_cs_pin(cs_pin),
    m_byte_index(0),
    m_message_started(false),
    m_new_setpoints_available(false)
{
    // Initialize SPI in slave mode
    m_spi.format(8, 0);  // 8-bit, mode 0
    // Note: SPISlave frequency is set by master
    
    // Initialize reply data with default robot status
    m_reply_data.data[0] = 0.0f;  // Current X position
    m_reply_data.data[1] = 0.0f;  // Current Y position  
    m_reply_data.data[2] = 0.0f;  // Current theta (orientation)
    m_reply_data.message_count = 0;
    m_reply_data.failed_count = 0;
    m_reply_data.last_delta_time_us = 0;
    
    // Prepare initial reply message
    prepareReplyMessage();
    
    // Start timer for timing measurements
    if (!m_timer_started) {
        m_timer.start();
        m_timer_started = true;
    }
    m_time_previous = m_timer.elapsed_time();
    
    // Print startup message similar to Python pinout
    printf("# Description        : Nucleo F446RE SPI Slave\r\n");
    printf("# SPI Configuration  : Slave Mode\r\n");
    printf("# Message Size       : %d bytes\r\n", SPI_MSG_SIZE);
    printf("# Float Count        : %d\r\n", SPI_NUM_FLOATS);
    printf("# Master Header      : 0x%02X\r\n", SPI_HEADER_MASTER);
    printf("# Slave Header       : 0x%02X\r\n", SPI_HEADER_SLAVE);
    printf("#\r\n");
    printf("# SPI Pin Configuration:\r\n");
    printf("# MOSI: %s\r\n", "PC_3");
    printf("# MISO: %s\r\n", "PC_2");
    printf("# SCK:  %s\r\n", "PB_10");
    printf("# CS:   %s\r\n", "PB_4");
    printf("#\r\n");
    printf("# Starting SPI communication...\r\n");
    printf("#\r\n");
    
    // Start thread
    m_thread.start(callback(this, &SPISlaveCom::threadTask));
}

SPISlaveCom::~SPISlaveCom()
{
    m_thread.terminate();   // Stop thread
}

bool SPISlaveCom::hasNewSetpoints()
{
    m_data_mutex.lock();
    bool result = m_new_setpoints_available;
    m_data_mutex.unlock();
    return result;
}

SPIData SPISlaveCom::getSetpoints()
{
    m_data_mutex.lock();
    SPIData result = m_received_data;
    m_new_setpoints_available = false;  // Mark as read
    m_data_mutex.unlock();
    return result;
}

void SPISlaveCom::updateReplyData(const SPIData& reply_data)
{
    m_data_mutex.lock();
    m_reply_data = reply_data;
    // Keep communication statistics from current reply data
    m_reply_data.message_count = m_received_data.message_count;
    m_reply_data.failed_count = m_received_data.failed_count;
    m_reply_data.last_delta_time_us = m_received_data.last_delta_time_us;
    prepareReplyMessage();
    m_data_mutex.unlock();
}

void SPISlaveCom::threadTask()
{
    while (true) {
        // Poll for SPI activity
        pollSPI();
        
        // Wait for signal or timeout
        uint32_t flags = ThisThread::flags_wait_any_for(0x01, 10ms);
        
        if (flags & 0x01) {
            // Process complete message
            processCompleteMessage();
        }
    }
}

void SPISlaveCom::pollSPI()
{
    // Check if CS is active (low)
    if (m_cs_pin.read() == 0) {
        if (!m_message_started) {
            m_message_started = true;
            m_byte_index = 0;
            // Prepare first reply byte
            m_spi.reply(m_buffer_tx[0]);
        }
        
        // Check if SPI has data
        if (m_spi.receive()) {
            uint8_t received_byte = m_spi.read();
            
            if (m_byte_index < SPI_MSG_SIZE) {
                m_buffer_rx[m_byte_index] = received_byte;
                m_byte_index++;
                
                // Prepare next reply byte
                if (m_byte_index < SPI_MSG_SIZE) {
                    m_spi.reply(m_buffer_tx[m_byte_index]);
                }
                
                // Check if message is complete
                if (m_byte_index >= SPI_MSG_SIZE) {
                    m_message_started = false;
                    // Signal thread to process message
                    m_thread.flags_set(0x01);
                }
            }
        }
    } else if (m_message_started) {
        // CS went high, reset for next message
        m_message_started = false;
        m_byte_index = 0;
    }
}

void SPISlaveCom::processCompleteMessage()
{
    // Create local copy for processing
    uint8_t local_buffer[SPI_MSG_SIZE];
    for (int i = 0; i < SPI_MSG_SIZE; i++) {
        local_buffer[i] = m_buffer_rx[i];
    }
    
    // Check header
    uint8_t header_received = local_buffer[0];
    uint8_t received_checksum = local_buffer[SPI_MSG_SIZE - 1];
    
    // Verify checksum (exclude checksum byte itself)
    uint8_t calculated_checksum = calculate_checksum(local_buffer, SPI_MSG_SIZE - 1);
    
    if (!verify_checksum(local_buffer, SPI_MSG_SIZE - 1, received_checksum)) {
        m_data_mutex.lock();
        m_received_data.failed_count++;
        m_data_mutex.unlock();
        printCommunicationStatus(false, header_received, calculated_checksum, received_checksum);
        return;
    }
    
    if (header_received != SPI_HEADER_MASTER) {
        m_data_mutex.lock();
        m_received_data.failed_count++;
        m_data_mutex.unlock();
        printCommunicationStatus(false, header_received);
        return;
    }
    
    // Calculate timing
    microseconds current_time = m_timer.elapsed_time();
    microseconds delta_time = current_time - m_time_previous;
    m_time_previous = current_time;
    
    // Extract setpoint data (3 floats after header)
    float* data_ptr = (float*)(local_buffer + 1);
    
    m_data_mutex.lock();
    m_received_data.data[0] = data_ptr[0];  // x setpoint
    m_received_data.data[1] = data_ptr[1];  // y setpoint  
    m_received_data.data[2] = data_ptr[2];  // theta setpoint
    m_received_data.message_count++;
    m_received_data.last_delta_time_us = duration_cast<microseconds>(delta_time).count();
    m_new_setpoints_available = true;
    
    // Prepare next reply message
    prepareReplyMessage();
    m_data_mutex.unlock();
    
    // Print successful communication in Python-compatible format
    printCommunicationStatus(true, header_received);
}

void SPISlaveCom::prepareReplyMessage()
{
    // Prepare reply message with robot status
    m_buffer_tx[0] = SPI_HEADER_SLAVE;
    
    // Copy float data (3 floats = 12 bytes)
    float* data_ptr = (float*)(m_buffer_tx + 1);
    data_ptr[0] = m_reply_data.data[0];  // current x
    data_ptr[1] = m_reply_data.data[1];  // current y
    data_ptr[2] = m_reply_data.data[2];  // current theta
    
    // Calculate and set checksum (exclude checksum byte itself)
    uint8_t checksum = calculate_checksum((const uint8_t*)m_buffer_tx, SPI_MSG_SIZE - 1);
    m_buffer_tx[SPI_MSG_SIZE - 1] = checksum;
}

void SPISlaveCom::printCommunicationStatus(bool success, uint8_t header_received, uint8_t expected_checksum, uint8_t received_checksum)
{
    if (success) {
        // Print success message in Python-compatible format
        printf("Message: %lu | "
               "Delta Time: %lu us | "
               "Received: [%.2f, %.2f, %.2f] | "
               "Header: 0x%02X | Failed: %lu\r\n",
               (unsigned long)m_received_data.message_count,
               (unsigned long)m_received_data.last_delta_time_us,
               m_received_data.data[0], m_received_data.data[1], m_received_data.data[2],
               header_received,
               (unsigned long)m_received_data.failed_count);
    } else {
        if (expected_checksum != 0 && received_checksum != 0) {
            // Checksum failure
            printf("Checksum failed! Expected: 0x%02X, Got: 0x%02X | Failed: %lu\r\n",
                   expected_checksum, received_checksum,
                   (unsigned long)m_received_data.failed_count);
        } else {
            // Header failure
            printf("Wrong header! Expected: 0x%02X, Got: 0x%02X | Failed: %lu\r\n",
                   SPI_HEADER_MASTER, header_received,
                   (unsigned long)m_received_data.failed_count);
        }
    }
}
