/*
 * SimpleLIN.h - Robust LIN bus implementation for ESP32-S3
 * Focuses on reliability and correct break field generation
 *
 * Features:
 * - Multiple break generation methods with automatic fallback
 * - LIN 2.0 enhanced checksum
 *
 * Copyright (C) 2024-2026 [Mihir Sanjay Fulari]
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef __SimpleLIN_h
#define __SimpleLIN_h

#include <Arduino.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_timer.h"

// LIN Protocol Constants
#define LIN_SYNC_BYTE 0x55
#define LIN_BREAK_BITS 13
#define LIN_DELIMITER_BITS 1

class SimpleLIN {
public:
    enum BreakMethod {
        BREAK_METHOD_AUTO = 0,      // Try hardware first, fallback to GPIO
        BREAK_METHOD_HARDWARE,      // Use UART hardware break (uart_set_break)
        BREAK_METHOD_GPIO,          // Bit-bang via GPIO
        BREAK_METHOD_BAUDRATE       // Send 0x00 at lower baud rate
    };

    enum ChecksumType {
        CHECKSUM_CLASSIC = 0,       // LIN 1.x - data only
        CHECKSUM_ENHANCED = 1       // LIN 2.x - PID + data (default)
    };

    /**
     * @brief Constructor
     * @param uart_num UART port (0, 1, 2)
     * @param tx_pin TX GPIO pin
     * @param rx_pin RX GPIO pin
     */
    SimpleLIN(uart_port_t uart_num, int tx_pin, int rx_pin)
        : m_uart(uart_num)
        , m_tx_pin(tx_pin)
        , m_rx_pin(rx_pin)
        , m_baudrate(19200)
        , m_break_bits(LIN_BREAK_BITS)
        , m_sync_byte(LIN_SYNC_BYTE)
        , m_break_method(BREAK_METHOD_AUTO)
        , m_checksum_type(CHECKSUM_ENHANCED)
        , m_initialized(false)
    {
    }

    /**
     * @brief Destructor - clean up resources
     */
    ~SimpleLIN() {
        end();
    }

    /**
     * @brief Initialize LIN bus
     * @param baudrate LIN baud rate (default: 19200)
     * @return true on success
     */
    bool begin(uint32_t baudrate = 19200) {
        if (m_initialized) {
            log_w("LIN already initialized");
            return true;
        }

        m_baudrate = baudrate;

        // Configure UART
        uart_config_t uart_config = {
            .baud_rate = (int)baudrate,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .rx_flow_ctrl_thresh = 122,
            .source_clk = UART_SCLK_APB,
        };

        // Install driver
        esp_err_t err = uart_driver_install(m_uart, 256, 256, 0, NULL, 0);
        if (err != ESP_OK) {
            log_e("UART driver install failed: %d", err);
            return false;
        }

        // Configure parameters
        err = uart_param_config(m_uart, &uart_config);
        if (err != ESP_OK) {
            log_e("UART config failed: %d", err);
            uart_driver_delete(m_uart);
            return false;
        }

        // Set pins
        err = uart_set_pin(m_uart, m_tx_pin, m_rx_pin, 
                           UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        if (err != ESP_OK) {
            log_e("UART pin config failed: %d", err);
            uart_driver_delete(m_uart);
            return false;
        }

        // Test break method capability
        detectBreakMethod();

        m_initialized = true;
        log_i("LIN initialized: UART%d, %lu baud, TX=%d, RX=%d", 
              m_uart, baudrate, m_tx_pin, m_rx_pin);
        return true;
    }

    /**
     * @brief Clean up and release resources
     */
    void end() {
        if (m_initialized) {
            uart_driver_delete(m_uart);
            gpio_reset_pin((gpio_num_t)m_tx_pin);
            gpio_reset_pin((gpio_num_t)m_rx_pin);
            m_initialized = false;
        }
    }

    /**
     * @brief Set number of break field bits (default: 13)
     */
    void setBreakBits(uint8_t bits) {
        if (bits >= 13) {
            m_break_bits = bits;
        } else {
            log_w("Break bits must be >= 13, keeping %d", m_break_bits);
        }
    }

    /**
     * @brief Set sync byte (default: 0x55)
     */
    void setSyncByte(uint8_t sync) {
        m_sync_byte = sync;
    }

    /**
     * @brief Set break generation method
     */
    void setBreakMethod(BreakMethod method) {
        m_break_method = method;
    }

    /**
     * @brief Set checksum type (default: Enhanced/LIN 2.0)
     */
    void setChecksumType(ChecksumType type) {
        m_checksum_type = type;
    }

    /**
     * @brief Send complete LIN frame (Master TX)
     * @param pid Protected ID (6-bit ID with parity)
     * @param data Data bytes (can be NULL for 0-length)
     * @param len Data length (0-8 bytes)
     * @return true on success
     */
    bool sendFrame(uint8_t pid, const uint8_t* data, uint8_t len) {
        if (!m_initialized) {
            log_e("LIN not initialized");
            return false;
        }

        if (len > 8) {
            log_e("Data length must be 0-8 bytes");
            return false;
        }

        // 1. Send Break Field
        if (!sendBreakField()) {
            log_e("Failed to send break field");
            return false;
        }

        // 2. Send Sync Byte
        if (uart_write_bytes(m_uart, (const char*)&m_sync_byte, 1) != 1) {
            log_e("Failed to send sync byte");
            return false;
        }

        // 3. Send PID (Protected ID with parity bits)
        uint8_t pid_with_parity = calculatePID(pid);
        if (uart_write_bytes(m_uart, (const char*)&pid_with_parity, 1) != 1) {
            log_e("Failed to send PID");
            return false;
        }   

        // 4. Send Data (if any)
        if (len > 0 && data != NULL) {
            if (uart_write_bytes(m_uart, (const char*)data, len) != len) {
                log_e("Failed to send data");
                return false;
            }
        }

        // 5. Calculate and send Checksum
        uint8_t checksum = calculateChecksum(pid_with_parity, data, len);
        if (uart_write_bytes(m_uart, (const char*)&checksum, 1) != 1) {
            log_e("Failed to send checksum");
            return false;
        }

        // Wait for transmission to complete
        uart_wait_tx_done(m_uart, pdMS_TO_TICKS(100));

        log_d("Frame sent: PID=0x%02X, Len=%d, Checksum=0x%02X", 
              pid_with_parity, len, checksum);
        return true;
    }

    /**
     * @brief Send LIN header only (Master requests data from slave)
     * @param pid Protected ID
     * @return true on success
     */
    bool sendHeader(uint8_t pid) {
        if (!m_initialized) {
            log_e("LIN not initialized");
            return false;
        }

        // 1. Send Break Field
        if (!sendBreakField()) {
            log_e("Failed to send break field");
            return false;
        }

        // 2. Send Sync Byte
        if (uart_write_bytes(m_uart, (const char*)&m_sync_byte, 1) != 1) {
            log_e("Failed to send sync byte");
            return false;
        }

        // 3. Send PID
        uint8_t pid_with_parity = calculatePID(pid);
        if (uart_write_bytes(m_uart, (const char*)&pid_with_parity, 1) != 1) {
            log_e("Failed to send PID");
            return false;
        }

        uart_wait_tx_done(m_uart, pdMS_TO_TICKS(50));

        log_d("Header sent: PID=0x%02X", pid_with_parity);
        return true;
    }

    /**
     * @brief Receive complete LIN frame from slave (with break detection)
     * @param frame_buffer Buffer to store complete frame
     * @param max_len Maximum buffer size
     * @param timeout_ms Receive timeout in milliseconds
     * @return Frame length if successful, -1 on error
     */
    int receiveFrame(uint8_t* frame_buffer, uint8_t max_len, uint32_t timeout_ms = 150) {
        if (!m_initialized || frame_buffer == NULL || max_len < 11) {
            return -1;
        }

        uint32_t start_time = millis();
        
        // 1. Detect break field
        if (!detectBreakField(timeout_ms)) {
            log_d("No break field detected");
            return -1;
        }

        // 2. Read and verify sync byte
        uint8_t sync_byte;
        int bytes_read = uart_read_bytes(m_uart, &sync_byte, 1, 
                                       pdMS_TO_TICKS(timeout_ms - (millis() - start_time)));
        if (bytes_read != 1 || sync_byte != LIN_SYNC_BYTE) {
            log_w("Invalid sync byte: 0x%02X", sync_byte);
            return -1;
        }
        frame_buffer[0] = sync_byte;

        // 3. Read PID
        uint8_t received_pid;
        bytes_read = uart_read_bytes(m_uart, &received_pid, 1, 
                                   pdMS_TO_TICKS(timeout_ms - (millis() - start_time)));
        if (bytes_read != 1) {
            return -1;
        }
        frame_buffer[1] = received_pid;

        // 4. Determine data length from PID
        uint8_t data_length = getDataLengthFromPID(received_pid & 0x3F);
        if (data_length == 0 || (2 + data_length + 1) > max_len) {
            log_w("Invalid data length: %d", data_length);
            return -1;
        }

        // 5. Read data + checksum
        int remaining_bytes = data_length + 1;
        bytes_read = uart_read_bytes(m_uart, &frame_buffer[2], remaining_bytes, 
                                   pdMS_TO_TICKS(timeout_ms - (millis() - start_time)));

        if (bytes_read != remaining_bytes) {
            log_w("Incomplete frame: expected %d, got %d", remaining_bytes, bytes_read);
            return -1;
        }

        // 6. Verify checksum
        uint8_t received_checksum = frame_buffer[2 + data_length];
        uint8_t calculated_checksum = calculateChecksum(received_pid, &frame_buffer[2], data_length);
        
        if (received_checksum != calculated_checksum) {
            log_w("Checksum error: received=0x%02X, calculated=0x%02X", 
                  received_checksum, calculated_checksum);
            return -1;
        }

        log_d("Frame received: PID=0x%02X, Len=%d, Checksum=0x%02X", 
              received_pid, data_length, received_checksum);
        
        return 2 + data_length + 1; // sync + pid + data + checksum
    }

    /**
     * @brief Calculate LIN 2.0 Enhanced Checksum
     * @param pid Protected ID (with parity)
     * @param data Data bytes
     * @param len Data length
     * @return Inverted 8-bit checksum
     */
    uint8_t calculateChecksum(uint8_t pid, const uint8_t* data, uint8_t len) {
        uint16_t sum = 0;

        // Enhanced checksum includes PID
        if (m_checksum_type == CHECKSUM_ENHANCED) {
            sum = pid;
        }

        // Add data bytes
        if (data != NULL) {
            for (uint8_t i = 0; i < len; i++) {
                sum += data[i];
                // Handle carry
                if (sum > 0xFF) {
                    sum = (sum & 0xFF) + 1;
                }
            }
        }

        // Return inverted checksum
        return (uint8_t)(~sum);
    }

    /**
     * @brief Calculate Protected ID (ID + parity bits)
     * @param id 6-bit ID (0-63)
     * @return 8-bit PID with parity
     */
    uint8_t calculatePID(uint8_t id) {
        id &= 0x3F; // Ensure only 6 bits

        // Calculate parity bits
        // P0 = ID0 ^ ID1 ^ ID2 ^ ID4
        uint8_t p0 = ((id >> 0) & 1) ^ ((id >> 1) & 1) ^ 
                     ((id >> 2) & 1) ^ ((id >> 4) & 1);
        
        // P1 = ~(ID1 ^ ID3 ^ ID4 ^ ID5)
        uint8_t p1 = ~(((id >> 1) & 1) ^ ((id >> 3) & 1) ^ 
                       ((id >> 4) & 1) ^ ((id >> 5) & 1));
        p1 &= 1;

        // Construct PID: [P1][P0][ID5:ID0]
        return id | (p0 << 6) | (p1 << 7);
    }

private:
    uart_port_t m_uart;
    int m_tx_pin;
    int m_rx_pin;
    uint32_t m_baudrate;
    uint8_t m_break_bits;
    uint8_t m_sync_byte;
    BreakMethod m_break_method;
    ChecksumType m_checksum_type;
    bool m_initialized;
    BreakMethod m_detected_method;

    /**
     * @brief Detect best break generation method
     */
    void detectBreakMethod() {
        m_detected_method = BREAK_METHOD_GPIO;
        log_i("Break method: GPIO bit-banging (most reliable)");
    }

    /**
     * @brief Send break field using selected method
     */
    bool sendBreakField() {
        BreakMethod method = (m_break_method == BREAK_METHOD_AUTO) ? 
                            m_detected_method : m_break_method;

        switch (method) {
            case BREAK_METHOD_HARDWARE:
                return sendBreakHardware();
            
            case BREAK_METHOD_GPIO:
                return sendBreakGPIO();
            
            case BREAK_METHOD_BAUDRATE:
                return sendBreakBaudrate();
            
            default:
                if (sendBreakGPIO()) return true;
                if (sendBreakHardware()) return true;
                return sendBreakBaudrate();
        }
    }

    /**
     * @brief Send break using UART hardware (uart_set_break)
     */
    bool sendBreakHardware() {
        uart_wait_tx_done(m_uart, pdMS_TO_TICKS(10));
        uart_send_break(m_uart); 
        uart_wait_tx_done(m_uart, pdMS_TO_TICKS(10));
        delayMicroseconds((LIN_DELIMITER_BITS * 1000000UL) / m_baudrate);

        log_d("Hardware break sent via uart_send_break()");
        return true;
    }

    /**
     * @brief Send break via GPIO bit-banging (MOST RELIABLE)
     */
    bool sendBreakGPIO() {
        uint32_t break_us = (m_break_bits * 1000000UL) / m_baudrate;
        uint32_t delim_us = (LIN_DELIMITER_BITS * 1000000UL) / m_baudrate;

        uart_wait_tx_done(m_uart, pdMS_TO_TICKS(10));
        gpio_matrix_out(m_tx_pin, SIG_GPIO_OUT_IDX, false, false);
        gpio_set_direction((gpio_num_t)m_tx_pin, GPIO_MODE_OUTPUT);
        gpio_set_pull_mode((gpio_num_t)m_tx_pin, GPIO_FLOATING);

        portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
        portENTER_CRITICAL(&mux);

        gpio_set_level((gpio_num_t)m_tx_pin, 0);
        delayMicroseconds(break_us);
        gpio_set_level((gpio_num_t)m_tx_pin, 1);
        delayMicroseconds(delim_us);

        portEXIT_CRITICAL(&mux);

        uart_set_pin(m_uart, m_tx_pin, UART_PIN_NO_CHANGE, 
                    UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        delayMicroseconds(50);

        log_d("Break sent via GPIO: %lu us", break_us);
        return true;
    }

    /**
     * @brief Send break by changing baud rate (FALLBACK METHOD)
     */
    bool sendBreakBaudrate() {
        uint32_t break_baud = (m_baudrate * 8) / m_break_bits; 
        uart_set_baudrate(m_uart, break_baud);
        
        uint8_t zero = 0x00;
        uart_write_bytes(m_uart, (const char*)&zero, 1);
        uart_wait_tx_done(m_uart, pdMS_TO_TICKS(10));
        
        uart_set_baudrate(m_uart, m_baudrate);
        
        log_d("Break: baud=%lu, low time improved", break_baud);
        return true;
    }

    /**
     * @brief Detect LIN break field (extended LOW period)
     */
    bool detectBreakField(uint32_t timeout_ms) {
        uint32_t start_time = millis();
        uint32_t low_start_time = 0;
        bool in_low = false;
        
        uint32_t expected_break_us = (m_break_bits * 1000000UL) / m_baudrate;
        uint32_t min_break_us = expected_break_us * 0.7;  
        uint32_t max_break_us = expected_break_us * 1.5;  
        
        while ((millis() - start_time) < timeout_ms) {
            int line_state = gpio_get_level((gpio_num_t)m_rx_pin);
            
            if (line_state == 0 && !in_low) {
                low_start_time = micros();
                in_low = true;
            } 
            else if (line_state == 1 && in_low) {
                uint32_t low_duration = micros() - low_start_time;
                in_low = false;
                
                if (low_duration >= min_break_us && low_duration <= max_break_us) {
                    log_d("Break detected: %lu us (expected: %lu us)", 
                          low_duration, expected_break_us);
                    return true;
                }
            }
            delayMicroseconds(5); 
        }
        return false;
    }

    /**
     * @brief Get expected data length from PID (LIN 2.0 specification)
     */
    uint8_t getDataLengthFromPID(uint8_t id) {
        if (id >= 0x00 && id <= 0x1F) return 2;  
        if (id >= 0x20 && id <= 0x2F) return 4;    
        if (id >= 0x30 && id <= 0x3F) return 8;  
        return 2; 
    }
};

#endif // __SimpleLIN_h