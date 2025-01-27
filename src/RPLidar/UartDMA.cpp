#include "HardwareSerial.h"
#include "esp32-hal.h"
#include "driver/uart.h"
#include "RPLidar.h"

RingbufHandle_t uart_ring_buf;
static uint8_t temp_buffer[UART_RX_BUF_SIZE];

typedef void (*node_callback_t)(sl_lidar_response_ultra_capsule_measurement_nodes_t* nodes, size_t count);

typedef struct {
    node_callback_t callback;
    // Add other parameters if needed
} task_params_t;

void setup_uart_dma(void) {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, UART_RX_BUF_SIZE, 0, 0, NULL, 0));
    
    // Create ring buffer
    uart_ring_buf = xRingbufferCreate(RING_BUFFER_SIZE, RINGBUF_TYPE_BYTEBUF);
    if (uart_ring_buf == NULL) {
      Serial.println("Failed to create ring buffer");
      // Handle error
    } else {
      Serial.println("uart_ring_buf created");
    }
}

// Task that reads from UART and puts into ring buffer
void uart_rx_task(void *arg) {
    size_t total_bytes = 0;
	size_t sends = 0;
	size_t fails = 0;
    unsigned long startMillis = millis();

    while(1) {
      if (uart_ring_buf == NULL) {
            Serial.println("Ring buffer is NULL in uart_rx_task");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        size_t length = 0;
        ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM, &length));
        
        if(length >= 0) {
            length = uart_read_bytes(UART_NUM, temp_buffer, 
                                   min(length, (size_t) UART_RX_BUF_SIZE), pdMS_TO_TICKS(20));
            if(length > 0) {
                xRingbufferSend(uart_ring_buf, temp_buffer, length, pdMS_TO_TICKS(10));
				total_bytes += length;
				sends++;
            }
        }
        unsigned long now = millis();
	    if (total_bytes >= 132000) {
			Serial.printf("Sends: %d, Bytes read:%d, bps: %5.0f\n",sends, total_bytes, total_bytes*9*1000.0/(now - startMillis));
			total_bytes = 0;
			sends = 0;
			startMillis = now;
        }
        vTaskDelay(pdMS_TO_TICKS(1));  // Prevent tight loop
    }
}

void process_data_task(void *arg) {
    task_params_t* params = (task_params_t*)arg;
    node_callback_t node_callback = params->callback;
    
    uint8_t tempBuffer[sizeof(sl_lidar_response_ultra_capsule_measurement_nodes_t)];  // Temporary buffer for processing chunks
    size_t processPos = 0;    // Position in current processing chunk
    uint32_t lastProcessTime = millis();

    // Buffer for accumulating nodes
    sl_lidar_response_ultra_capsule_measurement_nodes_t nodeBuffer[10];
    size_t nodeCount = 0;
    
    while(1) {
        size_t item_size;
        uint8_t *item = (uint8_t *)xRingbufferReceive(uart_ring_buf, &item_size, pdMS_TO_TICKS(100));
        
        // Check timeout
        uint32_t currentTime = millis();
        if (currentTime - lastProcessTime >= 100 && nodeCount > 0) {
            node_callback(nodeBuffer, nodeCount);
            nodeCount = 0;
            lastProcessTime = currentTime;
        }  

        if(item != NULL) {
            uint32_t startTs = millis();
            
            for(size_t i = 0; i < item_size; i++) {
                uint8_t currentByte = item[i];
                
                switch(processPos) {
                    case 0: {
                        uint8_t tmp = (currentByte >> 4);
                        if(tmp != RPLIDAR_RESP_MEASUREMENT_EXP_SYNC_1) {
                            continue;
                        }
                        tempBuffer[processPos++] = currentByte;
                        break;
                    }
                    case 1: {
                        uint8_t tmp = (currentByte >> 4);
                        if(tmp != RPLIDAR_RESP_MEASUREMENT_EXP_SYNC_2) {
                            processPos = 0;
                            continue;
                        }
                        tempBuffer[processPos++] = currentByte;
                        break;
                    }
                    default:
                        tempBuffer[processPos++] = currentByte;
                        
                        if(processPos == sizeof(sl_lidar_response_ultra_capsule_measurement_nodes_t)) {
                            // Process complete packet
                            sl_lidar_response_ultra_capsule_measurement_nodes_t* node = 
                                (sl_lidar_response_ultra_capsule_measurement_nodes_t*)tempBuffer;
                                
                            uint8_t checksum = 0;
                            uint8_t recvChecksum = ((node->s_checksum_1 & 0xF) | (node->s_checksum_2 << 4));
                            
                            for(size_t cpos = offsetof(sl_lidar_response_ultra_capsule_measurement_nodes_t, start_angle_sync_q6);
                                cpos < sizeof(sl_lidar_response_ultra_capsule_measurement_nodes_t); ++cpos) {
                                checksum ^= tempBuffer[cpos];
                            }
                            
                            if(recvChecksum == checksum) {
                                memcpy(&nodeBuffer[nodeCount++], node, sizeof(sl_lidar_response_ultra_capsule_measurement_nodes_t));
                                
                                if(nodeCount >= 10) {
                                    node_callback(nodeBuffer, nodeCount);
                                    nodeCount = 0;
                                    lastProcessTime = currentTime;
                                }
                            }
                            
                            processPos = 0;  // Reset for next packet
                        }
                        break;
                }
            }
            
            vRingbufferReturnItem(uart_ring_buf, item);
        }
    }
}