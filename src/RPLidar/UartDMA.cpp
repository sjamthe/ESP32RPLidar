#include "HardwareSerial.h"
#include "esp32-hal.h"
#include "driver/uart.h"
#include "RPLidar.h"

RingbufHandle_t uart_ring_buf;
static uint8_t temp_buffer[UART_RX_BUF_SIZE];

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
    size_t read_bytes = 0;
	size_t sends = 0;
	size_t lost_bytes = 0;
	size_t min_free_size = RING_BUFFER_SIZE;
    unsigned long startMillis = millis();

    while(1) {
      if (uart_ring_buf == NULL) {
            Serial.println("Ring buffer is NULL in uart_rx_task");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        size_t length = 0;
        ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM, &length));
        
        if(length > 0) {
            length = uart_read_bytes(UART_NUM, temp_buffer, 
                                   min(length, (size_t) UART_RX_BUF_SIZE), pdMS_TO_TICKS(20));
            if(length > 0) {
                BaseType_t retval = xRingbufferSend(uart_ring_buf, temp_buffer, length, pdMS_TO_TICKS(10));
				if(retval == pdTRUE) {
					read_bytes += length;
					sends++;
				} else { // ran out of buffer space.
					 lost_bytes += length; 
				}
				size_t free_size = xRingbufferGetCurFreeSize(uart_ring_buf);
				if(free_size < min_free_size) min_free_size = free_size;
            }
        }
        unsigned long now = millis();
	    if ((read_bytes+lost_bytes) >= 132000) {

			Serial.printf("\nSends: %d, Bytes read:%d, lost:%d, bps: %5.0f min_free_size: %d\n"
            ,sends, read_bytes, lost_bytes, read_bytes*9*1000.0/(now - startMillis), min_free_size);
			read_bytes = 0;
			sends = 0;
			lost_bytes = 0;
			startMillis = now;
        }
        vTaskDelay(pdMS_TO_TICKS(1));  // Prevent tight loop
    }
}

// Queue for communication between processing and publishing tasks
QueueHandle_t publish_queue;

#define SCANS_PER_PUBLISH 800  // For 10 QPS
#define MAX_MEASUREMENTS_PER_BATCH 1600  // Reduced size to prevent memory issues
#define PUBLISH_QUEUE_SIZE 3

struct LaserScanBatch {
    MeasurementData* measurements;
    size_t max_measurements;
    size_t total_measurements;
    size_t total_rotations;  // Track complete rotations instead of individual scans
};

void process_data_task(void *arg) {
    task_params_t* params = (task_params_t*)arg;
    RPLidar *rplidar = params->lidar;
    
    uint8_t tempBuffer[sizeof(sl_lidar_response_ultra_capsule_measurement_nodes_t)];
    size_t processPos = 0;
    
    // Create initial batch
    LaserScanBatch* currentBatch = new LaserScanBatch();
    if (!currentBatch) {
        Serial.println("Failed to allocate LaserScanBatch");
        return;
    }
    
    currentBatch->measurements = (MeasurementData*)malloc(MAX_MEASUREMENTS_PER_BATCH * sizeof(MeasurementData));
    if (!currentBatch->measurements) {
        Serial.println("Failed to allocate measurements array");
        delete currentBatch;
        return;
    }
    
    currentBatch->max_measurements = MAX_MEASUREMENTS_PER_BATCH;
    currentBatch->total_measurements = 0;
    currentBatch->total_rotations = 0;

    Serial.printf("Free heap before loop: %d\n", ESP.getFreeHeap());

    while(1) {
        size_t item_size;
        //Read buffer comnig from Uart task.
        uint8_t *item = (uint8_t *)xRingbufferReceive(uart_ring_buf, &item_size, pdMS_TO_TICKS(10));

        if(item != NULL) {
            // parse every byte of the buffer looking for the beginning of the node package 
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
                        
                        // If we got a complete node (132 bytes) verify checksum.
                        if(processPos == sizeof(sl_lidar_response_ultra_capsule_measurement_nodes_t)) {
                            sl_lidar_response_ultra_capsule_measurement_nodes_t* node = 
                                (sl_lidar_response_ultra_capsule_measurement_nodes_t*)tempBuffer;
                                
                            uint8_t checksum = 0;
                            uint8_t recvChecksum = ((node->s_checksum_1 & 0xF) | (node->s_checksum_2 << 4));
                            
                            for(size_t cpos = offsetof(sl_lidar_response_ultra_capsule_measurement_nodes_t, start_angle_sync_q6);
                                cpos < sizeof(sl_lidar_response_ultra_capsule_measurement_nodes_t); ++cpos) {
                                checksum ^= tempBuffer[cpos];
                            }
                            processPos = 0;
                            //if checksum is ok extract measurementdata (96 measurements) from this node.
                            if(recvChecksum == checksum) {
                                    MeasurementData measurements[rplidar->EXPRESS_MEASUREMENTS_PER_SCAN];
                                size_t count = 0;
                                rplidar->ultraCapsuleToNormal(*node, measurements, count);
                                
                                for(size_t i = 0; i < count; i++) {
                                    // First, add the measurement. not sure what currentBatch->max_measurements is for 
                                    if(currentBatch->total_measurements < currentBatch->max_measurements) {
                                        currentBatch->measurements[currentBatch->total_measurements++] = measurements[i];
                                    }
                                    
                                    // Then check if it's a start flag
                                    if(measurements[i].startFlag) {
                                        currentBatch->total_rotations++;
                                    }
                                        
                                    // Check if we have enough measurements (roughly SCANS_PER_PUBLISH)
                                    //if(currentBatch->total_rotations >= 12) {
                                    if(currentBatch->total_measurements >= SCANS_PER_PUBLISH) {
                                        // Queue the batch
                                        if(xQueueSend(publish_queue, &currentBatch, 0) != pdTRUE) {
                                            // Queue full, clean up
                                            free(currentBatch->measurements);
                                            delete currentBatch;
                                            Serial.println("Queue full, batch dropped");
                                        }
                                        
                                        // Create new batch
                                        currentBatch = new LaserScanBatch();
                                        if (!currentBatch) {
                                            Serial.println("Failed to allocate new batch");
                                            continue;
                                        }
                                        
                                        currentBatch->measurements = (MeasurementData*)malloc(MAX_MEASUREMENTS_PER_BATCH * sizeof(MeasurementData));
                                        if (!currentBatch->measurements) {
                                            Serial.println("Failed to allocate measurements array");
                                            delete currentBatch;
                                            continue;
                                        }
                                        
                                        currentBatch->max_measurements = MAX_MEASUREMENTS_PER_BATCH;
                                        currentBatch->total_measurements = 0;
                                        currentBatch->total_rotations = 0;
                                    }

                                }
                            }
						}
                        break;
                }
            }
            vRingbufferReturnItem(uart_ring_buf, item);
        }
        /* Periodically print heap info
        static uint32_t last_heap_check = 0;
        if (millis() - last_heap_check > 5000) {
            Serial.printf("Free heap: %d\n", ESP.getFreeHeap());
            Serial.printf("Current batch - measurements: %d, rotations: %d\n", 
                currentBatch->total_measurements, 
                currentBatch->total_rotations);
            last_heap_check = millis();
       }*/
    }
}

// Modified publish task
void publish_task(void *arg) {
    static unsigned long total_measurements = 0;
    static unsigned long total_rotations = 0;
    static unsigned long total_messages = 0;
    unsigned long start_ms = 0;
    const TickType_t xFrequency = pdMS_TO_TICKS(100);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while(1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        LaserScanBatch* batch_to_publish;
        if(xQueueReceive(publish_queue, &batch_to_publish, 0) == pdTRUE) {
            // Process the batch
            // Future: ROS2 publishing code here
            
            if(!start_ms) start_ms = millis();
            total_measurements += batch_to_publish->total_measurements;
            total_rotations += batch_to_publish->total_rotations;
            total_messages++;
            
            if((millis() - start_ms) >= 10000) {
                Serial.printf("Published: %d, Rotations: %d, Measurements: %d, Rate: %.1f measurements/s\n",
                    total_messages,
                    total_rotations,
                    total_measurements,
                    1000.0 * total_measurements / (millis() - start_ms));
                total_measurements = 0;
                total_rotations = 0;
                total_messages = 0;
                start_ms = millis();
            }
            
            // Clean up
            free(batch_to_publish->measurements);
            delete batch_to_publish;
        }
    }
}

// Modified setup function
void setupUartTasks(RPLidar* lidar) {
    // Create the publish queue
    publish_queue = xQueueCreate(PUBLISH_QUEUE_SIZE, sizeof(LaserScanBatch*));
    if(publish_queue == NULL) {
        Serial.println("Failed to create publish queue");
        return;
    }

    // Create tasks on Core 1
    xTaskCreatePinnedToCore(uart_rx_task, "uart_rx", 2048, NULL, 5, NULL, 1);

    task_params_t* params = (task_params_t*)malloc(sizeof(task_params_t));
    if(params == NULL) {
        Serial.println("Error: params is NULL");
        return;
    }
    params->lidar = lidar;
    // callback assignment removed
    
    xTaskCreatePinnedToCore(process_data_task, "process_data", 8192, (void*)params, 5, NULL, 1);
    xTaskCreatePinnedToCore(publish_task, "publish", 4096, NULL, 4, NULL, 1);
}