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

void process_data_task(void *arg) {
    task_params_t* params = (task_params_t*)arg;
    node_callback_t node_callback = params->callback;
    RPLidar *rplidar = params->lidar;
    
    uint8_t tempBuffer[sizeof(sl_lidar_response_ultra_capsule_measurement_nodes_t)];  // Temporary buffer for processing chunks
    size_t processPos = 0;    // Position in current processing chunk
    uint32_t lastProcessTime = millis();

    // Buffer for accumulating nodes
    //sl_lidar_response_ultra_capsule_measurement_nodes_t nodeBuffer[10];
    size_t nodeCount = 0;
    
    while(1) {
        size_t item_size;
        uint8_t *item = (uint8_t *)xRingbufferReceive(uart_ring_buf, &item_size, pdMS_TO_TICKS(100));
        
        /* //Check timeout
        uint32_t currentTime = millis();
        if (currentTime - lastProcessTime >= 100 && nodeCount > 0) {
            node_callback(nodeBuffer, nodeCount);
            nodeCount = 0;
            lastProcessTime = currentTime;
        }  */

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

                            // Process node if checksum is correct
                            MeasurementData measurements[rplidar->EXPRESS_MEASUREMENTS_PER_SCAN];
                            size_t count = 0;

                            if(recvChecksum == checksum) {
                                rplidar->ultraCapsuleToNormal(*node, measurements, count);
                                node_callback(measurements, count);
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

static unsigned long scans = 0;
unsigned long start_ms = 0;

void process_nodes(MeasurementData* measurements, size_t count) {
    //Serial.printf("node_callback for nodes: %d\n",count);
	for(int i=0; i<count; i++) {
		if(measurements[i].startFlag) {
			//Serial.print(".");
			// publish message here.
			vTaskDelay(pdMS_TO_TICKS(100));

		}
	}
	if(!start_ms) start_ms = millis();
	scans += count;
	if((millis() - start_ms) >= 10000) {
		Serial.printf(" Scans published:%5.0f\n", 1000.0*scans/(millis() - start_ms));
		scans = 0;
		start_ms = millis();
	}

	// Test buffer overflow scenario 
	// RING_BUFFER_SIZE 2048
	// 80ms - min_free_size = 1088
	// 100ms - min_free_size = 0, 100+ send fails causes problems soon with bps going down.
	// RING_BUFFER_SIZE 5120
	// 100ms min_free_size - 15, send fail 60-80 // barely makes it.
	// RING_BUFFER_SIZE 10240
	// 80ms min_free_size = 8496 fails 0 
	// 90ms min_free_size = 16 fails 90 
	// 100ms min_free_size = 416, send fail 0 // should work.

}

void setupUartTasks(RPLidar* lidar) {
    //xTaskCreate(uart_rx_task, "uart_rx", 2048, NULL, 5, NULL);
	xTaskCreatePinnedToCore(
		uart_rx_task,          // Task function
		"uart_rx",            // Name of task
		2048,                 // Stack size
		NULL,                 // Task input parameter
		5,                    // Priority
		NULL,                 // Task handle
		1                     // Core where the task should run (1)
	);

    task_params_t* params = (task_params_t*)malloc(sizeof(task_params_t));
    if(params == NULL) {
        Serial.println("Error: params is NULL");
        return;
    }
    params->lidar = lidar;
    params->callback = process_nodes;
    //xTaskCreate(process_data_task, "process_data", 8192, (void*)params, 4, NULL);
	xTaskCreatePinnedToCore(
		process_data_task,     // Task function
		"process_data",       // Name of task
		8192,                 // Stack size
		(void*)params,        // Task input parameter
		5,                    // Same priority as UART task since we want fair scheduling
		NULL,                 // Task handle
		1                     // Core where the task should run (1)
	);
}