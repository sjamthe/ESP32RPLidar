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
        
        if(length >= 132) {
            length = uart_read_bytes(UART_NUM, temp_buffer, 
                                   min(length, (size_t) UART_RX_BUF_SIZE), pdMS_TO_TICKS(20));
            if(length > 0) {
                xRingbufferSend(uart_ring_buf, temp_buffer, length, pdMS_TO_TICKS(10));
				total_bytes += length;
				sends++;
            }
        }
        unsigned long now = millis();
	    if ((sends) >= 1000) {
			Serial.printf("Sends: %d, Bytes read:%d, bps: %5.0f\n",sends, total_bytes, total_bytes*9*1000.0/(now - startMillis));
			total_bytes = 0;
			sends = 0;
			startMillis = now;
        }
        vTaskDelay(pdMS_TO_TICKS(1));  // Prevent tight loop
    }
}

void process_data_task(void *arg) {
	size_t total_bytes = 0;
	size_t items = 0;
    unsigned long startMillis = millis();

    while(1) {
		if (uart_ring_buf == NULL) {
            Serial.println("Ring buffer is NULL in process_data_task");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        size_t item_size;
        uint8_t *item = (uint8_t *)xRingbufferReceive(uart_ring_buf, &item_size, pdMS_TO_TICKS(100));
        
        if(item != NULL) {
            // Process your data here
            //handleLidar();
            // Return item to ring buffer
            vRingbufferReturnItem(uart_ring_buf, item);
			items++;
        }
		total_bytes += item_size;
        unsigned long now = millis();
	    if ((items) >= 1000) {
			Serial.printf("Received: %d, Bytes received:%d, bps: %5.0f\n",items, total_bytes, total_bytes*9*1000.0/(now - startMillis));
			total_bytes = 0;
			items = 0;
			startMillis = now;
        }
		vTaskDelay(pdMS_TO_TICKS(1));  // Prevent tight loop
    }
}