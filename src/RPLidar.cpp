#include <stdint.h>
#include <cstdlib>
#include <cstddef>
#include "Arduino.h"
#include <sys/_stdint.h>
#include "RPLidar.h"

uart_port_t RPLidar::_lidarPortNum = UART_NUM_2;

static uint32_t _varbitscale_decode(uint32_t scaled, uint32_t &scaleLevel) {
        static const uint32_t VBS_SCALED_BASE[] = {
        SL_LIDAR_VARBITSCALE_X16_DEST_VAL,
        SL_LIDAR_VARBITSCALE_X8_DEST_VAL,
        SL_LIDAR_VARBITSCALE_X4_DEST_VAL,
        SL_LIDAR_VARBITSCALE_X2_DEST_VAL,
		0,
	};

	static const uint32_t VBS_SCALED_LVL[] = {
			4,
			3,
			2,
			1,
			0,
	};

	static const uint32_t VBS_TARGET_BASE[] = {
			(0x1 << SL_LIDAR_VARBITSCALE_X16_SRC_BIT),
			(0x1 << SL_LIDAR_VARBITSCALE_X8_SRC_BIT),
			(0x1 << SL_LIDAR_VARBITSCALE_X4_SRC_BIT),
			(0x1 << SL_LIDAR_VARBITSCALE_X2_SRC_BIT),
			0,
	};

	for (size_t i = 0; i < _countof(VBS_SCALED_BASE); ++i) {
		int remain = ((int) scaled - (int) VBS_SCALED_BASE[i]);
		if (remain >= 0) {
			scaleLevel = VBS_SCALED_LVL[i];
			return VBS_TARGET_BASE[i] + (remain << scaleLevel);
		}
	}
	return 0;
}

static void convert(const sl_lidar_response_measurement_node_t &from, MeasurementData &to) {
    to.distance = from.distance_q2/4.0f;
    to.angle = (from.angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f;
    to.quality = (from.sync_quality>>RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
    to.startFlag = (from.sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT);
}

static void convert(const sl_lidar_response_measurement_node_hq_t &from, MeasurementData &measurement) {
    sl_lidar_response_measurement_node_t to;
	to.sync_quality = (from.flag & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) | ((from.quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT) << RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
	to.angle_q6_checkbit = 1 | (((from.angle_z_q14 * 90) >> 8) << RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT);
	to.distance_q2 = from.dist_mm_q2 > uint16_t(-1) ? uint16_t(0) : uint16_t(from.dist_mm_q2);

    convert(to, measurement);
}

RPLidar::RPLidar(uart_port_t lidarPortNum, int rxPin, int txPin, int motorPin, LaserScanBatchPool* pool)
    : _rxPin(rxPin), _txPin(txPin), _motorPin(motorPin), _isConnected(false), _motorEnabled(false),  _batchPool(pool) {
      _lidarPortNum = lidarPortNum;
}

size_t RPLidar::calculateRecommendedPoolSize(
    float lidarRotationHz,
    float processingTimeMs) {
    float batchesInFlight = (lidarRotationHz * processingTimeMs / 1000.0);
    return ceil(batchesInFlight) + 1;  // Add 1 for buffer
}

void RPLidar::setupUartDMA() {
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    esp_err_t err;
    
    // First delete any existing UART driver
    uart_driver_delete(_lidarPortNum);
    vTaskDelay(pdMS_TO_TICKS(10));  // Give some time for cleanup
    
    err = uart_param_config(_lidarPortNum, &uart_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure UART parameters: %d\n", err);
        return;
    }

    err = uart_set_pin(_lidarPortNum, _txPin, _rxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set UART pins: %d\n", err);
        return;
    }

    err = uart_driver_install(_lidarPortNum, UART_RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install UART driver: %d\n", err);
        return;
    }

    ESP_LOGV(TAG, "UART DMA setup completed successfully");
}

void RPLidar::stopUartTasks() {
    // Clean up tasks
    if (_uartTaskHandle!= NULL) {
        vTaskDelete(_uartTaskHandle);
        _uartTaskHandle = NULL;
    }
    if (_processTaskHandle!= NULL) {
        vTaskDelete(_processTaskHandle);
        _processTaskHandle = NULL;
    }
    if (_publishTaskHandle!= NULL) {
        vTaskDelete(_publishTaskHandle);
        _publishTaskHandle = NULL;
    }

    // Clean up ring buffer and queue
    if (_uartRingBuf) {
        vRingbufferDelete(_uartRingBuf);
        _uartRingBuf = NULL;
    }
    if (publishQueue) {
        vQueueDelete(publishQueue);
        publishQueue = NULL;
    }
}

void RPLidar::setupUartTasks() {
    ESP_LOGV(TAG, "Setting up UART tasks...");
    
    // Create ring buffer if not already created
    if (_uartRingBuf == NULL) {
        ESP_LOGV(TAG, "Creating ring buffer...");
        _uartRingBuf = xRingbufferCreate(RING_BUFFER_SIZE, RINGBUF_TYPE_BYTEBUF);
        if (_uartRingBuf == NULL) {
            ESP_LOGE(TAG, "Failed to create ring buffer");
            return;
        }
        ESP_LOGV(TAG, "Ring buffer created successfully");
    }
    
    // Create publish queue if not already created
    if (publishQueue == NULL && _batchPool != NULL) {
        size_t queueSize = _batchPool->getNumOfBatches();
        publishQueue = xQueueCreate(queueSize, sizeof(LaserScanBatch*));
        if (publishQueue == NULL) {
            ESP_LOGE(TAG, "Failed to create publish queue");
            return;
        }
        ESP_LOGV(TAG, "Publish queue created successfully");
    }

    // Create tasks if they don't exist
    if (_uartTaskHandle == NULL) {
        ESP_LOGV(TAG, "Creating UART RX task...");
        BaseType_t result = xTaskCreatePinnedToCore(
            uartRxTask,
            "uart_rx",
            4096,
            this,
            5,
            &_uartTaskHandle,
            1
        );
        if (result != pdPASS) {
            ESP_LOGE(TAG, "Failed to create uart_rx task");
            return;
        }
        ESP_LOGV(TAG, "UART RX task created successfully");
    }
    
    vTaskDelay(pdMS_TO_TICKS(50)); // Give UART task time to start
    
    // Create process task
    if (_processTaskHandle == NULL) {
        ESP_LOGV(TAG, "Creating process task...");
        BaseType_t result = xTaskCreatePinnedToCore(
            processDataTask,
            "process_data",
            16384,
            this,
            4,
            &_processTaskHandle,
            1
        );
        if (result != pdPASS) {
            ESP_LOGE(TAG, "Failed to create process_data task");
            return;
        }
        ESP_LOGV(TAG, "Process data task created successfully");
    }
    
    // Final check
    vTaskDelay(pdMS_TO_TICKS(100));
    if (_uartTaskHandle && _processTaskHandle) {
        ESP_LOGV(TAG, "All tasks running");
    } else {
        ESP_LOGE(TAG, "Task creation failed");
    }
}

void RPLidar::uartRxTask(void* arg) {
    RPLidar* lidar = static_cast<RPLidar*>(arg);
    uint8_t tempBuffer[UART_RX_BUF_SIZE];
    size_t readBytes = 0;
    size_t sends = 0;
    size_t lostBytes = 0;
    size_t minFreeSize = RING_BUFFER_SIZE;
    unsigned long startMillis = millis();

    while(1) {
        size_t length = 0;
        ESP_ERROR_CHECK(uart_get_buffered_data_len(_lidarPortNum, &length));
        
        if(length > 0) {
            length = uart_read_bytes(_lidarPortNum, tempBuffer, 
                                   min(length, (size_t)UART_RX_BUF_SIZE), 
                                   pdMS_TO_TICKS(20));
            if(length > 0) {
                //ESP_LOGV(TAG, "UART received %d bytes\n", length);
                BaseType_t retval = xRingbufferSend(lidar->_uartRingBuf, 
                                                   tempBuffer, length, 
                                                   pdMS_TO_TICKS(10));
                if(retval == pdTRUE) {
                    readBytes += length;
                    sends++;
                } else {
                    lostBytes += length;
                }
                size_t freeSize = xRingbufferGetCurFreeSize(lidar->_uartRingBuf);
                if(freeSize < minFreeSize) minFreeSize = freeSize;
            }
        }

        // Print statistics periodically
        unsigned long now = millis();
        if ((readBytes + lostBytes) >= 132000) {
            ESP_LOGI(TAG, "\nSends: %d, Bytes read:%d, lost:%d, bps: %5.0f min_free_size: %d\n",
                sends, readBytes, lostBytes, 
                readBytes*9*1000.0/(now - startMillis), minFreeSize);
            readBytes = 0;
            sends = 0;
            lostBytes = 0;
            startMillis = now;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void RPLidar::processDataTask(void* arg) {
    ESP_LOGV(TAG, "Process task starting...");
    vTaskDelay(pdMS_TO_TICKS(1));

    RPLidar* lidar = static_cast<RPLidar*>(arg);
    if (!lidar || !lidar->_uartRingBuf || !lidar->_batchPool || !lidar->publishQueue) {
        ESP_LOGE(TAG, "Invalid initialization in process task");
        vTaskDelete(NULL);
        return;
    }

    uint8_t* tempBuffer = (uint8_t*)pvPortMalloc(sizeof(sl_lidar_response_ultra_capsule_measurement_nodes_t));
    if (!tempBuffer) {
        ESP_LOGE(TAG, "Failed to allocate temp buffer");
        vTaskDelete(NULL);
        return;
    }
    
    size_t processPos = 0;
    LaserScanBatch* currentBatch = lidar->_batchPool->acquireBatch();
    if (!currentBatch) {
        ESP_LOGE(TAG, "Failed to acquire initial batch");
        vPortFree(tempBuffer);
        vTaskDelete(NULL);
        return;
    }

    size_t emptyCount = 0;
    while(1) {
        size_t itemSize;
        uint8_t* item = (uint8_t*)xRingbufferReceive(lidar->_uartRingBuf, &itemSize, pdMS_TO_TICKS(10));

        if(item != NULL) {
            // parse every byte of the buffer looking for the beginning of the node package 
            for(size_t i = 0; i < itemSize; i++) {
                /*if((i % 100) == 0) { // Yield every 100 bytes
                    taskYIELD();
                }*/
                uint8_t currentByte = item[i];
                
                switch(processPos) {
                    case 0: {
                        uint8_t tmp = (currentByte >> 4);
                        if(tmp != RPLIDAR_RESP_MEASUREMENT_EXP_SYNC_1) {
                            lidar->_is_previous_capsuledataRdy = false;
                            continue;
                        }
                        break;
                    }
                    case 1: {
                        uint8_t tmp = (currentByte >> 4);
                        if(tmp != RPLIDAR_RESP_MEASUREMENT_EXP_SYNC_2) {
                            processPos = 0;
                            lidar->_is_previous_capsuledataRdy = false;
                            continue;
                        }
                        break;
                    }
                    default:
                        break;
                }
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
                        if (node->start_angle_sync_q6 & RPLIDAR_RESP_MEASUREMENT_EXP_SYNCBIT) {
                            lidar->_is_previous_capsuledataRdy = false;
                        }                              
                        MeasurementData measurements[lidar->EXPRESS_MEASUREMENTS_PER_SCAN];
                        size_t count = 0;
                        lidar->ultraCapsuleToNormal(*node, measurements, count);
                        float prevAngle = -1;
                        for(size_t i = 0; i < count; i++) {
                            // First, add the measurement. not sure what currentBatch->max_measurements is for 
                            if(currentBatch->total_measurements < currentBatch->max_measurements) {
                                // If a new rotation is starting then send currentBatch
                                if((prevAngle - measurements[i].angle) > 300) {
                                    if(xQueueSend(lidar->publishQueue, &currentBatch, 0) != pdTRUE) {
                                        lidar->_is_previous_capsuledataRdy = false;
                                        ESP_LOGV(TAG, "Queue full, batch dropped");
                                    }
                                    // Always get new batch - the ring buffer will handle overwriting old data
                                    currentBatch = lidar->_batchPool->acquireBatch();
                                    //Serial.printf("Acquired batch %d, free queue %d\n", lidar->_batchPool->getCurrentIndex(),
                                    //        uxQueueSpacesAvailable(lidar->publishQueue));
                                } 
                                prevAngle = measurements[i].angle;
                                currentBatch->measurements[currentBatch->total_measurements++] = measurements[i];
                                // Then check if it's a start flag
                                if(measurements[i].startFlag) {
                                    currentBatch->total_rotations++;
                                }                            
                            }  else {
                                // We may get here if angle is wrong. get rid of current batch and start again.
                                lidar->_is_previous_capsuledataRdy = false;
                                currentBatch = lidar->_batchPool->acquireBatch();
                            }   
                            // Check if we have enough measurements (roughly SCANS_PER_PUBLISH)
                            // When batch is full:
                            /*if(currentBatch->total_rotations >= 1 
                                    || currentBatch->total_measurements >= currentBatch->max_measurements) {
                                if(xQueueSend(lidar->publishQueue, &currentBatch, 0) != pdTRUE) {
                                    lidar->_is_previous_capsuledataRdy = false;
                                    ESP_LOGV(TAG, "Queue full, batch dropped");
                                }
                                // Always get new batch - the ring buffer will handle overwriting old data
                                currentBatch = lidar->_batchPool->acquireBatch();
                                if (!currentBatch) {
                                    ESP_LOGE(TAG, "Failed to acquire batch - this should never happen with ring buffer");
                                    vPortFree(tempBuffer);
                                    vTaskDelete(NULL);
                                    return;
                                }
                            }*/
                        } // end for
                    } else { // bad checksum
                        lidar->_is_previous_capsuledataRdy = false;
                    } // end if
                } // end if(processPos
            } // end of for
            vRingbufferReturnItem(lidar->_uartRingBuf, item);
            emptyCount = 0;
        }  else {
            emptyCount++;
            if(emptyCount > 1000) { // If no data for a while
                emptyCount = 0;
                ESP_LOGV(TAG, "No data received for a while");
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        } // end of else
        // Add an explicit yield every few iterations
        taskYIELD();
    } // end of while
}

bool RPLidar::begin(unsigned long baud) {
    setupUartDMA();
    delay(1000);  // Give time for initialization
    _isConnected = true;
    
    if (_motorPin >= 0) {
        pinMode(_motorPin, OUTPUT);
        analogWrite(_motorPin, 0);
    }
    
    // Initialize task handles to NULL
    _uartTaskHandle = NULL;
    _processTaskHandle = NULL;
    _publishTaskHandle = NULL;
    _uartRingBuf = NULL;
    publishQueue = NULL;
    
    flushInput();
    return true;
}

RPLidar::~RPLidar() {
    // Clean up tasks
    if (_uartTaskHandle) vTaskDelete(_uartTaskHandle);
    if (_processTaskHandle) vTaskDelete(_processTaskHandle);
    if (_publishTaskHandle) vTaskDelete(_publishTaskHandle);
    
    // Clean up ring buffer and queue
    if (_uartRingBuf) vRingbufferDelete(_uartRingBuf);
    if (publishQueue) vQueueDelete(publishQueue);
    
    // Clean up UART
    uart_driver_delete(_lidarPortNum);
}

void RPLidar::end() {
    stopMotor();
    _isConnected = false;
}

bool RPLidar::stopScan() {
    sendCommand(CMD_STOP);
    delay(1); // Per protocol spec, give 1ms gap before other command.
    stopUartTasks();
    stopMotor();
    return true;
}

bool RPLidar::resetLidar() {
    sendCommand(CMD_RESET); // reboot lidar microcontroller
    delay(2); // Per protocol spec, give 2ms gap before other command.
    return true;
}

bool RPLidar::startScan() {
    if(!_isConnected) return false; // Don't start scan if not connected.

    // Stop any previous operation
    stopScan();
    delay(1);
    
    // Send scan command
    sendCommand(CMD_SCAN);

    // Wait for response header
    if (!waitResponseHeader()) {
        return false;
    }
    // Verify response descriptor
    if (!verifyResponseDescriptor(MULTI_RESP_MODE, RESP_TYPE_SCAN, 5)) {
        return false;
    }
    // Point function to correct type of response.
   _scanResponseMode = RESP_TYPE_SCAN;

    return true;
}

bool RPLidar::startExpressScan(uint8_t expressScanType) {
    if(!_isConnected) return false; // Don't start scan if not connected.

    sendCommand(CMD_STOP);
    delay(1); // Per protocol spec, give 1ms gap before other command.

    setupUartTasks();
    //wait a little for tasks to spawn.
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Enable motor
    if(!_motorEnabled) startMotor();

    // Express scan command,payload,checksum expected.
    // legacy   82 5 0 0 0 0 22
    // extended 82 5 2 0 0 0 20

    uint8_t payload[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
    if(expressScanType == EXPRESS_TYPE_EXTENDED)
      payload[0] = EXPRESS_TYPE_EXTENDED;
    else
      payload[0] = EXPRESS_TYPE_LEGACY; // For Legacy.
    
    sendCommand(CMD_EXPRESS_SCAN, payload, sizeof(payload));
    
    if (!waitResponseHeader()) {
        return false;
    }

	// Verify response descriptor
  if(expressScanType == EXPRESS_TYPE_LEGACY) {
    if (verifyResponseDescriptor(MULTI_RESP_MODE, RESP_TYPE_EXPRESS_LEGACY_SCAN, 84)) {
			ESP_LOGI(TAG, "Response is of type RESP_TYPE_EXPRESS_LEGACY_SCAN");
			_scanResponseMode = RESP_TYPE_EXPRESS_LEGACY_SCAN;
			return true;
    	} else if(verifyResponseDescriptor(MULTI_RESP_MODE, RESP_TYPE_EXPRESS_DENSE_SCAN, 84)) {
			ESP_LOGI(TAG, "Response is of type RESP_TYPE_EXPRESS_DENSE_SCAN");
			_scanResponseMode = RESP_TYPE_EXPRESS_DENSE_SCAN;
			return true;
	  	}
	} else {
    	if (verifyResponseDescriptor(MULTI_RESP_MODE, RESP_TYPE_EXPRESS_EXTENDED_SCAN, 132)) {
		    ESP_LOGI(TAG, "Response is of type RESP_TYPE_EXPRESS_EXTENDED_SCAN");
          	_scanResponseMode = RESP_TYPE_EXPRESS_EXTENDED_SCAN;
          	return true;
    	}
    }
    
    return false;
}

bool RPLidar::forceScan() {
    sendCommand(CMD_FORCE_SCAN);
    
    if (!waitResponseHeader()) {
        return false;
    }
    
    return true;
}

bool RPLidar::getHealth(DeviceHealth& health) {
    flushInput();
    
    sendCommand(CMD_GET_HEALTH);
    
    if (!waitResponseHeader()) {
        ESP_LOGE(TAG, "Failed to get health response header");
        return false;
    }

    // Verify response descriptor
    if (!verifyResponseDescriptor(SINGLE_RESP_MODE, RESP_TYPE_HEALTH, 3)) {
        return false;
    }
    
    // Read data according to length from descriptor
    uint8_t buffer[3];
    size_t bytesRead = readBytes(buffer, _responseDescriptor.length);
    if (bytesRead != _responseDescriptor.length) {
        ESP_LOGE(TAG, "Expected %lu bytes but got %d bytes\n", _responseDescriptor.length, bytesRead);
        return false;
    }
    
    health.status = buffer[0];
    health.error_code = (buffer[1] | (buffer[2] << 8));
    
    return true;
}

bool RPLidar::getSampleRate(DeviceScanRate &scanRate) {
    flushInput();
    
    sendCommand(GET_SAMPLERATE);
    
    if (!waitResponseHeader()) {
        ESP_LOGE(TAG, "Failed to get health response header");
        return false;
    }
	uint32_t expectedLength = 4; 
    // Verify response descriptor
    if (!verifyResponseDescriptor(SINGLE_RESP_MODE, RESP_TYPE_SCAN_RATE, expectedLength)) {
        return false;
    }
    
    // Read data according to length from descriptor
    uint8_t buffer[expectedLength];
    size_t bytesRead = readBytes(buffer, _responseDescriptor.length);
    if (bytesRead != _responseDescriptor.length) {
        ESP_LOGE(TAG, "Expected %lu bytes but got %d bytes\n", _responseDescriptor.length, bytesRead);
        return false;
    }
    
    scanRate.standard = (buffer[0] | (buffer[1] << 8));
	scanRate.express = (buffer[2] | (buffer[3] << 8));
    
    return true;
}

bool RPLidar::getInfo(DeviceInfo& info) {
    flushInput();
    
    sendCommand(CMD_GET_INFO);
    
    if (!waitResponseHeader()) {
        ESP_LOGE(TAG, "Failed to get info response header");
        return false;
    }

    // Verify response descriptor
    if (!verifyResponseDescriptor(SINGLE_RESP_MODE, RESP_TYPE_INFO, 20)) {
        return false;
    }
    
    // Read data according to length from descriptor
    uint8_t buffer[20];
    size_t bytesRead = readBytes(buffer, _responseDescriptor.length);
    if (bytesRead != _responseDescriptor.length) {
        ESP_LOGE(TAG, "Expected %lu bytes but got %d bytes\n", _responseDescriptor.length, bytesRead);
        return false;
    }
    
    // Parse according to reference implementation
    info.model = buffer[0];
    info.firmware_minor = buffer[1];
    info.firmware_major = buffer[2];
    info.hardware = buffer[3];
    memcpy(info.serialnum, &buffer[4], 16);
    
    return true;
}

sl_result RPLidar::readMeasurement(MeasurementData* measurements, size_t& count) {
    switch (_scanResponseMode) {
        case RESP_TYPE_SCAN:
            return readMeasurementTypeScan(measurements, count);
        	break;
		case RESP_TYPE_EXPRESS_EXTENDED_SCAN:
			return readMeasurementTypeExpExtended(measurements, count);
		case RESP_TYPE_EXPRESS_LEGACY_SCAN:
			return readMeasurementTypeExpLegacy(measurements, count);
        default:
            return SL_RESULT_FORMAT_NOT_SUPPORT;
    }
}


sl_result RPLidar::_waitUltraCapsuledNode(sl_lidar_response_ultra_capsule_measurement_nodes_t& node, uint32_t timeout)
        {
    if (!_isConnected) {
        _scanning = false;
        return SL_RESULT_OPERATION_FAIL;
    }

    int recvPos = 0;
    uint32_t startTs = millis();
    uint8_t recvBuffer[sizeof(sl_lidar_response_ultra_capsule_measurement_nodes_t)];
    uint8_t *nodeBuffer = (uint8_t*) &node;
    uint32_t waitTime;
    size_t recvSize = sizeof(sl_lidar_response_ultra_capsule_measurement_nodes_t);

    while ((waitTime = millis() - startTs) <= timeout) {

        if(available() < recvSize)
			continue;

		size_t bytesRead = readBytes(recvBuffer, recvSize);
		if(bytesRead < recvSize) {
			ESP_LOGE(TAG, "Error: read less than available should not happen");
			continue;
		}

        for (size_t pos = 0; pos < recvSize; ++pos) {
            uint8_t currentByte = recvBuffer[pos];
            switch (recvPos) {
                case 0: // expect the sync bit 1
                {
                    uint8_t tmp = (currentByte >> 4);
                    if (tmp == RPLIDAR_RESP_MEASUREMENT_EXP_SYNC_1) {
                        // pass
                    }
                    else {
                        _is_previous_capsuledataRdy = false;
                        continue;
                    }
                }
                    break;
                case 1: // expect the sync bit 2
                {
                    uint8_t tmp = (currentByte >> 4);
                    if (tmp == RPLIDAR_RESP_MEASUREMENT_EXP_SYNC_2) {
                        // pass
                    }
                    else {
                        recvPos = 0;
                        _is_previous_capsuledataRdy = false;
                        continue;
                    }
                }
                    break;
            }
            nodeBuffer[recvPos++] = currentByte;
            if (recvPos == sizeof(sl_lidar_response_ultra_capsule_measurement_nodes_t)) {
                // calc the checksum ...
                uint8_t checksum = 0;
                uint8_t recvChecksum = ((node.s_checksum_1 & 0xF) | (node.s_checksum_2 << 4));

                for (size_t cpos = offsetof(sl_lidar_response_ultra_capsule_measurement_nodes_t, start_angle_sync_q6);
                        cpos < sizeof(sl_lidar_response_ultra_capsule_measurement_nodes_t); ++cpos)
                        {
                    checksum ^= nodeBuffer[cpos];
                }

                if (recvChecksum == checksum) {
                    // only consider vaild if the checksum matches...
                    if (node.start_angle_sync_q6 & RPLIDAR_RESP_MEASUREMENT_EXP_SYNCBIT) {
                        _is_previous_capsuledataRdy = false;
                        return SL_RESULT_OK ;
                    }
                    return SL_RESULT_OK ;
                }
                _is_previous_capsuledataRdy = false;
                return SL_RESULT_INVALID_DATA ;
            }
        }
    }
    _is_previous_capsuledataRdy = false;
    return SL_RESULT_OPERATION_TIMEOUT ;
}

void RPLidar::ultraCapsuleToNormal(const sl_lidar_response_ultra_capsule_measurement_nodes_t &capsule, MeasurementData *measurements, size_t &nodeCount)
    {
    nodeCount = 0;
    if (_is_previous_capsuledataRdy) {
        int diffAngle_q8;
        int currentStartAngle_q8 = ((capsule.start_angle_sync_q6 & 0x7FFF) << 2);
        int prevStartAngle_q8 = ((_cached_previous_ultracapsuledata.start_angle_sync_q6 & 0x7FFF) << 2);

        diffAngle_q8 = (currentStartAngle_q8) - (prevStartAngle_q8);
        if (prevStartAngle_q8 > currentStartAngle_q8) {
            diffAngle_q8 += (360 << 8);
        }

        int angleInc_q16 = (diffAngle_q8 << 3) / 3;
        int currentAngle_raw_q16 = (prevStartAngle_q8 << 8);
        for (size_t pos = 0; pos < _countof(_cached_previous_ultracapsuledata.ultra_cabins); ++pos) {
            int dist_q2[3];
            int angle_q6[3];
            int syncBit[3];

            uint32_t combined_x3 = _cached_previous_ultracapsuledata.ultra_cabins[pos].combined_x3;

            // unpack ...
            int dist_major = (combined_x3 & 0xFFF);

            // signed partical integer, using the magic shift here
            // DO NOT TOUCH

            int dist_predict1 = (((int) (combined_x3 << 10)) >> 22);
            int dist_predict2 = (((int) combined_x3) >> 22);

            int dist_major2;

            uint32_t scalelvl1, scalelvl2;

            // prefetch next ...
            if (pos == _countof(_cached_previous_ultracapsuledata.ultra_cabins) - 1) {
                dist_major2 = (capsule.ultra_cabins[0].combined_x3 & 0xFFF);
            }
            else {
                dist_major2 = (_cached_previous_ultracapsuledata.ultra_cabins[pos + 1].combined_x3 & 0xFFF);
            }

            // decode with the var bit scale ...
            dist_major = _varbitscale_decode(dist_major, scalelvl1);
            dist_major2 = _varbitscale_decode(dist_major2, scalelvl2);

            int dist_base1 = dist_major;
            int dist_base2 = dist_major2;

            if ((!dist_major) && dist_major2) {
                dist_base1 = dist_major2;
                scalelvl1 = scalelvl2;
            }

            dist_q2[0] = (dist_major << 2);
            if ((dist_predict1 == 0xFFFFFE00) || (dist_predict1 == 0x1FF)) {
                dist_q2[1] = 0;
            }
            else {
                dist_predict1 = (dist_predict1 << scalelvl1);
                dist_q2[1] = (dist_predict1 + dist_base1) << 2;

            }

            if ((dist_predict2 == 0xFFFFFE00) || (dist_predict2 == 0x1FF)) {
                dist_q2[2] = 0;
            }
            else {
                dist_predict2 = (dist_predict2 << scalelvl2);
                dist_q2[2] = (dist_predict2 + dist_base2) << 2;
            }

            for (int cpos = 0; cpos < 3; ++cpos) {
                syncBit[cpos] = (((currentAngle_raw_q16 + angleInc_q16) % (360 << 16)) < angleInc_q16) ? 1 : 0;

                int offsetAngleMean_q16 = (int) (7.5 * 3.1415926535 * (1 << 16) / 180.0);

                if (dist_q2[cpos] >= (50 * 4))
                        {
                    const int k1 = 98361;
                    const int k2 = int(k1 / dist_q2[cpos]);

                    offsetAngleMean_q16 = (int) (8 * 3.1415926535 * (1 << 16) / 180) - (k2 << 6) - (k2 * k2 * k2) / 98304;
                }

                angle_q6[cpos] = ((currentAngle_raw_q16 - int(offsetAngleMean_q16 * 180 / 3.14159265)) >> 10);
                currentAngle_raw_q16 += angleInc_q16;

                if (angle_q6[cpos] < 0)
                    angle_q6[cpos] += (360 << 6);
                if (angle_q6[cpos] >= (360 << 6))
                    angle_q6[cpos] -= (360 << 6);

                sl_lidar_response_measurement_node_hq_t node;

                node.flag = (syncBit[cpos] | ((!syncBit[cpos]) << 1));
                node.quality = dist_q2[cpos] ? (0x2F << RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT) : 0;
                node.angle_z_q14 = uint16_t((angle_q6[cpos] << 8) / 90);
                node.dist_mm_q2 = dist_q2[cpos];

                convert(node, measurements[nodeCount]);
                nodeCount++;
            }
        }
    }

    _cached_previous_ultracapsuledata = capsule;
    _is_previous_capsuledataRdy = true;
}

sl_result RPLidar::readMeasurementTypeExpExtended(MeasurementData* measurements, size_t &count) {
    sl_lidar_response_ultra_capsule_measurement_nodes_t node;
	
    sl_result ans = _waitUltraCapsuledNode(node, READ_EXP_TIMEOUT_MS);
    if (ans != SL_RESULT_OK) {
        return ans;
    }
    ultraCapsuleToNormal(node, measurements, count);
	return SL_RESULT_OK;
}

sl_result RPLidar::readMeasurementTypeExpLegacy(MeasurementData* measurements, size_t &count) {
	//TODO:
	return SL_RESULT_OPERATION_NOT_SUPPORT;
}

sl_result RPLidar::readMeasurementTypeScan(MeasurementData* measurements, size_t& nodeCount) {
	int counter = 0;
	sl_lidar_response_measurement_node_t node;
	uint8_t *nodeBuffer = (uint8_t*)&node;
	uint8_t recvBuffer[sizeof(sl_lidar_response_measurement_node_t)];
    uint32_t startTs =  millis();
    uint32_t waitTime = 0;
    nodeCount = 0;

	uint8_t recvPos = 0;
	//_serial.setRxTimeout(0);
    //TODO: should we add timeout here intead of while(1)?
	while((waitTime =  millis() - startTs) <= READ_TIMEOUT_MS) {

		if(available() < sizeof(recvBuffer))
			continue;
		size_t bytesRead = readBytes(recvBuffer, sizeof(recvBuffer));
		if(bytesRead < sizeof(recvBuffer)) {
			ESP_LOGE(TAG, "Error: read less than available should not happen");
			continue;
		}

		//validation - NOTE: THIS CHECKBIT being false brings rate from 2100 to 1000
		for (size_t pos = 0; pos < bytesRead; ++pos) {
			uint8_t currentByte = recvBuffer[pos];
			switch (recvPos) {
				case 0: // expect the sync bit and its reverse in this byte
				{
					uint8_t tmp = (currentByte >> 1);
					if ((tmp ^ currentByte) & 0x1) {
						// pass
					}
					else {
						continue;
					}

				}
				break;
				case 1: // expect the highest bit to be 1
				{
					if (currentByte & RPLIDAR_RESP_MEASUREMENT_CHECKBIT) {
						// pass
					}
					else {
						recvPos = 0;
						continue;
					}
				}
				break;
			}
			nodeBuffer[recvPos++] = currentByte;

			if (recvPos == sizeof(sl_lidar_response_measurement_node_t)) {
				break;
			}
		}
		if (recvPos == sizeof(sl_lidar_response_measurement_node_t)) {
			break;
		}
	}

	// store the data ...
	if (recvPos == sizeof(sl_lidar_response_measurement_node_t)) {
        convert(node, measurements[nodeCount]);
		nodeCount++;
		return SL_RESULT_OK;
	}
    else {
        return SL_RESULT_INVALID_DATA;
    }
	return SL_RESULT_OPERATION_TIMEOUT;
}

void RPLidar::startMotor(uint8_t pwm) {
    if (_motorPin >= 0) {
        analogWrite(_motorPin, pwm);
        _motorEnabled = true;
    }
}

void RPLidar::stopMotor() {
    if (_motorPin >= 0) {
        analogWrite(_motorPin, 0);
        _motorEnabled = false;
    }
}

// Private methods
void RPLidar::sendCommand(uint8_t cmd, const uint8_t* payload, uint8_t payloadSize) {
    // Clear input buffer
    flushInput();
    
    // Send command header
    writeByte(CMD_SYNC_BYTE);
    writeByte(cmd);
    
    // Send payload if any
    if (payload && payloadSize > 0) {
        writeByte(payloadSize);
        writeBytes(payload, payloadSize);
        
        // Calculate and send checksum
        uint8_t checksum = CMD_SYNC_BYTE ^ cmd ^ payloadSize;
        for (uint8_t i = 0; i < payloadSize; i++) {
            checksum ^= payload[i];
        }
        writeByte(checksum);
    }
    
    //_serial.flush();
}

bool RPLidar::waitResponseHeader() {
    uint8_t byte;
    unsigned long startTime = millis();
        
    // Wait for first sync byte
    while ((millis() - startTime) < READ_TIMEOUT_MS*10) {
        if (available()) {
            byte = readByte();
            if (byte == RESP_SYNC_BYTE1) {
                
                // Wait for second sync byte
                startTime = millis();
                while ((millis() - startTime) < READ_TIMEOUT_MS) {
                    if (available()) {
                        byte = readByte();
                        if (byte == RESP_SYNC_BYTE2) {
                            // Read remaining 5 bytes of descriptor
                            uint8_t descriptor[5];
                            size_t bytesRead = readBytes(descriptor, 5);
                            if (bytesRead != 5) {
                                ESP_LOGE(TAG, "Failed to read complete descriptor");
                                return false;
                            }

                            // Parse 32-bit length/mode field (30 bits length, 2 bits mode)
                            uint32_t lengthAndMode = 
                                ((uint32_t)descriptor[0]) | 
                                ((uint32_t)descriptor[1] << 8) | 
                                ((uint32_t)descriptor[2] << 16) | 
                                ((uint32_t)descriptor[3] << 24);
                            
                            _responseDescriptor.length = lengthAndMode & 0x3FFFFFFF;
                            _responseDescriptor.mode = (lengthAndMode >> 30) & 0x03;
                            _responseDescriptor.dataType = descriptor[4];

                            ESP_LOGV(TAG, "Response descriptor: len=%lu mode=%u type=0x%02X\n", 
                                        _responseDescriptor.length, 
                                        _responseDescriptor.mode, 
                                        _responseDescriptor.dataType);

                            return true;
                        }
                    }
                }
                ESP_LOGE(TAG, "Timeout waiting for second sync byte");
                return false;
            }
        }
    }
    ESP_LOGE(TAG, "Timeout waiting for first sync byte");
    return false;
}

bool RPLidar::verifyResponseDescriptor(uint8_t expectedMode, uint8_t expectedType, uint32_t expectedLength) {
    if (_responseDescriptor.mode != expectedMode) {
        ESP_LOGE(TAG, "Wrong response Mode: got 0x%02X, expected 0x%02X\n", 
                     _responseDescriptor.mode, expectedMode);
        return false;
    }
    if (_responseDescriptor.dataType != expectedType) {
        ESP_LOGE(TAG, "Wrong response type: got 0x%02X, expected 0x%02X\n", 
                     _responseDescriptor.dataType, expectedType);
        return false;
    }
    if (expectedLength != 0 && _responseDescriptor.length != expectedLength) {
        ESP_LOGE(TAG, "Wrong response length: got %lu, expected %lu\n", 
                     _responseDescriptor.length, expectedLength);
        return false;
    }
    return true;
}

void RPLidar::flushInput() {
    while (available()) {
        readByte();
    }
}

uint8_t RPLidar::checksum(const uint8_t* data, uint8_t len) {
    uint8_t cs = 0;
    for (uint8_t i = 0; i < len; i++) {
        cs ^= data[i];
    }
    return cs;
}

// Single byte read
uint8_t RPLidar::readByte() {
    uint8_t byte;
    if(uart_read_bytes(_lidarPortNum, &byte, 1, pdMS_TO_TICKS(RX_TIMEOUT_MS)) == 1) {
        return byte;
    }
    return 0;
}

// Read multiple bytes
size_t RPLidar::readBytes(uint8_t* buffer, size_t length) {
    return uart_read_bytes(_lidarPortNum, buffer, length, pdMS_TO_TICKS(RX_TIMEOUT_MS));
}

// Check available bytes
size_t RPLidar::available() {
    size_t available_bytes;
    uart_get_buffered_data_len(_lidarPortNum, &available_bytes);
    return available_bytes;
}

void RPLidar::writeByte(uint8_t byte) {
    uart_write_bytes(_lidarPortNum, &byte, 1);
}

// Write multiple bytes
void RPLidar::writeBytes(const uint8_t* data, size_t length) {
    uart_write_bytes(_lidarPortNum, (const char*)data, length);
}