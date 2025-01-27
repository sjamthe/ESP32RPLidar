#include <stdint.h>
#include <cstdlib>
#include <cstddef>
#include "Arduino.h"
#include <sys/_stdint.h>
#include "RPLidar.h"

static uint32_t _varbitscale_decode(uint32_t scaled, uint32_t &scaleLevel)
		{
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

extern void setup_uart_dma(void);
RPLidar::RPLidar(HardwareSerial& serial, int rxPin, int txPin, int motorPin)
    : _serial(serial), _rxPin(rxPin), _txPin(txPin), _motorPin(motorPin), _isConnected(false), _motorEnabled(false) {
}

bool RPLidar::begin(unsigned long baud) {
    /** End any previous serial connection
    _serial.end();
    delay(100);  // Give time for serial to fully close
    
    // Initialize serial
    _serial.begin(baud, SERIAL_8N1, _rxPin, _txPin);*/

    setup_uart_dma();
    delay(1000);  // Give time for serial to initialize
    _isConnected = true;
    
    // Setup motor pin if provided
    if (_motorPin >= 0) {
        pinMode(_motorPin, OUTPUT);
        analogWrite(_motorPin, 0);
    }
    
    // Clear any stale data
    flushInput();
    
    return true;
}

void RPLidar::end() {
    stopMotor();
    _serial.end();
    _isConnected = false;
}

bool RPLidar::stop() {
    sendCommand(CMD_STOP);
    delay(1); // Per protocol spec, give 1ms gap before other command.
    return true;
}

bool RPLidar::reset() {
    sendCommand(CMD_RESET); // reboot lidar microcontroller
    delay(2); // Per protocol spec, give 2ms gap before other command.
    return true;
}

bool RPLidar::startScan() {
    if(!_isConnected) return false; // Don't start scan if not connected.

    // Stop any previous operation
    stop();
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

    stop();
    delay(1);

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
			Serial.println("Response is of type RESP_TYPE_EXPRESS_LEGACY_SCAN");
			_scanResponseMode = RESP_TYPE_EXPRESS_LEGACY_SCAN;
			return true;
    	} else if(verifyResponseDescriptor(MULTI_RESP_MODE, RESP_TYPE_EXPRESS_DENSE_SCAN, 84)) {
			Serial.println("Response is of type RESP_TYPE_EXPRESS_DENSE_SCAN");
			_scanResponseMode = RESP_TYPE_EXPRESS_DENSE_SCAN;
			return true;
	  	}
	} else {
    	if (verifyResponseDescriptor(MULTI_RESP_MODE, RESP_TYPE_EXPRESS_EXTENDED_SCAN, 132)) {
		    Serial.println("Response is of type RESP_TYPE_EXPRESS_EXTENDED_SCAN");
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
    //Serial.println("Requesting device health...");
    flushInput();
    
    sendCommand(CMD_GET_HEALTH);
    //Serial.println("Health command sent, waiting for response...");
    
    if (!waitResponseHeader()) {
        Serial.println("Failed to get health response header");
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
        Serial.printf("Expected %lu bytes but got %d bytes\n", _responseDescriptor.length, bytesRead);
        return false;
    }
    
    health.status = buffer[0];
    health.error_code = (buffer[1] | (buffer[2] << 8));
    
    return true;
}

bool RPLidar::getSampleRate(DeviceScanRate &scanRate) {
    //Serial.println("Requesting scan rate...");
    flushInput();
    
    sendCommand(GET_SAMPLERATE);
    //Serial.println("SampleRate command sent, waiting for response...");
    
    if (!waitResponseHeader()) {
        Serial.println("Failed to get health response header");
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
        Serial.printf("Expected %lu bytes but got %d bytes\n", _responseDescriptor.length, bytesRead);
        return false;
    }
    
    scanRate.standard = (buffer[0] | (buffer[1] << 8));
	scanRate.express = (buffer[2] | (buffer[3] << 8));
    
    return true;
}

bool RPLidar::getInfo(DeviceInfo& info) {
    //Serial.println("Requesting device info...");
    flushInput();
    
    sendCommand(CMD_GET_INFO);
    //Serial.println("Info command sent, waiting for response...");
    
    if (!waitResponseHeader()) {
        Serial.println("Failed to get info response header");
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
        Serial.printf("Expected %lu bytes but got %d bytes\n", _responseDescriptor.length, bytesRead);
        return false;
    }
    
    /* Print raw bytes
    Serial.print("Info raw data: ");
    for (size_t i = 0; i < bytesRead; i++) {
        Serial.printf("%02X ", buffer[i]);
    }
    Serial.println();*/
    
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
			Serial.println("Error: read less than available should not happen");
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

void RPLidar::_ultraCapsuleToNormal(const sl_lidar_response_ultra_capsule_measurement_nodes_t &capsule, MeasurementData *measurements, size_t &nodeCount)
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
    _ultraCapsuleToNormal(node, measurements, count);
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
			Serial.println("Error: read less than available should not happen");
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
    
    //Serial.println("Waiting for response header...");
    
    // Wait for first sync byte
    while ((millis() - startTime) < READ_TIMEOUT_MS*10) {
        if (available()) {
            byte = readByte();
            //Serial.printf("Got byte: %02X\n", byte);
            if (byte == RESP_SYNC_BYTE1) {
                //Serial.println("Found first sync byte");
                
                // Wait for second sync byte
                startTime = millis();
                while ((millis() - startTime) < READ_TIMEOUT_MS) {
                    if (available()) {
                        byte = readByte();
                        //Serial.printf("Got second byte: %02X\n", byte);
                        if (byte == RESP_SYNC_BYTE2) {
                            // Read remaining 5 bytes of descriptor
                            uint8_t descriptor[5];
                            size_t bytesRead = readBytes(descriptor, 5);
                            if (bytesRead != 5) {
                                Serial.println("Failed to read complete descriptor");
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

                            Serial.printf("Response descriptor: len=%lu mode=%u type=0x%02X\n", 
                                        _responseDescriptor.length, 
                                        _responseDescriptor.mode, 
                                        _responseDescriptor.dataType);

                            return true;
                        }
                    }
                }
                Serial.println("Timeout waiting for second sync byte");
                return false;
            }
        }
    }
    Serial.println("Timeout waiting for first sync byte");
    return false;
}

bool RPLidar::verifyResponseDescriptor(uint8_t expectedMode, uint8_t expectedType, uint32_t expectedLength) {
    if (_responseDescriptor.mode != expectedMode) {
        Serial.printf("Wrong response Mode: got 0x%02X, expected 0x%02X\n", 
                     _responseDescriptor.mode, expectedMode);
        return false;
    }
    if (_responseDescriptor.dataType != expectedType) {
        Serial.printf("Wrong response type: got 0x%02X, expected 0x%02X\n", 
                     _responseDescriptor.dataType, expectedType);
        return false;
    }
    if (expectedLength != 0 && _responseDescriptor.length != expectedLength) {
        Serial.printf("Wrong response length: got %lu, expected %lu\n", 
                     _responseDescriptor.length, expectedLength);
        return false;
    }
    return true;
}

void RPLidar::flushInput() {
    //while (_serial.available()) {
    //    _serial.read();
    //}
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
    if(uart_read_bytes(UART_NUM, &byte, 1, pdMS_TO_TICKS(RX_TIMEOUT_MS)) == 1) {
        return byte;
    }
    return 0;
}

// Read multiple bytes
size_t RPLidar::readBytes(uint8_t* buffer, size_t length) {
    return uart_read_bytes(UART_NUM, buffer, length, pdMS_TO_TICKS(RX_TIMEOUT_MS));
}

// Check available bytes
size_t RPLidar::available() {
    size_t available_bytes;
    uart_get_buffered_data_len(UART_NUM, &available_bytes);
    return available_bytes;
}

void RPLidar::writeByte(uint8_t byte) {
    uart_write_bytes(UART_NUM, &byte, 1);
}

// Write multiple bytes
void RPLidar::writeBytes(const uint8_t* data, size_t length) {
    uart_write_bytes(UART_NUM, (const char*)data, length);
}