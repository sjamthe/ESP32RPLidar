#include <sys/_stdint.h>
// RPLidar.cpp
#include "RPLidar.h"

RPLidar::RPLidar(HardwareSerial& serial, int rxPin, int txPin, int motorPin)
    : _serial(serial), _rxPin(rxPin), _txPin(txPin), _motorPin(motorPin), _motorEnabled(false) {
}

bool RPLidar::begin(unsigned long baud) {
    // End any previous serial connection
    _serial.end();
    delay(100);  // Give time for serial to fully close
    
    // Initialize serial
    _serial.begin(baud, SERIAL_8N1, _rxPin, _txPin);
    delay(500);  // Give time for serial to initialize
    
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
    _serial.setTimeout(FAST_READ_TIMEOUT_MS); // Ready to read measurements

    return true;
}

bool RPLidar::startExpressScan() {
    stop();
    delay(1);

    // Express scan payload
    uint8_t payload[5] = {0};
    payload[0] = 0; // Default working mode
    
    sendCommand(CMD_EXPRESS_SCAN, payload, sizeof(payload));
    
    if (!waitResponseHeader()) {
        return false;
    }
	// Verify response descriptor
	if (verifyResponseDescriptor(MULTI_RESP_MODE, RESP_TYPE_EXPRESS_LEGACY_SCAN, 84)) {
		Serial.println("Response is of type RESP_TYPE_EXPRESS_LEGACY_SCAN");
        return true;
    }
    else if (verifyResponseDescriptor(MULTI_RESP_MODE, RESP_TYPE_EXPRESS_EXTENDED_SCAN, 132)) {
		Serial.println("Response is of type RESP_TYPE_EXPRESS_EXTENDED_SCAN");
        return true;
    } else if(verifyResponseDescriptor(MULTI_RESP_MODE, RESP_TYPE_EXPRESS_DENSE_SCAN, 84)) {
		Serial.println("Response is of type RESP_TYPE_EXPRESS_DENSE_SCAN");
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
    size_t bytesRead = _serial.readBytes(buffer, _responseDescriptor.length);
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
    size_t bytesRead = _serial.readBytes(buffer, _responseDescriptor.length);
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
    size_t bytesRead = _serial.readBytes(buffer, _responseDescriptor.length);
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

/*bool RPLidar::readMeasurement2(MeasurementData& measurement) {
    
    // Read measurement data
    if (_serial.readBytes(buffer, sizeof(buffer)) != sizeof(buffer)) {
		Serial.println("Measurement Timeout Error");
        return false;
    }
    
    // Parse measurement data
    measurement.startFlag = (buffer[0] & 0x1) == 0x1; // set start flag is least significant bit is 1
    // Extract the two bits and compare them
    bool areNotEqual = ((buffer[0] & 0x2) >> 1) != (buffer[0] & 0x1);
    if(!areNotEqual) {
		Serial.println("Measurement Data Error: 1st & second bit are not inverse");
		for (int i = 7; i >= 0; i--) {
        	Serial.printf("%d", (buffer[0] >> i) & 1);
    	}
		Serial.println();
		return false;
    }
	// Extract quality of reflected laser light.
    measurement.quality = buffer[0] >> 2;
    
	// Extract check bit.
	if ((buffer[1] & 0x1) != 0x1) {
		Serial.println("Measurement Data Error: Checkbit is not 1");
		for (int i = 7; i >= 0; i--) {
        	Serial.printf("%d", (buffer[1] >> i) & 1);
    	}
		Serial.println();
		return false;
	}

    // Calculate angle (Q6 format)
    uint16_t angle_q6 = ((buffer[2] << 7) | (buffer[1] >> 1));
    measurement.angle = static_cast<float>(angle_q6) / 64.0f;
    
    // Calculate distance (Q2 format)
    uint16_t distance_q2 = (buffer[4] << 8) | buffer[3];
    measurement.distance = static_cast<float>(distance_q2) / 4.0f;
    
    return true;
}*/

bool RPLidar::readMeasurement(MeasurementData& measurement) {
    uint32_t currentTs = millis();
    uint32_t remainingtime;
    rplidar_response_measurement_node_t node;
   uint8_t *nodebuf = (uint8_t*)&node;

	uint8_t recvPos = 0;

	while ((remainingtime=millis() - currentTs) <= FAST_READ_TIMEOUT_MS) {
		int currentbyte = _serial.read();
		if (currentbyte<0) continue;

		switch (recvPos) {
			case 0: // expect the sync bit and its reverse in this byte          {
				{
					uint8_t tmp = (currentbyte>>1);
					if ( (tmp ^ currentbyte) & 0x1 ) {
						// pass
					} else {
						continue;
					}

				}
				break;
			case 1: // expect the highest bit to be 1
				{
					if (currentbyte & RPLIDAR_RESP_MEASUREMENT_CHECKBIT) {
						// pass
					} else {
						recvPos = 0;
						continue;
					}
				}
				break;
		}
		nodebuf[recvPos++] = currentbyte;

		if (recvPos == sizeof(measurement)) {
			// store the data ...
			measurement.distance = node.distance_q2/4.0f;
			measurement.angle = (node.angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f;
			measurement.quality = (node.sync_quality>>RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
			measurement.startFlag = (node.sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT);
			return true;
		}
			
	}

	return false;
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
    _serial.write(CMD_SYNC_BYTE);
    _serial.write(cmd);
    
    // Send payload if any
    if (payload && payloadSize > 0) {
        _serial.write(payloadSize);
        _serial.write(payload, payloadSize);
        
        // Calculate and send checksum
        uint8_t checksum = CMD_SYNC_BYTE ^ cmd ^ payloadSize;
        for (uint8_t i = 0; i < payloadSize; i++) {
            checksum ^= payload[i];
        }
        _serial.write(checksum);
    }
    
    _serial.flush();
}

bool RPLidar::waitResponseHeader() {
    uint8_t byte;
    unsigned long startTime = millis();
    
    //Serial.println("Waiting for response header...");
    
    // Wait for first sync byte
    while ((millis() - startTime) < READ_TIMEOUT_MS) {
        if (_serial.available()) {
            byte = _serial.read();
            //Serial.printf("Got byte: %02X\n", byte);
            if (byte == RESP_SYNC_BYTE1) {
                //Serial.println("Found first sync byte");
                
                // Wait for second sync byte
                startTime = millis();
                while ((millis() - startTime) < READ_TIMEOUT_MS) {
                    if (_serial.available()) {
                        byte = _serial.read();
                        //Serial.printf("Got second byte: %02X\n", byte);
                        if (byte == RESP_SYNC_BYTE2) {
                            // Read remaining 5 bytes of descriptor
                            uint8_t descriptor[5];
                            size_t bytesRead = _serial.readBytes(descriptor, 5);
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
    while (_serial.available()) {
        _serial.read();
    }
}

uint8_t RPLidar::checksum(const uint8_t* data, uint8_t len) {
    uint8_t cs = 0;
    for (uint8_t i = 0; i < len; i++) {
        cs ^= data[i];
    }
    return cs;
}