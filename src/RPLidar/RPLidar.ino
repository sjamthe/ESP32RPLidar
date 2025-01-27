/*
 * Expect output for getInfo
model: 24
firmware: 1.24
hardware: 5
serialNumber (HEX): 81 53 9D F1 C3 E3 9A C4 C3 E6 98 F9 71 84 34 0D 
 *
 */
#include "RPLidar.h"

typedef void (*node_callback_t)(sl_lidar_response_ultra_capsule_measurement_nodes_t* nodes, size_t count);
// Task parameters structure
typedef struct {
    node_callback_t callback;
    // Add other parameters if needed
} task_params_t;

// Example callback implementation
void process_nodes(sl_lidar_response_ultra_capsule_measurement_nodes_t* nodes, size_t count) {
    //Serial.printf("node_callback for nodes: %d\n",count);
	Serial.print(".");
    // Process batch of nodes here. We probably should send a message to another task and freeup this task.

}

void setupLidar();
extern void process_data_task(void *arg);
extern void uart_rx_task(void *arg);

void setup() {
  setupLidar();
}

RPLidar lidar(Serial2, 16, 17, 5); 

void setupLidar() {
    // Start USB serial for debugging and wait for port to be ready
    Serial.begin(115200);
    delay(2000);  // Give time for USB serial to properly initialize
    Serial.println("\n\nRPLidar Test Starting...");
    delay(500);   // Additional delay to ensure stability
    
    // Initialize RPLidar
	//Serial2.setRxBufferSize(1024);
    Serial.println("Initializing RPLidar...");
    if (!lidar.begin()) {
        Serial.println("Failed to start RPLidar");
        return;
    }
    
    // Get device info
    RPLidar::DeviceInfo info;
    if (lidar.getInfo(info)) {
        Serial.println("RPLidar Info:");
        Serial.printf("  Model: %02d ", info.model);
        switch(info.model) {
            case 0x18: Serial.println("(RPLIDAR A-series)"); break;
            default: Serial.println("(Unknown model)"); break;
        }
        // Swap minor and major for correct display
        Serial.printf("  Firmware Version: %d.%02d\n", info.firmware_major, info.firmware_minor);
        Serial.printf("  Hardware Version: 0x%02d\n", info.hardware);
        Serial.print("  Serial Number: ");
        for (int i = 0; i < 16; i++) {
            Serial.printf("%02X", info.serialnum[i]);
        }
        Serial.println();
    } else {
		Serial.println("Error: Failed to get device info");
		return;
	}

    // Get health status
    RPLidar::DeviceHealth health;
    if (lidar.getHealth(health)) {
        Serial.println("RPLidar Health:");
        Serial.printf("  Status: %d\n", health.status);
        Serial.printf("  Error Code: 0x%04X\n", health.error_code);
        if (health.status == 2) {  // Error state
            Serial.println("Device is in error state!");
            delay(1000);  // Give time for error message to be sent
            //ESP.restart(); // Restart on error
            return;
        }
    } else {
        Serial.println("Error: Failed to get device health");
		return;
    }

    //Get Scan Rate
    RPLidar::DeviceScanRate scanRate;
    if (lidar.getSampleRate(scanRate)) {
        Serial.println("RPLidar Scan Rate:");
        Serial.printf("  Standard Scan Rate: %d usecs\n", scanRate.standard);
        Serial.printf("  Express Scan Rate: %d usecs\n", scanRate.express);
    } else {
        Serial.println("Error: Failed to get scan rate");
		return;
    }

    // Reset device before starting
    Serial.println("Resetting RPLidar...");
    lidar.reset();
    delay(2000);  // Give it time to reset
    
    // Start motor with a clean delay sequence
    Serial.println("Starting motor...");
    lidar.startMotor();
    delay(1000);  // Give motor time to reach speed

    // Start scan
    Serial.println("Starting scan...");
    xTaskCreate(uart_rx_task, "uart_rx", 2048, NULL, 5, NULL);

    task_params_t* params = (task_params_t*)malloc(sizeof(task_params_t));
	if(params == NULL) {
		Serial.println("Error: params is NULL");
		return;
	}
    params->callback = process_nodes;
    xTaskCreate(process_data_task, "process_data", 8192, (void*)params, 4, NULL);

    if (!lidar.startExpressScan()) {
        Serial.println("Failed to start scan");
        delay(1000);  // Give time for error message to be sent
        //ESP.restart(); // Restart on scan failure (Should we restart lidar instead of ESP?)
        return;
    }
    Serial.println("Scan started successfully");

    // Wait for measurements to start
    delay(200);
}

unsigned long startMillis = 0;
unsigned long measurementCount = 0;
unsigned long rpsCount = 0;
unsigned long errorCount = 0;
unsigned long timeoutCount = 0;
bool firstMeasurement = true;

void handleLidar() {
  MeasurementData measurements[lidar.EXPRESS_MEASUREMENTS_PER_SCAN];
    size_t count = 0;
    
	sl_result ans = lidar.readMeasurement(measurements, count);
    if (ans == SL_RESULT_OK) {
        if (firstMeasurement) {
            Serial.println("First measurement received!");
            startMillis = millis();
            firstMeasurement = false;
        }
             
		measurementCount+= count;

		for (int i=0; i<count; i++) {
			if(measurements[i].startFlag) {
			rpsCount++;
			}
		}		
    } 
    else {
        if(ans == SL_RESULT_OPERATION_TIMEOUT)
			timeoutCount++;
		else if(ans == SL_RESULT_INVALID_DATA)
		 	errorCount++;
    }
	// Print stats every second
	unsigned long now = millis();
	if ((now - startMillis) > 10000) {
		Serial.printf("Errors: %u. Timeouts: %u, Measurements: %u, Measurements per second: %04.0f, rps: %04.0f bps: %5.0f\n",
			errorCount, timeoutCount, measurementCount, measurementCount/((now-startMillis)/1000.0), rpsCount/((now-startMillis)/1000.0),
			measurementCount*132*9/(96*(now-startMillis)/1000.0));
		Serial.printf("First measurement - Angle: %.2f°, Distance: %.2fmm, Quality: %d\n", 
						measurements[0].angle, measurements[0].distance, measurements[0].quality);
		Serial.printf("Last measurement - Angle: %.2f°, Distance: %.2fmm, Quality: %d\n", 
						measurements[lidar.EXPRESS_MEASUREMENTS_PER_SCAN-1].angle,
						measurements[lidar.EXPRESS_MEASUREMENTS_PER_SCAN-1].distance,
						measurements[lidar.EXPRESS_MEASUREMENTS_PER_SCAN-1].quality);
		startMillis = millis();
		measurementCount = 0;
		rpsCount = 0;
		errorCount = 0;
	}
}

void loop() {
    //handleLidar();
    //Notes: Only delay of 10ms is tolerated. anything more stalls reading lidar.
    delay(100);
}