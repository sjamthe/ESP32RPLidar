#include "RingLaserScanBatchPool.h"
#include "RPLidar.h"

// Declare as pointers instead of objects
RingLaserScanBatchPool* batchPool = nullptr;
RPLidar* lidar = nullptr;
TaskHandle_t publishTaskHandle = NULL;
bool scanning = false;

void createPublishTask();

void  getLidarInfo() {
// Get device info
    RPLidar::DeviceInfo info;
    if (lidar->getInfo(info)) {
        Serial.println("RPLidar Info:");
        Serial.printf("  Model: %02d ", info.model);
        switch(info.model) {
            case 0x18: Serial.println("(RPLIDAR A-series)"); break;
            default: Serial.println("(Unknown model)"); break;
        }
        Serial.printf("  Firmware Version: %d.%02d\n", info.firmware_major, info.firmware_minor);
        Serial.printf("  Hardware Version: 0x%02d\n", info.hardware);
        Serial.print("  Serial Number: ");
        for (int i = 0; i < 16; i++) {
            Serial.printf("%02X", info.serialnum[i]);
        }
        Serial.println();
    } else {
        Serial.println("Error: Failed to get device info");
        delay(10000); 
        //ESP.restart();
        return;
    }

    // Get health status
    RPLidar::DeviceHealth health;
    if (lidar->getHealth(health)) {
        Serial.println("RPLidar Health:");
        Serial.printf("  Status: %d\n", health.status);
        Serial.printf("  Error Code: 0x%04X\n", health.error_code);
        if (health.status == 2) {  // Error state
            Serial.println("Device is in error state!");
            delay(10000);  // Give time for error message to be sent
            ESP.restart(); // Restart on error
            return;
        }
    } else {
        Serial.println("Error: Failed to get device health");
        delay(10000); 
        //ESP.restart(); // Restart on error
        return;
    }

    // Get Scan Rate
    RPLidar::DeviceScanRate scanRate;
    if (lidar->getSampleRate(scanRate)) {
        Serial.println("RPLidar Scan Rate:");
        Serial.printf("  Standard Scan Rate: %d usecs\n", scanRate.standard);
        Serial.printf("  Express Scan Rate: %d usecs\n", scanRate.express);
    } else {
        Serial.println("Error: Failed to get scan rate");
        delay(10000); 
        ESP.restart(); // Restart on error
        return;
    }

}

void startLidarScan() {
        // Reset device before starting
    Serial.println("Resetting RPLidar...");
    lidar->resetLidar();
    delay(2000);  // Give it time to reset
    
    // Start motor with a clean delay sequence
    //Serial.println("Starting motor...");
    //lidar->startMotor();
    //delay(1000);  // Give motor time to reach speed

    // Start scan
    Serial.println("Starting scan...");
    
    if (!lidar->startExpressScan()) {
        Serial.println("Failed to start scan");
        delay(1000);  
        return;
    }
    Serial.println("Scan started successfully");
    createPublishTask();

	  scanning = true;
    // Wait for measurements to start
    delay(200);
}

void setup() {
    // Start USB serial for debugging and wait for port to be ready
    Serial.begin(115200);
    while(!Serial) {
        delay(100);
    }
    Serial.println("Starting initialization...");
    Serial.printf("Current core: %d, other core: %d\n", xPortGetCoreID(), !xPortGetCoreID());
    
    // Create batch pool first
    batchPool = new RingLaserScanBatchPool(10);
    if (!batchPool) {
        Serial.println("Failed to create batch pool");
        return;
    }
    Serial.println("Batch pool created");
    
    // Then create lidar with the pool
    lidar = new RPLidar(UART_NUM_2, 39, 40, 41, batchPool);
    if (!lidar) {
        Serial.println("Failed to create lidar");
        delete batchPool;
        return;
    }
    Serial.println("Lidar created");
    
        // Initialize lidar
    if (!lidar->begin()) {
        Serial.println("Failed to initialize lidar");
        delete lidar;
        delete batchPool;
        return;
    }
    
    Serial.println("Initialization complete");

    getLidarInfo();

    startLidarScan();
}

void createPublishTask() {
    if (publishTaskHandle == NULL) {
        BaseType_t result = xTaskCreatePinnedToCore(
            publishTask,
            "publish",
            8192,  // Increased stack size
            NULL,
            4, // priority
            &publishTaskHandle,
            1 // core
        );
        if (result != pdPASS) {
            Serial.println("Failed to create publish task");
            return;
        }
        Serial.println("Publish task created successfully");
    }
}

void publishTask(void* arg) {
    //RPLidar* lidar = static_cast<RPLidar*>(arg);
    static unsigned long totalMeasurements = 0;
    static unsigned long totalRotations = 0;
    static unsigned long totalMessages = 0;
    unsigned long startMs = 0;
    unsigned long delayMs = 0;
    unsigned long startmillis = 0;

    const TickType_t xFrequency = pdMS_TO_TICKS(80);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    BaseType_t xWasDelayed;

    Serial.println("Publish task started.");
    startMs = millis();
    startmillis = millis();
    scanning = true;
    while(lidar->publishQueue == NULL) {
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    while(1) {
        LaserScanBatch* batchToPublish;

        xWasDelayed = xTaskDelayUntil(&xLastWakeTime, xFrequency);
        delayMs += xWasDelayed;
        if(lidar->publishQueue != NULL && xQueueReceive(lidar->publishQueue, &batchToPublish, 0) == pdTRUE) {

            totalMeasurements += batchToPublish->total_measurements;
            totalRotations += batchToPublish->total_rotations;
            totalMessages++;
            
            if(totalMessages >= 120) {
                Serial.printf("%d, pubs (ms): %.0f, mes/rot: %.1f, "
                            "Measurements: %d, Rate: %.0f meas/s,"
                            "Free queue space: %d,  Num of Delays: %d\n",
                    totalMessages,
                    1.0*(millis() - startMs)/totalMessages,
                    1.0*totalMeasurements/totalRotations,
                    totalMeasurements,
                    1000.0 * totalMeasurements / (millis() - startMs),
                    uxQueueSpacesAvailable(lidar->publishQueue),
                    delayMs);

                totalMeasurements = 0;
                totalRotations = 0;
                totalMessages = 0;
                delayMs = 0;
                startMs = millis();
            }
        }
        // start & stop every minute
        if (scanning && (millis() - startmillis) > 1 * 60 * 1000) {
            lidar->stopScan();
            scanning = false;
            Serial.printf("Scan stopped at %d ms\n", millis());
        } else if (!scanning && (millis() - startmillis) > 1.5 * 60 * 1000) {
            if (lidar->startExpressScan()) {
                scanning = true;
                startmillis = millis();
    		        Serial.println("Scan restarted successfully.");
		        } else {
              Serial.println("Failed to restart scan.");
            }
        }
    }
    Serial.println("Out of while");
}

void loop() {

}