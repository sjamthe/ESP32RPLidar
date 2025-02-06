#include "RPLidar.h"
#include "RingLaserScanBatchPool.h"

RingLaserScanBatchPool::RingLaserScanBatchPool(size_t numBatches) :
    _numBatches(numBatches),
    _currentIndex(0),
    _mutex(portMUX_INITIALIZER_UNLOCKED) {
        
    ESP_LOGV(TAG, "Initializing ring batch pool...");
    
    _batches = new LaserScanBatch[numBatches];
    if (!_batches) {
        ESP_LOGE(TAG, "Failed to allocate batches array");
        return;
    }
    
    _measurements = new MeasurementData[numBatches * MAX_MEASUREMENTS_PER_BATCH];
    if (!_measurements) {
        ESP_LOGE(TAG, "Failed to allocate measurements array");
        delete[] _batches;
        _batches = nullptr;
        return;
    }
    
    // Initialize all batches
    for(size_t i = 0; i < numBatches; i++) {
        _batches[i].measurements = &_measurements[i * MAX_MEASUREMENTS_PER_BATCH];
        _batches[i].max_measurements = MAX_MEASUREMENTS_PER_BATCH;
    }
    
    ESP_LOGV(TAG, "Ring batch pool initialization complete");
}

RingLaserScanBatchPool::~RingLaserScanBatchPool() {
    delete[] _batches;
    delete[] _measurements;
}

LaserScanBatch* RingLaserScanBatchPool::acquireBatch() {
    portENTER_CRITICAL(&_mutex);
    
    // Get current batch
    LaserScanBatch* batch = &_batches[_currentIndex];
    
    // Reset the batch
    batch->total_measurements = 0;
    batch->total_rotations = 0;
    
    // Move to next slot
    _currentIndex = (_currentIndex + 1) % _numBatches;
    
    portEXIT_CRITICAL(&_mutex);
    
    return batch;
}

// No release needed - the ring buffer automatically overwrites old data
void RingLaserScanBatchPool::releaseBatch(LaserScanBatch*) {
    // No-op - we don't need to track releases anymore
}

size_t RingLaserScanBatchPool::getNumOfBatches() const { 
    return _numBatches; 
}

size_t RingLaserScanBatchPool::getCurrentIndex() const { 
    return _currentIndex; 
}