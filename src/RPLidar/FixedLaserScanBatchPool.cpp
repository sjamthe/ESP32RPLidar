
#include "RPLidar.h"
#include "FixedLaserScanBatchPool.h"

FixedLaserScanBatchPool::FixedLaserScanBatchPool(size_t numBatches) :
        _maxUsed(0),
        _mutex(portMUX_INITIALIZER_UNLOCKED) {
            
        Serial.println("Initializing batch pool...");
        
        _batches = new LaserScanBatch[numBatches];
        if (!_batches) {
            Serial.println("Failed to allocate batches array");
            return;
        }
        
        _measurements = new MeasurementData[numBatches * MAX_MEASUREMENTS_PER_BATCH];
        if (!_measurements) {
            Serial.println("Failed to allocate measurements array");
            delete[] _batches;
            _batches = nullptr;
            return;
        }
        
        _available = new bool[numBatches];
        if (!_available) {
            Serial.println("Failed to allocate available array");
            delete[] _batches;
            delete[] _measurements;
            _batches = nullptr;
            _measurements = nullptr;
            return;
        }
        
        _numBatches = numBatches;
        Serial.printf("Initializing %d batches\n", numBatches);
        
        for(size_t i = 0; i < numBatches; i++) {
            _batches[i].measurements = &_measurements[i * MAX_MEASUREMENTS_PER_BATCH];
            _batches[i].max_measurements = MAX_MEASUREMENTS_PER_BATCH;
            _available[i] = true;
            //Serial.printf("Batch %d initialized\n", i);
        }
        
        Serial.println("Batch pool initialization complete");
}

FixedLaserScanBatchPool::~FixedLaserScanBatchPool() {
    delete[] _batches;
    delete[] _measurements;
    delete[] _available;
}

LaserScanBatch* FixedLaserScanBatchPool::acquireBatch() {
    portENTER_CRITICAL(&_mutex);
    for(size_t i = 0; i < _numBatches; i++) {
        if(_available[i]) {
            _available[i] = false;
            LaserScanBatch* batch = &_batches[i];
            batch->total_measurements = 0;
            batch->total_rotations = 0;
            portEXIT_CRITICAL(&_mutex);
            return batch;
        }
    }
    size_t current = getCurrentUsedBatches();
    _maxUsed = max(_maxUsed, current);
    portEXIT_CRITICAL(&_mutex);

    return nullptr;
}

void FixedLaserScanBatchPool::releaseBatch(LaserScanBatch* batch) {
    portENTER_CRITICAL(&_mutex);
    size_t index = batch - _batches;
    if(index < _numBatches) {
        _available[index] = true;
    }
    portEXIT_CRITICAL(&_mutex);
}

// Add metrics
size_t FixedLaserScanBatchPool::getNumOfBatches() const { return _numBatches; }

size_t FixedLaserScanBatchPool::getMaxUsedBatches() const { return _maxUsed; }

size_t FixedLaserScanBatchPool::getCurrentUsedBatches() const { 
    size_t used = 0;
    for(size_t i = 0; i < _numBatches; i++) {
        if(!_available[i]) used++;
    }
    return used;
}