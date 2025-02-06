//#pragma once
#ifndef RING_LASER_SCAN_BATCH_POOL_H
#define RING_LASER_SCAN_BATCH_POOL_H

#include "RPLidar.h"

class RingLaserScanBatchPool  : public LaserScanBatchPool {
private:
    LaserScanBatch* _batches;
    MeasurementData* _measurements;
    size_t _numBatches;
    size_t _currentIndex;
    portMUX_TYPE _mutex;

public:
    RingLaserScanBatchPool(size_t numBatches);
    ~RingLaserScanBatchPool();
    LaserScanBatch* acquireBatch() override;
    void releaseBatch(LaserScanBatch* batch) override;
    
    size_t getNumOfBatches() const override;
    size_t getCurrentIndex() const;
};

#endif // RING_LASER_SCAN_BATCH_POOL_H