#ifndef FIXED_LASER_SCAN_BATCH_POOL_H
#define FIXED_LASER_SCAN_BATCH_POOL_H

#include "RPLidar.h"

class FixedLaserScanBatchPool : public LaserScanBatchPool {
public:
    FixedLaserScanBatchPool(size_t numBatches);
    ~FixedLaserScanBatchPool();
    
    LaserScanBatch* acquireBatch() override;
    void releaseBatch(LaserScanBatch* batch) override;
    
    size_t getNumOfBatches() const override;
    size_t getMaxUsedBatches() const;
    size_t getCurrentUsedBatches() const;

private:
    LaserScanBatch* _batches;
    MeasurementData* _measurements;
    bool* _available;
    size_t _numBatches;
    size_t _maxUsed;
    portMUX_TYPE _mutex;
};

#endif