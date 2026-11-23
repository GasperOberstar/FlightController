#include <fc/sim/gyro_model.h>
#include <fc/math/utils.h>
#include <vector>

namespace sim{
    bool GyroModel::configure(float noiseStd, unsigned seed, std::size_t delaySample, float bias, float initialRate){
        if(!(fc::isFinite(noiseStd) && noiseStd>=0 && fc::isFinite(bias) && fc::isFinite(initialRate) && delaySample<100))
            return false;
        noiseStd_=noiseStd;
        rng_.seed(seed);
        unitNormal_.reset();
        delayLine_.assign(delaySample+1, initialRate);
        head_=0;
        bias_=bias;
        lastOutput_=initialRate+bias;
        return true;
    }

    float GyroModel::sample(float trueRate) {
    if (fc::isFinite(trueRate)){
        delayLine_[head_] = trueRate;
        if (++head_ == delayLine_.size())
            head_ = 0;
        const float delayed = delayLine_[head_];
        lastOutput_=delayed+bias_+noiseStd_*unitNormal_(rng_);
    }
    return lastOutput_;
    }
    
    void GyroModel::reset(float initialRate) {
    if (!fc::isFinite(initialRate)) return;
    delayLine_.assign(delayLine_.size(), initialRate);
    head_ = 0;
    lastOutput_ = initialRate + bias_;
}
}