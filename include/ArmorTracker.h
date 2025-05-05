#ifndef ARMOR_TRACKER_H
#define ARMOR_TRACKER_H

#include "KalmanFilter.h"
#include <opencv2/opencv.hpp>
#include <memory>

class ArmorTracker {
public:
    ArmorTracker(const cv::Point3f& initial_position, double dt);
    
    void update(const cv::Point3f& new_position, double dt);
    cv::Point3f predictNextPosition() const;
    
    int getLostCount() const { return lost_count_; }
    void incrementLostCount() { lost_count_++; }
    void resetLostCount() { lost_count_ = 0; }
    
    double getLastUpdateTime() const { return last_update_time_; }

private:
    std::unique_ptr<KalmanFilter> kf_;
    int lost_count_ = 0;
    double last_dt_ = 0.1;
    double last_update_time_ = 0;
};

#endif // ARMOR_TRACKER_H