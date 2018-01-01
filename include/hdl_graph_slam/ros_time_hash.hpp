#ifndef ROS_TIME_HASH_HPP
#define ROS_TIME_HASH_HPP

#include <unordered_map>
#include <boost/functional/hash.hpp>

#include <ros/time.h>

/**
 * @brief Hash calculation for ros::Time
 */
class RosTimeHash {
public:
  size_t operator() (const ros::Time& val) const {
    size_t seed = 0;
    boost::hash_combine(seed, val.sec);
    boost::hash_combine(seed, val.nsec);
    return seed;
  }
};

#endif // ROS_TIME_HASHER_HPP
