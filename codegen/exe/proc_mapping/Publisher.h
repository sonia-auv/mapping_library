//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// Publisher.h
//
// Code generation for function 'Publisher'
//

#ifndef PUBLISHER_H
#define PUBLISHER_H

// Include files
#include "rtwtypes.h"
#include "mlroscpp_pub.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
namespace coder {
namespace ros {
class Publisher {
public:
  Publisher *init();
  char TopicName[20];
  double BufferSize;
  bool IsLatching;
  std::unique_ptr<MATLABPublisher<sensor_msgs::PointCloud2,
                                  sensor_msgs_PointCloud2Struct_T>>
      PublisherHelper;
};

} // namespace ros
} // namespace coder

#endif
// End of code generation (Publisher.h)
