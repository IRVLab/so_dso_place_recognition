#pragma once
#include "IOWrapper/Output3DWrapper.h"
#include "boost/thread.hpp"
#include "util/MinimalImage.h"

#include "FullSystem/HessianBlocks.h"
#include "util/FrameShell.h"

#include "PosesPts.h"

namespace dso {

class FrameHessian;
class CalibHessian;
class FrameShell;

namespace IOWrap {

class OutputWrapperSODSO : public Output3DWrapper {
private:
  std::vector<IDPose *> poses_history;
  std::vector<IDPtIntensity *> pts_history;

public:
  ~OutputWrapperSODSO();

  void publishKeyframes(std::vector<FrameHessian *> &frames, bool final,
                        CalibHessian *HCalib) override;
};

} // namespace IOWrap

} // namespace dso
