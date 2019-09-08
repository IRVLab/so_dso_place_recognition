#pragma once
#include "boost/thread.hpp"
#include "util/MinimalImage.h"
#include "IOWrapper/Output3DWrapper.h"

#include "FullSystem/HessianBlocks.h"
#include "util/FrameShell.h"

#include "PosesPts.h"

namespace dso
{

class FrameHessian;
class CalibHessian;
class FrameShell;

namespace IOWrap
{

class OutputWrapperLoop : public Output3DWrapper
{
private:
  std::vector<IDPose*> poses_history;
  std::vector<IDPtIntensity*> pts_history;

public:
  ~OutputWrapperLoop();

  void publishKeyframes( std::vector<FrameHessian*> &frames, bool final, CalibHessian* HCalib) override;

};



}



}
