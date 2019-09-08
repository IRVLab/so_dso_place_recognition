#ifndef GIST_H
#define GIST_H

#include "clany/clany_defs.h"
#include "libgist.h"
#include <opencv2/core/core.hpp>

_CLANY_BEGIN
struct GISTParams {
  bool use_color;
  int width;
  int height;
  int blocks;
  int scale;
  vector<int> orients;
};

class GIST {
public:
  GIST() = default;
  GIST(const GISTParams &gist_params) { setParams(gist_params); }

  void setParams(const GISTParams &gist_params);

  void extract(const cv::Mat &src, vector<float> &result) const;

private:
#if __cplusplus >= 201103L
  GISTParams params{false, 256, 256, 4, 4, {8, 8, 8, 8}};
#else
  GISTParams params;
#endif
  int desc_size;
};
_CLANY_END

#endif // GIST_H
