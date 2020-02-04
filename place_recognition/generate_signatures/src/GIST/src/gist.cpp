#include "../include/gist.h"
#include <memory>
#include <numeric>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;
using namespace cls;

namespace {
using DescPtr = unique_ptr<float, void (*)(void *)>;
using ColorImgPtr = unique_ptr<color_image_t, void (*)(color_image_t *)>;
using GrayImgPtr = unique_ptr<image_t, void (*)(image_t *)>;

class GISTImage {
public:
  GISTImage(const cv::Mat &src) {
    if (src.channels() == 1) {
      gray_img.reset(image_new(src.cols, src.rows));
      memcpy(gray_img->data, src.data, src.cols * src.rows * sizeof(float));
    } else {
      vector<cv::Mat> bgr;
      cv::split(src, bgr);

      color_img.reset(color_image_new(src.cols, src.rows));
      memcpy(color_img->c1, bgr[2].data, src.cols * src.rows * sizeof(float));
      memcpy(color_img->c2, bgr[1].data, src.cols * src.rows * sizeof(float));
      memcpy(color_img->c3, bgr[0].data, src.cols * src.rows * sizeof(float));
    }
  }

  operator color_image_t *() { return color_img.get(); }

  operator image_t *() { return gray_img.get(); }

private:
  ColorImgPtr color_img{nullptr, &color_image_delete};
  GrayImgPtr gray_img{nullptr, &image_delete};
};
} // Unnamed namespace

//////////////////////////////////////////////////////////////////////////
void GIST::setParams(const GISTParams &gist_params) {
  params = gist_params;

  desc_size = accumulate(params.orients.begin(), params.orients.end(), 0,
                         [&gist_params](int init, int orient) {
                           return init + gist_params.blocks *
                                             gist_params.blocks * orient;
                         });
  desc_size *= params.use_color ? 3 : 1;
}

void GIST::extract(const Mat &_src, vector<float> &result) const {
  assert(!_src.empty());
  cv::Mat src(_src.clone());

  double h = params.height;
  double w = params.width;
  double h_ratio = h / src.rows;
  double w_ratio = w / src.cols;

  Mat cropped(src);
  if (h_ratio < w_ratio) {
    // Resize to same width
    resize(src, src, Size(), w_ratio, w_ratio, INTER_LANCZOS4);
    // Crop to get same size
    cropped = src(Rect(0, (src.rows - h) / 2, w, h));
  }
  if (w_ratio < h_ratio) {
    // Resize to same height
    resize(src, src, Size(), h_ratio, h_ratio, INTER_LANCZOS4);
    // Crop to get same size
    cropped = src(Rect((src.cols - w) / 2, 0, w, h));
  }

  // Compute gist descriptor
  cropped.convertTo(cropped, CV_32F);
  DescPtr desc(nullptr, &free);

  if (params.use_color) {
    assert(cropped.channels() == 3);
    desc.reset(color_gist_scaletab(GISTImage(cropped), params.blocks,
                                   params.scale, params.orients.data()));
  } else {
    // cvtColor(cropped, cropped, CV_BGR2GRAY);
    desc.reset(bw_gist_scaletab(GISTImage(cropped), params.blocks, params.scale,
                                params.orients.data()));
  }

  // Copy to result
  result.resize(desc_size);
  memcpy(result.data(), desc.get(), sizeof(float) * desc_size);
}
