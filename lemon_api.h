/*******************************************************************************

  @file      lemon_api.h
  @brief
  @details   ~
  @author    cheng-ran@outlook.com
  @date      10.08.2021
  @copyright HengYiFeng, 2021-2023. All right reserved.

*******************************************************************************/
#ifndef LEMON_API_H_
#define LEMON_API_H_

#include <opencv2/opencv.hpp>

#include "datamatrix_decoder.h"
#include "datamatrix_locator.h"
#include "datamatrix_reader.h"
#include "image_processor.h"

namespace hyf_lemon {

/**
 * @brief decode from a image file
 * @param file - file directory *
 * @param output - if success, output the reult
 * @return true - if success
 */
bool Decode_file(const char* file, std::vector<std::vector<uchar>>* output);
/**
 * @brief decode from captured in-memory image
 * @param width - width of image
 * @param height  - height of image
 * @param image_data - pointer to image data
 * @param output - if success, output the reult
 * @return true - if success
 */
bool Decode_rt(const int width, const int height, const uchar* image_data,
               std::vector<std::vector<uchar>>* output);

class Lemon {
 public:
  Lemon();
  ~Lemon();

  bool Decode(std::vector<std::vector<uchar>>* output);
  cv::Mat image() const { return image_; };
  void SetImage(const cv::Mat& image);
  void SetReversed(const bool reversed);
  void SetBinMethod(const BinMethod method);
  void SetBinNormalTh(const unsigned val);
  void SetBinAdaptiveBlock(const unsigned val);

 private:
  ImageProcessor processor_;
  DatamatrixLocator locator_;
  DatamatrixReader reader_;
  cv::Mat image_;
};

namespace {

Lemon lemon;

}  // namespace

}  // namespace hyf_lemon

#endif  // LEMON_API_H_
