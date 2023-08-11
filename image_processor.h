/*******************************************************************************

  @file      image_processor.h
  @brief     binarize image with different methods/parameters, then get contours
  @details   ~
  @author    cheng-ran@outlook.com
  @date      6.06.2021
  @copyright HengYiFeng, 2021. All right reserved.

*******************************************************************************/
#ifndef IMAGE_PROCESSOR_H_
#define IMAGE_PROCESSOR_H_

#include <opencv2/opencv.hpp>

namespace hyf_lemon {

/**
  @typedef co_hengyifeng_lemon::PointSeq
  @brief   a collection/string of OpenCV points
**/
typedef std::vector<cv::Point> PointSeq;
/**
  @enum  co_hengyifeng_lemon::BinMethod
  @brief different binarizing methods
**/
enum BinMethod {
  BIN_NORMAL,    // threshold
  BIN_ADAPTIVE,  // adaptiveThreshold
};

/**
  @class   ImageProcessor
  @brief   binarize image then get contours. when constructed, certain fields
           will be set to default values:
  @details bin_convert <- false (normally the datamatrix is dark and its
           backgroud is bright, otherwise "true" should be set to converse it),
           bin_method <- BIN_ADAPTIVE,
           bin_adaptive_block <- 25,
           bin_nornal_th <- 127
**/
class ImageProcessor {
 public:
  /**
    @brief   ImageProcessor object constructor
    @param   source - will be copied, the process do not affect the source
  **/
  ImageProcessor();
  ImageProcessor(const bool reversed, const cv::Mat& source);
  ~ImageProcessor();

  /**
    @brief   the main method of ImageProcessor, be invoked directly or after
             setting bin_reverse, bin_method, and other paremeters.
             attention: must set_image(Mat) or construct with
             ImageProcessor(Mat) beforehead
    @param
             contours - output all the contours
  **/
  void Process(cv::Mat* output_binarized, std::vector<PointSeq>* contours);

  // setter & getter
  cv::Mat image() const { return image_; }
  void set_image(const cv::Mat& source);

  bool bin_reversed() const { return bin_reversed_; }
  void set_bin_reversed(const bool val) { bin_reversed_ = val; }

  BinMethod bin_method() const { return bin_method_; }
  void set_bin_method(const BinMethod method) { bin_method_ = method; }

  unsigned bin_normal_th() const { return bin_normal_th_; }
  void set_bin_normal_th(const unsigned val) { bin_normal_th_ = val; }

  unsigned bin_adaptive_block() const { return bin_adaptive_block_; }
  void set_bin_adaptive_block(const unsigned val) { bin_adaptive_block_ = val; }

 private:
  void Initialize();
  void BinarizeNormal();
  void BinarizeAdaptive();
  void Reverse();
  void GetContours(std::vector<PointSeq>* contours);
  void FilterContours(std::vector<PointSeq>* contours);
  bool CheckContour(const PointSeq& conour);

 private:
  cv::Mat image_;
  bool bin_reversed_;
  BinMethod bin_method_;
  int bin_normal_th_;
  int bin_adaptive_block_;
  // the min point count : each element > 4pix, each side has a minimum of 10
  // elememts, 4 sides in total
  const int kMin4PointCnt = 4 * 10 * 4;
  // the threshold of the aspect ratio of a datamatrix
  const float kTh4Aspect = 0.20f;
  // the min of the distances between a datamatrix and image's edges
  const int kMin4Gap2Edge = 4;
};

}  // namespace hyf_lemon

#endif  // IMAGE_PROCESSOR_H_
