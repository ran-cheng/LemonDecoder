/*******************************************************************************

  @file      image_processor.cpp
  @brief     binarize image with different methods/parameters, then get contours
  @details   ~
  @author    cheng-ran@outlook.com
  @date      6.06.2021
  @copyright HengYiFeng, 2021. All right reserved.

*******************************************************************************/
//#define DEBUG_IMG_PROC
#include "image_processor.h"
#include "datamatrix_locator.h"

using std::vector;
using namespace cv;

namespace hyf_lemon {

ImageProcessor::ImageProcessor() {
  set_bin_reversed(false);
  Initialize();
}
ImageProcessor::ImageProcessor(const bool reversed, const Mat& source) {
  set_bin_reversed(reversed);
  set_image(source);
  Initialize();
}
ImageProcessor::~ImageProcessor() { image_.release(); }

void ImageProcessor::Initialize() {
  bin_method_ = BIN_ADAPTIVE;
  bin_adaptive_block_ = 25;
  bin_normal_th_ = 127;
}

void ImageProcessor::set_image(const Mat& source) {
  if (!image_.empty()) {
    image_.release();
  }
  image_ = source.clone();
}

void ImageProcessor::Process(Mat* output_binarized,
                             vector<PointSeq>* contours) {
  if (image_.empty()) return;

  medianBlur(image_, image_, 3);
  switch (bin_method_) {
    case BIN_NORMAL:
      BinarizeNormal();
      break;
    case BIN_ADAPTIVE:
      BinarizeAdaptive();
      break;
    default:
      break;
  }

  GetContours(contours);
  FilterContours(contours);
  *output_binarized = image_.clone();

#ifdef DEBUG_IMG_PROC
  namedWindow("Binarized", 1);
  Mat drawing(image_.size(), CV_8UC3);
  cvtColor(image_, drawing, COLOR_GRAY2RGB);
  for (size_contour i = 0; i < contours->size(); i++) {
    Scalar color = Scalar(0, 255, 0); // BGR
    drawContours(drawing, *contours, (int)i, color, 1);
  }
  imshow("Binarized", drawing);
  waitKey(0);
  drawing.release();
#endif  // DEBUG_IMG_PROC
}

void ImageProcessor::BinarizeNormal() {
  threshold(image_, image_, bin_normal_th_, 255,
            !bin_reversed_ ? THRESH_BINARY_INV : THRESH_BINARY);
}

void ImageProcessor::BinarizeAdaptive() {
  adaptiveThreshold(image_, image_, 255, ADAPTIVE_THRESH_MEAN_C,
                    THRESH_BINARY_INV, bin_adaptive_block_, 0);
  if (bin_reversed_) {
    Reverse();
  }
}

void ImageProcessor::Reverse() {
  uchar* p = image_.data;
  for (size_contour i = 0; i < image_.cols * image_.rows; ++i) {
    *p++ = 255 - *p;
  }
}

void ImageProcessor::GetContours(vector<PointSeq>* contours) {
  findContours(image_, *contours, RETR_LIST, CHAIN_APPROX_NONE, Point(0, 0));
}

/**
 @brief check each contours by CheckContour, if not OK remove it
 @param contours - output with some contours removed
**/
void ImageProcessor::FilterContours(vector<PointSeq>* contours) {
  vector<PointSeq> filtered_contours;
  for (PointSeq contour : *contours) {
    if (CheckContour(contour)) {
      filtered_contours.push_back(contour);
    }
  }
  contours->swap(filtered_contours);
}
/**
 @brief  check contour using 3 conditions
 @param  contour
 @retval true: if all the requirements are met
**/
bool ImageProcessor::CheckContour(const PointSeq& contour) {
  // the amount of points that forms the contour should > kMin4PointCnt
  if (contour.size() < kMin4PointCnt) return false;

  // the rect(hori & verti) just bound the contour
  Rect bounding = boundingRect(contour);
  float aspect = bounding.height < bounding.width
                     ? (float)bounding.height / bounding.width
                     : (float)bounding.width / bounding.height;
  // the aspect should > kTh4Aspect
  if (aspect < kTh4Aspect) return false;

  // --
  if (bounding.x < kMin4Gap2Edge || bounding.y < kMin4Gap2Edge) return false;
  if (bounding.x + bounding.width + kMin4Gap2Edge > image_.cols ||
      bounding.y + bounding.height + kMin4Gap2Edge > image_.rows)
    return false;

  return true;
}

}  // namespace hyf_lemon
