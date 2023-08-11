/*******************************************************************************

  @file      datamatrix_locator.h
  @brief     check out each contour, try to build a area that bound a Datamatrix
  @details   ~
  @author    cheng-ran@outlook.com
  @date      15.06.2021
  @copyright HengYiFeng, 2021-2023. All right reserved.

*******************************************************************************/
#ifndef DATAMATRIX_LOCATOR_H
#define DATAMATRIX_LOCATOR_H

#include <opencv2/opencv.hpp>

#include "image_processor.h"

namespace hyf_lemon {
/**
  @typedef hyf_lemon::MatVec
  @brief   a vector of cv Mat(s)
**/
typedef std::vector<cv::Mat> MatVec;
/**
  @typedef hyf_lemon::size_contour
  @brief   when travel throgh a contour with a indicator(eg. i), the best type
           of i should be size_t. Howere, size_t is unsigned long long, when 
           opereation (eg. i-j) needed, something may get wrong.
           Notice: size_contour will be wrong when contour is very very huge.
**/
typedef long long size_contour;
/**
@struct XPoint_struct
@brief  cv Point whith its index
**/
typedef struct XPoint_struct {
  cv::Point location;
  size_contour index;
} XPoint;
/**
@struct LShape_struct
@brief  the "L" shape in a contour (of a DataMatrix 2D code),consists of
        vertexes p0-p2, angles of both 2 lines and other information.
        after px calculated, it finally becomes closed(has 4 sides).
**/
typedef struct LShape_struct {
  // the vertex  of the "L" shape
  XPoint p0;
  // top-left point
  XPoint p1;
  // bottom-right point
  XPoint p2;
  XPoint px;
  // the position of the "L" shape vertex
  // 0:topleft, 1:leftbottom, 2:bottomright, 3:righttop
  unsigned position;
  double angle1;
  double angle2;
  bool reversed;
} LShape;

int GetDistancePow(cv::Point p1, cv::Point p2);
double GetDistance(cv::Point p1, cv::Point p2);
double GetAngleF(cv::Point p0, cv::Point p1);
int GetPixValue8UC1(const cv::Mat image, cv::Point point);
int GetAngle(cv::Point p0, cv::Point p1);
cv::Point MovePixel(const cv::Point p0, const double angle,
                    const int step,
                    const int direction);
double GetBrightRateInALine(const cv::Mat binary, const cv::Point p0,
                            const double angle, const int L,
                            const int direction);
int GetDashNumberBright(const cv::Mat binary, const cv::Point p0,
                        const double angle, const int length,
                        const int direction);

/**
  @class   DatamatrixLocator
  @brief   check out each contour, get such areas that bound a Datamatrix
  @details ~
**/
class DatamatrixLocator {
 public:
  /**
    @brief DatamatrixLocator object constructor. when constructed, certain
           fields will be set to default values¡£
  **/
  DatamatrixLocator();
  /**
    @brief DatamatrixLocator object constructor. when constructed, certain
           fields will be set to default values.
    @param source   - the binarized image
    @param contours - the vector of contours
  **/
  DatamatrixLocator(const cv::Mat& source,
                    const std::vector<PointSeq>& contours);
  ~DatamatrixLocator();
  /**
    @brief  the main method of DatamatrixLocator. check each contour if it is
            possibly part of a Datamatrix, then output the binarized ROI of the
            possible Datamatrixs(backgound-dark, datamatrix-bright).
    @param  data_matrixs - output possible Datamatrix images
    @retval              - return the count of possible Datamatrix images
  **/
  int LocateDatamatrix(const cv::Mat& source, const ImageProcessor processor,
                       MatVec* datamatrixs);

  // setter & getter
  cv::Mat image() const { return image_; }
  void set_image(const cv::Mat& source);

  std::vector<PointSeq> contours() const { return contours_; };
  void set_contours(const std::vector<PointSeq> contours);

 private:
  /**
    @brief  Get bounding rect of a contour
    @param  coutour - input
    @param  vertex  - output points in the contour which are the closest
    @retval         - bounding rect
  **/
  cv::Rect GetBoundingRect(const PointSeq contour, XPoint* vertex);
  /**
    @brief  check if the coutour is orthogonal(horizontal/vertical), if true
            output the L shape
    @param  coutour - input
    @param  bound   - input: the bound of the contour
    @param  l_shape - output
    @retval         - return whether it is orthogonal
  **/
  bool CheckOrthogonal(const PointSeq contour, const cv::Rect bound,
                       LShape* l_shape);
  /**
    @brief  if the coutour is not orthogonal, use GetLShape
  **/
  bool GetLShape(const PointSeq contour, const cv::Rect bound,
                 const XPoint* vertex, LShape* l_shape);
  /**
    @brief  calibrate the angles of p0-p1 & p0-p2, and p1, p2 location
  **/
  bool CalibrateLShape(const PointSeq contour, LShape* l_shape);
  bool CalibrateAngle(const PointSeq contour, const XPoint p0,
                      const int direction, XPoint* p, double* angle);
  void CalibrateP1P2(const PointSeq contour, const XPoint best_point,
                     const int angle, const int direction, const int orient,
                     XPoint* p);
  void CalibrateP0(LShape* l_shape);
  void RedefineAnglePosition(LShape* l_shape);
  /**
    @brief  check if a blank L beside L shape, meanwhile ajust p1, p2
    @param  l_shape - 
    @retval         - false : if not 
  **/
  bool CheckBlankL(LShape* l_shape);
  /**
    @brief  calc the px pf LShape
    @param  padding - move p1/p2 inside several pixes beforehand
    @param  l_shape - 
    @retval         - false : if can not find px
  **/
  bool SetPx(const cv::Mat& image, const int padding, LShape* l_shape);
  /**
    @brief pushing L shape inside, until both reach a position that bright
           rate is big
    @param padding_back - true: after padding in, padding back 1 px
    @param lShape      - 
  **/
  void PaddingLShape(const cv::Mat& image,  const bool padding_back,
                     LShape* lShape);
  bool EnlargeLShape(LShape* l_shape);
  double Transform4LShape(const cv::Mat& src, const LShape& lShape,
                          cv::Mat* transformed, double w_h = -1.0);
  void Transform(const cv::Mat& src, const cv::Point* vertex, const double w_h,
                 cv::Mat* transformed);
  

  cv::Mat image_;
  std::vector<PointSeq> contours_;
};

}  // namespace hyf_lemon

#endif  // DATAMATRIX_LOCATOR_H
