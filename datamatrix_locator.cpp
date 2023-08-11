/*******************************************************************************

  @file      datamatrix_locator.cpp
  @brief     check out each contour, try to build a area that bound a Datamatrix
  @details   ~
  @author    cheng-ran@outlook.com
  @date      15.06.2021
  @copyright HengYiFeng, 2021-2023. All right reserved.

*******************************************************************************/
//#define DEBUG_DM_LOC

#include "datamatrix_locator.h"

#include <iostream>

using std::cout;
using std::endl;
using std::vector;
using namespace cv;

namespace hyf_lemon {

int GetDistancePow(Point p1, Point p2) {
  return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y);
}

double GetDistance(Point p1, Point p2) {
  return sqrt((double)(p1.x - p2.x) * (p1.x - p2.x) +
              (p1.y - p2.y) * (p1.y - p2.y));
}

double GetAngleF(Point p0, Point p1) {
  double deltaX = p0.x - p1.x;
  double deltaY = p0.y - p1.y;
  double thita = 0.0;
  double angle = 0.0;
  if (deltaX != 0.0) {
    thita = atan(deltaY / deltaX);
    angle = thita * 180 / CV_PI;
  } else
    angle = 90.0;
  if (angle > 0)
    angle = 180.0 - angle;
  else
    angle = -angle;
  return angle;
}

int GetAngle(Point p0, Point p1) {
  double deltaX = p0.x - p1.x;
  double deltaY = p0.y - p1.y;
  double thita = 0.0;
  int angle = 0;
  if (deltaX != 0.0) {
    thita = atan(deltaY / deltaX);
    angle = (int)floor(thita * 180 / CV_PI + 0.5);
  } else
    angle = 90;
  if (angle > 0)
    angle = 180 - angle;
  else
    angle = -angle;

  return angle;
}

int GetPixValue8UC1(const Mat image, Point point) {
  int channel = image.channels();
  int step = (int)image.step;
  int idx = point.y * step + point.x * channel;
  if (idx < 0) return 0;
  if (idx >= image.rows * step) return 0;
  return (uchar)image.data[idx];
}

Point MovePixel(const Point p0, const double angle, const int step,
                const int direction) {
  Point p;
  double x = p0.x - direction * cos(CV_PI * angle / 180.0) * step;
  double y = p0.y + direction * sin(CV_PI * angle / 180.0) * step;
  p.x = (int)floor(x + 0.5);
  p.y = (int)floor(y + 0.5);

  return p;
}

double GetBrightRateInALine(const Mat binary, const Point p0,
                            const double angle, const int L,
                            const int direction) {
  Point track;
  int n_bright = 0;
  for (int i = 0; i < L; i++) {
    track = MovePixel(p0, angle, i, direction);
    if (GetPixValue8UC1(binary, track) == 255) n_bright++;
  }
  return (double)n_bright / L;
}

int GetDashNumberBright(const Mat binary, const Point p0, const double angle,
                        const int length, const int direction) {
  int kMinIsland = 1;
  Point track;
  vector<int> bright_island;
  bool is_bright = false;
  int position;
  int n_dash = 0;
  // check each point
  for (int i = 0; i < length; i++) {
    track = MovePixel(p0, angle, i, direction);
    if (!is_bright && GetPixValue8UC1(binary, track) == 255) {
      is_bright = true;
      position = i;
    }
    if (is_bright) {
      if (GetPixValue8UC1(binary, track) == 0 || i == length - 1) {
        is_bright = false;
        bright_island.push_back(i - position);
      }
    }
  }
  // count and ignore little island
  for (int j = 0; j < bright_island.size(); j++) {
    if (bright_island[j] > kMinIsland) n_dash++;
  }
  vector<int>().swap(bright_island);
  return n_dash;
}

/****************************************************************************
 *                                   class                                   *
 ****************************************************************************/

DatamatrixLocator::DatamatrixLocator() { }
DatamatrixLocator::DatamatrixLocator(const Mat& source,
                                     const vector<PointSeq>& contours) {
  image_ = source;
  contours_ = contours;
}
DatamatrixLocator::~DatamatrixLocator() {
  image_.release();
  vector<PointSeq>().swap(contours_);
}

void DatamatrixLocator::set_image(const Mat& source) {
  if (!image_.empty()) {
    image_.release();
  }
  image_ = source;
}

void DatamatrixLocator::set_contours(const vector<PointSeq> contours) {
  vector<PointSeq>().swap(contours_);
  contours_ = contours;
}

int DatamatrixLocator::LocateDatamatrix(const Mat& source,
                                        const ImageProcessor processor,
                                        MatVec* datamatrixs) {
#ifdef DEBUG_DM_LOC
  namedWindow("Locator", 1);
  Mat drawing(image_.size(), CV_8UC3);
  cvtColor(image_, drawing, COLOR_GRAY2RGB);  
#endif
  int contour_index = 0;
  int n_good_matrix = 0;
  for (PointSeq contour : contours_) {    
    // the 4 vertex points in a contour
    XPoint vertex[4];
    // bounding rect
    Rect bound = GetBoundingRect(contour, vertex);
    // l_shape for each contour
    LShape l_shape;
    l_shape.position = -1;  // empty
    l_shape.angle1 = l_shape.angle2 = 0.0;
    l_shape.reversed = false;
    // Check if is Orthogonal, meanwhile get l_shape if is, it is faster
    if (!CheckOrthogonal(contour, bound, &l_shape)) {      
      // if not Orthogonal, another way to get l_shape
      if (!GetLShape(contour, bound, vertex, &l_shape)) continue;
      if (!CalibrateLShape(contour, &l_shape)) continue;
    }
    if (l_shape.position == -1) continue;

    CalibrateP0(&l_shape);
    RedefineAnglePosition(&l_shape);
    // check blank L and reset p1,p2 -> then p0
    if (!CheckBlankL(&l_shape)) continue;
    if (!SetPx(image_, 2, &l_shape)) continue;
    PaddingLShape(image_, true, &l_shape);       
    // transform 1 l_shape -> rectangle  
    if (!EnlargeLShape(&l_shape)) continue;    
    int image_size = source.cols > source.rows ? source.cols : source.rows;
    Mat transformed_1 = Mat::zeros(Size(image_size, image_size), CV_8UC1);    
    int image_w_h =
        floor(Transform4LShape(source, l_shape, &transformed_1, -1) + 0.5); 
    Mat binary_1 = transformed_1(Rect(0, 0, image_w_h, image_w_h)).clone();
    ImageProcessor p = processor;
    p.set_image(binary_1);
    vector<PointSeq> no_use;
    p.Process(&binary_1, &no_use);
    // modify L shape
    l_shape.p0.location = Point(0, image_w_h - 1);
    l_shape.p1.location = Point(0, 0);
    l_shape.p2.location = Point(image_w_h - 1, image_w_h - 1);
    l_shape.angle1 = 90.0;
    l_shape.angle2 = 0.0;
    l_shape.reversed = 0;
    if (!SetPx(binary_1, 5, &l_shape)) continue;
    PaddingLShape(binary_1, false, &l_shape);

    // transform again
    Mat transformed_2(image_w_h, image_w_h, CV_8UC1);
    Transform4LShape(transformed_1, l_shape, &transformed_2, image_w_h);
    n_good_matrix++; // success!
    datamatrixs->push_back(transformed_2);

  }
#ifdef DEBUG_DM_LOC
  imshow("Locator", drawing);
  waitKey(0);
#endif  // DEBUG_DM_LOC
  return n_good_matrix;
}

Rect DatamatrixLocator::GetBoundingRect(const PointSeq contour,
                                        XPoint* vertex) {
  /* return the boundary of contour
   * find 4 points in the contour which are the closest to bound's vertex,
   * output them to vertex */
  Rect bound = boundingRect(contour);
  // the 4 vertex points of bound
  Point bound_vertex[4];
  bound_vertex[0] = Point(bound.x, bound.y),
  bound_vertex[1] = Point(bound.x, bound.y + bound.height),
  bound_vertex[2] = Point(bound.x + bound.width, bound.y + bound.height),
  bound_vertex[3] = Point(bound.x + bound.width, bound.y);
  // get the closest point(of the contour) to each bound vertex
  int distance[] = {1000000, 1000000, 1000000, 1000000};
  size_contour i, j;
  for (i = 0; i < (size_contour)contour.size(); i++) {
    Point p = contour.at(i);
    int x = p.x, y = p.y;
    for (j = 0; j < 4; j++) {
      int d = GetDistancePow(p, bound_vertex[j]);
      if (d < distance[j]) {
        distance[j] = d;
        vertex[j].location.x = x;
        vertex[j].location.y = y;
        vertex[j].index = i;
      }
    }
  }
  // check rotation
  int top = 10000, right = -1, bottom = -1, left = 10000;
  for (j = 0; j < 4; j++) {
    int x = vertex[j].location.x, y = vertex[j].location.y;
    if (y < top) top = y;
    if (x < left) left = x;
    if (y > bottom) bottom = y;
    if (x > right) right = x;
  }
  int minArea, boundArea;
  double rotateRate;
  minArea = (right - left) * (bottom - top);
  boundArea = bound.width * bound.height;
  rotateRate = double(minArea) / boundArea;
  if (rotateRate < 0.75) {
    // if the rotation exceeds a certain degree, reassign vertex with the
    // most top\left\bottom\right points in the contour
    // get the most left,top,bottom,right point in the contour
    int top = 10000, right = -1, bottom = -1, left = 10000;
    for (i = 0; i < (size_contour)contour.size(); i++) {
      Point p = contour.at(i);
      int x = p.x, y = p.y;

      if (y < top) {
        top = y;
        vertex[0].location.x = x;
        vertex[0].location.y = y;
        vertex[0].index = i;
      }
      if (x < left) {
        left = x;
        vertex[1].location.x = x;
        vertex[1].location.y = y;
        vertex[1].index = i;
      }
      if (y > bottom) {
        bottom = y;
        vertex[2].location.x = x;
        vertex[2].location.y = y;
        vertex[2].index = i;
      }
      if (x > right) {
        right = x;
        vertex[3].location.x = x;
        vertex[3].location.y = y;
        vertex[3].index = i;
      }
    }
  }

  return bound;
}

bool DatamatrixLocator::CheckOrthogonal(const PointSeq contour,
                                        const Rect bound, LShape* l_shape) {
  /* if the contour is orthogonal(horiz/verti), get the "L" shape
   *  match every contour point to bound, make sure most(kOverlayRate) of the
   *  points not too far from bound(kGap).
   *  notice: bound is orthogonal.*/
  const int kGap = 4;
  const double kOverlayRate = 0.7;
  int counters[4] = {0};
  size_contour i;
  for (i = 0; i < (size_contour)contour.size(); i++) {
    Point p = contour.at(i);
    int x = p.x, y = p.y;
    if (y - bound.y < kGap) counters[0]++;                 // top
    if (x - bound.x < kGap) counters[1]++;                 // left
    if (bound.y + bound.height - y < kGap) counters[2]++;  // bottom
    if (bound.x + bound.width - x < kGap) counters[3]++;   // right
  }

  // 2 max lines
  int max1 = 0;
  size_contour maxIdx1 = 0;
  for (i = 0; i < 4; i++) {
    if (counters[i] > max1) {
      max1 = counters[i];
      maxIdx1 = i;
    }
  }
  double rate = 0.0;
  if (maxIdx1 % 2)
    rate = (double)max1 / bound.height;
  else
    rate = (double)max1 / bound.width;
  if (rate < kOverlayRate) return false;

  int max2 = 0;
  size_contour maxIdx2 = 0;
  for (i = 0; i < 4; i++) {
    if (i != maxIdx1 && counters[i] > max2) {
      max2 = counters[i];
      maxIdx2 = i;
    }
  }
  if (maxIdx2 % 2)
    rate = (double)max2 / bound.height;
  else
    rate = (double)max2 / bound.width;

  if (rate < kOverlayRate) return false;
  
  // initial l_shape
  l_shape->p0.index = -1;
  l_shape->p1.index = -1;
  l_shape->p2.index = -1;
  // 0:top-left,1:left-bottom,2:bottom-right,3:right-top
  if ((maxIdx1 == 0 && maxIdx2 == 1) || (maxIdx1 == 1 && maxIdx2 == 0)) {
    l_shape->position = 0;
    l_shape->p0.location = Point(bound.x, bound.y);
    l_shape->p1.location = Point(bound.x + bound.width, bound.y);
    l_shape->p2.location = Point(bound.x, bound.y + bound.height);
    l_shape->angle1 = 0.0;
    l_shape->angle2 = 90.0;
  } else if ((maxIdx1 == 1 && maxIdx2 == 2) || (maxIdx1 == 2 && maxIdx2 == 1)) {
    l_shape->position = 1;
    l_shape->p0.location = Point(bound.x, bound.y + bound.height);
    l_shape->p1.location = Point(bound.x, bound.y);
    l_shape->p2.location = Point(bound.x + bound.width, bound.y + bound.height);
    l_shape->angle1 = 90.0;
    l_shape->angle2 = 0.0;
  } else if ((maxIdx1 == 2 && maxIdx2 == 3) || (maxIdx1 == 3 && maxIdx2 == 2)) {
    l_shape->position = 2;
    l_shape->p0.location = Point(bound.x + bound.width, bound.y + bound.height);
    l_shape->p1.location = Point(bound.x, bound.y + bound.height);
    l_shape->p2.location = Point(bound.x + bound.width, bound.y);
    l_shape->angle1 = 0.0;
    l_shape->angle2 = 90.0;
  } else if ((maxIdx1 == 3 && maxIdx2 == 0) || (maxIdx1 == 0 && maxIdx2 == 3)) {
    l_shape->position = 3;
    l_shape->p0.location = Point(bound.x + bound.width, bound.y);
    l_shape->p1.location = Point(bound.x + bound.width, bound.y + bound.height);
    l_shape->p2.location = Point(bound.x, bound.y);
    l_shape->angle1 = 90.0;
    l_shape->angle2 = 0.0;
  }

  // get the most closed point(in contour) to each conner point(90L), and that
  // would be the new p1,p2
  int min1 = 10000, min2 = 10000;
  size_contour idx1=0, idx2=0;
  int x1 = l_shape->p1.location.x;
  int y1 = l_shape->p1.location.y;
  int x2 = l_shape->p2.location.x;
  int y2 = l_shape->p2.location.y;

  Point p1min, p2min;
  for (i = 0; i < (size_contour)contour.size(); i++) {
    Point p = contour.at(i);
    int x = p.x, y = p.y;
    int d = (x - x1) * (x - x1) + (y - y1) * (y - y1);
    if (d < min1) {
      p1min = p;
      idx1 = i;
      min1 = d;
    }

    d = (x - x2) * (x - x2) + (y - y2) * (y - y2);
    if (d < min2) {
      p2min = p;
      idx2 = i;
      min2 = d;
    }
  }
  l_shape->p1.location = p1min;
  l_shape->p1.index = idx1;
  l_shape->p2.location = p2min;
  l_shape->p2.index = idx2;

  return true;
}

bool DatamatrixLocator::GetLShape(const PointSeq contour, const Rect bound,
                                  const XPoint* vertex, LShape* l_shape) {
  /*  if the contour is not orthogonal, get the "L" shape
   *  by determining which vertexes fit 2 good lines in the contour.*/
  const double kLineError = 0.8 * 0.8;  // error^2
  const double kAspectError = 0.04;     // 1:4, so 0.25*0.25=0.0625
  const int kMinStep = 1600;            // (4*10)^2, 4px:min datamatrix element
  int line_length[4];
  double rates[4] = {0.0};

  int i;
  for (i = 0; i < 4; i++) {
    // calculate the straight line distance of the 4 vertex, and the contour
    // distance of the 4 vertex, calculate the rate
    int next = i + 1;
    if (i == 3) next = 0;
    line_length[i] = (vertex[i].location.x - vertex[next].location.x) *
                         (vertex[i].location.x - vertex[next].location.x) +
                     (vertex[i].location.y - vertex[next].location.y) *
                         (vertex[i].location.y - vertex[next].location.y);

    size_contour index0 = vertex[i].index;
    size_contour index1 = vertex[next].index;
    size_contour index_diff = index1 - index0;
    if (index1 < index0) index_diff = index1 + contour.size() - index0 + 1;
    size_contour steps_sqr = index_diff * index_diff;
    // i=0:top-left,1:left-bottom,2:bottom-right,3:right-top
    rates[i] = (double)line_length[i] / steps_sqr;
    if (line_length[i] < kMinStep) rates[i] = 0.0;
  }
  // 2 max lines
  // find 2 biggist rates£¬each rate must > kLineError
  double max1 = 0.0;
  size_contour maxIdx1 = 0;
  for (i = 0; i < 4; i++) {
    if (rates[i] > max1) {
      max1 = rates[i];
      maxIdx1 = i;
    }
  }
  if (max1 < kLineError) return false;
  
  double max2 = 0.0;
  size_contour maxIdx2 = 0;
  for (i = 0; i < 4; i++) {
    if (i != maxIdx1 && rates[i] > max2) {
      max2 = rates[i];
      maxIdx2 = i;
    }
  }
  if (max2 < kLineError) return false;
  // check if L1 is too much longer than L2
  // notice: lineLength is squared
  double aspect = line_length[maxIdx2] < line_length[maxIdx1]
                      ? (double)line_length[maxIdx2] / line_length[maxIdx1]
                      : (double)line_length[maxIdx1] / line_length[maxIdx2];
  if (aspect < kAspectError) return false;
  // initial l_shape
  // 0:top-left,1:left-bottom,2:bottom-right,3:right-top
  if ((maxIdx1 == 0 && maxIdx2 == 1) || (maxIdx1 == 1 && maxIdx2 == 0)) {
    l_shape->position = 0;
    l_shape->p0 = vertex[1];
    l_shape->p1 = vertex[0];
    l_shape->p2 = vertex[2];
    l_shape->angle1 = GetAngleF(l_shape->p0.location, l_shape->p1.location);
    l_shape->angle2 = GetAngleF(l_shape->p0.location, l_shape->p2.location);
  } else if ((maxIdx1 == 1 && maxIdx2 == 2) || (maxIdx1 == 2 && maxIdx2 == 1)) {
    l_shape->position = 1;
    l_shape->p0 = vertex[2];
    l_shape->p1 = vertex[1];
    l_shape->p2 = vertex[3];
    l_shape->angle1 = GetAngleF(l_shape->p0.location, l_shape->p1.location);
    l_shape->angle2 = GetAngleF(l_shape->p0.location, l_shape->p2.location);
  } else if ((maxIdx1 == 2 && maxIdx2 == 3) || (maxIdx1 == 3 && maxIdx2 == 2)) {
    l_shape->position = 2;
    l_shape->p0 = vertex[3];
    l_shape->p1 = vertex[2];
    l_shape->p2 = vertex[0];
    l_shape->angle1 = GetAngleF(l_shape->p0.location, l_shape->p1.location);
    l_shape->angle2 = GetAngleF(l_shape->p0.location, l_shape->p2.location);
  } else if ((maxIdx1 == 3 && maxIdx2 == 0) || (maxIdx1 == 0 && maxIdx2 == 3)) {
    l_shape->position = 3;
    l_shape->p0 = vertex[0];
    l_shape->p1 = vertex[3];
    l_shape->p2 = vertex[1];
    l_shape->angle1 = GetAngleF(l_shape->p0.location, l_shape->p1.location);
    l_shape->angle2 = GetAngleF(l_shape->p0.location, l_shape->p2.location);
  }

  if (fabs(l_shape->angle1 - l_shape->angle2) < 45.0f ||
      fabs(l_shape->angle1 - l_shape->angle2) > 135.0f)
    return false;

  return true;
}

bool DatamatrixLocator::CalibrateLShape(const PointSeq contour,
                                        LShape* l_shape) {
  // adjust angle1,angle2,p1,p2
  XPoint pHome1 = l_shape->p0, pHome2 = l_shape->p0;
  bool a = CalibrateAngle(contour, pHome1, +1, &l_shape->p1, &l_shape->angle1);
  bool b = CalibrateAngle(contour, pHome2, -1, &l_shape->p2, &l_shape->angle2);

  if (fabs(l_shape->angle1 - l_shape->angle2) < 45.0f ||
      fabs(l_shape->angle1 - l_shape->angle2) > 135.0f)
    return false;

  if (a && b) return true;
  return false;
}

bool DatamatrixLocator::CalibrateAngle(const PointSeq contour, const XPoint p0,
                                       const int direction, XPoint* p,
                                       double* angle) {
  /* get a better angle for lShape.angle1,lShape.angle2
   * set several sample points in the path from p0 to p(p1/p2), calc every angle
   * formed by p-sample and every point of the path. the angle value that
   * appears the most times wins, and the p-sample wins. */
  const int kSmapleSize = 6;
  size_contour kTotal = contour.size();
  XPoint samples[kSmapleSize];
  size_contour i, idx, path;
  // hough(count) for each sample by every angle
  int hough[kSmapleSize * 180];
  for (i = 0; i < kSmapleSize * 180; i++) hough[i] = 0;
  path = direction * ((size_contour)p0.index - (size_contour)p->index);
  if (path < 0) path = kTotal + path;
  const size_contour kMinHough = path / 4;

  // set sample points
  size_contour interval = path / (kSmapleSize + 1);
  for (i = 0; i < kSmapleSize; ++i) {
    idx = p->index + direction * interval * (i + 1);
    idx += idx < 0 ? kTotal : 0;
    idx -= idx >= kTotal ? kTotal : 0;
    Point sample = contour.at(idx);
    samples[i].location = sample;
    samples[i].index = idx;
  }
  // set angle formed by every point(of the path) to each sample points
  Point current_point;
  idx = p->index;
  int current_angle;
  int current_hough = 0, max_hough = 0, max_angle = -1;
  size_contour max_sample = -1;
  while (1) {
    idx += idx < 0 ? kTotal : 0;
    idx -= idx >= kTotal ? kTotal : 0;
    if (idx == p0.index + 1) break;
    if (idx == p0.index - 1) break;

    current_point = contour.at(idx);
    for (i = 0; i < kSmapleSize; i++) {
      if (samples[i].index == idx) continue;
      current_angle = GetAngle(samples[i].location, current_point);
      current_hough = ++hough[i * 180 + current_angle];
      if (current_hough > max_hough) {
        max_hough = current_hough;
        max_angle = current_angle;
        max_sample = i;
      }
    }
    idx = idx + direction;
  }
  if (max_hough < kMinHough) return false;
  // calibate
  XPoint best_point = samples[max_sample];
  double diff = max_angle - *angle;
  if ((diff < 0.0 && diff > -90.0) || diff > 90.0)  // clockwise
    CalibrateP1P2(contour, best_point, max_angle, -1, -direction, p);
  if ((diff > 0.0 && diff < 90.0) || diff < -90.0)  // counter clockwise
    CalibrateP1P2(contour, best_point, max_angle, +1, +direction, p);

  *angle = (double)max_angle;
  return true;
}

void DatamatrixLocator::CalibrateP1P2(const PointSeq contour,
                                      const XPoint best_point, const int angle,
                                      const int direction, const int orient,
                                      XPoint* p) {
  /*  ajust p1/p2. move p1/p2 forward and backward, until angle formed by it
   *   and best_point reach the error limit
   */
  const double kErrorLimit = 2.0;
  const size_contour kTotal = contour.size();
  Point current_point;
  XPoint final = *p;
  size_contour idx = p->index + direction;
  int counter = 0;
  while (1) {
    if (idx == -1) idx = kTotal - 1;
    if (idx == kTotal + 1) idx = 0;
    if (counter > 30) break;
    current_point = contour.at(idx);
    double diff = fabs(GetAngleF(best_point.location, current_point) - angle);
    if (orient == -1) {  // further to home
      if (diff > kErrorLimit) {
        break;
      } else {
        final.location = current_point;
        final.index = idx;
      }
    } else if (orient == +1) {  // closer to home
      if (diff < kErrorLimit) {
        final.location = current_point;
        final.index = idx;
        break;
      }
    }

    idx = idx + direction;
    counter++;
  }
  *p = final;
}

void DatamatrixLocator::CalibrateP0(LShape* l_shape) {
  /*  adjust home point according to new angle1,angle2 and new p1,p2 */
  XPoint p0;
  int x1 = l_shape->p1.location.x, y1 = l_shape->p1.location.y;
  int x2 = l_shape->p2.location.x, y2 = l_shape->p2.location.y;
  double a1 = CV_PI * l_shape->angle1 / 180.0;
  double a2 = CV_PI * l_shape->angle2 / 180.0;

  double x = 0.0, y = 0.0;
  if (l_shape->angle1 == 90.0 || l_shape->angle1 == 270.0) {
    x = x1;
    y = (x2 - x) * tan(a2) + y2;
  } else if (l_shape->angle1 == 0.0 || l_shape->angle1 == 180.0) {
    y = y1;
    x = (y2 - y) / tan(a2) + x2;
  } else if (l_shape->angle2 == 90.0 || l_shape->angle2 == 270.0) {
    x = x2;
    y = (x1 - x) * tan(a1) + y1;
  } else if (l_shape->angle2 == 0.0 || l_shape->angle2 == 180.0) {
    y = y2;
    x = (y1 - y) / tan(a1) + x1;
  } else {
    x = (tan(a2) * x2 - tan(a1) * x1 - y1 + y2) / (tan(a2) - tan(a1));
    y = y1 + tan(a1) * (x1 - x);
  }

  p0.location.x = (int)floor(x + 0.5);
  p0.location.y = (int)floor(y + 0.5);
  p0.index = -1;

  l_shape->p0 = p0;
}

void DatamatrixLocator::RedefineAnglePosition(LShape* l_shape) {
  /* redefine position, angle1, angl2 */
  XPoint p0 = l_shape->p0;
  int x1 = l_shape->p1.location.x, y1 = l_shape->p1.location.y;
  int x2 = l_shape->p2.location.x, y2 = l_shape->p2.location.y;
  // redefine position
  if (45.0 < l_shape->angle1 && l_shape->angle1 < 135.0) {
    if (p0.location.y > y1)
      l_shape->position = 1;
    else
      l_shape->position = 3;
  } else if (45.0 > l_shape->angle1 || l_shape->angle1 > 135.0) {
    if (p0.location.x < x1)
      l_shape->position = 0;
    else
      l_shape->position = 2;
  }
  // redefine angle1,angl2(0~180 -> 0~360)
  switch (l_shape->position) {
    case 0:
      if (l_shape->angle1 > 90.0) l_shape->angle1 += 180.0;
      l_shape->angle2 += 180.0;
      break;
    case 1:
      if (l_shape->angle2 > 90.0) l_shape->angle2 += 180.0;
      break;
    case 2:
      if (l_shape->angle1 < 90.0) l_shape->angle1 += 180.0;
      break;
    case 3:
      l_shape->angle1 += 180.0;
      if (l_shape->angle2 < 90.0) l_shape->angle2 += 180.0;
      break;
  }
}

bool DatamatrixLocator::CheckBlankL(LShape* l_shape) {
  const int kSteps = 10;
  const double kBrightRate = 0.05;
  Point p0 = l_shape->p0.location;
  Point p1 = l_shape->p1.location;
  Point p2 = l_shape->p2.location;
  const int kLength1 = floor(GetDistance(p0, p1) + 0.5);
  const int kLength2 = floor(GetDistance(p0, p2) + 0.5);

  int i;
  double angle90_p1 = l_shape->angle1 + 90.0;
  double angle90_p2 = l_shape->angle2 - 90.0;
  int move1 = 0, move2 = 0;

  // must get a blank L(border:1px) in kSteps pixels
  for (i = 0; i < kSteps; i++) {
    p1 = MovePixel(p1, angle90_p1, 1, -1);
    move1++;
    // track
    double rate =
        GetBrightRateInALine(image_, p1, l_shape->angle1, kLength1 + i, +1);
    if (rate < kBrightRate) break;
    
  }
  if (move1 == kSteps)
    return false;
 // else
 //  p1 = MovePixel(l_shape->p1.location, angle90_p1, move1, -1);

  for (i = 0; i < kSteps; i++) {  // must get a blank L(border:1px) in ? pixels
    p2 = MovePixel(p2, angle90_p2, 1, -1);
    move2++;
    // track
    double rate =
        GetBrightRateInALine(image_, p2, l_shape->angle2, kLength2 + i, +1);
    if (rate < kBrightRate) break;
  }
  if (move2 == kSteps)
    return false;
//  else
//    p2 = MovePixel(l_shape->p2.location, angle90_p2, move2, -1);
  l_shape->p1.location = p1;
  l_shape->p2.location = p2;
  CalibrateP0(l_shape);

  return true;
}

bool DatamatrixLocator::SetPx(const Mat& image, const int padding,
                              LShape* l_shape) {
  /* set px of L shape
   */
  const int kTrackLimit = 15; // pixel
  const double kBrightRate = 0.05;
  const int kRotateLimit = 15;  // degree
  double angleX1, angleX2;


  Point p0 = l_shape->p0.location;
  Point p1 = l_shape->p1.location;
  Point p2 = l_shape->p2.location;
  // track length
  double L1 = GetDistance(p0, p1);
  double L2 = GetDistance(p0, p2);
  int L;
  int j;

  // track 90  DEGREE_ALLOW degree line: p1,p2
  int rotate;
  double newAngle;
  Point ptrack, pTemp, pTemp1;
  double goodAngles[kTrackLimit] = {-1.0};
  int rates[kTrackLimit] = {-1};
  int maxRate = 0, maxIdx;

  // p1
  p1 = MovePixel(p1, l_shape->angle1, padding, +1);
  for (j = 0; j < kTrackLimit; j++) {
    // extend out
    pTemp = MovePixel(p1, l_shape->angle1, j, -1);
    pTemp1 = MovePixel(p1, l_shape->angle1, j + 2, -1);
    for (rotate = -kRotateLimit; rotate <= kRotateLimit; rotate++) {
      newAngle = l_shape->angle1 - 90.0 + (double)rotate;
      L = (int)floor(L2 / cos(CV_PI * rotate / 180.0) + 0.5);
      double rate = GetBrightRateInALine(image, pTemp1, newAngle, L, -1);
      if (rate < kBrightRate) {
        goodAngles[j] = newAngle;
        rates[j] = GetDashNumberBright(image, pTemp, newAngle, L, -1);
        break;
      }
    }
  }
  for (j = 0; j < kTrackLimit; j++) {
    if (rates[j] > maxRate) {
      maxRate = rates[j];
      maxIdx = j;
    }
  }
  if (maxRate < 3) return false;
  l_shape->p1.location = MovePixel(p1, l_shape->angle1, maxIdx + 1, -1);
  angleX1 = goodAngles[maxIdx];

  // p2
  for (j = 0; j < kTrackLimit; j++) {
    rates[j] = -1;
    goodAngles[j] = -1.0;
  }
  maxRate = 0;
  p2 = MovePixel(p2, l_shape->angle2, padding, +1);
  for (j = 0; j < kTrackLimit; j++) {
    // extend out
    pTemp = MovePixel(p2, l_shape->angle2, j, -1);
    pTemp1 = MovePixel(p2, l_shape->angle2, j + 2, -1);
    for (rotate = kRotateLimit; rotate >= -kRotateLimit; rotate--) {
      newAngle = l_shape->angle2 + 90.0 + (double)rotate;
      L = (int)floor(L1 / cos(CV_PI * rotate / 180.0) + 0.5);
      double rate = GetBrightRateInALine(image, pTemp1, newAngle, L, -1);
      if (rate < kBrightRate) {
        goodAngles[j] = newAngle;
        rates[j] = GetDashNumberBright(image, pTemp, newAngle, L, -1);
        break;
      }
    }
  }
  for (j = 0; j < kTrackLimit; j++) {
    if (rates[j] > maxRate) {
      maxRate = rates[j];
      maxIdx = j;
    }
  }
  if (maxRate < 3) return false;
  l_shape->p2.location = MovePixel(p2, l_shape->angle2, maxIdx + 1, -1);
  angleX2 = goodAngles[maxIdx];



  // get px
  LShape x_shape;
  x_shape.p1.location = l_shape->p2.location;
  x_shape.p2.location = l_shape->p1.location;
  x_shape.angle1 = angleX2;
  x_shape.angle2 = angleX1;
  CalibrateP0(&x_shape);
  l_shape->px = x_shape.p0;

  return true;
}

void DatamatrixLocator::PaddingLShape(const Mat& image, const bool padding_back,
                                      LShape* l_shape) {
  /* padding until the num of bright dots outside L shape is larger( >60% )*/
  const double kMinBrightRate = 0.6;
  const int kTryTimes = 5;
  Point p0 = l_shape->p0.location;
  Point p1 = l_shape->p1.location;
  Point p2 = l_shape->p2.location;
  const int kLenght1 = (int)floor(GetDistance(p0, p1) + 0.5);
  const int kLenght2 = (int)floor(GetDistance(p0, p2) + 0.5);

  int i;
  double angle90_p1 = l_shape->angle1 + 90.0;
  double angle90_p2 = l_shape->angle2 - 90.0;
  Point track;

  for (i = 0; i < kTryTimes; i++) {
    double rate =
        GetBrightRateInALine(image, p1, l_shape->angle1, kLenght1 + i, +1);
    if (rate >= kMinBrightRate)break;
    else
      p1 = MovePixel(p1, angle90_p1, 1, +1);
  }
  if (padding_back) p1 = MovePixel(p1, angle90_p1, 1, -1);
  for (i = 0; i < kTryTimes; i++) {
    double rate =
        GetBrightRateInALine(image, p2, l_shape->angle2, kLenght2 + i, +1);
    if (rate >= kMinBrightRate)break;
    else
      p2 = MovePixel(p2, angle90_p2, 1, +1);
  }
  if (padding_back) p2 = MovePixel(p2, angle90_p2, 1, -1);

  l_shape->p1.location = p1;
  l_shape->p2.location = p2;
  CalibrateP0(l_shape);
}

bool DatamatrixLocator::EnlargeLShape(LShape* l_shape) {
  const int kSize = 2;
  Point corners[4];
  corners[0] = l_shape->p1.location;
  corners[1] = l_shape->p0.location;
  corners[2] = l_shape->p2.location;
  corners[3] = l_shape->px.location;

  switch (l_shape->position) {
    case 0:
      corners[0].x += kSize;
      corners[0].y -= kSize;
      corners[1].x -= kSize;
      corners[1].y -= kSize;
      corners[2].x -= kSize;
      corners[2].y += kSize;
      corners[3].x += kSize;
      corners[3].y += kSize;
      break;
    case 1:
      corners[0].x -= kSize;
      corners[0].y -= kSize;
      corners[1].x -= kSize;
      corners[1].y += kSize;
      corners[2].x += kSize;
      corners[2].y += kSize;
      corners[3].x += kSize;
      corners[3].y -= kSize;
      break;
    case 2:
      corners[0].x -= kSize;
      corners[0].y += kSize;
      corners[1].x += kSize;
      corners[1].y += kSize;
      corners[2].x += kSize;
      corners[2].y -= kSize;
      corners[3].x -= kSize;
      corners[3].y -= kSize;
      break;
    case 3:
      corners[0].x += kSize;
      corners[0].y += kSize;
      corners[1].x += kSize;
      corners[1].y -= kSize;
      corners[2].x -= kSize;
      corners[2].y -= kSize;
      corners[3].x -= kSize;
      corners[3].y += kSize;
      break;
  }
  for (int i = 0; i < 4; i++) {
    if (corners[i].x < 0 || corners[i].x >= image_.cols) return false;
    if (corners[i].y < 0 || corners[i].y >= image_.rows) return false;
  }
  l_shape->p1.location = corners[0];
  l_shape->p0.location = corners[1];
  l_shape->p2.location = corners[2];
  l_shape->px.location = corners[3];
  return true;
}

double DatamatrixLocator::Transform4LShape(const Mat& src, const LShape& l_shape,
                                         Mat* transformed, double w_h) {
  double length;
  Point lshape_vertex[] = {l_shape.p1.location, l_shape.p0.location,
                           l_shape.p2.location, l_shape.px.location};
  if (w_h == -1.0) {
    for (int i = 0; i < 3; i++) {
      length = GetDistance(lshape_vertex[i], lshape_vertex[i + 1]);
      if (length > w_h) w_h = length;
    }
    length = GetDistance(lshape_vertex[3], lshape_vertex[0]);
    if (length > w_h) w_h = length;
  }
  Transform(src, lshape_vertex, w_h, transformed);
  return w_h;
}

void DatamatrixLocator::Transform(const Mat& src, const Point* vertex,
                                  const double w_h, Mat* transformed ) {
  Point2f src_pts[4];
  Point2f trans_pts[4];
  for (int i = 0; i < 4; i++) {
    src_pts[i] = Point2f((float)vertex[i].x, (float)vertex[i].y);
    trans_pts[i] = Point2f(0.0, 0.0);
  }
  trans_pts[1].y += (float)w_h;
  trans_pts[2].x += (float)w_h;
  trans_pts[2].y += (float)w_h;
  trans_pts[3].x += (float)w_h;

  Mat m = getPerspectiveTransform(src_pts, trans_pts);
  warpPerspective(src, *transformed, m, transformed->size());
  m.release();
}

}  // namespace hyf_lemon
