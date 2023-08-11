/*******************************************************************************

  @file      datamatrix_reader.cpp
  @brief     read the datamatrix binary codes from a image
  @details   ~
  @author    cheng-ran@outlook.com
  @date      9.07.2021
  @copyright HengYiFeng, 2021-2023. All right reserved.

*******************************************************************************/
 //#define DEBUG_DM_READER

#include "datamatrix_reader.h"

#include <iostream>

using std::cout;
using std::endl;
using std::vector;
using namespace cv;

namespace hyf_lemon {

DatamatrixReader::DatamatrixReader() {}
DatamatrixReader::DatamatrixReader(const Mat& source) { image_ = source; }
DatamatrixReader::~DatamatrixReader() {
  if (!image_.empty()) {
    image_.release();
  }
}

void DatamatrixReader::set_image(const Mat& source) {
  if (!image_.empty()) {
    image_.release();
  }
  image_ = source;
}

int DatamatrixReader::Read(const ImageProcessor& processor,
                           vector<int>* codes) {
  Mat binary = image_.clone();
  ImageProcessor p = processor;
  p.set_image(binary);
  vector<PointSeq> no_use;
  p.Process(&binary, &no_use);

  int image_w_h = image_.cols;

  int padding_down_count, padding_left_count;
  if (!PaddingDash(binary, &padding_down_count, &padding_left_count)) return -1;

  Rect roi(0, padding_down_count, image_w_h - padding_left_count,
           image_w_h - padding_down_count);
  Mat datamatrix_bin = binary(roi).clone();
  Mat datamatrix_orig = image_(roi).clone();
  int size_hori = -1, size_vert = -1;  // !!! datamatrix code size (m*n) !!!

  if (!GetCodeSize(datamatrix_bin, image_w_h, &size_hori, &size_vert)) {
    binary.release();
    datamatrix_orig.release();
    datamatrix_bin.release();
    return -1;
  }

  // set grid
  int* row_position = new int[size_vert + 1];
  int* col_position = new int[size_hori + 1];
  SetGrid(datamatrix_bin, size_vert, size_hori, row_position, col_position);
  // score
  double* scores = new double[size_hori * size_vert];
  double dark_avrage, bright_avrage;
  ScoreGrid(datamatrix_bin, datamatrix_orig, size_vert, size_hori, row_position,
            col_position, scores, &dark_avrage, &bright_avrage);

  // read code
  ReadCodes(processor, size_vert, size_hori, row_position, col_position,
            dark_avrage, bright_avrage, &datamatrix_bin, scores, codes);

#ifdef DEBUG_DM_READER
  printf("DataMatrix Reader: size_hori: %d, size_vert: %d\n", size_hori,
         size_vert);
#endif  // DEBUG_DM_READER

  binary.release();
  datamatrix_orig.release();
  datamatrix_bin.release();
  delete[] row_position;
  delete[] col_position;
  delete[] scores;
  return size_hori;
}

bool DatamatrixReader::PaddingDash(const Mat& binarized,
                                   int* padding_down_count,
                                   int* padding_left_count) {
  /* padding 2 image borders of dash sides until the num of bright dots rate
     is large( >20% )*/
  const double kMinBrightRate = 0.2;
  const int kTryTimes = 6;
  int w = image_.cols;
  int h = image_.rows;

  Point p0 = Point(w - 1, 0);
  Point p1 = Point(w - 1, h - 1);
  Point p2 = Point(0, 0);
  double angle1 = 270.0;
  double angle2 = 180.0;
  int length1 = h;
  int length2 = w;

  int i;
  double angle90_p1 = angle1 + 90.0;
  double angle90_p2 = angle2 - 90.0;
  Point track;

  for (i = 0; i < kTryTimes; i++) {
    double rate = GetBrightRateInALine(binarized, p1, angle1, length1, +1);
    if (rate >= kMinBrightRate)
      break;
    else
      p1 = MovePixel(p1, angle90_p1, 1, +1);
  }
  *padding_left_count = i;
  if (*padding_left_count == kTryTimes - 1) return false;

  for (i = 0; i < kTryTimes; i++) {
    double rate = GetBrightRateInALine(binarized, p2, angle2, length2, +1);
    if (rate >= kMinBrightRate)
      break;
    else
      p2 = MovePixel(p2, angle90_p2, 1, +1);
  }
  *padding_down_count = i;
  if (*padding_down_count == kTryTimes - 1) return false;

  return true;
}

bool DatamatrixReader::GetCodeSize(const Mat& datamatrix, const int image_w_h,
                                   int* size_hori, int* size_vert) {
  const int kTryTimes = 6;
  int size_x, size_y, max_size = -1;
  int j;
  int maxIdxX, maxIdxY;
  for (j = 0; j < kTryTimes; ++j) {  // horizontal
    Point p = Point(0, j);
    size_x = GetDashNumber(datamatrix, p, 0.0, image_w_h);
    if (size_x >= max_size) {
      max_size = size_x;
      maxIdxX = j;
    }
  }
  size_x = max_size;
  if (size_x < 10) return false;  // min count of elements is 10

  max_size = -1;
  for (j = 0; j < kTryTimes; ++j) {  // vertical
    Point p = Point(image_w_h - j - 1, image_w_h - 1);
    size_y = GetDashNumber(datamatrix, p, 90.0, image_w_h);
    if (size_y >= max_size) {
      max_size = size_y;
      maxIdxY = j;
    }
  }
  size_y = max_size;
  if (size_y < 8) return false;  // min count of elements is 8

  *size_hori = size_x;
  *size_vert = size_y;

  return true;
}

int DatamatrixReader::GetDashNumber(const Mat& datamatrix, const Point p,
                                    const double angle, const int length,
                                    int direction) {
  const int kMinIsland = 1;
  const double kMin2MaxRate = 0.3;
  Point track;
  vector<int> bright_island;
  vector<int> dark_island;
  bool is_bright = false;
  int position_bright = -1;
  int position_dark = -1;
  int n_bright = 0, n_dark = 0;

  // check each point
  for (int i = 0; i < length; i++) {
    track = MovePixel(p, angle, i, direction);
    if (!is_bright) {
      if (length - 1 == i)
        dark_island.push_back(i - position_dark + 1);
      else if (GetPixValue8UC1(datamatrix, track) == 255) {
        is_bright = true;
        position_bright = i;
        if (position_dark != -1) dark_island.push_back(i - position_dark);
      }
    }
    if (is_bright) {
      if (i == length - 1)
        bright_island.push_back(i - position_bright + 1);
      else if (GetPixValue8UC1(datamatrix, track) == 0) {
        is_bright = false;
        bright_island.push_back(i - position_bright);
        position_dark = i;
      }
    }
  }

  // count and ignore little island
  int min = 10000, max = 0;
  int current, j;
  for (j = 0; j < bright_island.size(); j++) {
    current = bright_island[j];
    if (current > kMinIsland) {
      n_bright++;
      if (current < min) min = current;
      if (current > max) max = current;
    }
  }
  bright_island.clear();
  vector<int>().swap(bright_island);
  if (double(min) / max < kMin2MaxRate) {
    dark_island.clear();
    vector<int>().swap(dark_island);
    return -1;
  }
  //
  min = 10000, max = 0;
  for (j = 0; j < dark_island.size(); j++) {
    current = dark_island[j];
    if (current > kMinIsland) {
      n_dark++;
      if (current < min) min = current;
      if (current > max) max = current;
    }
  }
  dark_island.clear();
  vector<int>().swap(dark_island);
  if (double(min) / max < kMin2MaxRate) return -1;
  // printf("bright+dark:%d,%d\n", n_bright, n_dark);
  if (n_bright == n_dark || n_bright - n_dark == 1) return n_dark + n_dark;

  return -1;
}

void DatamatrixReader::SetGrid(const Mat& datamatrix, const int size_vert,
                               const int size_hori, int* row_position,
                               int* col_position) {
  // paint contours
  Mat img_contours = Mat::zeros(datamatrix.size(), CV_8UC1);
  vector<PointSeq> contours;
  findContours(datamatrix, contours, RETR_LIST, CHAIN_APPROX_NONE, Point(0, 0));
  for (size_contour i = 0; i < (size_contour)contours.size(); i++) {
    drawContours(img_contours, contours, (int)i, Scalar(255, 255, 255), 1);
  }
  vector<PointSeq>().swap(contours);

  // calculate every row & col
  double block_hori = (double)datamatrix.cols / size_hori;
  double block_vert = (double)datamatrix.rows / size_vert;

  row_position[0] = col_position[0] = 0;
  row_position[size_vert] = datamatrix.rows - 1;
  col_position[size_hori] = datamatrix.cols - 1;

  int j;
  for (j = 0; j < size_vert; j++) {
    int y = (int)floor(block_vert * j + 0.5f);
    y = FitRow(img_contours, y);
    row_position[j] = y;
  }
  for (j = 0; j < size_hori; j++) {
    int x = (int)floor(block_hori * j + 0.5f);
    x = FitCol(img_contours, x);
    col_position[j] = x;
  }
  img_contours.release();
}

int DatamatrixReader::FitRow(const Mat& img_contours, int y) {
  int x0, x1, y0;
  int n_bright;
  Point p;
  int max = 0, max_inx = -1;

  x0 = 0;
  x1 = img_contours.cols - 1;
  for (int i = 0; i < 5; i++) {
    n_bright = 0;
    y0 = y - 2 + i;
    for (int j = x0; j <= x1; j++) {
      p.x = j;
      p.y = y0;
      if (GetPixValue8UC1(img_contours, p) == 255) n_bright++;
    }
    if (n_bright > max) {
      max = n_bright;
      max_inx = i;
    }
  }

  if (max_inx != -1) y = y - 2 + max_inx;

  return y;
}

int DatamatrixReader::FitCol(const Mat& img_contours, int x) {
  int y0, y1, x0;
  int n_bright;
  Point p;
  int max = 0, max_idx = -1;

  y0 = 0;
  y1 = img_contours.rows - 1;
  for (int i = 0; i < 5; i++) {
    n_bright = 0;
    x0 = x - 2 + i;
    for (int j = y0; j <= y1; j++) {
      p.x = x0;
      p.y = j;
      if (GetPixValue8UC1(img_contours, p) == 255) n_bright++;
    }
    if (n_bright > max) {
      max = n_bright;
      max_idx = i;
    }
  }

  if (max_idx != -1) x = x - 2 + max_idx;

  return x;
}

void DatamatrixReader::ScoreGrid(const Mat& datamatrix_bin,
                                 const Mat& datamatrix_orig,
                                 const int size_vert, const int size_hori,
                                 const int* row_position,
                                 const int* col_position, double* scores,
                                 double* dark_avrage, double* bright_avrage) {
  const double kGate1 = 0.25, kGate2 = 0.75;
  double dark_avr = 0.0, bright_avr = 0.0;
  int n_dark = 0, n_bright = 0;
  double* averages = new double[size_hori * size_vert];

  // set dash line
  bool odd = false;
  int i, j;
  for (i = 0; i < size_hori; i++) {
    if (odd)
      scores[i] = 0.0;
    else
      scores[i] = 1.0;
    odd = !odd;
    scores[(size_vert - 1) * size_hori + i] = 1.0;
  }
  odd = false;
  for (i = size_vert; i > 0; i--) {
    if (odd)
      scores[i * size_hori - 1] = 0.0;
    else
      scores[i * size_hori - 1] = 1.0;
    odd = !odd;
    scores[(i - 1) * size_hori] = 1.0;
  }

  for (j = 0; j < size_vert; j++) {
    int y0 = row_position[j];
    int y1 = row_position[j + 1];
    for (int i = 0; i < size_hori; i++) {
      int idx = size_hori * j + i;
      int x0 = col_position[i];
      int x1 = col_position[i + 1];
      double score = GetScore(datamatrix_bin, x0, y0, x1, y1);
      scores[idx] = score;
      double average = GetAverage(datamatrix_orig, x0, y0, x1, y1);
      averages[idx] = average;

      if (score <= kGate1) {
        scores[idx] = 0.0;
        dark_avr += average;
        n_dark++;

      } else if (score >= kGate2) {
        scores[idx] = 1.0;
        bright_avr += average;
        n_bright++;
      }
      // printf("%1.2f ", score);
    }
    // printf("\n");
  }
  *dark_avrage = round(dark_avr / n_dark);
  *bright_avrage = round(bright_avr / n_bright);
  delete[] averages;
}

double DatamatrixReader::GetScore(const Mat& src, int x0, int y0, int x1,
                                  int y1) {
  int n_bright = 0;
  int n_total = 0;
  int value;

  for (int j = y0 + 1; j < y1; j++) {
    for (int i = x0 + 1; i < x1; i++) {
      value = GetPixValue8UC1(src, Point(i, j));
      if (value > 0) n_bright++;
      n_total++;
    }
  }
  return (double)n_bright / n_total;
}

double DatamatrixReader::GetCenterScore(const Mat& src, int x0, int y0, int x1,
                                        int y1) {
  int brightNum = 0;
  int totalNum = 0;
  int value;
  int xBegin, xEnd;
  int yBegin, yEnd;

  xEnd = (x1 + x0) / 2 + 1;
  if ((x1 - x0) % 2 == 0)  // enven number
    xBegin = (x1 + x0) / 2 - 1;
  else
    xBegin = (x1 + x0) / 2;
  yEnd = (y1 + y0) / 2 + 1;
  if ((y1 - y0) % 2 == 0)
    yBegin = (y1 + y0) / 2 - 1;
  else
    yBegin = (y1 + y0) / 2;

  for (int j = yBegin; j <= yEnd; j++) {
    for (int i = xBegin; i <= xEnd; i++) {
      value = GetPixValue8UC1(src, Point(i, j));
      if (value > 0) brightNum++;
      totalNum++;
    }
  }
  return (double)brightNum / totalNum;
}

double DatamatrixReader::GetAverage(const Mat& src, int x0, int y0, int x1,
                                    int y1) {
  int total_value = 0;
  int n_total = 0;
  int value;

  for (int j = y0 + 1; j < y1; j++) {
    for (int i = x0 + 1; i < x1; i++) {
      value = GetPixValue8UC1(src, Point(i, j));
      total_value += value;
      n_total++;
    }
  }
  return (double)total_value / n_total;
}

void DatamatrixReader::ReadCodes(const ImageProcessor& processor,
                                 const int size_vert, const int size_hori,
                                 const int* row_position,
                                 const int* col_position,
                                 const double dark_avrage,
                                 const double bright_avrage, Mat* datamatrix,
                                 double* scores, vector<int>* codes) {
  /*
   */
  const double kGate1 = 0.25, kGate2 = 0.75, kGate3 = 0.66;

  int j;
  for (j = 0; j < size_vert; j++) {
    for (int i = 0; i < size_hori; i++) {
      int idx = size_hori * j + i;
      double score = scores[idx];
      if (score >= kGate2)
        PaintGrid(row_position, col_position, i, j, bright_avrage, datamatrix);
      else if (score <= kGate1)
        PaintGrid(row_position, col_position, i, j, dark_avrage, datamatrix);
    }
  }

  ImageProcessor p = processor;
  p.set_image(*datamatrix);
  p.set_bin_reversed(true);
  vector<PointSeq> no_use;
  p.Process(datamatrix, &no_use);

  // get center score for each grid those
  for (j = 0; j < size_vert; j++) {
    int y0 = row_position[j];
    int y1 = row_position[j + 1];
    for (int i = 0; i < size_hori; i++) {
      int idx = size_hori * j + i;
      int x0 = col_position[i];
      int x1 = col_position[i + 1];
      if (scores[idx] > kGate1 && scores[idx] < kGate2) {
        double score = GetCenterScore(*datamatrix, x0, y0, x1, y1);
        if (score > kGate3)
          scores[idx] = 1.0;
        else
          scores[idx] = 0.0;
      }
      codes->push_back((int)scores[idx]);
      // printf("%1.2f ", scores[idx]);
    }
    // printf("\n");
  }
}

void DatamatrixReader::PaintGrid(const int* rows, const int* cols, const int i,
                                 const int j, const double color, Mat* src) {
  int x0 = cols[i];
  int x1 = cols[i + 1];
  int y0 = rows[j];
  int y1 = rows[j + 1];

  int colour = (int)floor(color + 0.5);
  if (colour > 255) colour = 255;

  for (int m = x0; m <= x1; m++) {
    for (int n = y0; n <= y1; n++) src->data[n * src->cols + m] = colour;
  }
}

}  // namespace hyf_lemon