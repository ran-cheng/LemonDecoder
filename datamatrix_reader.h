/*******************************************************************************

  @file      datamatrix_reader.h
  @brief     read the datamatrix binary codes from a image
  @details   ~
  @author    cheng-ran@outlook.com
  @date      9.07.2021
  @copyright HengYiFeng, 2021-2023. All right reserved.

*******************************************************************************/
#ifndef DATAMATRIX_READER_H_
#define DATAMATRIX_READER_H_

#include <opencv2/opencv.hpp>

#include "datamatrix_locator.h"

namespace hyf_lemon {

class DatamatrixReader {
 public:
  DatamatrixReader();
  DatamatrixReader(const cv::Mat& source);
  ~DatamatrixReader();

  // setter & getter
  cv::Mat image() const { return image_; }
  void set_image(const cv::Mat& source);
  /**
   * @brief main method of DatamatrixReader, read binary code from image
   * @param code - output
   * @return size_hori (if fail, return -1)
  */
  int Read(const ImageProcessor& processor, std::vector<int>* code);

 private:
  /**
   * @brief push inside 2 borders(dash side) -> trim them a little bit
   * @param padding_down_count - output
   * @param padding_left_count - output
   * @return true - if success
  */
  bool PaddingDash(const cv::Mat& binarized,int* padding_down_count, int* padding_left_count);
  /**
   * @brief get number of size, size_hori*size_vert
   * @param datamatrix - input
   * @param size_hori - output
   * @param size_vert - output
   * @return true - if success
   */
  bool GetCodeSize(const cv::Mat& datamatrix, const int image_w_h,
                   int* size_hori, int* size_vert);
  int GetDashNumber(const cv::Mat& datamatrix, const cv::Point p,
                    const double angle, const int length, int direction = -1);
  /**
   * @brief set the grid of datamatrix, grid is orthogonal(horizontal/vertical)
   * @param datamatrix 
   * @param row_position - output horizontal lines of the grid
   * @param col_position - output vertival lines of the grid
  */
  void SetGrid(const cv::Mat& datamatrix, const int size_vert, const int size_hori,
               int* row_position, int* col_position); 
  int FitRow(const cv::Mat& img_contours, int y);
  int FitCol(const cv::Mat& img_contours, int x);
  void ScoreGrid(const cv::Mat& datamatrix_bin, const cv::Mat& datamatrix_orig,
                 const int size_vert, const int size_hori,
                 const int* row_position, const int* col_position,
                 double* scores, double* dark_avrage, double* bright_avrage);

  double GetScore(const cv::Mat& src, int x0, int y0, int x1, int y1); 
  double GetCenterScore(const cv::Mat& src, int x0, int y0, int x1, int y1);
  double GetAverage(const cv::Mat& src, int x0, int y0, int x1, int y1);

  void ReadCodes(const ImageProcessor& processor, const int size_vert,
                 const int size_hori, const int* row_position,
                 const int* col_position, const double dark_avrage,
                 const double bright_avrage, cv::Mat* datamatrix, double* scores,
                 std::vector<int>* codes);
  void PaintGrid(const int* rows, const int* cols, const int i, const int j,
                 const double color, cv::Mat* src);


  cv::Mat image_;
};

}  // namespace hyf_lemon

#endif  // DATAMATRIX_READER_H_
