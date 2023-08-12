/*******************************************************************************

  @file      lemon_api.cpp
  @brief
  @details   ~
  @author    cheng-ran@outlook.com
  @date      10.08.2021
  @copyright HengYiFeng, 2021-2023. All right reserved.

*******************************************************************************/
//#define DEBUG_MAIN

#include "lemon_api.h"

#include <iostream>

using std::cout;
using std::endl;
using std::vector;
using namespace cv;

namespace hyf_lemon {

bool Decode(const Mat& image, vector<vector<uchar>>* output) { 
  Mat src(image.size(), CV_8UC1);
  cvtColor(image, src, COLOR_BGR2GRAY);
  lemon.SetImage(src);
  return lemon.Decode(output);
}

bool Decode_file(const char* file, vector<vector<uchar>>* output) {
  Mat src = imread(file, IMREAD_GRAYSCALE);
  lemon.SetImage(src);
  return lemon.Decode(output);
}

bool Decode_rt(const int width, const int height, const uchar* image_data,
               vector<vector<uchar>>* output) {
  Mat image(Size(width, height), CV_8UC3);
  image.data = (uchar*)image_data;
  if (image.empty()) return false;
  flip(image, image, 1);
  cvtColor(image, image, COLOR_BGR2GRAY);
  lemon.SetImage(image);
  return lemon.Decode(output);
}

/****************************************************************************
 *                                   class                                   *
 ****************************************************************************/

Lemon::Lemon() {}
Lemon::~Lemon() { image_.release(); }

void Lemon::SetImage(const Mat& image) {
  processor_.set_image(image);
  image_ = image;
}
void Lemon::SetReversed(const bool reversed) {
  processor_.set_bin_reversed(reversed);
}
void Lemon::SetBinMethod(const BinMethod method) {
  processor_.set_bin_method(method);
}
void Lemon::SetBinNormalTh(const unsigned val) {
  processor_.set_bin_normal_th(val);
}
void Lemon::SetBinAdaptiveBlock(const unsigned val) {
  processor_.set_bin_adaptive_block(val);
}

bool Lemon::Decode(vector<vector<uchar>>* output) {

#ifdef DEBUG_MAIN
  double time_begin = getTickCount();
#endif  // DEBUG_MAIN

  bool flag_success = false;
  int n_takes = 0;
  while (!flag_success && n_takes < 4) {
    switch (n_takes++) {
      case 0:
        // default or last successful method
        break;
      case 1:
        SetReversed(true);
        break;
      case 2:
        SetReversed(false);
        SetBinAdaptiveBlock(35);
        break;
      case 3:
        SetReversed(true);
        SetBinMethod(BIN_NORMAL);
        break;
      default:
        break;
    }

#ifdef DEBUG_MAIN
    cout << ">>>  Take " << n_takes << endl;
#endif  // DEBUG_MAIN

    /* ****************************  step 1  *********************************/
    Mat binarized;
    vector<PointSeq> contours;
    processor_.Process(&binarized, &contours);
    if (contours.size() < 1) {
#ifdef DEBUG_MAIN
      cout << "Step 1 - Image Process: No possible contours found." << endl;
#endif  // DEBUG_MAIN

      binarized.release();
      vector<PointSeq>().swap(contours);
      continue;
    }
#ifdef DEBUG_MAIN
    cout << "Step 1 - Image Process: " << contours.size() << " possible contours found."
         << endl;
#endif  // DEBUG_MAIN

    /* ****************************  step 2  *********************************/
    locator_.set_image(binarized);
    locator_.set_contours(contours);
    MatVec datamatrixs;
    int count = locator_.LocateDatamatrix(image(), processor_, &datamatrixs);
    if (count < 1) {
#ifdef DEBUG_MAIN
      cout << "Step 2 - Datamatrix Locator: No possible Datamatrix found." << endl;
#endif  // DEBUG_MAIN

      binarized.release();
      vector<PointSeq>().swap(contours);
      MatVec().swap(datamatrixs);
      continue;
    }
#ifdef DEBUG_MAIN
    cout << "Step 2 - Datamatrix Locator: " << count << " possible Datamatrix found."
         << endl;
#endif  // DEBUG_MAIN

    /* ****************************  step 3  *********************************/
    for (Mat datamatrix : datamatrixs) {
      reader_.set_image(datamatrix);
      // read
      vector<int> codes;
      int size_hori = reader_.Read(processor_, &codes);
      int size_vert = codes.size() / size_hori;
      if (size_hori < 8 || size_vert < 8) continue;
      if (size_hori % 2 == 1 || size_vert % 2 == 1) continue;

#ifdef DEBUG_MAIN
      cout << "Step 3 - Datamatrix Reader: " << endl;
      for (int j = 0; j < size_vert; j++) {
        for (int i = 0; i < size_hori; i++) {
          int idx = size_hori * j + i;
          cout << codes[idx] << " ";
        }
        cout << endl;
      }
#endif

      // decode
      DatamatrixDecoder decoder(size_vert, size_hori, codes);
      vector<int> message;
      vector<uchar> text;
      if (decoder.decode(&message)) {
        flag_success = true;

#ifdef DEBUG_MAIN
        cout << "Step 4 - Decode Result: ";
#endif
        for (int i = 0; i < message.size(); i++) {
          uchar c = (uchar)message[i];
          text.push_back(c);

#ifdef DEBUG_MAIN
          cout << c;
#endif
        }
        output->push_back(text);
#ifdef DEBUG_MAIN
        cout << endl;
#endif
      }
      vector<int>().swap(codes);
      vector<int>().swap(message);
    }  // for(step 3)

    binarized.release();
    vector<PointSeq>().swap(contours);
    MatVec().swap(datamatrixs);

  }  // while

#ifdef DEBUG_MAIN
  double time_end = getTickCount();
  cout << "time spend: " << (time_end - time_begin) * 1000 / getTickFrequency()
       << "ms" << endl;
#endif  // DEBUG_MAIN

  return flag_success;
}

}  // namespace hyf_lemon