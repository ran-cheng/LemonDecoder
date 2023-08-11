/*******************************************************************************

  @file      lemon_decoder.cpp
  @brief     entry of the demo
  @details   ~
  @author    cheng-ran@outlook.com
  @date      7.08.2023
  @copyright HengYiFeng, 2021-2023. All right reserved.

*******************************************************************************/
#include <iostream>

#include "lemon_api.h"

using std::vector;

int main() {
  vector<vector<uchar>> message;

  // namespace hyf_lemon is used
  // return true if decoding is success
  // if success, output each datamatrix code in the image file via
  // vector<vector<uchar>> message
  if (hyf_lemon::Decode_file("1.jpg", &message)) {
    // TODO
  }
}
