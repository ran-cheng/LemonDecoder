/*******************************************************************************

  @file      datamatrix_decoder.h
  @brief     decode datamatrix codewords
  @details   ~
  @author    cheng-ran@outlook.com
  @date      10.08.2023
  @copyright HengYiFeng, 2021-2023. All right reserved.

*******************************************************************************/
#ifndef DATAMATRIX_DECODER_H_
#define DATAMATRIX_DECODER_H_

#include <opencv2/opencv.hpp>

namespace hyf_lemon {

class DatamatrixDecoder {
 public:
  DatamatrixDecoder(const int numRows, const int numCols,
                    const std::vector<int> &codes);
  ~DatamatrixDecoder();
  bool decode(std::vector<int> *message);

 private:
  int numRows;
  int numColumns;
  std::vector<int> codes;
  std::vector<bool> codesMirror;
  std::vector<int> words;
  int eccIndex;
  int dataNum;
  int correctorNum;
  int totalNum;

 private:
  //-------------------get codewords from matrix--------
  void mergeRegion(
      std::vector<int> codesTotal,
      std::vector<int> &codesUseful);  // lose all "L" & "Dash",merge all region
  void getWords();                     // matrix"codes" --> std::vector"words"
  bool isRead(int row, int column);
  bool readModule(int row, int column, int numRows, int numColumns);
  int readUtah(int row, int column, int numRows, int numColumns);
  int readCorner1(int numRows, int numColumns);
  int readCorner2(int numRows, int numColumns);
  int readCorner3(int numRows, int numColumns);
  int readCorner4(int numRows, int numColumns);

  //---------------------error correction(RS) of codewords--------
  int repair();  // if corrected, change std::vector"words"
  bool checkVector(std::vector<int> v, int value);

  //------------------convert codes to message--------
  bool getMessage(std::vector<int>* message);
  int getEncodeType(int codeword);
  int decodeAscii(int index, std::vector<int> &message);
  int decodeC40Text(int index, std::vector<int> &message, int encType);
  void pushC40Text(std::vector<int> &message, int value, bool upperShift);
  int decodeX12(int index, std::vector<int> &message);
  int decodeEdifact(int index, std::vector<int> &message);
  int decodeBase256(int index, std::vector<int> &message);
  int UnRandomize255State(int value, int n);
};

}  // namespace hyf_lemon

#endif  // DATAMATRIX_DECODER_H_ 
