/*******************************************************************************

  @file      datamatrix_decoder.cpp
  @brief     decode datamatrix codewords
  @details   ~
  @author    cheng-ran@outlook.com
  @date      10.08.2023
  @copyright HengYiFeng, 2021-2023. All right reserved.

*******************************************************************************/
#include "datamatrix_decoder.h"

using std::vector;

namespace hyf_lemon {

/****************************************************************************
 *                                  utility                                 *
 ****************************************************************************/

#define nn 255
#define NOERROR 1
#define CANT_REPAIR 0
#define REPAIR_OK 2
// p(x)=x^8+x^5+x^3+x^2+1
int expOf[] = {
    255, 0,   1,   240, 2,   225, 241, 53,  3,   38,  226, 133, 242, 43,  54,
    210, 4,   195, 39,  114, 227, 106, 134, 28,  243, 140, 44,  23,  55,  118,
    211, 234, 5,   219, 196, 96,  40,  222, 115, 103, 228, 78,  107, 125, 135,
    8,   29,  162, 244, 186, 141, 180, 45,  99,  24,  49,  56,  13,  119, 153,
    212, 199, 235, 91,  6,   76,  220, 217, 197, 11,  97,  184, 41,  36,  223,
    253, 116, 138, 104, 193, 229, 86,  79,  171, 108, 165, 126, 145, 136, 34,
    9,   74,  30,  32,  163, 84,  245, 173, 187, 204, 142, 81,  181, 190, 46,
    88,  100, 159, 25,  231, 50,  207, 57,  147, 14,  67,  120, 128, 154, 248,
    213, 167, 200, 63,  236, 110, 92,  176, 7,   161, 77,  124, 221, 102, 218,
    95,  198, 90,  12,  152, 98,  48,  185, 179, 42,  209, 37,  132, 224, 52,
    254, 239, 117, 233, 139, 22,  105, 27,  194, 113, 230, 206, 87,  158, 80,
    189, 172, 203, 109, 175, 166, 62,  127, 247, 146, 66,  137, 192, 35,  252,
    10,  183, 75,  216, 31,  83,  33,  73,  164, 144, 85,  170, 246, 65,  174,
    61,  188, 202, 205, 157, 143, 169, 82,  72,  182, 215, 191, 251, 47,  178,
    89,  151, 101, 94,  160, 123, 26,  112, 232, 21,  51,  238, 208, 131, 58,
    69,  148, 18,  15,  16,  68,  17,  121, 149, 129, 19,  155, 59,  249, 70,
    214, 250, 168, 71,  201, 156, 64,  60,  237, 130, 111, 20,  93,  122, 177,
    150};
int alphaTo[] = {
    1,   2,   4,   8,   16,  32,  64,  128, 45,  90,  180, 69,  138, 57,  114,
    228, 229, 231, 227, 235, 251, 219, 155, 27,  54,  108, 216, 157, 23,  46,
    92,  184, 93,  186, 89,  178, 73,  146, 9,   18,  36,  72,  144, 13,  26,
    52,  104, 208, 141, 55,  110, 220, 149, 7,   14,  28,  56,  112, 224, 237,
    247, 195, 171, 123, 246, 193, 175, 115, 230, 225, 239, 243, 203, 187, 91,
    182, 65,  130, 41,  82,  164, 101, 202, 185, 95,  190, 81,  162, 105, 210,
    137, 63,  126, 252, 213, 135, 35,  70,  140, 53,  106, 212, 133, 39,  78,
    156, 21,  42,  84,  168, 125, 250, 217, 159, 19,  38,  76,  152, 29,  58,
    116, 232, 253, 215, 131, 43,  86,  172, 117, 234, 249, 223, 147, 11,  22,
    44,  88,  176, 77,  154, 25,  50,  100, 200, 189, 87,  174, 113, 226, 233,
    255, 211, 139, 59,  118, 236, 245, 199, 163, 107, 214, 129, 47,  94,  188,
    85,  170, 121, 242, 201, 191, 83,  166, 97,  194, 169, 127, 254, 209, 143,
    51,  102, 204, 181, 71,  142, 49,  98,  196, 165, 103, 206, 177, 79,  158,
    17,  34,  68,  136, 61,  122, 244, 197, 167, 99,  198, 161, 111, 222, 145,
    15,  30,  60,  120, 240, 205, 183, 67,  134, 33,  66,  132, 37,  74,  148,
    5,   10,  20,  40,  80,  160, 109, 218, 153, 31,  62,  124, 248, 221, 151,
    3,   6,   12,  24,  48,  96,  192, 173, 119, 238, 241, 207, 179, 75,  150,
    0};
// a+b
int GfAdd(int a, int b) { return a ^ b; }
// a * b
int GfMult(int a, int b) {
  return (a == 0 || b == 0) ? 0 : alphaTo[(expOf[a] + expOf[b]) % nn];
}
// a * alpha^b
int GfMult2(int a, int b) { return a == 0 ? 0 : alphaTo[(expOf[a] + b) % nn]; }
// a/b
int GfDiv(int a, int b) {
  if (a == 0) return 0;
  if (a == b) return 1;
  return expOf[a] > expOf[b] ? alphaTo[expOf[a] - expOf[b]]
                             : alphaTo[nn + expOf[a] - expOf[b]];
}
// a/alpha^b
int GfDiv2(int a, int b) {
  if (a == 0) return 0;
  if (b == 0) return a;
  return expOf[a] >= b ? alphaTo[expOf[a] - b]
                       : alphaTo[nn + (expOf[a] - b) % nn];
}
// resolve equation set
void gaussion(int *polys, int *sums, int size) {
  int i, j, k, d;
  for (i = 0; i < size; i++) {
    // diagonal postion
    d = i * size + i;
    int diagonal = polys[d];
    // diagonal --> 1
    int index = i * size;
    for (j = 0; j < size; j++)
      polys[index + j] = GfDiv(polys[index + j], diagonal);
    sums[i] = GfDiv(sums[i], diagonal);
    // otherrows - thisrow
    for (k = 0; k < size; k++) {
      if (k != i) {  // another row
        int index2 = k * size;
        int coefficient = polys[index2 + i];
        // each column( polynomial[k] = polynomial[k] -
        // polynomial[i]*coefficient )
        for (int m = 0; m < size; m++) {
          polys[index2 + m] =
              GfAdd(polys[index2 + m], GfMult(coefficient, polys[index + m]));
        }
        sums[k] = GfAdd(sums[k], GfMult(coefficient, sums[i]));
      }
    }
  }
}
// from ECC200 rules table
static const int totalRows[] = {10,  12,  14,  16,  18, 20, 22, 24, 26, 32,
                                36,  40,  44,  48,  52, 64, 72, 80, 88, 96,
                                104, 120, 132, 144, 8,  8,  12, 12, 16, 16};
static const int totalCols[] = {10,  12,  14,  16,  18, 20, 22, 24, 26, 32,
                                36,  40,  44,  48,  52, 64, 72, 80, 88, 96,
                                104, 120, 132, 144, 18, 32, 26, 36, 36, 48};
static const int numRegionRows[] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 2,
                                    2, 2, 2, 2, 2, 4, 4, 4, 4, 4,
                                    4, 6, 6, 6, 1, 1, 1, 1, 1, 1};
static const int numRegionCols[] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 2,
                                    2, 2, 2, 2, 2, 4, 4, 4, 4, 4,
                                    4, 6, 6, 6, 1, 2, 1, 2, 2, 2};
static const int dataRows[] = {8,  10, 12, 14, 16, 18, 20, 22, 24, 14,
                               16, 18, 20, 22, 24, 14, 16, 18, 20, 22,
                               24, 18, 20, 22, 6,  6,  10, 10, 14, 14};
static const int dataCols[] = {8,  10, 12, 14, 16, 18, 20, 22, 24, 14,
                               16, 18, 20, 22, 24, 14, 16, 18, 20, 22,
                               24, 18, 20, 22, 16, 14, 24, 16, 16, 22};
static const int dataWords[] = {
    3,   5,   8,   12,  18,  22,  30,   36,   44,   62, 86, 114, 144, 174, 204,
    280, 368, 456, 576, 696, 816, 1050, 1304, 1558, 5,  10, 16,  22,  32,  49};
static const int errorWords[] = {5,  7,  10, 12, 14, 18, 20, 24, 28, 36,
                                 42, 48, 56, 68, 42, 56, 36, 48, 56, 68,
                                 56, 68, 62, 62, 7,  11, 14, 18, 24, 28};


/****************************************************************************
 *                                   class                                   *
 ****************************************************************************/

DatamatrixDecoder::DatamatrixDecoder(const int numRows, const int numColumns,
                     const vector<int>& codesTotal) {
  this->numRows = numRows;
  this->numColumns = numColumns;
  // ECC info
  int i;
  for (i = 0; i < sizeof(totalRows) / sizeof(int); i++) {
    if (totalRows[i] == numRows && totalCols[i] == numColumns) {
      this->eccIndex = i;
      break;
    }
  }

  mergeRegion(codesTotal, codes);
  for (i = 0; i < codes.size(); i++) this->codesMirror.push_back(false);

  this->dataNum = dataWords[this->eccIndex];
  this->correctorNum = errorWords[this->eccIndex];
  this->totalNum = this->dataNum + this->correctorNum;
}

void DatamatrixDecoder::mergeRegion(vector<int> codesTotal, vector<int> &codesUseful) {
  int numRegionRow = numRegionRows[this->eccIndex];
  int numRegionCol = numRegionCols[this->eccIndex];
  int numDataRow = dataRows[this->eccIndex];
  int numDataCol = dataCols[this->eccIndex];

  int idxR = 0, idxC = 0;
  for (int i = 1; i < this->numRows - 1; i++) {
    idxR++;
    if (i == numRows - 2) {
      idxR = 0;
    } else {
      if (idxR == numDataRow + 1) continue;
      if (idxR == numDataRow + 2) {
        idxR = 0;
        continue;
      }
    }

    for (int j = 1; j < this->numColumns - 1; j++) {
      idxC++;
      if (j == numColumns - 2) {
        idxC = 0;
      } else {
        if (idxC == numDataCol + 1) continue;
        if (idxC == numDataCol + 2) {
          idxC = 0;
          continue;
        }
      }

      int idx = numColumns * i + j;
      codesUseful.push_back(codesTotal[idx]);
    }
  }

  this->numRows = this->numRows - numRegionRow * 2;
  this->numColumns = this->numColumns - numRegionCol * 2;
}

DatamatrixDecoder::~DatamatrixDecoder(void) {
  codes.clear();
  codesMirror.clear();
  words.clear();
  vector<int>().swap(codes);
  vector<bool>().swap(codesMirror);
  vector<int>().swap(words);
}

bool DatamatrixDecoder::decode(vector<int>* message) {
  getWords();
  if (repair() == CANT_REPAIR) return false;
  return getMessage(message);
}


/****************************************************************************
 *                       get codewords frommatrix                           *
 ****************************************************************************/

void DatamatrixDecoder::getWords() {
  int resultOffset = 0;
  int row = 4;
  int column = 0;
  // printf("numRows:%d*%d", numRows, numColumns);

  bool corner1Read = false;
  bool corner2Read = false;
  bool corner3Read = false;
  bool corner4Read = false;

  // Read all of the codewords
  do {
    // Check the four corner cases
    if ((row == numRows) && (column == 0) && !corner1Read) {
      words.push_back(readCorner1(numRows, numColumns));
      row -= 2;
      column += 2;
      corner1Read = true;
    } else if ((row == numRows - 2) && (column == 0) &&
               ((numColumns & 0x03) != 0) && !corner2Read) {
      words.push_back(readCorner2(numRows, numColumns));
      row -= 2;
      column += 2;
      corner2Read = true;
    } else if ((row == numRows + 4) && (column == 2) &&
               ((numColumns & 0x07) == 0) && !corner3Read) {
      words.push_back(readCorner3(numRows, numColumns));
      row -= 2;
      column += 2;
      corner3Read = true;
    } else if ((row == numRows - 2) && (column == 0) &&
               ((numColumns & 0x07) == 4) && !corner4Read) {
      words.push_back(readCorner4(numRows, numColumns));
      row -= 2;
      column += 2;
      corner4Read = true;
    } else {
      // Sweep upward diagonally to the right
      do {
        if ((row < numRows) && (column >= 0) && !isRead(row, column)) {
          words.push_back(readUtah(row, column, numRows, numColumns));
        }
        row -= 2;
        column += 2;
      } while ((row >= 0) && (column < numColumns));
      row += 1;
      column += 3;

      // Sweep downward diagonally to the left
      do {
        if ((row >= 0) && (column < numColumns) && !isRead(row, column)) {
          words.push_back(readUtah(row, column, numRows, numColumns));
        }
        row += 2;
        column -= 2;
      } while ((row < numRows) && (column >= 0));
      row += 3;
      column += 1;
    }
  } while ((row < numRows) || (column < numColumns));
}

bool DatamatrixDecoder::isRead(int row, int column) {
  int idx = numColumns * row + column;
  return codesMirror[idx];
}

bool DatamatrixDecoder::readModule(int row, int column, int numRows, int numColumns) {
  // Adjust the row and column indices based on boundary wrapping
  if (row < 0) {
    row += numRows;
    column += 4 - ((numRows + 4) & 0x07);
  }
  if (column < 0) {
    column += numColumns;
    row += 4 - ((numColumns + 4) & 0x07);
  }
  int idx = numColumns * row + column;
  codesMirror[idx] = true;
  return codes[idx] == 1 ? true : false;
}

int DatamatrixDecoder::readUtah(int row, int column, int numRows, int numColumns) {
  int currentByte = 0;
  if (readModule(row - 2, column - 2, numRows, numColumns)) {
    currentByte |= 1;
  }
  currentByte <<= 1;
  if (readModule(row - 2, column - 1, numRows, numColumns)) {
    currentByte |= 1;
  }
  currentByte <<= 1;
  if (readModule(row - 1, column - 2, numRows, numColumns)) {
    currentByte |= 1;
  }
  currentByte <<= 1;
  if (readModule(row - 1, column - 1, numRows, numColumns)) {
    currentByte |= 1;
  }
  currentByte <<= 1;
  if (readModule(row - 1, column, numRows, numColumns)) {
    currentByte |= 1;
  }
  currentByte <<= 1;
  if (readModule(row, column - 2, numRows, numColumns)) {
    currentByte |= 1;
  }
  currentByte <<= 1;
  if (readModule(row, column - 1, numRows, numColumns)) {
    currentByte |= 1;
  }
  currentByte <<= 1;
  if (readModule(row, column, numRows, numColumns)) {
    currentByte |= 1;
  }
  return currentByte;
}

int DatamatrixDecoder::readCorner1(int numRows, int numColumns) {
  int currentByte = 0;
  if (readModule(numRows - 1, 0, numRows, numColumns)) {
    currentByte |= 1;
  }
  currentByte <<= 1;
  if (readModule(numRows - 1, 1, numRows, numColumns)) {
    currentByte |= 1;
  }
  currentByte <<= 1;
  if (readModule(numRows - 1, 2, numRows, numColumns)) {
    currentByte |= 1;
  }
  currentByte <<= 1;
  if (readModule(0, numColumns - 2, numRows, numColumns)) {
    currentByte |= 1;
  }
  currentByte <<= 1;
  if (readModule(0, numColumns - 1, numRows, numColumns)) {
    currentByte |= 1;
  }
  currentByte <<= 1;
  if (readModule(1, numColumns - 1, numRows, numColumns)) {
    currentByte |= 1;
  }
  currentByte <<= 1;
  if (readModule(2, numColumns - 1, numRows, numColumns)) {
    currentByte |= 1;
  }
  currentByte <<= 1;
  if (readModule(3, numColumns - 1, numRows, numColumns)) {
    currentByte |= 1;
  }
  return currentByte;
}

int DatamatrixDecoder::readCorner2(int numRows, int numColumns) {
  int currentByte = 0;
  if (readModule(numRows - 3, 0, numRows, numColumns)) {
    currentByte |= 1;
  }
  currentByte <<= 1;
  if (readModule(numRows - 2, 0, numRows, numColumns)) {
    currentByte |= 1;
  }
  currentByte <<= 1;
  if (readModule(numRows - 1, 0, numRows, numColumns)) {
    currentByte |= 1;
  }
  currentByte <<= 1;
  if (readModule(0, numColumns - 4, numRows, numColumns)) {
    currentByte |= 1;
  }
  currentByte <<= 1;
  if (readModule(0, numColumns - 3, numRows, numColumns)) {
    currentByte |= 1;
  }
  currentByte <<= 1;
  if (readModule(0, numColumns - 2, numRows, numColumns)) {
    currentByte |= 1;
  }
  currentByte <<= 1;
  if (readModule(0, numColumns - 1, numRows, numColumns)) {
    currentByte |= 1;
  }
  currentByte <<= 1;
  if (readModule(1, numColumns - 1, numRows, numColumns)) {
    currentByte |= 1;
  }
  return currentByte;
}

int DatamatrixDecoder::readCorner3(int numRows, int numColumns) {
  int currentByte = 0;
  if (readModule(numRows - 1, 0, numRows, numColumns)) {
    currentByte |= 1;
  }
  currentByte <<= 1;
  if (readModule(numRows - 1, numColumns - 1, numRows, numColumns)) {
    currentByte |= 1;
  }
  currentByte <<= 1;
  if (readModule(0, numColumns - 3, numRows, numColumns)) {
    currentByte |= 1;
  }
  currentByte <<= 1;
  if (readModule(0, numColumns - 2, numRows, numColumns)) {
    currentByte |= 1;
  }
  currentByte <<= 1;
  if (readModule(0, numColumns - 1, numRows, numColumns)) {
    currentByte |= 1;
  }
  currentByte <<= 1;
  if (readModule(1, numColumns - 3, numRows, numColumns)) {
    currentByte |= 1;
  }
  currentByte <<= 1;
  if (readModule(1, numColumns - 2, numRows, numColumns)) {
    currentByte |= 1;
  }
  currentByte <<= 1;
  if (readModule(1, numColumns - 1, numRows, numColumns)) {
    currentByte |= 1;
  }
  return currentByte;
}

int DatamatrixDecoder::readCorner4(int numRows, int numColumns) {
  int currentByte = 0;
  if (readModule(numRows - 3, 0, numRows, numColumns)) {
    currentByte |= 1;
  }
  currentByte <<= 1;
  if (readModule(numRows - 2, 0, numRows, numColumns)) {
    currentByte |= 1;
  }
  currentByte <<= 1;
  if (readModule(numRows - 1, 0, numRows, numColumns)) {
    currentByte |= 1;
  }
  currentByte <<= 1;
  if (readModule(0, numColumns - 2, numRows, numColumns)) {
    currentByte |= 1;
  }
  currentByte <<= 1;
  if (readModule(0, numColumns - 1, numRows, numColumns)) {
    currentByte |= 1;
  }
  currentByte <<= 1;
  if (readModule(1, numColumns - 1, numRows, numColumns)) {
    currentByte |= 1;
  }
  currentByte <<= 1;
  if (readModule(2, numColumns - 1, numRows, numColumns)) {
    currentByte |= 1;
  }
  currentByte <<= 1;
  if (readModule(3, numColumns - 1, numRows, numColumns)) {
    currentByte |= 1;
  }
  return currentByte;
}

/****************************************************************************
 *                 correct codewords from vector"words"                     *
 ****************************************************************************/

int DatamatrixDecoder::repair() {
  int i, j, k;
  int t = correctorNum / 2;
  // cumpute syndromes
  int *syndromes = new int[t * 2 + 1];
  bool allright = true;
  for (i = 1; i <= t * 2; i++) {
    syndromes[i] = words[totalNum - 1];
    for (j = 1; j < totalNum; j++) {
      syndromes[i] =
          GfAdd(syndromes[i], GfMult2(words[totalNum - j - 1], j * i));
    }
    if (syndromes[i] != 0) allright = false;
  }
  if (allright) return NOERROR;
  // compute sigma(x)
  int *sigmaPoly = new int[t * t];
  int *sigmaPolySums = new int[t];
  for (i = 0; i < t; i++) {  // inintial equations set
    int index = i * t;
    sigmaPolySums[i] = syndromes[t + i + 1];
    for (j = t + i, k = 0; k < t; j--, k++) sigmaPoly[index + k] = syndromes[j];
  }
  gaussion(sigmaPoly, sigmaPolySums,
           t);  // solve equations by gaussion,
                // results(考1,考2...考t) stored in sigmaPolySums
  // compute Xi: substitute 2^0,2^1......2^totalNum-1 in 考(x) onebyone, if
  // 考(x)=0, we got it
  vector<int> errorPlaces;
  for (i = 0; i < totalNum; i++) {
    int sum = 1;
    for (j = 0; j < t; j++)
      sum = GfAdd(sum, GfDiv2(sigmaPolySums[j], i * j + i));
    if (sum == 0) errorPlaces.push_back(i);
  }
  if (errorPlaces.size() <= 0 || errorPlaces.size() > t) return CANT_REPAIR;
  // compute Ci: using equations set { Sj = ﹉Ci*Xi^j = 0 }
  int errorNum = (int)errorPlaces.size();
  int *yPolySum = new int[errorNum];
  int *yPoly = new int[errorNum * errorNum];
  for (i = 0; i < errorNum; i++) {  // inintial equations set
    int index = i * errorNum;
    k = 0;
    yPolySum[i] = 0;
    for (j = 0; j < totalNum; j++) {
      if (!checkVector(errorPlaces, j)) {  // correct data --> sum
        yPolySum[i] =
            GfAdd(yPolySum[i], GfMult2(words[totalNum - 1 - j], (i + 1) * j));
      } else {  // error place --> polynomial
        yPoly[index + k] = GfMult2(1, (i + 1) * j);
        k++;
      }
    }
  }
  gaussion(yPoly, yPolySum, errorNum);
  for (i = 0; i < errorNum; i++)
    words[totalNum - errorPlaces[i] - 1] = yPolySum[i];
  return REPAIR_OK;
}

bool DatamatrixDecoder::checkVector(vector<int> v, int value) {
  for (int i = 0; i < v.size(); i++)
    if (value == v[i]) return true;
  return false;
}

/****************************************************************************
 *                convert corrected codewords to message                    *
 ****************************************************************************/

#define encTypeAsciiEndValue 129
#define encUnlatchValue 254
#define encEdifactUnlatchValue 31
#define encTypeC40Value 230
#define encTypeBase256Value 231
#define encTypeFNC1Value 232
#define encTypeStructAppendValue 233
#define encTypeAscUpperValue 235
#define encType05MacroValue 236
#define encType06MacroValue 237
#define encTypeX12Value 238
#define encTypeTextValue 239
#define encTypeEdifactValue 240
#define encTypeECIValue 241

typedef enum {
  encTypeAscii = 1,
  encTypeC40,
  encTypeBase256,
  encTypeX12,
  encTypeText,
  encTypeEdifact
} encType;

typedef enum { C40set0 = 0, C40set1, C40set2, C40set3 } c40set;

bool DatamatrixDecoder::getMessage(vector<int>* message) {
  int index = 0;
  bool isMacro = false;

  // macro standard
  if (words[index] == encType05MacroValue ||
      words[index] == encType06MacroValue) {
    message->push_back('[');
    message->push_back(')');
    message->push_back('>');
    message->push_back(30); /* ASCII RS */
    message->push_back('0');
    if (words[index] == encType05MacroValue)
      message->push_back('5');
    else
      message->push_back('6');
    message->push_back(29); /* ASCII GS */

    isMacro = true;
    index++;
  }
  // check each word
  while (index < this->dataNum) {
    if (index < 0) return false;  // any decode function fails, it'll return -1

    int enctype = getEncodeType(words[index]);
    if (enctype != encTypeAscii) index++;

    switch (enctype) {
      case encTypeAscii:
        index = decodeAscii(index, *message);
        break;
      case encTypeC40:
      case encTypeText:
        index = decodeC40Text(index, *message, enctype);
        break;
      case encTypeX12:
        index = decodeX12(index, *message);
        break;
      case encTypeEdifact:
        index = decodeEdifact(index, *message);
        break;
      case encTypeBase256:
        index = decodeBase256(index, *message);
        break;
      default:
        break;
    }
  }

  if (isMacro) {
    message->push_back(30);
    message->push_back(4);
  }

  return true;
}

int DatamatrixDecoder::getEncodeType(int codeword) {
  encType type;
  switch (codeword) {
    case encTypeC40Value:
      type = encTypeC40;
      break;
    case encTypeBase256Value:
      type = encTypeBase256;
      break;
    case encTypeX12Value:
      type = encTypeX12;
      break;
    case encTypeEdifactValue:
      type = encTypeEdifact;
      break;
    case encTypeTextValue:
      type = encTypeText;
      break;
    default:
      type = encTypeAscii;
      break;
  }
  return type;
}

int DatamatrixDecoder::decodeAscii(int index, vector<int> &message) {
  int codeword, digits;
  bool upperShift = false;

  while (index < this->dataNum) {
    codeword = this->words[index];

    if (getEncodeType(codeword) != encTypeAscii)
      return index;
    else
      index++;

    if (upperShift == true) {
      message.push_back(codeword + 127);
      upperShift = false;
    } else if (codeword == encTypeAscUpperValue) {
      upperShift = true;
    } else if (codeword == encTypeAsciiEndValue) {
      return this->dataNum;
    } else if (codeword <= 128) {
      message.push_back(codeword - 1);
    } else if (codeword <= 229) {
      digits = codeword - 130;
      message.push_back(digits / 10 + '0');
      message.push_back(digits - (digits / 10) * 10 + '0');
    }
  }

  return index;
}

void DatamatrixDecoder::pushC40Text(vector<int> &message, int value, bool upperShift) {
  unsigned char m = (unsigned char)value;

  if (upperShift)
    message.push_back(m + 128);
  else
    message.push_back(m);
}

int DatamatrixDecoder::decodeC40Text(int index, vector<int> &message, int encType) {
  int i;
  int packed;
  c40set set = C40set0;
  bool upperShift = false;
  int c40Values[3];

  if (this->dataNum - index < 2)  // only 1 word left
    return index;

  while (index < this->dataNum) {
    packed = (words[index] << 8) | words[index + 1];  // a1*256+a2
    c40Values[0] = ((packed - 1) / 1600);  // a1*256+a2 = b0*1600+b1*40+b2+1
    c40Values[1] = ((packed - 1) / 40) % 40;
    c40Values[2] = (packed - 1) % 40;
    index += 2;

    for (i = 0; i < 3; i++) {
      if (set == C40set0) { /* Basic set */
        if (c40Values[i] <= 2) {
          set = (c40set)(c40Values[i] + 1);  // change set
        } else if (c40Values[i] == 3) {
          pushC40Text(message, ' ', upperShift);
        } else if (c40Values[i] <= 13) {
          pushC40Text(message, c40Values[i] - 13 + '9', upperShift); /* 0-9 */
        } else if (c40Values[i] <= 39) {
          if (encType == encTypeC40) {
            pushC40Text(message, c40Values[i] - 39 + 'Z', upperShift); /* A-Z */
          } else if (encType == encTypeText) {
            pushC40Text(message, c40Values[i] - 39 + 'z', upperShift); /* a-z */
          }
        }
      }

      else if (set == C40set1) {                        /* Shift 1 set */
        pushC40Text(message, c40Values[i], upperShift); /* ASCII 0 - 31 */
      }

      else if (set == C40set2) { /* Shift 2 set */
        if (c40Values[i] <= 14) {
          pushC40Text(message, c40Values[i] + 33,
                      upperShift); /* ASCII 33 - 47 */
        } else if (c40Values[i] <= 21) {
          pushC40Text(message, c40Values[i] + 43,
                      upperShift); /* ASCII 58 - 64 */
        } else if (c40Values[i] <= 26) {
          pushC40Text(message, c40Values[i] + 69,
                      upperShift); /* ASCII 91 - 95 */
        } else if (c40Values[i] == 27) {
          pushC40Text(message, 29,
                      upperShift); /* FNC1 -- XXX depends on position? */
        } else if (c40Values[i] == 30) {
          upperShift = true;
          set = C40set0;
        }
      }

      else if (set == C40set3) { /* Shift 3 set */
        if (encType == encTypeC40) {
          pushC40Text(message, c40Values[i] + 96, upperShift);
        } else if (encType == encTypeText) {
          if (c40Values[i] == 0)
            pushC40Text(message, c40Values[i] + 96, upperShift);
          else if (c40Values[i] <= 26)
            pushC40Text(message, c40Values[i] - 26 + 'Z', upperShift); /* A-Z */
          else
            pushC40Text(message, c40Values[i] - 31 + 127,
                        upperShift); /* { | } ~ DEL */
        }
      }

      upperShift = false;
      set = C40set0;
    }

    if (this->words[index] == encUnlatchValue)  // alter to ascii mode
      return index + 1;

    if (this->dataNum - index < 2)  // only 1 word left
      return index;
  }

  return index;
}

int DatamatrixDecoder::decodeX12(int index, vector<int> &message) {
  int i;
  int packed;
  int x12Values[3];

  if (this->dataNum - index < 2)  // only 1 word left
    return index;

  while (index < this->dataNum) {
    packed = (words[index] << 8) | words[index + 1];  // a1*256+a2
    x12Values[0] = ((packed - 1) / 1600);  // a1*256+a2 = b0*1600+b1*40+b2+1
    x12Values[1] = ((packed - 1) / 40) % 40;
    x12Values[2] = (packed - 1) % 40;
    index += 2;

    for (i = 0; i < 3; i++) {
      if (x12Values[i] == 0)
        message.push_back(13);
      else if (x12Values[i] == 1)
        message.push_back(42);
      else if (x12Values[i] == 2)
        message.push_back(62);
      else if (x12Values[i] == 3)
        message.push_back(32);
      else if (x12Values[i] <= 13)
        message.push_back(x12Values[i] + 44);
      else if (x12Values[i] <= 90)
        message.push_back(x12Values[i] + 51);
    }

    if (this->words[index] == encUnlatchValue)  // alter to ascii mode
      return index + 1;

    if (this->dataNum - index < 2)  // only 1 word left
      return index;
  }

  return index;
}

int DatamatrixDecoder::decodeEdifact(int index, vector<int> &message) {
  /*
                       --->
  Edifact Code      Ascii code
  0 (000000)        64(1 000000)
  30(011110)        94(1 011110)
31                return to Ascii mode
  32(100000)		  32(100000)
  63(111111)        63(111111)

  <---

  4 Edifact Codes(6*4) merge to 3 Ascii codes(8*3):
  0            30         32            63
  000000       011110     100000        111111

  <---

  000000 01      1110 1000       00 111111
  1              232             63

  */
  int i;
  unsigned char unpacked[4];

  if (this->dataNum - index < 3)  // fewer than 3 words
    return index;

  while (index < this->dataNum) {
    unpacked[0] = (this->words[index] & 0xfc) >> 2;  // 0xfc=111111 00
    unpacked[1] =
        (this->words[index] & 0x03) << 4 | (this->words[index + 1] & 0xf0) >> 4;
    unpacked[2] = (this->words[index + 1] & 0x0f) << 2 |
                  (this->words[index + 2] & 0xc0) >> 6;  // 0xc0=11 000000
    unpacked[3] = this->words[index + 2] & 0x3f;         // 0x3f=00 111111

    for (i = 0; i < 4; i++) {
      if (i < 3) index++;

      if (unpacked[i] == encEdifactUnlatchValue) return index;

      if (unpacked[i] <= 30)
        message.push_back(unpacked[i] | 0x40);  // 0x40=0100 0000
      else
        message.push_back(unpacked[i]);
    }

    if (this->dataNum - index < 3)  // fewer than 3 words
      return index;
  }
  return index;
}

int DatamatrixDecoder::decodeBase256(int index, vector<int> &message) {
  int d0, d1;
  int i, endIndex;

  i = index + 1;
  d0 = UnRandomize255State(this->words[index++], i++);
  if (d0 == 0) {
    endIndex = this->dataNum;
  } else if (d0 <= 249) {
    endIndex = index + d0;
  } else {
    d1 = UnRandomize255State(this->words[index++], i++);
    endIndex = index + (d0 - 249) * 250 + d1;
  }

  if (endIndex > this->dataNum) return -1;

  while (index < endIndex)
    message.push_back(UnRandomize255State(this->words[index++], i++));

  return index;
}

int DatamatrixDecoder::UnRandomize255State(int value, int n) {
  int pseudoRandom;
  int tmp;

  pseudoRandom = ((149 * n) % 255) + 1;
  tmp = value - pseudoRandom;
  if (tmp < 0) tmp += 256;

  return tmp;
}

}  // namespace hyf_lemon