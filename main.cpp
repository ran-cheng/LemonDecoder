/*******************************************************************************

  @file      lemon_decoder.cpp
  @brief     entry of the demo
  @details   ~
  @author    cheng-ran@outlook.com
  @date      12.08.2023
  @copyright HengYiFeng, 2021-2023. All right reserved.

*******************************************************************************/
#include <iostream>

#include "lemon_api.h"
using std::vector;
using std::cout;
using std::cin;
using std::cerr;
using std::endl;
using std::string;
using namespace cv;


int main() {
    
    VideoCapture cap;
    cout << "type the camera number: " << endl;
    for (;;) {
      int camera_number;
      cin >> camera_number;
      if (cin.fail()) {
        cin.clear();
        cin.get();
        continue;
      }

      cap.open(camera_number); 
      if (!cap.isOpened()) {
        cerr << "cannot open camera, try another one." << endl;
      } else 
        break;
    }
    cin.clear();
    cout << "press space to decode" << endl;

    // display video
    namedWindow("HYF - SDPC", WINDOW_AUTOSIZE);
    int key;
    Mat frame;
    for (;;) {
      cap >> frame;
      //flip(frame, frame, 1);
      if (frame.empty()) break;  // Ran out
      imshow("HYF - SDPC", frame);
      key = waitKey(5);
      
      if (key == 32) { // space
        cout << "decoding..." << endl;
        vector<vector<uchar>> message;
        if (hyf_lemon::Decode(frame, &message)) {
          for (vector<uchar> text : message) {
            for (uchar c : text) {
              cout << c;
            }
            cout << endl;

            // save file
            string file_name("D:/hyf/");
            file_name.append(string((char*)text.data(), text.size()));
            file_name.append(".jpg");
            if (imwrite(file_name, frame)) {
              cout << "Image saved: ";
              cout << file_name << endl;
            }
          }
        } else
          cout << "fail" << endl;
      }
    }

    cap.release();
}
