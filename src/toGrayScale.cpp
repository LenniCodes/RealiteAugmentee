#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>


cv::Mat toGrayScale(const cv::Mat input){
  cv::Mat output;
  cv::cvtColor(input, output, cv::COLOR_BGR2GRAY);
  return output;
}

// int main(int argc, char *argv[]) {
//   if (argc < 3){
//     std::cout << "usage : " << argv[0] << " [inputImage] [outputPath]" << std::endl;
//     return 1;
//   }
//   toGrayScale(argv[1], argv[2]);
//   return 0;
// }



