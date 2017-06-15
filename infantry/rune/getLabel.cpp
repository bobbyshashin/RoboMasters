/*
    Copyright (c) 2013, Taiga Nomi and the respective contributors
    All rights reserved.

    Use of this source code is governed by a BSD-style license that can be found
    in the LICENSE file.
*/
#include <iostream>
#include "tiny_dnn/tiny_dnn.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace tiny_dnn;
using namespace tiny_dnn::activation;
using namespace std;
using namespace cv;

// rescale output to 0-100
template <typename Activation>
//static network<sequential> nn;
double rescale(double x) {
  Activation a(1);
  return 100.0 * (x - a.scale().first) / (a.scale().second - a.scale().first);
}

void convert_image(cv::Mat img,
                   double minv,
					double maxv,
                   int w,
                   int h,
                   vec_t &data)
{
    if (img.data == nullptr) return; // cannot open, or it's not an image

	cv::Mat_<uint8_t> resized;
    cv::resize(img, resized, cv::Size(w, h));

    // mnist dataset is "white on black", so negate required
    std::transform(resized.begin(), resized.end(), std::back_inserter(data),
        [=](uint8_t c) { return (255 - c) * (maxv - minv) / 255.0 + minv; });
}
/*
void cnn_init(const std::string &dictionary){
	nn.load(dictionary);
}
*/
// Construct vec_t image representation
tiny_dnn::vec_t mat2vec_t(cv::Mat _mat)
{
    tiny_dnn::vec_t ovect;
    switch(_mat.channels()) {
        case 1: {
            tiny_dnn::float_t *ptr = _mat.ptr<tiny_dnn::float_t>(0);
            ovect = tiny_dnn::vec_t(ptr, ptr + _mat.cols * _mat.rows );
        } break;
        case 3: {
            std::vector<cv::Mat> _vmats;
            cv::split(_mat, _vmats);
            for(int i = 0; i < 3; i++) {
                cv::Mat _chanmat = _vmats[i];
                tiny_dnn::float_t *ptr = _chanmat.ptr<tiny_dnn::float_t>(0);
                ovect.insert(ovect.end(), ptr, ptr + _mat.cols * _mat.rows);
            }
        } break;
    }
	return ovect;
}
int recognize(cv::Mat img, const std::string &dictionary) {

    static network<sequential> nn;
    nn.load(dictionary);
    // convert imagefile to vec_t
    vec_t data;
    convert_image(img, -1.0, 1.0, 32, 32, data);
    
    // recognize
    auto res = nn.predict(data);
    vector<pair<double, int>> scores;

    // sort & print top-3
    for (int i = 0; i < 10; i++)
    	scores.emplace_back(rescale<tanh_layer>(res[i]), i);

    sort(scores.begin(), scores.end(), greater<pair<double, int>>());
    cout << endl << endl;
    for(int i = 0; i < 10; i++) 
	cout << "Score: " << scores[i].first << "     Predicted Label: " << scores[i].second << endl;
	return scores[0].second;
}

