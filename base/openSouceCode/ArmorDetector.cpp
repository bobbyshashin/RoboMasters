/*******************************************************************************************************************
Copyright 2017 Dajiang Innovations Technology Co., Ltd (DJI)

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
documentation files(the "Software"), to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense, and / or sell copies of the Software, and 
to permit persons to whom the Software is furnished to do so, subject to the following conditions : 

The above copyright notice and this permission notice shall be included in all copies or substantial portions of
the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO
THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
IN THE SOFTWARE.
*******************************************************************************************************************/

#include "ArmorDetector.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <queue>

#ifndef SHOW_DEBUG_IMG
//#define SHOW_DEBUG_IMG
#endif

#ifndef COUT_LOG
//#define COUT_LOG
#endif

#ifndef USE_NEON
#define USE_NEON
#endif

using namespace cv;
using namespace std;

void ArmorDetector::setImage(const cv::Mat & src){
    _size = src.size();
    const cv::Point & last_result = _res_last.center;
    if(last_result.x == 0 || last_result.y == 0){
        _src = src;
        _dect_rect = Rect(0, 0, src.cols, src.rows);
    }
    else{
        Rect rect = _res_last.boundingRect();
        int max_half_w = _para.max_light_delta_h * 1.3;
        int max_half_h = 300;
        double scale = src.rows == 480 ? 1.8 : 2.5;

        int exp_half_w = min(max_half_w / 2, int(rect.width * scale));
        int exp_half_h = min(max_half_h / 2, int(rect.height * scale));

        int w = std::min(max_half_w, exp_half_w);
        int h = std::min(max_half_h, exp_half_h);
        Point center = last_result;
        int x = std::max(center.x - w, 0);
        int y = std::max(center.y - h, 0);
        Point lu = Point(x, y);
        x = std::min(center.x + w, src.cols);
        y = std::min(center.y + h, src.rows);
        Point rd = Point(x, y);


//        int exp_half_w = 100;
//        int exp_half_h = 40;
//        int exp_half_gap_w = 60;
//        int exp_half_gap_h = 10;

//        if(src.rows == 720){
//            exp_half_w = 150;
//            exp_half_h = 60;
//            exp_half_gap_w = 80;
//            exp_half_gap_h = 15;
//        }

//        Rect rect = _res_last.boundingRect();
//        int x = std::max(int(rect.x - exp_half_gap_w), 0);
//        int y = std::max(int(rect.y - exp_half_gap_h), 0);
//        Point center = last_result;
//        int _x = std::max(center.x - exp_half_w, 0);
//        int _y = std::max(center.y - exp_half_h, 0);
//        Point lu = Point(min(x, _x), min(y, _y));

//        x = std::min(int(rect.x+rect.width + exp_half_gap_w), src.cols);
//        y = std::min(int(rect.y+rect.height + exp_half_gap_h), src.rows);
//        _x = std::min(center.x + exp_half_w, src.cols);
//        _y = std::min(center.y + exp_half_h, src.rows);
//        Point rd = Point(max(x, _x), max(y, _y));
        _dect_rect = Rect(lu, rd);
        if (makeRectSafe(_dect_rect, src.size()) == false){
            _res_last = cv::RotatedRect();
            _dect_rect = Rect(0, 0, src.cols, src.rows);
            _src = src;
        }
        else
            src(_dect_rect).copyTo(_src);
    }

    int total_pixel = _src.cols * _src.rows;
	const uchar * ptr_src = _src.data;
	const uchar * ptr_src_end = _src.data + total_pixel * 3;

    _g.create(_src.size(), CV_8UC1);
    _ec.create(_src.size(), CV_8UC1);
    _max_color = cv::Mat(_src.size(), CV_8UC1, cv::Scalar(0));
    uchar *ptr_g = _g.data, *ptr_ec = _ec.data, *ptr_max_color = _max_color.data;
    if (_para.enemy_color == RED){
        for (; ptr_src != ptr_src_end; ++ptr_src, ++ptr_g, ++ptr_max_color, ++ptr_ec)	{
            uchar b = *ptr_src;
            uchar g = *(++ptr_src);
            uchar r = *(++ptr_src);
            *ptr_g = g;
            *ptr_ec = r;
            //*ptr_g = b;
            if (r > _para.min_light_gray)
                *ptr_max_color = 255;
//            if (r - b > _para.br_threshold && r >= g)
//                *ptr_max_color = 255;
        }
    }
    else {
        for (; ptr_src != ptr_src_end; ++ptr_src, ++ptr_g, ++ptr_max_color, ++ptr_ec)	{
            uchar b = *ptr_src;
            uchar g = *(++ptr_src);
            uchar r = *(++ptr_src);
            *ptr_g = g;
            *ptr_ec = b;
            //*ptr_g = r;
            if (b > _para.min_light_gray)
                *ptr_max_color = 255;
//            if (b - r > _para.br_threshold && b >= g)
//                *ptr_max_color = 255;
        }
    }

#ifdef SHOW_DEBUG_IMG
	cv::imshow("g", _g);
	cv::imshow("_max_color", _max_color);
#endif
}

void ArmorDetector::initTemplate(const cv::Mat & _template, const cv::Mat & _template_small){
    std::vector<cv::Mat> bgr;
    bgr.resize(3);
    split(_template, bgr);

    Mat temp_green;
    resize(bgr[1], temp_green, Size(100,25));
    cv::threshold(temp_green, _binary_template, 128, 255, THRESH_OTSU);

    split(_template_small, bgr);
    resize(bgr[1], temp_green, Size(100,25));
    cv::threshold(temp_green, _binary_template_small, 128, 255, THRESH_OTSU);
}

int ArmorDetector::templateDist(const cv::Mat & img, bool is_small){
    int dist = 0;
    const uchar threshold_value = _para.min_light_gray - 15;
    int total_piexl = img.rows* img.cols;
    const uchar * p1 = is_small ? _binary_template_small.data :_binary_template.data;
    const uchar * p2 = img.data;
    for(int i = 0; i < total_piexl;  ++i, p1+=1, p2+=3){
        uchar v = (*p2+1) > threshold_value ? 255 : 0;
        dist += (*p1) == v ? 0 : 1;
    }
    return dist;
}

#ifndef USE_NEON
void ArmorDetector::findContourInEnemyColor(
		cv::Mat & left, cv::Mat & right, 
        vector<vector<Point2i> > &contours_left,
        vector<vector<Point2i> > &contours_right){
    // find contour in sub image of blue and red
    vector<vector<Point2i> > contours_br;
	vector<Vec4i> hierarchy;
    findContours(_max_color, contours_br, hierarchy, CV_RETR_EXTERNAL , CV_CHAIN_APPROX_SIMPLE);
    vector<vector<Point2i> >::const_iterator it = contours_br.begin();

    // left lamp template
    // o o x x x x x x
    // o o x x x x x x
    // o o x x x x x x

    // right lamp template
    // x x x x x x o o
    // x x x x x x o o
    // x x x x x x o o

    // margin_l -> col of 'o'
    // margin_r -> col of 'x'
    // margin_h -> row of 'o' or 'x'

    // using average gray value of 'x' minus average gray value of 'o'
    // if the value lager than threshold, the point is consider as a lamp point

	left = Mat::zeros(_max_color.size(), CV_8UC1);
	right = Mat::zeros(_max_color.size(), CV_8UC1);
    const int margin_l = 1, margin_r = 10;
    const int margin_h = 3;

    while (it != contours_br.end()){
		Rect rect = cv::boundingRect(*it);
		if (rect.height < _para.min_light_height){
			++it;
			continue;
		}

		int max_i = rect.x + rect.width;
		max_i = std::min(_max_color.cols, max_i + margin_r);
		int half_j = (margin_h >> 1), max_j = rect.y + rect.height, min_j = rect.y;
		max_j = std::min(_max_color.rows - 1, max_j + half_j);
		min_j = std::max(min_j, half_j);
		int count_left = 0, count_right = 0;

        const uchar * ptr_gray_base = _g.data;
        for (size_t j = min_j; j < max_j; ++j)	{
            const uchar * ptr_gray = ptr_gray_base + j * _g.cols;
            for (size_t i = rect.x; i < max_i; ++i)	{
                if (*(ptr_gray + i) < _para.min_light_gray)
                    continue;

                float block0 = 0, block1 = 0, block_1 = 0;
                // do margin protection
                if (i >= margin_r){
                    for (int m = -half_j; m <= half_j; ++m)	{
                        const uchar * ptr = ptr_gray + m * _g.cols + i;
                        // for common 'o' of template
                        for (int k = 0; k < margin_l; k++)
                            block0 += *(ptr + k);
                        // for 'x' of left template
                        for (int k = margin_l; k < margin_r; k++)
                            block1 += *(ptr + k);
                        // for 'x' of right template
                        for (int k = -margin_l; k > -margin_r; --k)
                            block_1 += *(ptr + k);
                    }
                    block0 /= margin_h * margin_l;
                    block1 /= margin_h * (margin_r - margin_l);
                    block_1 /= margin_h * (margin_r - margin_l);
                    int avgdist = block0 - block1;
                    left.at<uchar>(j, i) = avgdist > _para.avg_contrast_threshold ? (++count_left, 255) : 0;
                    avgdist = block0 - block_1;
                    right.at<uchar>(j, i) = avgdist > _para.avg_contrast_threshold ? (++count_right, 255) : 0;
                }
                else {
                    for (int m = -half_j; m <= half_j; ++m)	{
                        const uchar * ptr = ptr_gray + m * _g.cols + i;
                        // for 'o' of left template
                        for (int k = 0; k < margin_l; k++)
                            block0 += *(ptr + k);
                        // for 'x' of left template
                        for (int k = margin_l; k < margin_r; k++)
                            block1 += *(ptr + k);;
                    }
                    block0 /= margin_h * margin_l;
                    block1 /= margin_h * (margin_r - margin_l);
                    int avgdist = block0 - block1;
                    left.at<uchar>(j, i) = avgdist > _para.avg_contrast_threshold ? (++count_left, 255) : 0;
                }
            }
        }

        // find the lamp contours
		if (count_left > 10){
            vector<vector<Point2i> > contour;
			vector<Vec4i> hierarchy;
			findContours(left(rect), contour, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cv::Point2i(rect.x, rect.y));
			contours_left.insert(contours_left.end(), contour.begin(), contour.end());
		}

		if (count_right > 10){
            vector<vector<Point2i> > contour;
			vector<Vec4i> hierarchy;
			findContours(right(rect), contour, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cv::Point2i(rect.x, rect.y));
			contours_right.insert(contours_right.end(), contour.begin(), contour.end());
		}
		++it;
	}
#ifdef COUT_LOG
//	cout << "contours_br:" << contours_br.size() << endl;
//	cout << "contours_left:" << contours_left.size() << endl;
//	cout << "contours_right:" << contours_right.size() << endl;
#endif
}

#else

#include <sse_to_neon.hpp>
#include <arm_neon.h>
#include <stdlib.h>

void ArmorDetector::findContourInEnemyColor(
		cv::Mat & left, cv::Mat & right,
        vector<vector<Point2i> > &contours_left,
        vector<vector<Point2i> > &contours_right){
    // find contour in sub image of blue and red
    vector<vector<Point2i> > contours_br;
	vector<Vec4i> hierarchy;
    findContours(_max_color, contours_br, hierarchy, CV_RETR_EXTERNAL , CV_CHAIN_APPROX_SIMPLE);
    vector<vector<Point2i> >::const_iterator it = contours_br.begin();

    // left lamp template
    // o o x x x x x x
    // o o x x x x x x
    // o o x x x x x x

    // right lamp template
    // x x x x x x o o
    // x x x x x x o o
    // x x x x x x o o

    // margin_l -> col of 'o'
    // margin_r -> col of 'x'
    // margin_h -> row of 'o' or 'x'

    // using average gray value of 'x' minus average gray value of 'o'
    // if the value lager than threshold, the point is consider as a lamp point

	left = Mat::zeros(_max_color.size(), CV_8UC1);
	right = Mat::zeros(_max_color.size(), CV_8UC1);
    const int margin_o = 1, margin_x = 16;
    const int margin_h = 3;

    const __m128i vk0 = _mm_setzero_si128();       // constant vector of all 0s for use with _mm_unpacklo_epi8/_mm_unpackhi_epi8

#ifdef SHOW_DEBUG_IMG
    Mat _max_color_rgb;
    cvtColor(_max_color, _max_color_rgb, CV_GRAY2BGR);
#endif
    while (it != contours_br.end()){
		Rect rect = cv::boundingRect(*it);
#ifdef SHOW_DEBUG_IMG
        Scalar color(rand() & 255, rand() & 255, rand() & 255);
        rectangle(_max_color_rgb, rect, color, 2);
        char slope_str[15];
        sprintf(slope_str, "%d,%d", rect.width, rect.height);
        putText(_max_color_rgb, slope_str, Point(rect.x, rect.y), CV_FONT_HERSHEY_COMPLEX_SMALL, 0.5, color, 1);
#endif
        if ((rect.height < _para.min_light_height) ||
            (rect.height > 15 && rect.width > 0.6 * rect.height + 0.5) ||
            (rect.height <= 15 && rect.width > 0.9 * rect.height)){
            ++it;
            continue;
        }

        Rect b_rect = rect;
        if(broadenRect(b_rect, 3, 3, _src.size()) == false) continue;
        Scalar m = mean(_src(b_rect));
        if (_para.enemy_color == RED && (m[0] > m[2] || m[1] > m[2])){
            ++it;
            continue;
        }
        else if(_para.enemy_color == BLUE && (m[2] > m[0] || m[1] > m[0])){
            ++it;
            continue;
        }

        int min_i = rect.x;
		int max_i = std::min(_max_color.cols - margin_o, rect.x + rect.width);
		int min_j = rect.y;
        int max_j = std::min(_max_color.rows - margin_h, rect.y + rect.height);

        int count_left = 0, count_right = 0;
        const uchar * ptr_gray_base = _g.data;
        const uchar * ptr_ec_base = _ec.data;
        for (size_t j = min_j; j < max_j; ++j)	{
            size_t offset = j * _g.cols;
            const uchar * ptr_gray = ptr_gray_base + offset;
            const uchar * ptr_ec = ptr_ec_base + offset;
            uchar * ptr_left = left.data + offset;
            uchar * ptr_right = right.data + offset;

            for (size_t i = min_i; i < max_i; ++i)	{
                if (*(ptr_ec + i) < _para.min_light_gray)
                    continue;

                if((*(ptr_left + i) != 0) && (*(ptr_right + i) != 0))
                    continue;

                int offset_0 = i;
                int offset_1 = offset_0 + _g.cols;
                int offset_2 = offset_1 + _g.cols;

                const uchar * cur_point_0 = ptr_gray + offset_0;
                const uchar * cur_point_1 = ptr_gray + offset_1;
                const uchar * cur_point_2 = ptr_gray + offset_2;

//                int sum_o = (cur_point_0[0] + cur_point_0[1]) +
//                            (cur_point_1[0] + cur_point_1[1]) +
//                            (cur_point_2[0] + cur_point_2[1]);
//                sum_o /= 6;
                int sum_o = ((ptr_ec + offset_0)[0]) +
                            ((ptr_ec + offset_1)[0]) +
                            ((ptr_ec + offset_2)[0]);
                sum_o /= 3;
//                if(sum_o < _para.min_light_gray)
//                    continue;

                if (i + margin_o + margin_x < _max_color.cols) {
                //if (*(ptr_left + i) == 0 && i + margin_o + margin_x < _max_color.cols) {
                    __m128i vsum = _mm_setzero_si128();
                    __m128i v = _mm_load_si128((__m128i*)(cur_point_0+margin_o));
                    __m128i vl = _mm_unpacklo_epi8(v, vk0);
                    __m128i vh = _mm_unpackhi_epi8(v, vk0);
                    vsum = _mm_add_epi16(vsum, _mm_add_epi16(vl, vh));

                    v = _mm_load_si128((__m128i*)(cur_point_1+margin_o));
                    vl = _mm_unpacklo_epi8(v, vk0);
                    vh = _mm_unpackhi_epi8(v, vk0);
                    vsum = _mm_add_epi16(vsum, _mm_add_epi16(vl, vh));

                    v = _mm_load_si128((__m128i*)(cur_point_2+margin_o));
                    vl = _mm_unpacklo_epi8(v, vk0);
                    vh = _mm_unpackhi_epi8(v, vk0);
                    vsum = _mm_add_epi16(vsum, _mm_add_epi16(vl, vh));

                    int sum_x  = _mm_extract_epi16(vsum,0) + _mm_extract_epi16(vsum,1) + _mm_extract_epi16(vsum,2) + _mm_extract_epi16(vsum,3)+
                            _mm_extract_epi16(vsum,4) + _mm_extract_epi16(vsum,5) + _mm_extract_epi16(vsum,6) + _mm_extract_epi16(vsum,7);

                    int avgdist = sum_o - sum_x/48;
                    //cout << "sum_o, sum_x, avgdist:\t" << sum_o/16 << ", " << sum_x/128 << ", " << avgdist << "\n";
                    if (avgdist > _para.avg_contrast_threshold){
                        *(unsigned char*)(ptr_left+offset_0) = 0xff;
                        *(unsigned char*)(ptr_left+offset_1) = 0xff;
                        *(unsigned char*)(ptr_left+offset_2) = 0xff;

                        count_left += 3;
                        //cout << "sum_x:" << sum_x << "\tsum_o:" << sum_o << endl;
                    }
                }

                if (i > margin_x) {
                //if (*(ptr_right + i) == 0 && i > margin_x) {
                    __m128i vsum = _mm_setzero_si128();
                    __m128i v = _mm_load_si128((__m128i*)(cur_point_0-margin_x));
                    __m128i vl = _mm_unpacklo_epi8(v, vk0);
                    __m128i vh = _mm_unpackhi_epi8(v, vk0);
                    vsum = _mm_add_epi16(vsum, _mm_add_epi16(vl, vh));

                    v = _mm_load_si128((__m128i*)(cur_point_1-margin_x));
                    vl = _mm_unpacklo_epi8(v, vk0);
                    vh = _mm_unpackhi_epi8(v, vk0);
                    vsum = _mm_add_epi16(vsum, _mm_add_epi16(vl, vh));

                    v = _mm_load_si128((__m128i*)(cur_point_2-margin_x));
                    vl = _mm_unpacklo_epi8(v, vk0);
                    vh = _mm_unpackhi_epi8(v, vk0);
                    vsum = _mm_add_epi16(vsum, _mm_add_epi16(vl, vh));

                    int sum_x  = _mm_extract_epi16(vsum,0) + _mm_extract_epi16(vsum,1) + _mm_extract_epi16(vsum,2) + _mm_extract_epi16(vsum,3)+
                            _mm_extract_epi16(vsum,4) + _mm_extract_epi16(vsum,5) + _mm_extract_epi16(vsum,6) + _mm_extract_epi16(vsum,7);

                    int avgdist = sum_o - sum_x/48;
                    //cout << "sum_o, sum_x, avgdist:\t" << sum_o/16 << ", " << sum_x/128 << ", " << avgdist << "\n";
                    if (avgdist > _para.avg_contrast_threshold){
                        *(unsigned char*)(ptr_right+offset_0) = 0xff;
                        *(unsigned char*)(ptr_right+offset_1) = 0xff;
                        *(unsigned char*)(ptr_right+offset_2) = 0xff;

                        count_right += 3;
                        //cout << "sum_x:" << sum_x << "\tsum_o:" << sum_o << endl;
                    }
                }
            }
        }

        // find the lamp contours
//		if (count_left > 10){
//            vector<vector<Point2i> > contour;
//			vector<Vec4i> hierarchy;
//			findContours(left(rect), contour, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cv::Point2i(rect.x, rect.y));
//			contours_left.insert(contours_left.end(), contour.begin(), contour.end());
//		}

//		if (count_right > 10){
//            vector<vector<Point2i> > contour;
//			vector<Vec4i> hierarchy;
//			findContours(right(rect), contour, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cv::Point2i(rect.x, rect.y));
//			contours_right.insert(contours_right.end(), contour.begin(), contour.end());
//		}
		++it;
	}

#ifdef SHOW_DEBUG_IMG
    cv::imshow("_max_color_rgb", _max_color_rgb);
    cv::imshow("contrast_left", left);
	cv::imshow("contrast_right", right);
#endif

    findContours(left, contours_left, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
    findContours(right, contours_right, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

#ifdef COUT_LOG
//	cout << "contours_br:" << contours_br.size() << endl;
	cout << "contours_left:" << contours_left.size() << endl;
	cout << "contours_right:" << contours_right.size() << endl;
#endif
}
#endif


cv::RotatedRect ArmorDetector::boundingRRect(const cv::RotatedRect & left, const cv::RotatedRect & right){
	const Point & pl = left.center, & pr = right.center;
	Point2f center = (pl + pr) / 2.0;
	cv::Size2f wh_l = left.size;
	cv::Size2f wh_r = right.size;
	float width = POINT_DIST(pl, pr) - (wh_l.width + wh_r.width) / 2.0;
	float height = std::max(wh_l.height, wh_r.height);
    //float height = (wh_l.height + wh_r.height) / 2.0;
	float angle = std::atan2(right.center.y - left.center.y, right.center.x - left.center.x);
	return RotatedRect(center, Size2f(width, height), angle * 180 / CV_PI);
}

RotatedRect ArmorDetector::adjustRRect(const RotatedRect & rect){
	const Size2f & s = rect.size;
	if (s.width < s.height)
		return rect;
	return RotatedRect(rect.center, Size2f(s.height, s.width), rect.angle + 90.0);
}

void ArmorDetector::findTargetInContours(const vector<vector<Point> > & contours_left, const vector<vector<Point> > & contours_right, vector<RotatedRect> & rects, std::vector<double> & score) {
	// ��ֱ������������ҳ�����б�ʷ�Χ������
	vector<RotatedRect> final_contour_rect_left, final_contour_rect_right;
    vector<double> score_left, score_right;
#ifdef SHOW_DEBUG_IMG
	Mat contours_show_left, contours_show_right;
	_src.copyTo(contours_show_left);
	_src.copyTo(contours_show_right);
#endif

	for (size_t i = 0; i < contours_left.size(); ++i){
        // fit the lamp contour as a eclipse
		RotatedRect rrect = minAreaRect(contours_left[i]);
		rrect = adjustRRect(rrect);
		double angle = rrect.angle;
		angle = 90 - angle;
		angle = angle < 0 ? angle + 180 : angle;

#ifdef SHOW_DEBUG_IMG
		//cout << "(angle)\t(" << angle << ")\n";
		Scalar color(rand() & 255, rand() & 255, rand() & 255);
		char slope_str[15];
        sprintf(slope_str, "%.1f", angle);
		putText(contours_show_left, slope_str, contours_left[i][0], CV_FONT_HERSHEY_COMPLEX_SMALL, 0.5, color, 1);
		drawContours(contours_show_left, contours_left, i, color, CV_FILLED, 8);
#endif
        // the contour must be near-vertical
        float delta_angle = abs(angle - 90);
        if (delta_angle < _para.light_slope_offset){
			final_contour_rect_left.push_back(rrect);
            score_left.push_back(delta_angle);
		}
	}

	for (size_t i = 0; i < contours_right.size(); ++i){
        // fit the lamp contour as a eclipse
		RotatedRect rrect = minAreaRect(contours_right[i]);
		rrect = adjustRRect(rrect);
		double angle = rrect.angle;
		angle = 90 - angle;
		angle = angle < 0 ? angle + 180 : angle;

#ifdef SHOW_DEBUG_IMG
		//cout << "(angle)\t(" << angle << ")\n";
		Scalar color(rand() & 255, rand() & 255, rand() & 255);
		char slope_str[15];
        sprintf(slope_str, "%.1f", angle);
		putText(contours_show_right, slope_str, contours_right[i][0], CV_FONT_HERSHEY_COMPLEX_SMALL, 0.5, color, 1);
		drawContours(contours_show_right, contours_right, i, color, CV_FILLED, 8);
#endif
        // the contour must be near-vertical
        float delta_angle = abs(angle - 90);
		if (delta_angle < _para.light_slope_offset){
			final_contour_rect_right.push_back(rrect);
            score_right.push_back(delta_angle);
		}
	}

    // using all the left edge and right edge to make up rectangles
    for (size_t i = 0; i < final_contour_rect_left.size(); ++i) {
        const RotatedRect & rect_i = final_contour_rect_left[i];
        const Point & center_i = rect_i.center;
		float xi = center_i.x;
		float yi = center_i.y;

        for (size_t j = 0; j < final_contour_rect_right.size(); j++) {
            const RotatedRect & rect_j = final_contour_rect_right[j];
            const Point & center_j = rect_j.center;
			float xj = center_j.x;
			float yj = center_j.y;
			float delta_h = xj - xi;
            float delta_angle = abs(rect_j.angle - rect_i.angle);

            // if rectangle is match condition, put it in candidate vector
			if (delta_h > _para.min_light_delta_h && delta_h < _para.max_light_delta_h &&
				abs(yi - yj) < _para.max_light_delta_v &&
                delta_angle < _para.max_light_delta_angle) {
				RotatedRect rect = boundingRRect(rect_i, rect_j);
				rects.push_back(rect);
                score.push_back((score_right[j] + score_left[i]) / 6.0 + delta_angle);
			}
		}
	}
#ifdef SHOW_DEBUG_IMG
	imshow("4.contours_l", contours_show_left);
	imshow("4.contours_r", contours_show_right);
#endif
}


cv::RotatedRect ArmorDetector::chooseTarget(const std::vector<cv::RotatedRect> & rects, const std::vector<double> & score) {
    if (rects.size() < 1){
        _is_lost = true;
        return RotatedRect();
    }

	int ret_idx = -1;
    double avg_score = 0.0;
    for(int i = 0; i < score.size(); ++i){
        avg_score += score[i];
    }
    avg_score /= score.size();
    const double small_armor_wh_threshold = 3.6;
    const double avg_slope = 4.0;
    const double exp_weight_scale = 15.0;
    const double degree2rad_scale = 3.1415926 / 180.0;
    double percent_large_grad_threshold = 0.25;
    double template_dist_threshold = 0.20;
    double max_wh_ratio = 5.2, min_wh_ratio = 1.25;
    if( _is_lost == false){
        template_dist_threshold = 0.4;
        percent_large_grad_threshold = 0.5;
        max_wh_ratio += 0.5;
        min_wh_ratio -= 0.2;
    }
    double weight = template_dist_threshold * percent_large_grad_threshold / exp(-(avg_slope + avg_score) * degree2rad_scale * exp_weight_scale);
    bool is_small = false;

    #define SafeRect(rect, max_size) {if (makeRectSafe(rect, max_size) == false) continue;}

	for (size_t i = 0; i < rects.size(); ++i){
		const RotatedRect & rect = rects[i];

        // the ratio of width and height must be matched
        double w = rect.size.width;
        double h = rect.size.height;
        double wh_ratio = w / h;
        if (wh_ratio > max_wh_ratio || wh_ratio < min_wh_ratio)
            continue;

        AngleSolver * slover = NULL;
        if(_size.height == 480)
            slover = s_solver;
        else if(_size.height == 720)
            slover = l_solver;

        if (wh_ratio < small_armor_wh_threshold)
            is_small = true;
        else
            is_small = false;

        if (slover != NULL && _is_lost){
            is_small == true ? slover->setTargetSize(12.4, 5.4) : slover->setTargetSize(21.6, 5.4);
            double angle_y = 0.0, angle_x = 0.0;
            if (false == slover->getAngle(rect, angle_x, angle_y, 0., 0., cv::Point(_dect_rect.x, _dect_rect.y)))
                continue;
            Mat xyz_in_ptz = slover->position_in_ptz;
            double d = sqrt(xyz_in_ptz.at<double>(1) * xyz_in_ptz.at<double>(1) + xyz_in_ptz.at<double>(2) * xyz_in_ptz.at<double>(2));
            double t_offset = sin((pitch_angle -  angle_y) * 3.1415926 / 180.0) * d;
            if(t_offset < -40.0 || t_offset > 35.0){
#ifdef COUT_LOG
                cout << "refused : target out of range: " <<
                        "\n\tcurrent ptz angle: " << pitch_angle <<
                        "\n\tcurrent target position: "  << xyz_in_ptz.t() <<
                        "\n\tcurrent target offet set: " << t_offset << "\n";
#endif
                continue;
            }
            if(t_offset > 5.0)
                is_small = false;
            else
                is_small = true;
        }

        // width must close to the last result
        const Size2f size_last = _res_last.size;
        if(_is_lost == false && size_last.width > _para.min_light_delta_h){
            double percent = 0.50 * size_last.width;
            if (abs(w - size_last.width) > percent){
#ifdef COUT_LOG
                cout << "refused 0 : size_last.width: " << size_last.width << "\tcur width: "  << w << endl;
#endif
                continue;
            }
        }

        // rotate the area
        int lamp_width = max((int)w / (is_small ? 12 : 20), 1);
        cv::Rect bounding_roi = rect.boundingRect();
        bounding_roi.x -= w / 8;
        bounding_roi.width += w / 4;
        SafeRect(bounding_roi, _src.size());

		Point2f new_center = rect.center - Point2f(bounding_roi.x, bounding_roi.y);
		Mat roi_src = _src(bounding_roi);
		Mat rotation = getRotationMatrix2D(new_center, rect.angle, 1);
		Mat rectify_target;
        cv::warpAffine(roi_src, rectify_target, rotation, bounding_roi.size());

        // get the black board of the armor
        cv::Point ul = Point(std::max(int(new_center.x - (w / 2.0)) + 1, 0), std::max((int)(new_center.y - h / 2.0), 0));
        cv::Point dr = Point(new_center.x + w / 2.0, new_center.y + h / 2.0);
        Rect roi_black = Rect(cv::Point(ul.x, ul.y), cv::Point(dr.x, dr.y));
        // get the left lamp and right lamp of the armor
        Rect roi_left_lamp = Rect(Point(max(0, ul.x - lamp_width), ul.y), Point(max(rectify_target.cols, ul.x), dr.y));
        Rect roi_right_lamp = Rect(Point(dr.x , ul.y), Point(min(dr.x + lamp_width, rectify_target.cols), dr.y));

        SafeRect(roi_left_lamp, rectify_target.size());
        SafeRect(roi_right_lamp, rectify_target.size());
        SafeRect(roi_black, rectify_target.size());

        // valid the gray value of black area
        Mat black_part;
        rectify_target(roi_black).copyTo(black_part);
        int black_side = min(_para.min_light_delta_h / 2, 4);
        Mat gray_mid_black(Size(roi_black.width - black_side * 2, roi_black.height), CV_8UC1);
        const uchar * ptr = black_part.data;
        uchar * ptr_gray = gray_mid_black.data;
        int avg_green_mid = 0;
        int avg_red_side = 0;
        int avg_blue_side = 0;
        int avg_green_side = 0;
        int cf = black_part.cols - black_side;
        for(int j = 0; j < black_part.rows; ++j){
            for (int k = 0; k < black_side; ++k, ++ptr){
                uchar b = *ptr;
                uchar g = *(++ptr);
                uchar r = *(++ptr);
                avg_red_side += r;
                avg_blue_side += b;
                avg_green_side += g;
            }
            for (int k = black_side; k < cf; ++k, ++ptr, ++ptr_gray){
                uchar b = *ptr;
                uchar g = *(++ptr);
                uchar r = *(++ptr);
                avg_green_mid += g;
                *ptr_gray = (uchar)((r * 38 + g * 75 + b * 15) >> 7);
            }
            for (int k = cf; k < black_part.cols; ++k, ++ptr){
                uchar b = *ptr;
                uchar g = *(++ptr);
                uchar r = *(++ptr);
                avg_red_side += r;
                avg_blue_side += b;
                avg_green_side += g;
            }
        }
        avg_green_mid /= (gray_mid_black.cols * gray_mid_black.rows);
        int side_total = black_side * 2 * gray_mid_black.rows;
        avg_green_side /= side_total;
        if (avg_green_mid > _para.avg_board_gray_threshold){
#ifdef COUT_LOG
            cout << "refused 1 : avg_green: " << avg_green_mid << "\tavg_board_gray_threshold: "  << (int)_para.avg_board_gray_threshold << endl;
#endif
            continue;
        }

        if (_para.enemy_color == RED && avg_red_side - 10 < avg_blue_side){
#ifdef COUT_LOG
            cout << "refused 1.1 : red < blue:  red:" << avg_red_side/side_total << "\tblue: "  << avg_blue_side/side_total << endl;
#endif
            continue;
        }
        else if (_para.enemy_color == BLUE && avg_blue_side - 10 < avg_red_side){
#ifdef COUT_LOG
            cout << "refused 1.2 : red > blue:  red:" << avg_red_side/side_total << "\tblue: "  << avg_blue_side/side_total << endl;
#endif
            continue;
        }

        // valid the gradient of the black area
        Mat gradX, gradY;
        cv::Sobel(gray_mid_black, gradX, CV_16S, 1, 0);
        cv::Sobel(gray_mid_black, gradY, CV_16S, 0, 1);

        int y_grad = 0, x_grad = 0;
        int large_grad_count = 0;
        int side_width = gray_mid_black.cols * 10.0 / 100; // jump over the side��
        short * ptr_x = (short *)gradX.data;
        short * ptr_y = (short *)gradY.data;
        for (size_t j = 0; j < gradX.rows; ++j){
            // jump over left side part
            ptr_x+= side_width;
            ptr_y+= side_width;

            // compute the middle part
            int up_b = gradX.cols - side_width;
            for (size_t k = side_width; k < up_b; ++k, ++ptr_x, ++ptr_y)	{
                int x = abs(*ptr_x);
                int y = abs(*ptr_y);
                x_grad += x;
                y_grad += y;
                large_grad_count += y / _para.grad_threshold;
            }
            // jump over right side part
            ptr_x+= gradX.cols-up_b;
            ptr_y+= gradX.cols-up_b;
        }

        // kick out the area of large gradients
        int total_pixel = (gradX.cols - (side_width << 1)) * gradX.rows;
        double large_grad_percent =(double)large_grad_count/total_pixel;
        if(large_grad_percent > percent_large_grad_threshold){
#ifdef COUT_LOG
            cout << "refused 2: large_grad_percent: " << large_grad_percent <<  endl;
#endif
            continue;
        }

        // valid the average gradient of the black area
        double avg_x = (double)x_grad / total_pixel;
        double avg_y = (double)y_grad / total_pixel;
        if (avg_x < _para.avg_board_grad_threshold + 30 && avg_y < _para.avg_board_grad_threshold ){
            Point p1(roi_left_lamp.x, roi_left_lamp.y);
            Point p2(roi_right_lamp.x+roi_right_lamp.width, roi_right_lamp.y+roi_right_lamp.height);
            // get the whole area of armor
            Rect armor_rect(p1, p2);
            SafeRect(armor_rect, rectify_target.size());
            Mat armor = rectify_target(armor_rect);

            // compute the distance of template
            cv::Size cur_size;
            if (is_small)
                cur_size = _binary_template_small.size();
            else
                cur_size = _binary_template.size();
            resize(armor, armor, cur_size);
            double dist = templateDist(armor, is_small);
            dist = dist / (cur_size.width * cur_size.height);
            if(dist >  template_dist_threshold){
#ifdef COUT_LOG
                cout << "refused 3: dist: " << dist << "\tdist threshold:" << (cur_size.width * cur_size.height) / 4 << endl;
#endif
                continue;
            }

            // choose the best rectangle with minimum (large_grad_percent * dist)
            dist = max(dist, 0.001);
            large_grad_percent = max(large_grad_percent, 0.001);
            double cur_weight = large_grad_percent * dist / exp(-(abs(rect.angle) + score[i]) * degree2rad_scale * exp_weight_scale);
            if(cur_weight < weight){
                weight = cur_weight;
                ret_idx = i;
                _is_small_armor = is_small;
                //cout << "red:" << avg_red_side/side_total << "\tblue: "  << avg_blue_side/side_total << endl;
                //imwrite("armor_org.bmp", rectify_target(armor_rect));
            }
#ifdef COUT_LOG
            else
                cout << "refused 4: cur_weight: " << cur_weight << "\tweight threshold:" << weight << endl;
#endif
        }
#ifdef COUT_LOG
        else
            cout << "refused 3: (x_grad, y_grad): (" << avg_x << ", " << avg_y << ")\t avg_grad_threshold: " <<  (int)_para.avg_board_grad_threshold << endl;
#endif
    }
    //return ret_idx == -1 ? RotatedRect() : rects[ret_idx];
    if (ret_idx == -1){
        _is_lost = true;
        return RotatedRect();
    }
    _is_lost = false;
    // broaden the height of target
    RotatedRect ret_rect = rects[ret_idx];
    Rect ret_rect1 = ret_rect.boundingRect();
    if (broadenRect(ret_rect1, 3, 3, _src.size()) == false)
        return ret_rect;
    //rectangle(_src, ret_rect1, CV_RGB(255,128,128),2);
    //imshow("_src", _src);

    Mat s_b = _ec(ret_rect1);
    threshold(s_b, s_b, 128, 255, CV_THRESH_OTSU);
    vector<vector<Point2i> > contours_br;
	vector<Vec4i> hierarchy;
    findContours(s_b, contours_br, hierarchy, CV_RETR_EXTERNAL , CV_CHAIN_APPROX_SIMPLE);
    for (int k = 0; k < contours_br.size(); ++ k){
        Rect r = boundingRect(contours_br[k]);
        if (r.height > ret_rect.size.height){
            ret_rect.size.height = r.height;
            if(ret_rect.size.width / ret_rect.size.height < small_armor_wh_threshold)
                _is_small_armor = true;
        }
    }
    return ret_rect;
}

cv::RotatedRect ArmorDetector::getTargetAera(const cv::Mat & src){
	setImage(src);
	cv::Mat contrast_left, contrast_right;
    vector<vector<Point2i> > contours_left;
    vector<vector<Point2i> > contours_right;
	findContourInEnemyColor(contrast_left, contrast_right, contours_left, contours_right);
    vector<RotatedRect> rects;
    vector<double> score;
    findTargetInContours(contours_left, contours_right, rects, score);
    RotatedRect final_rect = chooseTarget(rects, score);

    //cout << "w:" << final_rect.size.width << ", h:" << final_rect.size.height << ", w/h:" << (double)final_rect.size.width/final_rect.size.height << endl;
#ifdef SHOW_DEBUG_IMG
    Mat rect_show;
    _src.copyTo(rect_show);
	for (size_t i = 0; i < rects.size(); i++)	{
		Scalar color(rand() & 255, rand() & 255, rand() & 255);
		Point2f vertices[4];
		rects[i].points(vertices);
		for (int i = 0; i < 4; i++)
			line(rect_show, vertices[i], vertices[(i + 1) % 4], color);
	}

	Point2f vertices[4];
	final_rect.points(vertices);
	for (int i = 0; i < 4; i++)
		line(rect_show, vertices[i], vertices[(i + 1) % 4], CV_RGB(255, 0, 0));
	imshow("5.rect", rect_show);
#endif

//    for (size_t i = 0; i < rects.size(); i++)	{
//        //Point offset = Point(_dect_rect.x, _dect_rect);
//        Scalar color(rand() & 255, rand() & 255, rand() & 255);
//        Point2f vertices[4];
//        rects[i].points(vertices);
//        for (int i = 0; i < 4; i++)
//            line(_src, vertices[i], vertices[(i + 1) % 4], color);
//    }
//    imshow("5.rect", _src);

    if(final_rect.size.width != 0){
        final_rect.center.x += _dect_rect.x;
        final_rect.center.y += _dect_rect.y;
        _res_last = final_rect;
        _lost_cnt = 0;
    }
    else{
        ++_lost_cnt;

        if (_lost_cnt < 3)
            _res_last.size =Size2f(_res_last.size.width * 2, _res_last.size.height * 1.5);
        else if(_lost_cnt == 6)
            _res_last.size =Size2f(_res_last.size.width * 1.5, _res_last.size.height * 1.5);
        else if(_lost_cnt == 12)
            _res_last.size =Size2f(_res_last.size.width * 1.5, _res_last.size.height * 1.5);
        else if(_lost_cnt == 18)
            _res_last.size =Size2f(_res_last.size.width * 1.5, _res_last.size.height * 1.5);
        else if (_lost_cnt > 60 )
            _res_last = RotatedRect();
    }
	return final_rect;
}

