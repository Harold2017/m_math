//
// Created by Harold on 2021/7/6.
//

/*
 * Definitions from Octave doc:
 *
 * Regional minima should not be mistaken with local minima. 
 * Local minima are pixels whose value is less or equal to all of its neighbors. 
 * A regional minima is the connected component of pixels whose values are all less than the neighborhood of the minima (the connected component, not its individual pixels). 
 * All pixels belonging to a regional minima are local minima, but the inverse is not true.
 */

// code from: https://stackoverflow.com/questions/11205025/imregionalmax-matlab-functions-equivalent-in-opencv

#ifndef M_MATH_M_IMREGION_MIN_H
#define M_MATH_M_IMREGION_MIN_H

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace M_MATH {
    namespace detail {
        int inline neighborCleanup(float* in, uchar* out, int i, int x, int y, int x_lim, int y_lim)
        {
            int index;
            for (int xx = x - 1; xx < x + 2; ++xx) {
                for (int yy = y - 1; yy < y + 2; ++yy) {
                    if (((xx == x) && (yy==y)) || xx < 0 || yy < 0 || xx >= x_lim || yy >= y_lim)
                        continue;
                    index = xx*y_lim + yy;
                    if ((in[i] == in[index]) && (out[index] == 0))
                        return 1;
                }
            }

            return 0;
        }

        void inline neighborCheck(float* in, uchar* out, int i, int x, int y, int x_lim)
        {   
            int indexes[8], cur_index;
            indexes[0] = x*x_lim + y;
            indexes[1] = x*x_lim + y+1;
            indexes[2] = x*x_lim + y+2;
            indexes[3] = (x+1)*x_lim + y+2;
            indexes[4] = (x + 2)*x_lim + y+2;
            indexes[5] = (x + 2)*x_lim + y + 1;
            indexes[6] = (x + 2)*x_lim + y;
            indexes[7] = (x + 1)*x_lim + y;
            cur_index = (x + 1)*x_lim + y+1;

            for (int t = 0; t < 8; t++) {
                if (in[indexes[t]] < in[cur_index]) {
                    out[i] = 0;
                    break;
                }
            }

            if (out[i] == 3)
                out[i] = 1;
        }
    }

    //  output is a binary image
    //  1: not a min region
    //  0: part of a min region
    //  2: not sure if min or not
    //  3: uninitialized
    void imregionalmin(cv::Mat& img, cv::Mat& out_img)
    {
        // pad the border of img with 1 and copy to img_pad
        cv::Mat img_pad;
        cv::copyMakeBorder(img, img_pad, 1, 1, 1, 1, cv::BORDER_CONSTANT, 1);

        //  initialize binary output to 2, unknown if min
        out_img = cv::Mat::ones(img.rows, img.cols, CV_8U)+2;

        //  initialize pointers to matrices
        float* in = (float *)(img_pad.data);
        uchar* out = (uchar *)(out_img.data);

        //  size of matrix
        int in_size = img_pad.cols*img_pad.rows;
        int out_size = img.cols*img.rows;

        int x, y;
        for (int i = 0; i < out_size; i++) {
            //  find x, y indexes
            y = i % img.cols;
            x = i / img.cols;

            detail::neighborCheck(in, out, i, x, y, img_pad.cols);  //  all regions are either min or max
        }

        cv::Mat label;
        cv::connectedComponents(out_img, label);

        int* lab = (int *)(label.data);

        in = (float *)(img.data);
        in_size = img.cols*img.rows;

        std::vector<int> bad_labels;

        for (int i = 0; i < out_size; i++) {
            //  find x, y indexes
            y = i % img.cols;
            x = i / img.cols;

            if (lab[i] != 0) {
                if (detail::neighborCleanup(in, out, i, x, y, img.rows, img.cols) == 1) {
                    bad_labels.push_back(lab[i]);
                }
            }
        }

        std::sort(bad_labels.begin(), bad_labels.end());
        bad_labels.erase(std::unique(bad_labels.begin(), bad_labels.end()), bad_labels.end());

        for (int i = 0; i < out_size; ++i) {
            if (lab[i] != 0) {
                if (std::find(bad_labels.begin(), bad_labels.end(), lab[i]) != bad_labels.end()) {
                    out[i] = 0;
                }
            }
        }
    }
}

#endif //M_MATH_M_IMREGION_MIN_H