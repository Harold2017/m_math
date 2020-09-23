//
// Created by Harold on 2020/9/23.
//

#ifndef M_MATH_M_PEAK_H
#define M_MATH_M_PEAK_H

#include <vector>
#include <algorithm>
#include <cmath>

namespace M_MATH {
    class Peak1D {
    public:
        constexpr static float EPS = 2.2204e-16f;
        template<typename T>
        static void find_peaks(std::vector<T> x0, std::vector<int>& peak_idx_vec, double sel = 0);

    private:
        template<typename T>
        static void diff(std::vector<T> const& in, std::vector<T>& out) {
            out.resize(in.size()-1);
            for(int i=1; i<in.size(); ++i)
                out[i-1] = in[i] - in[i-1];
        }

        template<typename U>
        static void select_elements(std::vector<U> const& in, std::vector<int> const& indices, std::vector<U>& out) {
            for(int idx : indices)
                out.push_back(in[idx]);
        }
    };

    template<typename T>
    void Peak1D::find_peaks(std::vector<T> x0, std::vector<int>& peak_idx_vec, double sel) {
        int minIdx = std::distance(x0.begin(), std::min_element(x0.begin(), x0.end()));
        int maxIdx = std::distance(x0.begin(), std::max_element(x0.begin(), x0.end()));

        if (sel < EPS)
            sel = (x0[maxIdx]-x0[minIdx])/4.0;

        int len0 = x0.size();

        std::vector<T> dx;
        diff(x0, dx);
        replace(dx.begin(), dx.end(), 0.0f, -EPS);
        std::vector<T> dx0(dx.begin(), dx.end()-1);
        std::vector<T> dx1(dx.begin()+1, dx.end());
        // product to find sign changes
        std::vector<T> dx2(dx0.size());
        for(int i=0; i<dx0.size(); ++i)
            dx2[i] = dx0[i] * dx1[i];

        // find where the derivative changes sign
        std::vector<int> ind;
        for(int i=0; i<dx2.size(); ++i)
            if(dx2[i]<0.f)
                ind.push_back(i+1);

        std::vector<T> x;

        std::vector<int> indAux(ind.begin(), ind.end());
        select_elements(x0, indAux, x);
        x.insert(x.begin(), x0[0]);
        x.insert(x.end(), x0[x0.size()-1]);


        ind.insert(ind.begin(), 0);
        ind.insert(ind.end(), len0);

        T minMag = *(std::min_element(x.begin(), x.end()));
        T leftMin = minMag;
        int len = x.size();

        if(len>2)
        {
            T tempMag = minMag;
            bool foundPeak = false;
            int ii;

            std::vector<T> xSub0(x.begin(), x.begin()+3);
            std::vector<T> xDiff;
            diff(xSub0, xDiff);

            // get sign vector
            std::vector<int> signDx(xDiff.size());
            for(int i=0; i<xDiff.size(); ++i) {
                if(xDiff[i]>0)
                    signDx[i]=1;
                else if(xDiff[i]<0)
                    signDx[i]=-1;
                else
                    signDx[i]=0;
            }

            // The first point is larger or equal to the second
            if (signDx[0] <= 0) {
                // Want alternating signs
                if (signDx[0] == signDx[1]) {
                    x.erase(x.begin()+1);
                    ind.erase(ind.begin()+1);
                    len = len-1;
                }
            }
                // First point is smaller than the second
            else {
                // Want alternating signs
                if (signDx[0] == signDx[1]) {
                    x.erase(x.begin());
                    ind.erase(ind.begin());
                    len = len-1;
                }
            }

            if ( x[0] >= x[1] )
                ii = 0;
            else
                ii = 1;

            T maxPeaks = std::ceil((T)len/2.0);
            std::vector<int> peakLoc(maxPeaks,0);
            int cInd = 1;
            int tempLoc;

            while(ii < len)
            {
                ii = ii+1;//This is a peak
                //Reset peak finding if we had a peak and the next peak is bigger
                //than the last or the left min was small enough to reset.
                if(foundPeak) {
                    tempMag = minMag;
                    foundPeak = false;
                }

                //Found new peak that was lager than temp mag and selectivity larger
                //than the minimum to its left.

                if( x[ii-1] > tempMag && x[ii-1] > leftMin + sel ) {
                    tempLoc = ii-1;
                    tempMag = x[ii-1];
                }

                //Make sure we don't iterate past the length of our std::vector
                if(ii == len)
                    break; //We assign the last point differently out of the loop

                ii = ii+1; // Move onto the valley

                //Come down at least sel from peak
                if(tempMag > sel + x[ii-1]) {
                    foundPeak = true; //We have found a peak
                    leftMin = x[ii-1];
                    peakLoc[cInd-1] = tempLoc; // Add peak to index
                    cInd = cInd+1;
                }
                else if(x[ii-1] < leftMin) // New left minima
                    leftMin = x[ii-1];

            }

            // Check end point
            if ( x[x.size()-1] > tempMag && x[x.size()-1] > leftMin + sel ) {
                peakLoc[cInd-1] = len-1;
                cInd = cInd + 1;
            }
                // Check if we still need to add the last point
            else if( !foundPeak && tempMag > minMag ) {
                peakLoc[cInd-1] = tempLoc;
                cInd = cInd + 1;
            }

            //Create output
            if( cInd > 0 ) {
                std::vector<int> peakLocTmp(peakLoc.begin(), peakLoc.begin()+cInd-1);
                select_elements(ind, peakLocTmp, peak_idx_vec);
            }
        }
    }
}

#endif //M_MATH_M_PEAK_H
