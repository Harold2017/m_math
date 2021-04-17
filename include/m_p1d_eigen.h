//
// Created by Harold on 2021/4/16.
//

#ifndef M_MATH_M_P1D_EIGEN_HPP
#define M_MATH_M_P1D_EIGEN_HPP

#include <vector>
#include <numeric>
#include <Eigen/Core>
#include <ostream>

// reference: https://www.sthu.org/blog/13-perstopology-peakdetection/index.html
namespace M_MATH {
	struct Peak {
		int born, died, left, right;  // born indicates peak (local maxima) idx in seq
		explicit Peak(int startIdx) { born = left = right = startIdx; died = -1; }

		template<typename T>
		T get_persistence(std::vector<T> seq) {
			return died == -1 ? std::numeric_limits<T>::infinity() : seq[born] - seq[died];
		}

		friend std::ostream& operator<<(std::ostream& os, Peak const& peak) {
			os << "{ " << peak.born << ", " << peak.died << ", " << peak.left << ", " << peak.right << " }";
			return os;
		}
	};

	template<typename T>
	std::vector<Peak> get_persistent_homology(std::vector<T> const& seq) {
		auto N = seq.size();
		std::vector<Peak> peaks;
		peaks.reserve(N);
		// maps indices to peaks
		std::vector<int> idxtopeaks(N, -1);
		// sequence indices sorted by values
		Eigen::VectorXi indices = Eigen::VectorXi::LinSpaced(N, 0, N-1);
		std::sort(indices.data(), indices.data() + N, [&](int i, int j) { return seq[i] > seq[j]; });
		// process each sample in descending order
		bool leftdone, rightdone;
		int il, ir, idx;
		for (auto i = 0; i < N; ++i) {
			idx = indices[i];
			leftdone = idx > 0 && idxtopeaks[idx - 1] != -1;
			rightdone = idx < N - 1 && idxtopeaks[idx + 1] != -1;
			il = leftdone ? idxtopeaks[idx - 1] : -1;
			ir = rightdone ? idxtopeaks[idx + 1] : -1;
			// new peak born
			if (!leftdone && !rightdone) {
				peaks.push_back(Peak(idx));
				idxtopeaks[idx] = peaks.size() - 1;
			}
			// directly merge to next peak left
			if (leftdone && !rightdone) {
				peaks[il].right += 1;
				idxtopeaks[idx] = il;
			}
			// directly merge to next peak right
			if (!leftdone && rightdone) {
				peaks[ir].left -= 1;
				idxtopeaks[idx] = ir;
			}
			// merge left and right peaks
			if (leftdone && rightdone)
				// left was born earlier: merge right to left
				if (seq[peaks[il].born] > seq[peaks[ir].born]) {
					peaks[ir].died = idx;
					peaks[il].right = peaks[ir].right;
					idxtopeaks[peaks[il].right] = idxtopeaks[idx] = il;
				}
				else {
					peaks[il].died = idx;
					peaks[ir].left = peaks[il].left;
					idxtopeaks[peaks[ir].left] = idxtopeaks[idx] = ir;
				}
		}

		std::sort(peaks.begin(), peaks.end(), [&](Peak& p1, Peak& p2) { return p1.get_persistence(seq) > p2.get_persistence(seq); });
		return peaks;
	}
}

#endif //M_MATH_M_P1D_EIGEN_HPP