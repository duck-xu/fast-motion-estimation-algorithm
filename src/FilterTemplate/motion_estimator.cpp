#include <unordered_map>
#include <algorithm>
#include <iterator>

#include "metric.hpp"
#include "motion_estimator.hpp"



MotionEstimator::MotionEstimator(int width, int height, uint8_t quality, bool use_half_pixel)
	: width(width)
	, height(height)
	, quality(quality)
	, use_half_pixel(use_half_pixel)
	, width_ext(width + 2 * BORDER)
	, num_blocks_hor((width + BLOCK_SIZE - 1) / BLOCK_SIZE)
	, num_blocks_vert((height + BLOCK_SIZE - 1) / BLOCK_SIZE)
	, first_row_offset(width_ext * BORDER + BORDER) {

	// PUT YOUR CODE HERE
	prev_vectors = new MV[num_blocks_hor * num_blocks_vert];
	mxvec = new int[num_blocks_hor * num_blocks_vert];
}

MotionEstimator::~MotionEstimator() {
	// PUT YOUR CODE HERE
	delete[] prev_vectors;
	delete[] mxvec;
}

void MotionEstimator::Estimate(const uint8_t* cur_Y,
                               const uint8_t* prev_Y,
                               const uint8_t* prev_Y_up,
                               const uint8_t* prev_Y_left,
                               const uint8_t* prev_Y_upleft,
                               MV* mvectors) {
	//std::unordered_map<ShiftDir, const uint8_t*> prev_map {
		//{ ShiftDir::NONE, prev_Y }
	//};
	/*
	if (use_half_pixel) {
		prev_map.emplace(ShiftDir::UP, prev_Y_up);
		prev_map.emplace(ShiftDir::LEFT, prev_Y_left);
		prev_map.emplace(ShiftDir::UPLEFT, prev_Y_upleft);
	}*/

	//ìåäèàííàÿ ôèëüòðàöèÿ î÷åíü ìåäëåííàÿ 
	/*
	int win = 3;
	int edge = win / 2;
	uint8_t *out;
	uint8_t *w;
	out = new uint8_t[width_ext * height];
	w = new uint8_t[win * win];
	for (int i = edge; i < height - edge; i++) {
		for (int j = edge; j < width_ext - edge; j++) {
			for (int fi = 0; fi < win; fi++) {
				for (int fj = 0; fj < win; fj++) {
					w[fi*win + fj] = cur_Y[(i + fi - edge) * width_ext + (j + fj - edge)];
				}
			}
			std::sort(w, w + win);
			out[i*width_ext + j] = w[edge * win + edge];
		}
	}
	*/
	
	for (int i = 0; i < num_blocks_vert; ++i) {
		for (int j = 0; j < num_blocks_hor; ++j) {
			const auto block_id = i * num_blocks_hor + j;
			const auto hor_offset = j * BLOCK_SIZE;
			const auto vert_offset = first_row_offset + i * BLOCK_SIZE * width_ext;
			const auto cur = cur_Y + vert_offset + hor_offset;

			const auto prev = prev_Y + vert_offset + hor_offset;

			MV best_vector;
			best_vector.x = 0;
			best_vector.y = 0;
			best_vector.shift_dir = ShiftDir::NONE;
			best_vector.error = GetErrorSAD_16x16(cur, prev, width_ext);

			// Local function to shorten syntax. Returns bool: if the checked vector has smaller error
			const int &width_local = width_ext;
			auto verror = [&cur, &width_local, &best_vector](const uint8_t *prev, int y, int x) -> bool {
				const uint8_t *comp = prev + y * width_local + x;
				long elem = GetErrorSAD_16x16(cur, comp, width_local);
				if (elem < best_vector.error) {
					best_vector.x = x;
					best_vector.y = y;
					best_vector.error = elem;
					return true;
				}
				return false;
			};





			if (best_vector.error < threshold) {
				mvectors[block_id] = best_vector;
				prev_vectors[block_id] = best_vector;
				mxvec[block_id] = std::max(abs(best_vector.y), abs(best_vector.x));
				continue;
			}

			const int block_id_up = (i - 1) * num_blocks_hor + j;
			const int block_id_lf = i * num_blocks_hor + j - 1;

			// Set step size
			int step_size = 0;
			if (i == 0 && j == 0) {
				step_size = 3;
			}
			else {
				int max = 0;
				if (i > 0) {
					max = mxvec[block_id_up];
				}
				if (j > 0) {
					max = std::max(max, mxvec[block_id_lf]);
				}

				if (!first && quality >= 40) {
					step_size = std::max(abs(prev_vectors[block_id].x), abs(prev_vectors[block_id].y));
					step_size = std::max(step_size, max);
				}
				else {
					step_size = max;
				}
			}

			// Prediction by previous and current computed motion vectors
			if (i > 0) {
				verror(prev, mvectors[block_id_up].y, mvectors[block_id_up].x);
			}
			if (j > 0) {
				verror(prev, mvectors[block_id_lf].y, mvectors[block_id_lf].x);
			}
			if (step_size >= 4) {
				if (quality >= 40) {
					if (!first) {
						verror(prev, prev_vectors[block_id].y, prev_vectors[block_id].x);
					}
				}
			}
			else if (step_size >= 2) { // Step size in [2, 3]
				int rood_y[] = { 1, 0, -1, 0 };
				int rood_x[] = { 0, 1, 0, -1 };
				for (int h = 0; h < 4; h++) {
					int y = (rood_y[h] + best_vector.y) * step_size;
					int x = (rood_x[h] + best_vector.x) * step_size;
					if (-BORDER <= x && x <= BORDER && -BORDER <= y && y <= BORDER) {
						verror(prev, y, x);
					}
				}

				// extra
				if (quality >= 60) {
					if (!first) {
						verror(prev, prev_vectors[block_id].y, prev_vectors[block_id].x);
					}
				}
			}

			// Search with step size 1
			bool flag = true;

			int search_lim = 4;
			int border_lim = BORDER;
			bool launch_search = true;

			if (quality >= 100) {
				search_lim = 4;
			}
			else if (quality >= 20) {
				search_lim = 4;
			}
			else {
				border_lim = BORDER / 2;
				launch_search = threshold < best_vector.error && best_vector.error < 2 * threshold;
			}

			if (launch_search) {
				while (flag) {
					flag = false;
					int rood_y[] = { 1, 0, -1, 0, 1, -1, -1, 1 };
					int rood_x[] = { 0, 1, 0, -1, 1, 1, -1, -1 };
					for (int h = 0; h < search_lim; h++) {
						int y = rood_y[h] + best_vector.y;
						int x = rood_x[h] + best_vector.x;
						if (-border_lim <= x && x <= border_lim && -border_lim <= y && y <= border_lim) {
							// verror returns and produces a side effect
							flag |= verror(prev, y, x);
						}
					}
				}
			}




			//halfpixel
			if (use_half_pixel) {
				if (j != 0) {
					const auto half_prev = prev_Y_up + vert_offset + hor_offset;
					if (verror(half_prev, best_vector.y, best_vector.x)) {
						best_vector.shift_dir = ShiftDir::UP;
					}
				}
				if (i != 0) {
					const auto half_prev = prev_Y_left + vert_offset + hor_offset;
					if (verror(half_prev, best_vector.y, best_vector.x)) {
						best_vector.shift_dir = ShiftDir::LEFT;
					}
				}
				if (j != 0 && i != 0) {
					auto half_prev = prev_Y_upleft + vert_offset + hor_offset;
					if (verror(half_prev, best_vector.y, best_vector.x)) {
						best_vector.shift_dir = ShiftDir::UPLEFT;
					}
				}
			}

			mvectors[block_id] = best_vector;
			prev_vectors[block_id] = best_vector;
			mxvec[block_id] = std::max(abs(best_vector.y), abs(best_vector.x));
		}
	}

	if (first) {
		first = false;
	}
	//delete[] w;
	//delete[] out;
}
