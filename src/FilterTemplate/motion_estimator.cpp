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
}

MotionEstimator::~MotionEstimator() {
	// PUT YOUR CODE HERE
	delete[] prev_vectors;
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

	//медианная фильтрация очень медленная 
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

			MV best_vector;
			best_vector.error = std::numeric_limits<long>::max();

			// PUT YOUR CODE HERE
			for (int u = 0; u < 1; u++) {
				const auto prev = prev_Y + vert_offset + hor_offset;
				best_vector.x = 0;
				best_vector.y = 0;
				best_vector.shift_dir = ShiftDir::NONE;
				best_vector.error = GetErrorSAD_16x16(cur, prev, width_ext);

				if (best_vector.error < threshold) {
					break;
				}

				const auto block_id_up = (i - 1) * num_blocks_hor + j;
				const auto block_id_lf = i * num_blocks_hor + j - 1;

				int step_size = 0;
				if (i == 0 && j == 0) {
					step_size = 3;
				}
				else {
					int max = 0;
					if (i == 0) {
						max = std::max(abs(mvectors[block_id_lf].x), abs(mvectors[block_id_lf].y));
					}
					else if (j == 0) {
						max = std::max(abs(mvectors[block_id_up].x), abs(mvectors[block_id_up].y));
					}
					else {
						max = std::max({ abs(mvectors[block_id_up].x), abs(mvectors[block_id_up].y), abs(mvectors[block_id_lf].x), abs(mvectors[block_id_lf].y) });
					}
					if (!first && quality >= 40) {
						step_size = std::max(abs(prev_vectors[block_id].x), abs(prev_vectors[block_id].y));
						step_size = std::max(step_size, max);
					} 
					else {
						step_size = max;
					}
				}

				if (step_size >= 4) {
					if (quality >= 40) {
						if (!first) {
							const auto comp = prev + prev_vectors[block_id].y * width_ext + prev_vectors[block_id].x;
							int error = GetErrorSAD_16x16(cur, comp, width_ext);
							if (error < best_vector.error) {
								best_vector.x = prev_vectors[block_id].x;
								best_vector.y = prev_vectors[block_id].y;
								best_vector.shift_dir = ShiftDir::NONE;
								best_vector.error = error;
							}
						}
					}
					if (!(i == 0 && j == 0)) {
						if (i == 0) {
							const auto comp = prev + mvectors[block_id_lf].y * width_ext + mvectors[block_id_lf].x;
							int error = GetErrorSAD_16x16(cur, comp, width_ext);
							if (error < best_vector.error) {
								best_vector.x = mvectors[block_id_lf].x;
								best_vector.y = mvectors[block_id_lf].y;
								best_vector.shift_dir = ShiftDir::NONE;
								best_vector.error = error;
							}
						}
						else if (j == 0) {
							const auto comp = prev + mvectors[block_id_up].y * width_ext + mvectors[block_id_up].x;
							int error = GetErrorSAD_16x16(cur, comp, width_ext);
							if (error < best_vector.error) {
								best_vector.x = mvectors[block_id_up].x;
								best_vector.y = mvectors[block_id_up].y;
								best_vector.shift_dir = ShiftDir::NONE;
								best_vector.error = error;
							}
						}
						else {
							const auto comp1 = prev + mvectors[block_id_up].y * width_ext + mvectors[block_id_up].x;
							int error1 = GetErrorSAD_16x16(cur, comp1, width_ext);
							const auto comp2 = prev + mvectors[block_id_lf].y * width_ext + mvectors[block_id_lf].x;
							int error2 = GetErrorSAD_16x16(cur, comp2, width_ext);
							if (error1 < best_vector.error) {
								if (error2 < error1) {
									best_vector.x = mvectors[block_id_lf].x;
									best_vector.y = mvectors[block_id_lf].y;
									best_vector.shift_dir = ShiftDir::NONE;
									best_vector.error = error2;
								}
								else {
									best_vector.x = mvectors[block_id_up].x;
									best_vector.y = mvectors[block_id_up].y;
									best_vector.shift_dir = ShiftDir::NONE;
									best_vector.error = error1;
								}
							}
							else {
								if (error2 < best_vector.error) {
									best_vector.x = mvectors[block_id_lf].x;
									best_vector.y = mvectors[block_id_lf].y;
									best_vector.shift_dir = ShiftDir::NONE;
									best_vector.error = error2;
								}
							}
						}
					}
				}
				else if (step_size >= 2) {
					int error = 0;
					for (int h = 0; h < 4; h++) {
						int x = 0, y = 0;
						switch (h) {
						case 0:
							y = step_size;
							break;
						case 1:
							y = -step_size;
							break;
						case 2:
							x = step_size;
							break;
						case 3:
							x = -step_size;
							break;
						}
						x += best_vector.x;
						y += best_vector.y;
						const auto comp = prev + y * width_ext + x;
						error = GetErrorSAD_16x16(cur, comp, width_ext);
						if (error < best_vector.error) {
							best_vector.x = x;
							best_vector.y = y;
							best_vector.shift_dir = ShiftDir::NONE;
							best_vector.error = error;
						}
					}
					//extra
					if ( quality >= 60) {
						if (!first) {
							int px = prev_vectors[block_id].x;
							int py = prev_vectors[block_id].y;
							auto comp = prev + py * width_ext + px;
							error = GetErrorSAD_16x16(cur, comp, width_ext);
							if (error < best_vector.error) {
								best_vector.x = px;
								best_vector.y = py;
								best_vector.shift_dir = ShiftDir::NONE;
								best_vector.error = error;
							}
						}
						if (!(i == 0 && j == 0)) {
							if (i == 0) {
								auto comp = prev + mvectors[block_id_lf].y * width_ext + mvectors[block_id_lf].x;
								error = GetErrorSAD_16x16(cur, comp, width_ext);
								if (error < best_vector.error) {
									best_vector.x = mvectors[block_id_lf].x;
									best_vector.y = mvectors[block_id_lf].y;
									best_vector.shift_dir = ShiftDir::NONE;
									best_vector.error = error;
								}
							}
							else if (j == 0) {
								auto comp = prev + mvectors[block_id_up].y * width_ext + mvectors[block_id_up].x;
								error = GetErrorSAD_16x16(cur, comp, width_ext);
								if (error < best_vector.error) {
									best_vector.x = mvectors[block_id_up].x;
									best_vector.y = mvectors[block_id_up].y;
									best_vector.shift_dir = ShiftDir::NONE;
									best_vector.error = error;
								}
							}
							else {
								auto comp1 = prev + mvectors[block_id_up].y * width_ext + mvectors[block_id_up].x;
								int error1 = GetErrorSAD_16x16(cur, comp1, width_ext);
								auto comp2 = prev + mvectors[block_id_lf].y * width_ext + mvectors[block_id_lf].x;
								int error2 = GetErrorSAD_16x16(cur, comp2, width_ext);
								if (error1 < best_vector.error) {
									if (error2 < error1) {
										best_vector.x = mvectors[block_id_lf].x;
										best_vector.y = mvectors[block_id_lf].y;
										best_vector.shift_dir = ShiftDir::NONE;
										best_vector.error = error2;
									}
									else {
										best_vector.x = mvectors[block_id_up].x;
										best_vector.y = mvectors[block_id_up].y;
										best_vector.shift_dir = ShiftDir::NONE;
										best_vector.error = error1;
									}
								}
								else {
									if (error2 < best_vector.error) {
										best_vector.x = mvectors[block_id_lf].x;
										best_vector.y = mvectors[block_id_lf].y;
										best_vector.shift_dir = ShiftDir::NONE;
										best_vector.error = error2;
									}
								}
							}
						}
					}
				}
				bool flag = true;
				if (quality >= 100) {
					while (flag) {
						flag = false;
						for (int h = 0; h < 4; h++) {
							int x = 0, y = 0;
							step_size = 1;
							switch (h) {
							case 0:
								y = step_size;
								break;
							case 1:
								y = -step_size;
								break;
							case 2:
								x = step_size;
								break;
							case 3:
								x = -step_size;
								break;
							case 4:
								x = step_size;
								y = step_size;
								break;
							case 5:
								x = step_size;
								y = -step_size;
								break;
							case 6:
								x = -step_size;
								y = step_size;
								break;
							case 7:
								x = -step_size;
								y = -step_size;
								break;
							}
							x += best_vector.x;
							y += best_vector.y;
							if (x >= BORDER || x <= -BORDER || y >= BORDER || y <= -BORDER)
								break;
							const auto comp = prev + y * width_ext + x;
							int error = GetErrorSAD_16x16(cur, comp, width_ext);
							if (error < best_vector.error) {
								flag = true;
								best_vector.x = x;
								best_vector.y = y;
								best_vector.shift_dir = ShiftDir::NONE;
								best_vector.error = error;
							}
						}
					}
				} else if (quality >= 20) {
					while (flag) {
						flag = false;
						for (int h = 0; h < 4; h++) {
							int x = 0, y = 0;
							step_size = 1;
							switch (h) {
							case 0:
								y = step_size;
								break;
							case 1:
								y = -step_size;
								break;
							case 2:
								x = step_size;
								break;
							case 3:
								x = -step_size;
								break;
							}
							x += best_vector.x;
							y += best_vector.y;
							if (x >= BORDER || x <= -BORDER || y >= BORDER || y <= -BORDER)
								break;
							const auto comp = prev + y * width_ext + x;
							int error = GetErrorSAD_16x16(cur, comp, width_ext);
							if (error < best_vector.error) {
								flag = true;
								best_vector.x = x;
								best_vector.y = y;
								best_vector.shift_dir = ShiftDir::NONE;
								best_vector.error = error;
							}
						}
					}
				}
				else {
					if (best_vector.error < 2 * threshold && best_vector.error > threshold) {
						while (flag) {
							flag = false;
							for (int h = 0; h < 4; h++) {
								int x = 0, y = 0;
								step_size = 1;
								switch (h) {
								case 0:
									y = step_size;
									break;
								case 1:
									y = -step_size;
									break;
								case 2:
									x = step_size;
									break;
								case 3:
									x = -step_size;
									break;
								}
								x += best_vector.x;
								y += best_vector.y;
								int b = BORDER / 2;
								if (x >= b || x <= -b || y >= b || y <= -b)
									break;
								const auto comp = prev + y * width_ext + x;
								int error = GetErrorSAD_16x16(cur, comp, width_ext);
								if (error < best_vector.error) {
									flag = true;
									best_vector.x = x;
									best_vector.y = y;
									best_vector.shift_dir = ShiftDir::NONE;
									best_vector.error = error;
								}
							}
						}
					}
				}
			}

			//halfpixel
			if (use_half_pixel) {
				if (j != 0) {
					auto prev = prev_Y_up + vert_offset + hor_offset;
					auto comp = prev + best_vector.y * width_ext + best_vector.x;
					int error = GetErrorSAD_16x16(cur, comp, width_ext);
					if (error < best_vector.error) {
						best_vector.shift_dir = ShiftDir::UP;
						best_vector.error = error;
					}
				}
				if (i != 0) {
					auto prev = prev_Y_left + vert_offset + hor_offset;
					auto comp = prev + best_vector.y * width_ext + best_vector.x;
					int  error = GetErrorSAD_16x16(cur, comp, width_ext);
					if (error < best_vector.error) {
						best_vector.shift_dir = ShiftDir::LEFT;
						best_vector.error = error;
					}
				}
				if (j != 0 && i != 0) {
					auto prev = prev_Y_upleft + vert_offset + hor_offset;
					auto comp = prev + best_vector.y * width_ext + best_vector.x;
					int error = GetErrorSAD_16x16(cur, comp, width_ext);
					if (error < best_vector.error) {
						best_vector.shift_dir = ShiftDir::UPLEFT;
						best_vector.error = error;
					}
				}
				/*if (j != num_blocks_hor - 1) {
					auto prev = prev_Y_up + vert_offset + hor_offset;
					auto comp = prev + (best_vector.y - 1) * width_ext + best_vector.x;
					int  error = GetErrorSAD_16x16(cur, comp, width_ext);
					if (error < best_vector.error) {
						best_vector.y--;
						best_vector.shift_dir = ShiftDir::UP;
						best_vector.error = error;
					}
				}
				if (i != num_blocks_vert - 1) {
					auto prev = prev_Y_left + vert_offset + hor_offset;
					auto comp = prev + best_vector.y * width_ext + best_vector.x - 1;
					int  error = GetErrorSAD_16x16(cur, comp, width_ext);
					if (error < best_vector.error) {
						best_vector.x--;
						best_vector.shift_dir = ShiftDir::LEFT;
						best_vector.error = error;
					}
				}
				if (quality >= 80) {
					if (j != 0 && i != 0) {
						auto prev = prev_Y_upleft + vert_offset + hor_offset;
						auto comp = prev + best_vector.y * width_ext + best_vector.x;
						int error = GetErrorSAD_16x16(cur, comp, width_ext);
						if (error < best_vector.error) {
							best_vector.shift_dir = ShiftDir::UPLEFT;
							best_vector.error = error;
						}
					}
					if (j != num_blocks_hor - 1) {
						auto prev = prev_Y_upleft + vert_offset + hor_offset;
						auto comp = prev + (best_vector.y - 1) * width_ext + best_vector.x;
						int  error = GetErrorSAD_16x16(cur, comp, width_ext);
						if (error < best_vector.error) {
							best_vector.y--;
							best_vector.shift_dir = ShiftDir::UPLEFT;
							best_vector.error = error;
						}
					}
					if (i != num_blocks_vert - 1) {
						auto prev = prev_Y_upleft + vert_offset + hor_offset;
						auto comp = prev + best_vector.y * width_ext + best_vector.x - 1;
						int  error = GetErrorSAD_16x16(cur, comp, width_ext);
						if (error < best_vector.error) {
							best_vector.x--;
							best_vector.shift_dir = ShiftDir::UPLEFT;
							best_vector.error = error;
						}
					}
					if (i != num_blocks_vert - 1 && j != num_blocks_hor - 1) {
						auto prev = prev_Y_upleft + vert_offset + hor_offset;
						auto comp = prev + (best_vector.y - 1) * width_ext + best_vector.x - 1;
						int  error = GetErrorSAD_16x16(cur, comp, width_ext);
						if (error < best_vector.error) {
							best_vector.x--;
							best_vector.y--;
							best_vector.shift_dir = ShiftDir::UPLEFT;
							best_vector.error = error;
						}
					}
				}*/
			}

			mvectors[block_id] = best_vector;
			prev_vectors[block_id] = best_vector;
		}
	}
	if (first) {
		first = false;
	}
	//delete[] w;
	//delete[] out;
}
