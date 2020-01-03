/// @file	LowPassFilter2pDGhpp
/// @brief	A class to implement a second order low pass filter on a 2D marix

#pragma once

#include <matrix/math.hpp>

namespace math
{
class LowPassFilter2DG
{
public:

        LowPassFilter2DG(const float am[2][2], const float bm[2], const float cm[2], const float d)
	{
		// set initial parameters
                set_matrix(am, bm, cm, d);
	}

	// Change filter parameters
        void set_matrix(const float am[2][2], const float bm[2], const float cm[2], const float d);

	/**
	 * Add a new raw value to the filter
	 *
	 * @return retrieve the filtered result
	 */
	inline matrix::Vector3f apply(const matrix::Vector3f &sample)
	{
		// do the filtering
                const matrix::Vector3f element_1{_delay_element_1 *_a11 + _delay_element_2 * _a12 +_b1 * sample};
                const matrix::Vector3f element_2{_delay_element_1 *_a21 + _delay_element_2 * _a22 + _b2 * sample};

                const matrix::Vector3f output{element_1 *_c1 + element_2 *_c2 + sample * _d1};

                _delay_element_2 = element_2;
                _delay_element_1 = element_1;

		return output;
	}

	// Return the cutoff frequency
        //float get_cutoff_freq() const { return _cutoff_freq; }

	// Reset the filter state to this value
	matrix::Vector3f reset(const matrix::Vector3f &sample);

private:

        float _a11{0.0f};
        float _a12{0.0f};
        float _a21{0.0f};
        float _a22{0.0f};

	float _b1{0.0f};
	float _b2{0.0f};

        float _c1{0.0f};
        float _c2{0.0f};

        float _d1{0.0f};

	matrix::Vector3f _delay_element_1{0.0f, 0.0f, 0.0f};	// buffered sample -1
	matrix::Vector3f _delay_element_2{0.0f, 0.0f, 0.0f};	// buffered sample -2
};

} // namespace math
