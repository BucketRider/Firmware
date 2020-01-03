/// @file	LowPassFilter2pDGhpp
/// @brief	A class to implement a second order low pass filter on a 2D marix

#pragma once

#include <matrix/math.hpp>

namespace math
{
class LowPassFilter3DG
{
public:

        LowPassFilter3DG(const float am[3][3], const float bm[3], const float cm[3], const float d)
	{
		// set initial parameters
                set_matrix(am, bm, cm, d);
	}

	// Change filter parameters
        void set_matrix(const float am[3][3], const float bm[3], const float cm[3], const float d);

	/**
	 * Add a new raw value to the filter
	 *
	 * @return retrieve the filtered result
	 */
	inline matrix::Vector3f apply(const matrix::Vector3f &sample)
	{
		// do the filtering
                const matrix::Vector3f element_1{_delay_element_1 *_a11 + _delay_element_2 * _a12
                                                             +_delay_element_3 * _a13 +_b1 * sample};
                const matrix::Vector3f element_2{_delay_element_1 *_a21 + _delay_element_2 * _a22
                                                             +_delay_element_3 * _a23 +_b2 * sample};
                const matrix::Vector3f element_3{_delay_element_1 *_a31 + _delay_element_2 * _a32
                                                             +_delay_element_3 * _a33 +_b3 * sample};
                const matrix::Vector3f output{element_1 *_c1 + element_2 *_c2 +  element_3 *_c3 + sample * _d1};

                _delay_element_3 = element_3;
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
        float _a13{0.0f};
        float _a21{0.0f};
        float _a22{0.0f};
        float _a23{0.0f};
        float _a31{0.0f};
        float _a32{0.0f};
        float _a33{0.0f};

	float _b1{0.0f};
	float _b2{0.0f};
        float _b3{0.0f};

        float _c1{0.0f};
        float _c2{0.0f};
        float _c3{0.0f};

        float _d1{0.0f};

	matrix::Vector3f _delay_element_1{0.0f, 0.0f, 0.0f};	// buffered sample -1
	matrix::Vector3f _delay_element_2{0.0f, 0.0f, 0.0f};	// buffered sample -2
        matrix::Vector3f _delay_element_3{0.0f, 0.0f, 0.0f};	// buffered sample -3
};

} // namespace math
