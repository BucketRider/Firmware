#include "LowPassFilter3DG.hpp"

#include <px4_defines.h>

#include <cmath>

namespace math
{

void LowPassFilter3DG::set_matrix(const float am[3][3],const  float bm[3], const float cm[3], const float d)
{

	// reset delay elements on filter change
	_delay_element_1.zero();
	_delay_element_2.zero();
    _delay_element_3.zero();

        _a11 = am[0][0];
        _a12 = am[0][1];
        _a13 = am[0][2];
        _a21 = am[1][0];
        _a22 = am[1][1];
        _a23 = am[1][2];
        _a31 = am[2][0];
        _a32 = am[2][1];
        _a33 = am[2][2];

        _b1 = bm[0];
        _b2 = bm[1];
        _b3 = bm[2];

        _c1 = cm[0];
        _c2 = cm[1];
        _c3 = cm[2];

        _d1 = d;

}

matrix::Vector3f LowPassFilter3DG::reset(const matrix::Vector3f &sample)
{
        const matrix::Vector3f dval = sample / (_c1 + _c2 + _c3 + _d1);

	if (PX4_ISFINITE(dval(0)) && PX4_ISFINITE(dval(1)) && PX4_ISFINITE(dval(2))) {
		_delay_element_1 = dval;
		_delay_element_2 = dval;
        _delay_element_3 = dval;

	} else {
		_delay_element_1 = sample;
		_delay_element_2 = sample;
        _delay_element_3 = sample;
	}

	return apply(sample);
}

} // namespace math
