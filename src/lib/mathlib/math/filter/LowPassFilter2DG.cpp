#include "LowPassFilter2DG.hpp"

#include <px4_defines.h>

#include <cmath>

namespace math
{

void LowPassFilter2DG::set_matrix(const float am[2][2], const float bm[2], const float cm[2], const float d)
{

	// reset delay elements on filter change
	_delay_element_1.zero();
	_delay_element_2.zero();

        _a11 = am[0][0];
        _a12 = am[0][1];
        _a21 = am[1][0];
        _a22 = am[1][1];

        _b1 = bm[0];
        _b2 = bm[1];

        _c1 = cm[0];
        _c2 = cm[1];

        _d1 = d;

}

matrix::Vector3f LowPassFilter2DG::reset(const matrix::Vector3f &sample)
{
        const matrix::Vector3f dval = sample / (_c1 + _c2 +_d1);

	if (PX4_ISFINITE(dval(0)) && PX4_ISFINITE(dval(1)) && PX4_ISFINITE(dval(2))) {
		_delay_element_1 = dval;
		_delay_element_2 = dval;

	} else {
		_delay_element_1 = sample;
		_delay_element_2 = sample;
	}

	return apply(sample);
}

} // namespace math
