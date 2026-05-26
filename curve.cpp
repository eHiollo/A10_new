#include "curve.hpp"
#include<cmath>
#include<iostream>
using namespace std;


auto TCurve2::getCurveParam()->void
{
	this->ta_ = v_ / a_;
	this->pa_ = 0.5 * a_ * ta_ * ta_;
	this->tm_ = (p_ - 2 * pa_) / v_;
	this->tc_ = tm_ + 2 * ta_;
}



auto TCurve2::getTCurve(int count)->double
{
	int t = count + 1;
	double s = 0;

	if (t < tc_ * 1000 + 1)
	{
		if (tc_ - 2 * ta_ > 0)
		{
			if (t < ta_ * 1000 + 1)
			{
				s = 0.5 * a_ * (t / 1000.0) * (t / 1000.0);
			}
			else if (ta_ * 1000 < t && t < tc_ * 1000 - ta_ * 1000 + 1)
			{
				s = 0.5 * a_ * ta_ * ta_ + a_ * ta_ * (t / 1000.0 - ta_);
			}
			else
			{
				s = p_ - 0.5 * a_ * (tc_ - t / 1000.0) * (tc_ - t / 1000.0);
			}
		}

		else
		{
			ta_ = sqrt(p_ / a_);
			tc_ = 2 * ta_;

			if (t < ta_ * 1000 + 1)
			{
				s = 0.5 * a_ * (t / 1000.0) * (t / 1000.0);
			}
			else
			{
				s = p_ - 0.5 * a_ * (tc_ - t / 1000.0) * (tc_ - t / 1000.0);
			}
		}


		return s;
	}
	else
	{
		return p_;
	}


}