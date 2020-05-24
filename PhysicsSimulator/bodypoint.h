#ifndef BODYPOINT_H
#define BODYPOINT_H

#include "vector.h"

class BodyPoint : public Vector
{
	double mass_;

public:
	BodyPoint();
    BodyPoint(double x, double y, double mass);
	BodyPoint(const BodyPoint& bp);

	double mass() const {return mass_; }

	void set(double x, double y, double mass);
};

#endif // BODYPOINT_H
