#include "bodypoint.h"

#include "negativemassexception.h"

#include <cmath>

BodyPoint::BodyPoint()
    : BodyPoint(0, 0, 0)
{

}

BodyPoint::BodyPoint(double x, double y, double mass)
    : Vector(x, y)
{
    if (mass < 0)
        throw NegativeMassException();
    else
		this->mass_ = mass;
}

BodyPoint::BodyPoint(const BodyPoint &bp)
	: BodyPoint(bp.x_, bp.y_, bp.mass_)
{

}

void BodyPoint::set(double x, double y, double mass)
{
	Vector::set(x, y);
	this->mass_ = mass;
}
