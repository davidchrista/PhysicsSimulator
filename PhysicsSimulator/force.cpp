#include "force.h"

Force::Force()
	: Force(Vector(0, 0), Vector(0, 0), FORCE)
{

}

Force::Force(const Vector & attackPoint, const Vector & direction, ForceType type)
	: attackPoint(attackPoint), direction(direction), type(type)
{

}

Force::Force(const Force &f)
	: Force(f.attackPoint, f.direction, f.type)
{

}
