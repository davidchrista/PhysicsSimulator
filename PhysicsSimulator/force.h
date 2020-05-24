#ifndef FORCE_H
#define FORCE_H

#include "vector.h"

enum ForceType
{
	FORCE,
	ACCELLERATION,
};

class Force
{
public:
    Force();
	Force(const Vector & attackPoint, const Vector & direction, ForceType type);
	Force(const Force& f);

	Vector getAttackPoint() const;
	Vector getDirection() const;

private:
	Vector attackPoint;
	Vector direction;
	ForceType type;
};

#endif // FORCE_H
