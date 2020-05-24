#include "rigidbody.h"

#include "timing.h"
#include "precision.h"

RigidBody::RigidBody()
	: bodyPoints()
	, internalForces()
	, externalForces()
	, position(R2(0.0, 0.0))
	, velocity(R2(0.0, 0.0))
	, angularVelocity(0.0)
{
	BodyPoint bp;
	bp.position = R2(0.0, 0.0);
	bp.mass = 0.0;

	bodyPoints.push_back(bp);

	computeMassAndMassCenter();
}

RigidBody::RigidBody(const vector<BodyPoint> & bodyPoints,
					 const R2 & position)
	: bodyPoints(bodyPoints)
	, internalForces()
	, externalForces()
	, position(position)
	, rotation(0.0)
	, velocity(R2(0.0, 0.0))
	, angularVelocity(0.0)
{
	computeMassAndMassCenter();
}

RigidBody::RigidBody(const RigidBody & rb)
	: RigidBody(rb.bodyPoints, rb.position)
{
}

RigidBody::~RigidBody()
{
	if (internalForces != nullptr)
	{
		for (Force * f : *internalForces)
		{
			if (f != nullptr)
				delete f;
		}

		delete internalForces;
	}

	if (externalForces != nullptr)
	{
		for (Force * f : *externalForces)
		{
			if (f != nullptr)
				delete f;
		}

		delete externalForces;
	}
}

void RigidBody::addBodyPoint(const BodyPoint & bp)
{
	bodyPoints.push_back(bp);
	rotate((bodyPoints.end() - 1)->position, rotation);

	computeMassAndMassCenter();
}

void RigidBody::setExternalForces(vector<Force *> * f)
{
	this->externalForces = f;
}

void RigidBody::setInternalForces(vector<Force *> * f)
{
	this->internalForces = f;
}

void RigidBody::move(const R2 & v)
{
	position += v;
}

void RigidBody::rotateBp(double angle)
{
	rotation += angle;

	R2 temp;
	R2 massCenterOld = massCenter;

	rotate(massCenter, angle);
	position += (massCenterOld - massCenter);
	// For alternative (0,0) != massCenter.

	for (BodyPoint & bp : bodyPoints)
	{
		temp = bp.position - massCenterOld;
		rotate(temp, angle);
		bp.position = massCenter
					  + temp;
	}

	if (internalForces != nullptr)
	{
		for (Force * f : *internalForces)
		{
			temp = f->attackPoint - massCenterOld;
			rotate(temp, angle);
			f->attackPoint = massCenter
							 + temp;
			rotate(f->direction, angle);
		}
	}

	if (externalForces != nullptr)
	{
		for (Force * f : *externalForces)
		{
			temp = f->attackPoint - massCenterOld;
			rotate(temp, angle);
			f->attackPoint = massCenter
							 + temp;
		}
	}
}

void RigidBody::computeMassAndMassCenter()
{
	mass = 0.0;
	massCenter = R2(0.0, 0.0);
	momentOfInertia = 0.0;

	for (auto bp : bodyPoints)
	{
		mass += bp.mass;
		massCenter += (bp.mass * bp.position);
	}

	massCenter *= (1.0 / mass);

	for (auto bp : bodyPoints)
	{
		momentOfInertia += bp.mass
						   * (bp.position - massCenter).norm();
	}
}

vector<R2> RigidBody::getBpPositions() const
{
	vector<R2> v;

	for (auto bp : bodyPoints)
	{
		//		v.push_back(position + bp.position - massCenter);
		v.push_back(position + bp.position);
		// For alternative (0,0) != massCenter.
	}

	return v;
}

R2 RigidBody::getPosition() const
{
	return position;
}

R2 RigidBody::getMassCenterPosition() const
{
	//	return position;
	return position + massCenter;
	// For alternative (0,0) != massCenter.
}

double RigidBody::getMass() const
{
	return mass;
}

vector<Force> RigidBody::getInternalForces() const
{
	vector<Force> forces;

	if (internalForces != nullptr)
	{
		for (Force * f : *internalForces)
		{
			Force force = *f;
			force.attackPoint += this->position;
			forces.push_back(force);
		}
	}

	return forces;
}

vector<Force> RigidBody::getExternalForces() const
{
	vector<Force> forces;

	if (externalForces != nullptr)
	{
		for (Force * f : *externalForces)
		{
			Force force = *f;
			force.attackPoint += this->position;
			forces.push_back(force);
		}
	}

	return forces;
}

R2 RigidBody::getVelocity() const
{
	return velocity;
}

double RigidBody::getAngularVelocity() const
{
	return angularVelocity;
}

void RigidBody::step()
{
	vector<Force> resultingForces = computeResultingForces();

	velocity += computeAccelleration(resultingForces)
				* timing::TIME_DELTA;
	angularVelocity += computeAngularAccelleration(resultingForces)
					   * timing::TIME_DELTA;

	move(velocity * timing::TIME_DELTA);
	rotateBp(angularVelocity * timing::TIME_DELTA);
}

R2 RigidBody::computeAccelleration(vector<Force> presentForces)
{
	if (presentForces.size() > 0)
	{
		return presentForces[0].direction * (1.0 / mass);
	}
	else
	{
		return R2(0.0, 0.0);
	}
}

double RigidBody::computeAngularAccelleration(vector<Force> presentForces)
{
	if (presentForces.size() > 0)
	{
		return presentForces[1].direction.norm()
			   * cross(normed(presentForces[1].attackPoint - massCenter),
					   normed(presentForces[1].direction))
			   * (presentForces[1].attackPoint - massCenter).norm()
			   / momentOfInertia;
	}
	else
	{
		return 0.0;
	}
}

//////////TEST PURPOSES//////////
void RigidBody::addMovementAndRotation(const R2 & movement, double rotation)
{
	velocity += movement;
	angularVelocity += rotation;
}
//////////TEST PURPOSES//////////

vector<Force> RigidBody::computeResultingForces()
{
	vector<Force> remainingForces;

	if (internalForces != nullptr)
	{
		for (Force * f : *internalForces)
		{
			if (f != nullptr)
				remainingForces.push_back(*f);
		}
	}

	if (externalForces != nullptr)
	{
		for (Force * f : *externalForces)
		{
			if (f != nullptr)
				remainingForces.push_back(*f);
		}
	}

	vector<Force> resultingForces;

	Force f;
	f.attackPoint = R2(0.0, 0.0);
	f.direction = f.attackPoint;
	for (int i = 1; i <= 2; ++i)
		resultingForces.push_back(f);
	resultingForces[0].attackPoint = massCenter;

	if (remainingForces.size() < 1)
		return resultingForces;

	vector<Force>::iterator it = remainingForces.begin();

	f = *it;
	it++;

	while (it != remainingForces.end() && f.direction.norm() < precision::ZERO_TOLERANCE)
	{
		f = *it;
		it++;
	}

	f.attackPoint += projection(f.direction, massCenter - f.attackPoint);

#ifdef V1
	while (it != remainingForces.end() && (f.attackPoint - massCenter).norm() < precision::ZERO_TOLERANCE)
	{
		resultingForces[0].direction += f.direction;
		f = *it;
		it++;
	}
#endif

	while (it != remainingForces.end())
	{
		Force curr = *it;

		if (f.direction.norm() < precision::ZERO_TOLERANCE)
		{
			f = curr;
			it++;
			continue;
		}

		if (curr.direction.norm() < precision::ZERO_TOLERANCE)
		{
			it++;
			continue;
		}

		curr.attackPoint += projection(curr.direction,
									   massCenter - curr.attackPoint);
		f.attackPoint += projection(f.direction, massCenter - f.attackPoint);

#ifdef V1
		if ((curr.attackPoint - massCenter).norm()
				< precision::ZERO_TOLERANCE)
		{
			// Case "central force":
			// add to central forces and continue
			resultingForces[0].direction += curr.direction;
		}
		else if (fabs(fabs(angle(f.direction, curr.direction)) - M_PI)
				 < precision::ZERO_TOLERANCE)
#else
		if (fabs(fabs(angle(f.direction, curr.direction)) - M_PI)
				< precision::ZERO_TOLERANCE)
#endif
		{
			// Case "practically antiparallel"
			if ((fabs(f.direction.norm() - curr.direction.norm())
					< precision::ZERO_TOLERANCE))
			{
				// Case "practically antiparallel and same length":
				// postpone if more, otherwise pure rotation
				it++;

				if (it == remainingForces.end())
				{
					f.attackPoint += projection(
										 f.direction, massCenter - f.attackPoint);
					curr.attackPoint += projection(curr.direction,
												   massCenter - curr.attackPoint);

					double dist = (curr.attackPoint - f.attackPoint).norm();

					f.attackPoint = massCenter
									+ normed(f.attackPoint - massCenter) * dist;

					resultingForces[1] = f;

					return resultingForces;
				}
				else
				{
					remainingForces.push_back(f);
					f = curr;

					continue;
				}
			}
			else
			{
				// Case "antiparallel and different lengths":
				// add correction forces and add up to one
				unparallelizeForces(f, curr);

				f = addUpForces(f, curr);
			}
		}
		else if (fabs(angle(f.direction, curr.direction))
				 < precision::ZERO_TOLERANCE)
		{
			// Case "parallel":
			// add correction forces and add up to one
			unparallelizeForces(f, curr);

			f = addUpForces(f, curr);
		}
		else
		{
			// Case "not parallel":
			// add up to one
			f = addUpForces(f, curr);
		}

		it++;
	}

	if ((f.attackPoint - massCenter).norm() > precision::ZERO_TOLERANCE)
	{
		f.attackPoint += projection(f.direction, massCenter - f.attackPoint);
		resultingForces[1] = f;
	}

	resultingForces[0].direction += f.direction;

	return resultingForces;
}

Force addUpForces(const Force & f1, const Force & f2)
{
	if (f1.direction.norm() < precision::ZERO_TOLERANCE)
		return f2;

	if (f2.direction.norm() < precision::ZERO_TOLERANCE)
		return f1;

	Force newForce;

	newForce.direction = f1.direction + f2.direction;
	newForce.attackPoint = findAffineIntersection(f1.attackPoint, f1.direction,
						   f2.attackPoint, f2.direction);
	newForce.attackPoint
		= findAffineIntersection(newForce.attackPoint, newForce.direction,
								 f1.attackPoint,
								 f2.attackPoint - f1.attackPoint);

	return newForce;
}

void unparallelizeForces(Force & f1, Force & f2)
{
	// Fails if f1 and f2 antiparallel and same length!
	R2 correction = normed(f1.attackPoint - f2.attackPoint);

	if (f1.direction.norm() >= f2.direction.norm())
	{
		correction *= f1.direction.norm();

		f1.direction -= correction;
		f2.direction += correction;
	}
	else
	{
		correction *= f2.direction.norm();

		f1.direction += correction;
		f2.direction -= correction;
	}

	if (f1.direction.norm() < precision::ZERO_TOLERANCE)
	{
		R2 dirF2 = f2.direction;
		rotate(f2.direction, M_PI / 4.0);
		f2.direction *= (1.0 / sqrt(2.0));
		f1.direction = dirF2 - f2.direction;
	}
	if (f2.direction.norm() < precision::ZERO_TOLERANCE)
	{
		R2 dirF1 = f1.direction;
		rotate(f1.direction, M_PI / 4.0);
		f1.direction *= (1.0 / sqrt(2.0));
		f2.direction = dirF1 - f1.direction;
	}
}
