#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include "vector.h"

#include <algorithm>
#include <vector>

using namespace std;

// Possible optmization. Treats central forces seperately.
#define V1

struct BodyPoint
{
	R2 position;
	double mass;

	BodyPoint()
		: position(R2(0.0, 0.0))
		, mass(0.0) { }
};

struct Force
{
	R2 attackPoint;
	R2 direction;

	Force()
		: attackPoint(R2(0.0, 0.0))
		, direction(R2(0.0, 0.0)) { }
};

class RigidBody
{
public:
	RigidBody();
	RigidBody(const vector<BodyPoint> & bodyPoints, const R2 & position);
	RigidBody(const RigidBody & rb);
	~RigidBody();

	void addBodyPoint(const BodyPoint & bp);

	void setExternalForces(vector<Force *> * f);
	void setInternalForces(vector<Force *> * f);
	void setZeroTolerance(double zeroTolerance);

	R2 getPosition() const;
	R2 getMassCenterPosition() const;
	vector<R2> getBpPositions() const;
	vector<Force> getInternalForces() const;
	vector<Force> getExternalForces() const;
	double getMass() const;
	R2 getVelocity() const;
	double getAngularVelocity() const;

	void step();

	//////////TEST PURPOSES//////////
	void addMovementAndRotation(const R2 & movement, double rotation);
	//////////TEST PURPOSES//////////

private:
	vector<BodyPoint> bodyPoints; // positions relative to local coordinates.

	// internalForces:
	// Attack Points relative to local coords. Attack Points will automatically
	// be moved (rel. to local coords) when body is rotated.
	// Directions will be rotated when body is rotated
	vector<Force *> * internalForces;

	// externalForces:
	// Like internal Force, BUT:
	// Directions won't be rotated when body is rotated.
	vector<Force *> * externalForces;

	double mass;
	double momentOfInertia;

	R2 massCenter; // Relative to local coordinates.

	R2 position; // Global.
	double rotation;

	R2 velocity;
	double angularVelocity;

	void computeMassAndMassCenter();
	vector<Force> computeResultingForces();
	R2 computeAccelleration(vector<Force> presentForces);
	double computeAngularAccelleration(vector<Force> presentForces);

	void move(const R2 & v);
	void rotateBp(double angle);
};

Force addUpForces(const Force & f1, const Force & f2);

// unparallelizeForces:
// Do not use if f1 and f2 antiparallel and same length!
void unparallelizeForces(Force & f1, Force & f2);

#endif // RIGIDBODY_H
