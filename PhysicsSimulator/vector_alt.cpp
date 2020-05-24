#include "vector_alt.h"

#include <cmath>

#include <eigen3/Eigen/Dense>

using namespace Eigen;

Vector::Vector()
	: Vector(0, 0)
{
}

Vector::Vector(double x, double y)
{
	set(x, y);
}

Vector::Vector(const Vector & v)
	: Vector(v.x, v.y)
{
}

void Vector::set(double x, double y)
{
	this->x = x;
	this->y = y;

	if (x > 0)
		this->angle = atan(y / x);
	else if (x < 0)
		this->angle = atan(y / x) + (y > 0 ? M_PI : -M_PI);
	else if (y > 0)
		this->angle = M_PI / 2;
	else if (y < 0)
		this->angle = -M_PI / 2;
	else
		this->angle = 0;

	this->length = sqrt(pow(x, 2) + pow(y, 2));
}

void Vector::setPolar(double angle, double length)
{
	if (angle > M_PI)
		angle -= floor(angle / M_PI) * 2 * M_PI;
	else if (angle < -M_PI)
		angle += floor(-angle / M_PI) * 2 * M_PI;

	this->angle = angle;
	this->length = length;

	this->x = length * cos(angle);
	this->y = length * sin(angle);
}

Vector & Vector::rotate(double angle)
{
	setPolar(this->angle + angle, this->length);

	return *this;
}

Vector Vector::normed()
{
	if (length != 0)
		return (*this) * (1 / length);
	else
		return Vector(0, 0);
}

Vector & Vector::operator=(const Vector & v)
{
	this->set(v.x, v.y);

	return *this;
}

Vector & Vector::operator+=(const Vector & v)
{
	set(x + v.x, y + v.y);

	return *this;
}

Vector & Vector::operator-=(const Vector & v)
{
	set(x - v.x, y - v.y);

	return *this;
}

Vector operator-(const Vector & a)
{
	return Vector(-a.x, -a.y);
}

Vector operator+(const Vector & a, const Vector & b)
{
	return Vector(a.x + b.x, a.y + b.y);
}

Vector operator-(const Vector & a, const Vector & b)
{
	return Vector(a.x - b.x, a.y - b.y);
}

double dot(const Vector & a, const Vector & b)
{
	return a.x * b.x + a.y * b.y;
}

double operator*(const Vector & a, const Vector & b)
{
	return a.x * b.x + a.y * b.y;
}

double cross(const Vector & a, const Vector & b)
{
	return a.x * b.y - a.y * b.x;
}

Vector projection(const Vector & direction, const Vector & v)
{
	if (direction.getLength() == 0)
		return Vector(0, 0);
	else
		return direction * (dot(v, direction) / dot(direction, direction));
}

double angle(const Vector & a, const Vector & b)
{
	return atan2(cross(a, b), dot(a, b));
}

Vector findAffineIntersection(const Vector & pos1, const Vector & dir1,
							  const Vector & pos2, const Vector & dir2)
{
	Matrix2d A;
	Vector2d b;
	Vector2d x;

	A << dir2.x, -dir1.x, dir2.y, -dir1.y;
	b << (pos1 - pos2).x, (pos1 - pos2).y;

	x = A.colPivHouseholderQr().solve(b);

	return pos2 + (dir2 * x(0));
}
