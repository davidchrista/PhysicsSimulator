#ifndef VECTOR_H
#define VECTOR_H

class Vector
{
public:
	Vector();
	Vector(double x, double y);
	Vector(const Vector & v);

	void set(double y, double x);
	void setPolar(double angle, double length);

	double getX() const
	{
		return this->x;
	}
	double getY() const
	{
		return this->y;
	}
	double getAngle() const
	{
		return this->angle;
	}
	double getLength() const
	{
		return this->length;
	}

	Vector normed();

	Vector & rotate(double getAngle);
	Vector & operator=(const Vector & v);
	Vector & operator+=(const Vector & v);
	Vector & operator-=(const Vector & v);
	template <typename S>
	Vector & operator*=(const S & s)
	{
		set(x * s, y * s);

		return *this;
	}

private:
	double x;
	double y;
	double angle;
	double length;

private:
	friend Vector operator-(const Vector & a);
	friend Vector operator+(const Vector & a, const Vector & b);
	friend Vector operator-(const Vector & a, const Vector & b);
	template <typename S>
	friend Vector operator*(const S & s, const Vector & v);
	template <typename S>
	friend Vector operator*(const Vector & v, const S & s);

	friend double dot(const Vector & a, const Vector & b);
	friend double operator*(const Vector & a, const Vector & b);
	friend double cross(const Vector & a, const Vector & b);
	friend Vector projection(const Vector & direction, const Vector & v);
	friend double angle(const Vector & a, const Vector & b);

	friend Vector findAffineIntersection(const Vector & pos1, const Vector & dir1,
										 const Vector & pos2, const Vector & dir2);
};

Vector operator-(const Vector & a);
Vector operator+(const Vector & a, const Vector & b);
Vector operator-(const Vector & a, const Vector & b);
template <typename S>
Vector operator*(const S & s, const Vector & v)
{
	return Vector(s * v.getX(), s * v.getY());
}
template <typename S>
Vector operator*(const Vector & v, const S & s)
{
	return Vector(s * v.getX(), s * v.getY());
}

double dot(const Vector & a, const Vector & b);
double operator*(const Vector & a, const Vector & b);
double cross(const Vector & a, const Vector & b);
Vector projection(const Vector & direction, const Vector & v);
double angle(const Vector & a, const Vector & b);

Vector findAffineIntersection(const Vector & pos1, const Vector & dir1,
							  const Vector & pos2, const Vector & dir2);

#endif // VECTOR_H
