#include "vector.h"

Matrix<double, 2, 1> projection(const Matrix<double, 2, 1> & direction,
						   const Matrix<double, 2, 1> & v)
{
	if (direction.norm() == 0.0)
		return Matrix<double, 2, 1>(0.0, 0.0);
	else
		return direction * (v.dot(direction) / direction.dot(direction));
}

double cross(const Matrix<double, 2, 1> & a,
			  const Matrix<double, 2, 1> & b)
{
	return a(0) * b(1) - a(1) * b(0);
}

double angle(const Matrix<double, 2, 1> & a, const Matrix<double, 2, 1> & b)
{
	return atan2(cross(a, b), a.dot(b));
}

//Matrix<double, 2, 1> & rotate(Matrix<double, 2, 1> & v, double angle)
//{
//	double c = cos(angle), s = sin(angle);

//	Matrix<double, 2, 2> rotMatrix;
//	rotMatrix << c, -s, s, c;

//	v = (rotMatrix * v);

//	return v;
//}

//Matrix<double, 2, 1> rotate(const Matrix<double, 2, 1> & v, double angle)
//{
//	Matrix<double, 2, 1> a;

//	double c = cos(angle), s = sin(angle);

//	Matrix<double, 2, 2> rotMatrix;
//	rotMatrix << c, -s, s, c;

//	a = (rotMatrix * v);

//	return a;
//}

void rotate(Matrix<double, 2, 1> & v, double angle)
{
	double c = cos(angle), s = sin(angle);

	Matrix<double, 2, 2> rotMatrix;
	rotMatrix << c, -s, s, c;

	v = (rotMatrix * v);
}

Matrix<double, 2, 1> normed(const Matrix<double, 2, 1> & v)
{
	double n = v.norm();

	if (n > 0.0)
		return v * (1.0 / v.norm());
	else
		return Matrix<double, 2, 1>(0.0, 0.0);
}

Matrix<double, 2, 1> findAffineIntersection(
		const Matrix<double, 2, 1> & pos1, const Matrix<double, 2, 1> & dir1,
		const Matrix<double, 2, 1> & pos2, const Matrix<double, 2, 1> & dir2)
{
	Matrix<double, 2, 2> A;
	Matrix<double, 2, 1> b;
	Matrix<double, 2, 1> x;

	A << dir2(0), -dir1(0), dir2(1), -dir1(1);
	b = pos1 - pos2;

	x = A.colPivHouseholderQr().solve(b);

	return pos2 + (dir2 * x(0));
}
