#ifndef VECTOR_H
#define VECTOR_H

#include <eigen3/Eigen/Dense>

using namespace Eigen;

Matrix<double, 2, 1> projection(const Matrix<double, 2, 1> & direction,
								const Matrix<double, 2, 1> & v);

double cross(const Matrix<double, 2, 1> & a,
			 const Matrix<double, 2, 1> & b);

double angle(const Matrix<double, 2, 1> & a, const Matrix<double, 2, 1> & b);

void rotate(Matrix<double, 2, 1> & v, double angle);

Matrix<double, 2, 1> normed(const Matrix<double, 2, 1> & v);

Matrix<double, 2, 1> findAffineIntersection(
		const Matrix<double, 2, 1> & pos1, const Matrix<double, 2, 1> & dir1,
		const Matrix<double, 2, 1> & pos2, const Matrix<double, 2, 1> & dir2);

typedef Matrix<double, 2, 1> R2;
typedef Matrix<double, 3, 1> R3;

typedef Matrix<double, 2, 2> R22;
typedef Matrix<double, 3, 3> R33;

#endif // VECTOR_H
