#ifndef NEGATIVEMASSEXCEPTION_H
#define NEGATIVEMASSEXCEPTION_H

#include <exception>

using namespace std;

class NegativeMassException : public exception
{
	virtual const char * what() const throw()
	{
		return "Negative mass not allowed";
	}
};

#endif // NEGATIVEMASSEXCEPTION_H
