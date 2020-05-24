#ifndef SIMULATOR_H
#define SIMULATOR_H

#include "painterwidget.h"
#include "rigidbody.h"

#include <vector>

#include <QObject>

class Simulator : public QObject
{
	Q_OBJECT
public:
	Simulator(PainterWidget * painter, vector<RigidBody *> * rigidBodies);

private:
	PainterWidget * painter;
	vector<RigidBody *> * rigidBodies;
};

#endif // SIMULATOR_H
