#include "mainwindow.h"

#include "ui_mainwindow.h"
#include "timing.h"

#include <exception>

#include <QGridLayout>
#include <QHBoxLayout>
#include <QSizePolicy>

MainWindow::MainWindow(QWidget * parent)
	: QMainWindow(parent)
	, ui(new Ui::MainWindow)
	, painterWidget(nullptr)
	, timer(nullptr)
	, rigidBodies(nullptr)
{
	ui->setupUi(this);

	painterWidget = new PainterWidget(this->centralWidget());
	rigidBodies = new vector<RigidBody *>;
	internalForces = new vector<vector<Force *>*>;
	externalForces = new vector<vector<Force *>*>;

	static_cast<QHBoxLayout *>(this->centralWidget()->layout())
        ->removeItem(
		static_cast<QHBoxLayout *>(this->centralWidget()->layout())->itemAt(1));
	static_cast<QHBoxLayout *>(this->centralWidget()->layout())
        ->addWidget(painterWidget);
	this->painterWidget->setFocus();
	this->painterWidget->setZoomFactor(100.0);
	this->painterWidget->setOrigin(50.0, 50.0);
	this->painterWidget->setPaintForces(true);
	this->painterWidget->setBodiesToPaint(rigidBodies);

	configureExample();

	timer = new QTimer(this);
	connect(timer, SIGNAL(timeout()), painterWidget, SLOT(animate()));
	connect(timer, SIGNAL(timeout()), this, SLOT(step()));
	timer->start(timing::TIME_UNIT_IN_MILLISECONDS);
}

void MainWindow::configureExample()
{
	//	BodyPoint bp1, bp2, bp3;
	//	bp1.mass = 1;
	//	bp1.position = Vector(-30.0, 0.0);
	//	bp2.mass = 10.0;
	//	bp2.position = Vector(0.0, 30.0);
	//	bp3.mass = 1;
	//	bp3.position = Vector(30.0, 0.0);

	//	BodyPoint bp4, bp5;
	//	bp4.mass = 1;
	//	bp4.position = Vector(-20.0, -10.0);
	//	bp5.mass = 1;
	//	bp5.position = Vector(20.0, 20.0);

	//	vector<BodyPoint> bodyPoints;
	//	bodyPoints.push_back(bp4);
	//	bodyPoints.push_back(bp1);
	//	bodyPoints.push_back(bp2);
	//	bodyPoints.push_back(bp5);
	//	bodyPoints.push_back(bp3);

	BodyPoint head, tail;
	head.mass = 1.0;
	tail.mass = 2.0;
	head.position = R2(0.0, 10.0);
	tail.position = R2(0.0, 0.0);

	vector<BodyPoint> bodyPoints;
	bodyPoints.push_back(head);
	bodyPoints.push_back(tail);

	RigidBody * rb = new RigidBody(bodyPoints, R2(0.0, 0.0));
	rigidBodies->push_back(rb);

	vector<Force *> * externalForcesRb = new vector<Force *>;
	vector<Force *> * internalForcesRb = new vector<Force *>;
	rb->setInternalForces(internalForcesRb);
	rb->setExternalForces(externalForcesRb);
	internalForces->push_back(internalForcesRb);
	externalForces->push_back(externalForcesRb);

	Force * gravitation = new Force;
	gravitation->attackPoint = rb->getMassCenterPosition() - rb->getPosition();
	gravitation->direction = R2(0.0, -10.0) * (rb->getMass() / 10.0);
	// "/ xx" for slower movement ...
	externalForcesRb->push_back(gravitation);

	Force * sideForce = new Force;
	Force * tailForce = new Force;
	tailForce->attackPoint = tail.position;
	//tailForce->direction = R2(0.0, 0.0);
	sideForce->attackPoint = head.position;
	//sideForce->direction = R2(0.0, 0.0);
	internalForcesRb->push_back(sideForce);
	internalForcesRb->push_back(tailForce);

	//	Force * f1 = new Force;
	//	Force * f2 = new Force;
	//	Force * f3 = new Force;

	//	f1->attackPoint = rb->getMassCenterPosition() + Vector(1, 0.0);
	//	f2->attackPoint = rb->getMassCenterPosition() - Vector(2, 0.0);
	//	f1->direction = Vector(0.0, 1);
	//	f2->direction = Vector(0.0, -1.5);
	//	f1->direction.rotate(M_PI / 4);
	//	f2->direction.rotate(M_PI / 4);
	//	f1->attackPoint -= rb->getMassCenterPosition();
	//	f2->attackPoint -= rb->getMassCenterPosition();
	//	f1->attackPoint.rotate(M_PI / 4);
	//	f2->attackPoint.rotate(M_PI / 4);
	//	f1->attackPoint += rb->getMassCenterPosition();
	//	f2->attackPoint += rb->getMassCenterPosition();
	//	internalForcesRb->push_back(f1);
	//	internalForcesRb->push_back(f2);

	//	f1->attackPoint = rb->getMassCenterPosition() + Vector(3, 0.0);
	//	f1->direction = Vector(0.0, -1.5);
	//	internalForcesRb->push_back(f1);
	//	f2->attackPoint = rb->getMassCenterPosition();
	//	f2->direction = Vector(0.0, 3);
	//	internalForcesRb->push_back(f2);
	//	f3->attackPoint = rb->getMassCenterPosition() + Vector(-3, 0.0);
	//	f3->direction = Vector(0.0, -1.5);
	//	internalForcesRb->push_back(f3);
}

MainWindow::~MainWindow()
{
	delete ui;
	delete painterWidget;
	delete timer;
	delete rigidBodies;
}

void MainWindow::step()
{
	for (RigidBody * rb : *rigidBodies)
	{
		rb->step();
	}
}

void MainWindow::keyPressEvent(QKeyEvent * event)
{
	R2 v(0.0, 0.0);

	switch (event->key())
	{
	case Qt::Key_Left:
		(*rigidBodies)[0]->addMovementAndRotation(R2(-1.0, 0.0), 0.0);
		break;
	case Qt::Key_Right:
		(*rigidBodies)[0]->addMovementAndRotation(R2(1.0, 0.0), 0.0);
		break;
	case Qt::Key_Up:
		(*rigidBodies)[0]->addMovementAndRotation(R2(0.0, 1.0), 0.0);
		break;
	case Qt::Key_Down:
		(*rigidBodies)[0]->addMovementAndRotation(R2(0.0, -1.0), 0.0);
		break;
	case Qt::Key_D:
		v += ((*rigidBodies)[0]->getBpPositions()[0] - (*rigidBodies)[0]->getBpPositions()[1]);
		rotate(v, -M_PI / 2.0);
		(*(*internalForces)[0])[0]->direction = normed(v) * 1.0;
		break;
	case Qt::Key_A:
		v += ((*rigidBodies)[0]->getBpPositions()[0] - (*rigidBodies)[0]->getBpPositions()[1]);
		rotate(v, M_PI / 2.0);
		(*(*internalForces)[0])[0]->direction = normed(v) * 1.0;
		break;
	case Qt::Key_W:
		(*(*internalForces)[0])[1]->direction = normed(
					(*rigidBodies)[0]->getBpPositions()[0]
				- (*rigidBodies)[0]->getBpPositions()[1]
				) * 10.0;
		break;
	case Qt::Key_Space:
		if (timer->isActive() == true)
			timer->stop();
		else
			timer->start(timing::TIME_UNIT_IN_MILLISECONDS);
		break;
	}
}

void MainWindow::keyReleaseEvent(QKeyEvent * event)
{
	switch (event->key())
	{
	case Qt::Key_D:
		(*(*internalForces)[0])[0]->direction = R2(0.0, 0.0);
		break;
	case Qt::Key_A:
		(*(*internalForces)[0])[0]->direction = R2(0.0, 0.0);
		break;
	case Qt::Key_W:
		(*(*internalForces)[0])[1]->direction = R2(0.0, 0.0);
		break;
	case Qt::Key_Space:
		break;
	}
}


void MainWindow::on_pushButtonRestart_clicked()
{

}
