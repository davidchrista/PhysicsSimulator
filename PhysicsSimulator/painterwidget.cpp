#include "painterwidget.h"

#include <iomanip>
#include <sstream>

using namespace std;

PainterWidget::PainterWidget(QWidget * parent)
	: QWidget(parent)
	, painter()
	, paintStepCount(true)
	, paintForces(true)
	, paintCoordinates(true)
	, stepCount(0)
	, zoomFactor(1.0)
	, originX(0.0)
	, originY(0.0)
{
	widgetWidth = this->geometry().width();
	widgetHeigth = this->geometry().height();

	background = QBrush(QColor(64, 32, 64));
	circleBrush = QBrush(Qt::white);
	circlePen = QPen(Qt::white);
	circlePen.setWidth(1);
}

void PainterWidget::animate()
{
	update();
}

void PainterWidget::resizeEvent(QResizeEvent * event)
{
	event->accept();

	widgetWidth = this->geometry().width();
	widgetHeigth = this->geometry().height();

	updateEnvironment();
}

void PainterWidget::paintEvent(QPaintEvent * event)
{
	this->stepCount++;

	painter.begin(this);

	painter.setRenderHint(QPainter::Antialiasing);
	painter.setBrush(circleBrush);
	painter.setPen(circlePen);

	painter.fillRect(event->rect(), background);

	painter.translate(originX * lengthUnitSize, originY * lengthUnitSize);

	painter.save();

	if (paintStepCount == true)
	{
		string stepCount = "Step count: " + to_string(this->stepCount);
		painter.drawText(QPointF(-originX * lengthUnitSize + 10.0,
								 -originY * lengthUnitSize + 18.0),
						 QString(stepCount.c_str()));
	}

	painter.drawLine(QPoint(-widgetWidth, 0),
					 QPoint(widgetWidth, 0));
	painter.drawLine(QPoint(0, -widgetHeigth),
					 QPoint(0, widgetHeigth));

	for (int i = 1;
			i <= static_cast<int>(
				ceil(zoomFactor / widgetWidth * max(widgetWidth, widgetHeigth) / 2.0));
			i++)
	{
		painter.drawLine(QPointF(-i * lengthUnitSize, -0.2 * lengthUnitSize),
						 QPointF(-i * lengthUnitSize, 0.2 * lengthUnitSize));
		painter.drawLine(QPointF(i * lengthUnitSize, -0.2 * lengthUnitSize),
						 QPointF(i * lengthUnitSize, 0.2 * lengthUnitSize));
		painter.drawLine(QPointF(-0.2 * lengthUnitSize, -i * lengthUnitSize),
						 QPointF(0.2 * lengthUnitSize, -i * lengthUnitSize));
		painter.drawLine(QPointF(-0.2 * lengthUnitSize, i * lengthUnitSize),
						 QPointF(0.2 * lengthUnitSize, i * lengthUnitSize));
	}

	if (bodiesToPaint != nullptr)
	{
		for (RigidBody * rb : (*bodiesToPaint))
		{
			paintRigidBody(rb);

			stringstream stream;
			stream << fixed << setprecision(3) << rb->getVelocity().norm();
			string veloString = "v = " + stream.str();
			painter.drawText(QPointF(-originX * lengthUnitSize + 10.0,
									 -originY * lengthUnitSize + 34.0),
							 QString(veloString.c_str()));
			stream = stringstream();
			stream << fixed << setprecision(3) << rb->getAngularVelocity();
			string angVeloString = "omega = " + stream.str();
			painter.drawText(QPointF(-originX * lengthUnitSize + 10.0,
									 -originY * lengthUnitSize + 50.0),
							 QString(angVeloString.c_str()));

			if (paintForces == true)
			{
				for (Force f : rb->getInternalForces())
					paintForce(f);
				for (Force f : rb->getExternalForces())
					paintForce(f);
			}
		}
	}

	painter.restore();

	painter.end();
}

void PainterWidget::mousePressEvent(QMouseEvent * event)
{
	event->accept();
	this->setFocus();
}

void PainterWidget::paintRigidBody(const RigidBody * const rb)
{
	double pointDrawingSize = max(1.0, lengthUnitSize / 4.0);
	vector<R2> points = rb->getBpPositions();
	vector<R2>::iterator it = points.begin();

	double x = (*it)(0) * lengthUnitSize;
	double y = -(*it)(1) * lengthUnitSize;
	painter.drawEllipse(QPointF(x, y), pointDrawingSize, pointDrawingSize);
	if (paintCoordinates == true)
		paintCoords(*it);

	double nextX, nextY;

	it++;
	while (it != points.end())
	{
		nextX = (*it)(0) * lengthUnitSize;
		nextY = -(*it)(1) * lengthUnitSize;

		painter.drawEllipse(QPointF(nextX, nextY),
							pointDrawingSize, pointDrawingSize);
		if (paintCoordinates == true)
			paintCoords(*it);
		painter.drawLine(QPointF(x, y), QPointF(nextX, nextY));

		x = nextX;
		y = nextY;

		it++;
	}

	x = rb->getMassCenterPosition()(0) * lengthUnitSize;
	y = -rb->getMassCenterPosition()(1) * lengthUnitSize;
	painter.drawEllipse(QPointF(x, y), pointDrawingSize, pointDrawingSize);
	if (paintCoordinates == true)
		paintCoords(rb->getMassCenterPosition());
}

void PainterWidget::paintForce(const Force f)
{
	double apx = f.attackPoint(0) * lengthUnitSize;
	double apy = f.attackPoint(1) * lengthUnitSize;
	R2 dir = f.direction;
	//	painter.drawLine(QPointF(apx, -apy),
	//					 QPointF(apx - dir.getX() * lengthUnitSize / 10.0,
	//							 -apy + dir.getY() * lengthUnitSize / 10.0));
	painter.drawLine(QPointF(apx, -apy),
					 QPointF(apx - dir(0) * lengthUnitSize,
							 -apy + dir(1) * lengthUnitSize));
	rotate(dir, M_PI / 4.0);
	dir = normed(dir);
	painter.drawLine(QPointF(apx, -apy),
					 QPointF(apx - dir(0) * lengthUnitSize,
							 -apy + dir(1) * lengthUnitSize));
	rotate(dir, -M_PI / 2.0);
	painter.drawLine(QPointF(apx, -apy),
					 QPointF(apx - dir(0) * lengthUnitSize,
							 -apy + dir(1) * lengthUnitSize));
}

void PainterWidget::paintCoords(const R2 & v)
{
	stringstream stream;
	stream << fixed << setprecision(3) << v(0) << "|" << v(1);
	string s = "(" + stream.str() + ")";
	painter.drawText(QPointF(v(0) * lengthUnitSize + 1.0,
							 -v(1) * lengthUnitSize - 1.0),
					 QString(s.c_str()));
}

void PainterWidget::setBodiesToPaint(const vector<RigidBody *> * bodiesToPaint)
{
	this->bodiesToPaint = bodiesToPaint;
}

void PainterWidget::setPaintForces(bool paintForces)
{
	this->paintForces = paintForces;
}

void PainterWidget::setZoomFactor(double zoomFactor)
{
	this->zoomFactor = zoomFactor;

	updateEnvironment();
}

void PainterWidget::setOrigin(double x, double y)
{
	originX = x;
	originY = y;
}

void PainterWidget::setPaintStepCount(bool paintStepCount)
{
	this->paintStepCount = paintStepCount;
}

void PainterWidget::updateEnvironment()
{
	lengthUnitSize = widgetWidth / zoomFactor;
}
