#ifndef PAINTERWIDGET_H
#define PAINTERWIDGET_H

#include "rigidbody.h"

#include <vector>

#include <QBrush>
#include <QPaintEvent>
#include <QPainter>
#include <QPen>
#include <QWidget>
#include <QWidget>

using namespace std;

class PainterWidget : public QWidget
{
	Q_OBJECT

public:
	explicit PainterWidget(QWidget * parent = nullptr);

	void setBodiesToPaint(const vector<RigidBody *> * bodiesToPaint);
	void setPaintForces(bool paintForces);
	void setZoomFactor(double zoomFactor);
	void setOrigin(double x, double y);
	void setPaintStepCount(bool paintStepCount);

public slots:
	void animate();

protected:
	void resizeEvent(QResizeEvent * event) Q_DECL_OVERRIDE;
	void paintEvent(QPaintEvent * event) Q_DECL_OVERRIDE;
	void mousePressEvent(QMouseEvent * event) Q_DECL_OVERRIDE;

private:
	QPainter painter;
	QBrush background;
	QBrush circleBrush;
	QPen circlePen;

	// widgetWidth, widgetHeigth:
	// Width and Heigth of Widget in pixels.
	int widgetWidth;
	int widgetHeigth;

	const vector<RigidBody *> * bodiesToPaint;

	bool paintStepCount;
	bool paintForces;
	bool paintCoordinates;

	long stepCount;

	// zoomFactor, unitSize:
	// Determines the size of the drawn objects.
	// zoomFactor = "displayed length units on whole widget width"
	// lengthUnitSize = ceil(widgetWidth / zoomFactor)
	//          = "size of one lengthUnit in pixels"
	double zoomFactor;
	double lengthUnitSize;

	// originX, originY:
	// Location of the coordinate origin in length units
	double originX;
	double originY;

	void paintRigidBody(const RigidBody * const rb);
	void paintForce(const Force f);
	void paintCoords(const R2 & v);

	void updateEnvironment();
};

#endif // PAINTERWIDGET_H
