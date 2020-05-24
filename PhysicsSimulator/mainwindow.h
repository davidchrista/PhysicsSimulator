#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "painterwidget.h"
#include "rigidbody.h"
#include "simulator.h"

#include <vector>

#include <QMainWindow>
#include <QTimer>

namespace Ui
{
class MainWindow;
}

class MainWindow : public QMainWindow
{
	Q_OBJECT

public:
	explicit MainWindow(QWidget * parent = nullptr);
	~MainWindow();

public slots:
	void step();

protected:
	void keyReleaseEvent(QKeyEvent * event) Q_DECL_OVERRIDE;
	void keyPressEvent(QKeyEvent * event) Q_DECL_OVERRIDE;

private slots:
    void on_pushButtonRestart_clicked();

private:
	Ui::MainWindow * ui;
	PainterWidget * painterWidget;
	QTimer * timer;

	vector<RigidBody *> * rigidBodies;
	vector<vector<Force *>*> * internalForces;
	vector<vector<Force *>*> * externalForces;

	void configureExample();
};

#endif // MAINWINDOW_H
