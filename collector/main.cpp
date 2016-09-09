#include <QtCore/qglobal.h>

#if QT_VERSION < QT_VERSION_CHECK(5, 0, 0)
	#include <QtGui/QApplication>
#else
	#include <QtWidgets/QApplication>
#endif

#include <QtCore/QScopedPointer>
#include <QtCore/QTimer>

#include <trikControl/brickInterface.h>
#include <trikControl/brickFactory.h>

#include "collector.h"

int main(int argc, char* argv[])
{
	QApplication app(argc, argv);
	QScopedPointer<trikControl::BrickInterface> brick(trikControl::BrickFactory::create());

	Collector collector(*brick);

	QObject::connect(brick->gyroscope(), SIGNAL(newData(QVector<int>,const trikKernel::TimeVal))
					 , &collector, SLOT(writeInfo(QVector<int>,trikKernel::TimeVal)));

	QObject::connect(brick->accelerometer(), SIGNAL(newData(QVector<int>,const trikKernel::TimeVal))
					 , &collector, SLOT(updateAccel(QVector<int>,trikKernel::TimeVal)));

	QObject::connect(&collector, SIGNAL(endWriting())
					 , &collector, SLOT(closeFile()));

	return app.exec();
}
