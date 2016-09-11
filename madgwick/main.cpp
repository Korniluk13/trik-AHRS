#include <QtCore/qglobal.h>

#if QT_VERSION < QT_VERSION_CHECK(5, 0, 0)
	#include <QtGui/QApplication>
#else
	#include <QtWidgets/QApplication>
#endif

#include <QtCore/QScopedPointer>
#include <iostream>

#include <trikControl/brickInterface.h>
#include <trikControl/brickFactory.h>

#include "filter.h"

int main(int argc, char* argv[])
{
	QApplication app(argc, argv);

	QScopedPointer<trikControl::BrickInterface> brick(trikControl::BrickFactory::create());

//	float magwickConst = 0.0;
//	std::cin >> magwickConst;

	Filter filter(*brick, 0.8);

	QObject::connect(brick->gyroscope(), SIGNAL(newData(QVector<int>,const trikKernel::TimeVal)),
					 &filter, SLOT(updateNavigation(QVector<int>,const trikKernel::TimeVal)));

	return app.exec();
}
