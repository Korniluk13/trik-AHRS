#include <QtCore/qglobal.h>

#if QT_VERSION < QT_VERSION_CHECK(5, 0, 0)
	#include <QtGui/QApplication>
#else
	#include <QtWidgets/QApplication>
#endif

#include <QtCore/QScopedPointer>

#include <trikControl/brickInterface.h>
#include <trikControl/brickFactory.h>

#include "integrator.h"

int main(int argc, char* argv[])
{
	QApplication app(argc, argv);

	QScopedPointer<trikControl::BrickInterface> brick(trikControl::BrickFactory::create());

	MatrixIntegrator integrator(*brick);

	QObject::connect(brick->gyroscope(), SIGNAL(newData(QVector<int>,const trikKernel::TimeVal)),
					 &integrator, SLOT(integrate(QVector<int>,const trikKernel::TimeVal)));

	return app.exec();
}
