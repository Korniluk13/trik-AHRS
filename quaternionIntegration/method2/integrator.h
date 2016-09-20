#include <trikControl/brickInterface.h>
#include <trikKernel/timeVal.h>

#include <QtCore/QObject>
#include <QQuaternion>

class MatrixIntegrator : public QObject
{
	Q_OBJECT
public:

	MatrixIntegrator(trikControl::BrickInterface &brick);

	~MatrixIntegrator();

public slots:

	void integrate(const QVector<int> &gyro, const trikKernel::TimeVal &t);

private:

	double getRoll(const QQuaternion &q) const;

	double getPitch(const QQuaternion &q) const;

	double getYaw(const QQuaternion &q) const;

	void showNavigation();

	trikControl::BrickInterface &mBrick;

	QQuaternion mQ;
	QQuaternion mBias;

	int mInitCount;
	trikKernel::TimeVal mLastUpdate;

	const int mGyroCalibrCount = 300;
	const double mRadToDeg = 180.0 / M_PI;
	const double mGyroRadToDeg = 0.07 / mRadToDeg; //(2000dps)
};
