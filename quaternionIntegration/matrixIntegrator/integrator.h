#include <trikControl/brickInterface.h>
#include <trikKernel/timeVal.h>

#include <QtCore/QObject>
#include <QVector4D>

class MatrixIntegrator : public QObject
{
	Q_OBJECT
public:

	MatrixIntegrator(trikControl::BrickInterface &brick);

	~MatrixIntegrator();

public slots:

	void integrate(const QVector<int> &gyro, const trikKernel::TimeVal &t);

private:

	double getRoll(const QVector4D &q) const;

	double getPitch(const QVector4D &q) const;

	double getYaw(const QVector4D &q) const;

	void showNavigation();

	trikControl::BrickInterface &mBrick;

	QVector4D mQ;
	QVector4D mBias;

	int mInitCount;
	trikKernel::TimeVal mLastUpdate;

	const int mGyroCalibrCount = 300;
	const double mRadToDeg = 180.0 / M_PI;
	const double mGyroToRad = 0.07 / mRadToDeg; //(2000dps)
};
