#include <trikControl/brickInterface.h>
#include <trikKernel/timeVal.h>

#include <QtCore/QObject>
#include <QVector4D>
#include <QMatrix4x4>

class Integrator : public QObject
{
	Q_OBJECT
public:

	Integrator(trikControl::BrickInterface &brick);

	~Integrator();

public slots:

	void updateNavigation(const QVector<int> &gyro, const trikKernel::TimeVal &t);	

private:

	double getPitch(const QVector4D &q) const;

	double getRoll(const QVector4D &q) const;

	double getYaw(const QVector4D &q) const;

	void showNavigation();

	trikControl::BrickInterface &mBrick;

	const int mGyroCalibrCount = 300;
	int mInitCount;

	QVector4D q;
	QVector4D bias;
	QMatrix4x4 P;
	QMatrix4x4 Q;
	QMatrix4x4 R;

	trikKernel::TimeVal mLastUpdate;

	const double mRadToDeg = 180.0 / M_PI;
	const double mGyroToRad = 0.07 / mRadToDeg; //0.00875 - 0.0175 - 0.07(2000dps)
};
