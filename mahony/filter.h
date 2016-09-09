#include <trikControl/brickInterface.h>
#include <trikKernel/timeVal.h>

#include <QtCore/QObject>
#include <QQuaternion>

class Filter : public QObject
{
	Q_OBJECT
public:

	Filter(trikControl::BrickInterface &brick, int filterCoeff = 300);

	~Filter();

signals:

	void navigationReady();

public slots:

	double getPitch(const QQuaternion &q) const;
	double getRoll(const QQuaternion &q) const;
	double getYaw(const QQuaternion &q) const;

	void showNavigation();

	void updateNavigation(const QVector<int> &gyro, const trikKernel::TimeVal &t);

private:

	trikControl::BrickInterface &mBrick;
	int mFilterCoeff;

	QQuaternion mQ;
	QQuaternion mBias;

	int mInitCount;
	trikKernel::TimeVal mLastUpdate;

	const int mGyroCalibrCount = 200;
	const double mRadToDeg = 180.0 / M_PI;
	const double mGyroRadToDeg = 0.07 / mRadToDeg; //0.00875 - 0.0175 - 0.07(2000dps)
};
