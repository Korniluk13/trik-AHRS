#include "integrator.h"

#include <QDebug>

Integrator::Integrator(trikControl::BrickInterface &brick)
	: mBrick(brick)
	, mQ(1, 0, 0, 0)
	, mBias(0, 0, 0, 0)
{
	mInitCount = mGyroCalibrCount;
}

Integrator::~Integrator()
{}

void Integrator::integrate(const QVector<int> &gyro, const trikKernel::TimeVal &t)
{
	const QQuaternion qGyro(0, gyro[1], -gyro[0], gyro[2]);

	if (mInitCount) {
		mLastUpdate = t;
		mBias += qGyro;
		mInitCount--;

		if (!mInitCount) {
			mBias /= mGyroCalibrCount;
		}

		return;
    }

	const double dt = (t - mLastUpdate).toMcSec() / 1000000.0;
	mLastUpdate = t;

	const QQuaternion deltaW = (qGyro - mBias) * mGyroRadToDeg * dt;

	QQuaternion deltaQ;

	const double c1 = cos(deltaW.x() / 2);
	const double s1 = sin(deltaW.x() / 2);
	const double c2 = cos(deltaW.y() / 2);
	const double s2 = sin(deltaW.y() / 2);
	const double c3 = cos(deltaW.z() / 2);
	const double s3 = sin(deltaW.z() / 2);

	deltaQ.setScalar(c1 * c2 * c3 + s1 * s2 * s3);
	deltaQ.setX(s1 * c2 * c3 - c1 * s2 * s3);
	deltaQ.setY(c1 * s2 * c3 + s1 * c2 * s3);
	deltaQ.setZ(c1 * c2 * s3 - s1 * s2 * c3);

	mQ *= deltaQ;
	mQ.normalize();

	showNavigation();
}

double Integrator::getRoll(const QQuaternion &q) const
{
	return asin(-2 * q.x() * q.z()+2 * q.scalar() * q.y()) * mRadToDeg;
}

double Integrator::getPitch(const QQuaternion &q) const
{
	return atan2(2 * q.y()*q.z() + 2 * q.scalar() * q.x()
				 , 1 - 2 * q.x() * q.x() - 2 * q.y() * q.y())
			* mRadToDeg;
}

double Integrator::getYaw(const QQuaternion &q) const
{
	return atan2(2 * q.x() * q.y() + 2 * q.scalar() * q.z()
				 , 1 - 2 * q.y() * q.y() - 2 * q.z() * q.z())
			* mRadToDeg;
}

void Integrator::showNavigation()
{
	qDebug() << getPitch(mQ) << " " << getRoll(mQ) << " " << getYaw(mQ);
}
