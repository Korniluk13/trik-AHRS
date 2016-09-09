#include "filter.h"

#include <QDebug>

double Filter::getRoll(const QQuaternion &q) const
{
	return asin(2 * q.scalar() * q.y() - 2 * q.x() * q.y()) * mRadToDeg;
}

double Filter::getPitch(const QQuaternion &q) const
{
	return atan2(2 * q.y()*q.z() + 2 * q.scalar() * q.x()
				 , 1 - 2 * q.x() * q.x() - 2 * q.y() * q.y())
			* mRadToDeg;
}

double Filter::getYaw(const QQuaternion &q) const
{
	return atan2(2 * q.x() * q.y() + 2 * q.scalar() * q.z()
				 , 1 - 2 * q.y() * q.y() - 2 * q.z() * q.z())
			* mRadToDeg;
}

Filter::Filter(trikControl::BrickInterface &brick, int filterCoeff)
    : mBrick(brick)
	, mFilterCoeff(filterCoeff)
	, mQ(1, 0, 0, 0)
	, mBias(0, 0, 0, 0)
{
	mInitCount = mGyroCalibrCount;
}

Filter::~Filter()
{}

void Filter::showNavigation()
{
	qDebug() << getPitch(mQ) << " " << getRoll(mQ);
}

void Filter::updateNavigation(const QVector<int> &gyro, const trikKernel::TimeVal &t)
{
	const QQuaternion qGyro(0, gyro[1], -gyro[0], gyro[2]);

	if (mInitCount) {
		mLastUpdate = t;
		mBias += qGyro;
		mInitCount--;

		if (!mInitCount) {
			mBias /= mGyroCalibrCount;
			qDebug() << mBias.x() << " " << mBias.y() << " " << mBias.z();
		}

		return;
    }

	double dt = (t - mLastUpdate).toMcSec() / 1000000.0;
	mLastUpdate = t;

	const QVector<int> accel = mBrick.accelerometer()->read();
	QQuaternion qAccel(0, accel[0], accel[1], -accel[2]);
	qAccel.normalize();

	QQuaternion gravity(0,0,0,-1);
	gravity = mQ.conjugate() * gravity * mQ;

	const QQuaternion qErr = (qAccel * gravity - gravity * qAccel) / 2;
	const QQuaternion deltaW = (qGyro - mBias + mFilterCoeff * qErr) * mGyroRadToDeg * dt;

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
