#include "filter.h"

#include <QDebug>

double Filter::getRoll(const QQuaternion &q) const
{
	return asin(-2 * q.x() * q.z()+2 * q.scalar() * q.y()) * mRadToDeg;
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
	qDebug() << getPitch(mQ) << " " << getRoll(mQ) << " " << getYaw(mQ);
}

void Filter::updateNavigation(const QVector<int> &gyro, const trikKernel::TimeVal &t)
{
	QQuaternion qGyro(0, -gyro[1], gyro[0], gyro[2]);

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
	QQuaternion qAccel(0, accel[0], accel[1], accel[2]);

	qGyro = (qGyro - mBias) * mGyroRadToDeg;

	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	qDot1 = 0.5f * (-mQ.x() * qGyro.x() - mQ.y() * qGyro.y() - mQ.z() * qGyro.z());
	qDot2 = 0.5f * (mQ.scalar() * qGyro.x() + mQ.y() * qGyro.z() - mQ.z() * qGyro.y());
	qDot3 = 0.5f * (mQ.scalar() * qGyro.y() - mQ.x() * qGyro.z() + mQ.z() * qGyro.x());
	qDot4 = 0.5f * (mQ.scalar() * qGyro.z() + mQ.x() * qGyro.y() - mQ.y() * qGyro.x());

	QQuaternion qDot = 0.5 * mQ * qGyro;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((qAccel.x() == 0.0f) && (qAccel.y() == 0.0f) && (qAccel.z() == 0.0f)))
	{

		qAccel.normalize();

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * mQ.scalar();
		_2q1 = 2.0f * mQ.x();
		_2q2 = 2.0f * mQ.y();
		_2q3 = 2.0f * mQ.z();
		_4q0 = 4.0f * mQ.scalar();
		_4q1 = 4.0f * mQ.x();
		_4q2 = 4.0f * mQ.y();
		_8q1 = 8.0f * mQ.x();
		_8q2 = 8.0f * mQ.y();
		q0q0 = mQ.scalar() * mQ.scalar();
		q1q1 = mQ.x() * mQ.x();
		q2q2 = mQ.y() * mQ.y();
		q3q3 = mQ.z() * mQ.z();

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * qAccel.x() + _4q0 * q1q1 - _2q1 * qAccel.y();
		s1 = _4q1 * q3q3 - _2q3 * qAccel.x() + 4.0f * q0q0 * mQ.x() - _2q0 * qAccel.y() - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * qAccel.z();
		s2 = 4.0f * q0q0 * mQ.y() + _2q0 * qAccel.x() + _4q2 * q3q3 - _2q3 * qAccel.y() - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * qAccel.z();
		s3 = 4.0f * q1q1 * mQ.z() - _2q1 * qAccel.x() + 4.0f * q2q2 * mQ.z() - _2q2 * qAccel.y();
		recipNorm = sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 /= recipNorm;
		s1 /= recipNorm;
		s2 /= recipNorm;
		s3 /= recipNorm;

		qDot -= mFilterCoeff * QQuaternion(s0, s1, s2, s3);
	}

	// Integrate rate of change of quaternion to yield quaternion

	QQuaternion deltaQ;

/*	const double c1 = cos(deltaW.x() / 2);
	const double s1 = sin(deltaW.x() / 2);
	const double c2 = cos(deltaW.y() / 2);
	const double s2 = sin(deltaW.y() / 2);
	const double c3 = cos(deltaW.z() / 2);
	const double s3 = sin(deltaW.z() / 2);

	deltaQ.setScalar(c1 * c2 * c3 + s1 * s2 * s3);
	deltaQ.setX(s1 * c2 * c3 - c1 * s2 * s3);
	deltaQ.setY(c1 * s2 * c3 + s1 * c2 * s3);
	deltaQ.setZ(c1 * c2 * s3 - s1 * s2 * c3);*/


	mQ += qDot * dt;
	mQ.normalize();

	showNavigation();
}
