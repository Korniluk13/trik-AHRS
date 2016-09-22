#include "integrator.h"

#include <QDebug>
#include <QMatrix4x4>

MatrixIntegrator::MatrixIntegrator(trikControl::BrickInterface &brick)
	: mBrick(brick)
	, mQ(0, 0, 0, 1)
	, mBias(0, 0, 0, 0)
{
	mInitCount = mGyroCalibrCount;
}

MatrixIntegrator::~MatrixIntegrator()
{}

void MatrixIntegrator::integrate(const QVector<int> &gyro, const trikKernel::TimeVal &t)
{
	QVector4D qGyro(gyro[1], -gyro[0], gyro[2], 0);

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

	qGyro -= mBias;
	qGyro *= mGyroToRad;
	const double theta = qGyro.length() * dt;

	const double c = (1 - theta * theta / 8);
	QMatrix4x4 C(c, 0, 0, 0,
				 0, c, 0, 0,
				 0, 0, c, 0,
				 0, 0, 0, c);

	const double wx = qGyro.x();
	const double wy	= qGyro.y();
	const double wz = qGyro.z();

	QMatrix4x4 Lambda(0, wz, -wy, wx,
					  -wz, 0, wx, wy,
					  wy, -wx, 0, wz,
					  -wx, -wy, -wz, 0);

	QMatrix4x4 F = C + 0.5 * dt * Lambda;

//	qDebug() << F(0, 0) << " " << F(0, 1) << " " << F(0, 2) << " " << F(0,3) << "\n"
//			 << F(1, 0) << " " << F(1, 1) << " " << F(1, 2) << " " << F(1,3) << "\n"
//			 << F(2, 0) << " " << F(2, 1) << " " << F(2, 2) << " " << F(2,3) << "\n"
//			 << F(3, 0) << " " << F(3, 1) << " " << F(3, 2) << " " << F(3,3) << "\n";

	mQ = F * mQ;
	mQ.normalize();
//	qDebug() << mQ.x() << " " << mQ.y() << " " << mQ.z() << " " << mQ.w();

	showNavigation();
}

double MatrixIntegrator::getRoll(const QVector4D &q) const
{
	return asin(-2 * q.x() * q.z()+2 * q.w() * q.y()) * mRadToDeg;
}

double MatrixIntegrator::getPitch(const QVector4D &q) const
{
	return atan2(2 * q.y() * q.z() + 2 * q.w() * q.x()
				 , 1 - 2 * q.x() * q.x() - 2 * q.y() * q.y())
			* mRadToDeg;
}

double MatrixIntegrator::getYaw(const QVector4D &q) const
{
	return atan2(2 * q.x() * q.y() + 2 * q.w() * q.z()
				 , 1 - 2 * q.y() * q.y() - 2 * q.z() * q.z())
			* mRadToDeg;
}

void MatrixIntegrator::showNavigation()
{
	qDebug() << getPitch(mQ) << " " << getRoll(mQ) << " " << getYaw(mQ);
}
