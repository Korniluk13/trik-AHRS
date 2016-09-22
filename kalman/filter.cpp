#include "filter.h"

#include <QDebug>
#include <QMatrix4x4>

Integrator::Integrator(trikControl::BrickInterface &brick)
	: mBrick(brick)
	, mInitCount(mGyroCalibrCount)
	, q(QVector4D(0, 0, 0, 1))
	, bias(QVector4D(0, 0, 0, 0))
{
	P(0,0) = 1;
	P(1,1) = 1;
	P(2,2) = 1;
	P(3,3) = 1;

	Q(0,0) = 1;
	Q(1,1) = 1;
	Q(2,2) = 1;
	Q(3,3) = 1;

	const double pConst = 0.0001;
	const double qConst = 0.00001;

	P = pConst * P;
	Q = qConst * Q;

	qDebug() << mInitCount;
}

Integrator::~Integrator()
{}

void Integrator::updateNavigation(const QVector<int> &gyro, const trikKernel::TimeVal &t)
{
	QVector4D qGyro(gyro[1], -gyro[0], gyro[2], 0);

	if (mInitCount) {
		mLastUpdate = t;
		bias += qGyro;
		mInitCount--;

		if (!mInitCount) {
			bias /= mGyroCalibrCount;
		}

		return;
	}

	const double dt = (t - mLastUpdate).toMcSec() / 1000000.0;
	mLastUpdate = t;

	qGyro -= bias;
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

	q = F * q;
	q.normalize();

	P = F * P * F.transposed() + Q;

	QVector3D gravity(0, 0, -1);
	const QVector<int> accel = mBrick.accelerometer()->read();
	QVector3D accelN(accel[0], accel[1], -accel[2]);
	accelN.normalize();
	QVector3D a = QVector3D::crossProduct(accelN, gravity);

	QVector4D z(a, sqrt(2) + QVector3D::dotProduct(gravity, accelN));
	z.normalize();

	QVector4D y = q - z;

	// H is identity matrix
	QMatrix4x4 S;
	S = P + R;

	QMatrix4x4 K;
	K = P * S.inverted();

	q = q + K * y;
	q.normalize();

	QMatrix4x4 I;
	I(0,0) = 1; I(0,1) = 0; I(0,2) = 0; I(0,3) = 0;
	I(1,0) = 0; I(1,1) = 1; I(1,2) = 0; I(1,3) = 0;
	I(2,0) = 0; I(2,1) = 0; I(2,2) = 1; I(2,3) = 0;
	I(3,0) = 0; I(3,1) = 0; I(3,2) = 0; I(3,3) = 1;

	P = (I - K) * P;

	showNavigation();
}

double Integrator::getRoll(const QVector4D &q) const
{
	return asin(-2 * q.x() * q.z()+2 * q.w() * q.y()) * mRadToDeg;
}

double Integrator::getPitch(const QVector4D &q) const
{
	return atan2(2 * q.y() * q.z() + 2 * q.w() * q.x()
				 , 1 - 2 * q.x() * q.x() - 2 * q.y() * q.y())
			* mRadToDeg;
}

double Integrator::getYaw(const QVector4D &q) const
{
	return atan2(2 * q.x() * q.y() + 2 * q.w() * q.z()
				 , 1 - 2 * q.y() * q.y() - 2 * q.z() * q.z())
			* mRadToDeg;
}

void Integrator::showNavigation()
{
	qDebug() << getPitch(q) << " " << getRoll(q) << " " << getYaw(q);
}
