#include "collector.h"

#include <QDebug>

Collector::Collector(trikControl::BrickInterface &brick)
	: mBrick(brick)
	, mCounter(0)
{
	mResult.open("1.txt");
	mAccel << 0 << 0 << 0;
}

Collector::~Collector()
{
	mResult.close();
}

void Collector::writeInfo(QVector<int> gyro, const trikKernel::TimeVal t)
{
	if (mCounter == 0)
		mStartTime = t;

	if (mCounter == 500) {
		qDebug() << "init end";
	}

	if (mCounter == maxCount) {
		emit endWriting();
		return;
	}

	mResult << (t - mStartTime).toMcSec() << " " <<
			   gyro[0] << " " << gyro[1] << " " << gyro[2] << " " <<
			   mAccel[0] << " " << mAccel[1] << " " << mAccel[2] << std::endl;

	mCounter++;
}

void Collector::updateAccel(QVector<int> accel, trikKernel::TimeVal t)
{
	mAccel[0] = accel[0];
	mAccel[1] = accel[1];
	mAccel[2] = accel[2];
}

void Collector::closeFile()
{
	mResult.close();
	qDebug() << "end";
}
