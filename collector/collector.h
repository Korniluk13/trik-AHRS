#include <QtCore/QObject>

#include <fstream>

#include <trikControl/brickInterface.h>
#include <trikKernel/timeVal.h>

class Collector : public QObject
{
	Q_OBJECT
public:

	Collector(trikControl::BrickInterface &brick);

	~Collector();

signals:

	void endWriting();

public slots:

	void writeInfo(QVector<int> gyro, trikKernel::TimeVal t);

	void updateAccel(QVector<int> accel, trikKernel::TimeVal t);

	void closeFile();

private:

	trikControl::BrickInterface &mBrick;
	QVector<int> mAccel;
	int mCounter;
	trikKernel::TimeVal mStartTime;

	std::ofstream mResult;

	//Count for
	const int maxCount = 5000;
};
