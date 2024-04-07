#ifndef NAVIGATION_H
#define NAVIGATION_H


#include <iostream>
#include <QCoreApplication>
#include <QQueue>
#include <QMutex>
#include <QWaitCondition>
#include <QThread>
//#include <QObject>
#include <Eigen/Dense>

using namespace Eigen;

class Navigation : public QThread {
    Q_OBJECT
public:
    Navigation();

    void setavp0(MatrixXd avp0,MatrixXd ts);

    void setParameters(int aligntime, int frequency);

    void startProcessing();

    void stopProcessing();

    void receiveData(double data[7]);

    MatrixXd getavp();
protected:
    void run() override;
private:
    QQueue<MatrixXd> dataQueue_;
    QMutex mutex_;
    QWaitCondition cv_;
    bool running_;
    bool dataProcessed_;
    uint len;
    uint imunum;
    uint alignTime;
    uint f;
    uint avp1snum;
    double ts;
    MatrixXd alignData;
    MatrixXd NavData;
    //VectorXd avp0;
    VectorXd avp;
signals:
    void sendavp(const VectorXd avp);
};
#endif // NAVIGATION_H
