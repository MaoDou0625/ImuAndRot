#include "navigation.h"
#include <iostream>
#include <QCoreApplication>
#include <QQueue>
#include <QMutex>
#include <QWaitCondition>
#include <QThread>
#include <Eigen/Eigen>
#include "INS.h"

using namespace Eigen;
using namespace std;


Navigation::Navigation(){
    dataQueue_.clear();
    running_=false;
    dataProcessed_=false;
    alignTime=1000;
    f=1000;
    ts=1.0/f;
    nav.init();
    nav.ts=ts;
    imunum=0;
    avp1snum=0;
    //avp0.resize(9);
    avp.resize(10);
    NavData.resize(4,7);
}


void Navigation::startProcessing() {
    running_ = true;
    len=0;
    //alignData=Matrix<double,Dynamic,Dynamic>();
    alignData.resize(1,7);
    alignData.setZero();
    dataQueue_.clear();
    //avp0=VectorXd::Zero(9);
    avp=VectorXd::Zero(10);
    start();
}

void Navigation::stopProcessing() {
    running_ = false;
    cv_.wakeOne();  // 唤醒线程以允许停止
    wait();
}

// 接收数据
void Navigation::receiveData(double data[7]) {
    QMutexLocker locker(&mutex_);
    // 校正后的imu数据
    VectorXd imuCorrect(7);
    // 标定数据
    VectorXd Ng(3);//陀螺仪数据
    VectorXd Na(3);//加速度计数据
    VectorXd gout(3);//校正后的陀螺仪数据
    VectorXd aout(3);//校正后的加速度计数据

    Ng<<data[0],data[1],data[2];
    Na<<data[3],data[4],data[5];

    gout=INSerr.Eg.inverse()*(((Ng-INSerr.dg/f).array()/INSerr.Kg.array()).matrix());
    aout=(INSerr.Ea.inverse()*(((Na-INSerr.da/f).array()/INSerr.Ka.array()).matrix()))*glv.g0;
    double dt=data[6];
    imuCorrect<<gout,aout,dt;
    dataQueue_.enqueue(imuCorrect.transpose());
    cv_.wakeOne();  // 唤醒线程以处理数据
}

// 处理数据
void Navigation::run() {
    // 线程主循环
    while (running_ || dataQueue_.size() >= 1) {
        // 等待数据
        QMutexLocker locker(&mutex_);
        // 等待数据或停止信号
        while (dataQueue_.size() < 1 && running_) {
            cv_.wait(&mutex_);  // 等待数据到达或停止信号
        }
        if (len<alignTime*f&&dataQueue_.isEmpty()==false) {
            alignData=alignData+dataQueue_.dequeue()/alignTime/f;len++;
        }else if(len==alignTime*f&&dataQueue_.isEmpty()==false){
            //对准
            len++;
            NavData.row(0)= dataQueue_.dequeue();
            alignData(0,6)=NavData(0,6);
            nav.alignsb(alignData);
            avp1snum++;
            imunum++;
        }else{
            NavData.row(imunum)= dataQueue_.dequeue();
            imunum++;
            avp1snum++;
        }
        locker.unlock();

        if (imunum>=nav.nn) {
            // 导航解算
            nav.insupdate(NavData);
            nav.vn(2)=glv.vn0(2);
            avp<<nav.avp,NavData(3,6)-glv.t0;
            cout<<"avp:"<<nav.avp.transpose()<<"  t:  "<<NavData(3,6)-glv.t0<<endl;
            imunum=0;
            if(avp1snum>=f){
                emit this->sendavp(avp);
                avp1snum=0;
            }
        }

    }
}

void Navigation::setParameters(int aligntime, int frequency){
    alignTime=aligntime;
    f=frequency;
}

MatrixXd Navigation::getavp(){
    //return matlab.getData("avpout", 1, 10);
    return avp;
}
