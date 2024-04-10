#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSerialPortInfo>
#include <QObject>
#include <QDir>
#include <QFile>
#include <QtCharts>
#include <Eigen/Eigen>
#include "portthread.h"
#include "navigation.h"
#include "mainassist.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

//class MainWindow;
//class Plot;
using namespace Eigen;
//class DataFrame;





// 主窗口类
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
    bool P_Read_Con;
    bool P_Write_Con;
    bool Saving;
    bool Naving;

    void Init();//初始化
    void ReadPort();//读取串口
    void SetPortRead();//设置读取串口
    void SetPortWrite();//设置输出串口
    void SetReadState(bool yes);
    void SetWriteState(bool yes);
    void SetButton();
    void StartWork();
    void recieve(const QByteArray &s);//接受数据，解帧
    void decode(QByteArray datain,int type,Data &dataout);//解帧，无用
    void shakehand();//发送握手协议（100惯导用）
    void shakehand2();//发送握手协议（小三轴转台用）
    void sendPort();
    void ShowErrot(const QString &s);//无用
    void ShowImu(const double imu[7]);//展示数据
    void ShowRot(const double rot[4]);//展示数据
    void ShowNav(const VectorXd avp);//展示数据

    void StartNav();//开始导航

    void RotSet();//发送转台命令

    QSerialPortInfo portInfo;//串口信息
    PortThread readThread;//读取串口
    PortThread writeThread;//发送串口
    QDir dir;//文件夹
    QFile fileimu;//文件名
    QFile filerot;//文件名

    struct inData{
        int index;
        int wx;
        int wy;
        int wz;
        int ax;
        int ay;
        int az;//待添加
    };
    struct dataLine
    {
        long int num=0;
        QValueAxis* yaxis=new QValueAxis();
        int ymin=-10;
        int ymax=10;
        QLineSeries* data=new QLineSeries();
    };
    QValueAxis* xaxis1=new QValueAxis();
    QValueAxis* xaxis2=new QValueAxis();
    int xmin=0;
    int xmax=1000;
    //inDataFrame ind_frame;
    //inData ind_resolve;
    long int point[6];
    double rot[3];
    double imusp[7];
    double imu1s[7];

    dataLine groline[3];
    dataLine accline[3];
    dataLine rotline[2];

    QByteArray dataTemp;
    long int lostDataNum;
    int dataImuNum;
    int dataRotNum;

    QChart* groChart;
    QChart* accChart;

    Plot groPlot;
    Plot accPlot;

    Data datain;

    Navigation navthread;
};



#endif // MAINWINDOW_H
