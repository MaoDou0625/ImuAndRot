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

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow;
class Plot;
using namespace Eigen;
//class DataFrame;

// 绘图类
class Plot{
public:
    //Plot();
    //图表
    QChart* Chart;
    //横坐标轴
    QValueAxis* xaxis;
    //横坐标轴最小值
    double xmin;
    //横坐标轴最大值
    double xmax;

    //曲线数量
    int num;
    //纵坐标轴列表
    QList<QValueAxis*> yaxis;
    //曲线列表
    QList<QLineSeries*> line;
    //曲线颜色列表
    QList<QColor> color;
    //曲线名称列表
    QList<QString> name;
    // 曲线最小值
    QList<double> ymin;
    // 曲线最大值
    QList<double> ymax;

    // 初始化
    void init();
    // 添加曲线
    void addLine(QString name,QColor color,double ymin,double ymax);
    // 删除曲线
    void delLine(int index);
    // 更新曲线
    void updateLine(int index,double x,double y);
    // 更新曲线
    void updateLine(int index,double x,double y,double z);
    // 更新曲线
    void updateLine(int index,double x,double y,double z,double w);
    // 设置纵坐标轴范围
    void setYRange(int index,double ymin,double ymax);
};

// 数据帧类
class Data:public QObject{
    Q_OBJECT
public:
    // 构造函数
    //Data(QObject *parent=nullptr);
    // imu单点数据
    double imu[7];
    // rot单点数据
    double rot[3];
    // imu1s累加
    double imu1s[7];
    // rot1s累加
    double rot1s[3];
    // imu1s累加次数
    uint imu1sNum=0;
    // rot1s累加次数
    uint rot1sNum=0;

    // 对准数据数量
    int alignnum;
    // 对准数据
    MatrixXd alignData;

    Data();
    // 初始化
    void init();

    // 更新imu数据
    bool updateImu(double imu[7]);
    // 更新rot数据
    bool updateRot(double rot[3]);

    // imu数据转QString
    QString ImutoString();
    // rot数据转QString
    QString RottoString();
    // imu1s数据转QString
    QString Imu1stoString();
    // rot1s数据转QString
    QString Rot1stoString();

signals:
    void sendImu(const double imu[7]);
    void sendRot(const double rot[3]);
};



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

    void Init();
    void ReadPort();
    void SetPortRead();
    void SetPortWrite();
    void SetReadState(bool yes);
    void SetWriteState(bool yes);
    void SetButton();
    void StartWork();
    void recieve(const QByteArray &s);
    void shakehand();
    void sendPort();
    void ShowErrot(const QString &s);
    void ShowImu(const double imu[7]);
    void ShowRot(const double rot[3]);
    void ShowNav(const VectorXd avp);

    void StartNav();

    void outRxisSet();

    QSerialPortInfo portInfo;
    PortThread readThread;
    PortThread writeThread;
    QDir dir;
    QFile fileimu;
    QFile filerot;
    //QString filepath;

    struct inDataFrame{
        uint start;
        uint type1;
        uint lens1;
        uint type2;
        uint lens2;
        uint end;//可去除
    };
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
    inDataFrame ind_frame;
    inData ind_resolve;
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

    Data tmpdata;

    Navigation navthread;
};



#endif // MAINWINDOW_H
