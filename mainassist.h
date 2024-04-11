#ifndef MAINASSIST_H
#define MAINASSIST_H

#include <QtCharts>
#include <Eigen/Eigen>

using namespace Eigen;

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
    //数据帧格式
    struct inDataFrame{
        uint start;//帧头
        uint type1;//分类1
        uint lens1;//帧长1
        uint type2;//分类2
        uint lens2;//帧长2
        uint end;//帧尾
    }inframe;

    struct outDataFrame{
        uint len;
        quint8 start1;
        quint8 start2;
        quint8 start3;

        quint8 innpos;
        quint8 innrelapos;
        quint8 innv;
        quint8 innsway;
        quint8 innenable;
        quint8 innstop;

        quint8 midpos;
        quint8 midrelapos;
        quint8 midv;
        quint8 midsway;
        quint8 midenable;
        quint8 midstop;

        quint8 outpos;
        quint8 outrelapos;
        quint8 outv;
        quint8 outsway;
        quint8 outenable;
        quint8 outstop;

        quint32 para1;
        quint32 para2;
        quint8 ccwdirect;
        quint8 cwdirect;
        quint8 shortdirect;
        quint8 enable;
        quint8 disenable;

        quint8 end;

        double inn0;
        double mid0;
        double out0;

    }outframe;

    struct imuFrame{
        quint32 time;
        qint32 wx;
        quint16 tx;
        qint32 wy;
        quint16 ty;
        qint32 wz;
        quint16 tz;
        quint8 ax1;
        quint8 ax2;
        quint8 ay1;
        quint8 ay2;
        quint8 az1;
        quint8 az2;
    }imutmp;

    struct rotFrame{
        quint32 time;
        qint32 out;
        qint32 tmp1;
        qint32 mid;
        qint32 tmp2;
        qint32 inn;
        qint32 tmp3;
    }rottmp;

    //惯导握手数据
    QByteArray imushakehand;
    //转台握手数据
    QByteArray rotshakehand;

    // 单点数据
    double imu[7]; double rot[4];
    // 1s累加
    double imu1s[7]; double rot1s[4];
    // 累加次数
    uint imu1sNum=0; uint rot1sNum=0;

    // 频率
    uint freq1;uint freq2;


    // 对准数据数量
    int alignnum;
    // 对准数据
    MatrixXd alignData;

    Data();
    // 初始化
    void init();

    // 更新imu数据
    bool updateImu(QByteArray data);
    // 更新rot数据
    bool updateRot(QByteArray data);

    // imu数据转QString
    QString ImuToString();
    // rot数据转QString
    QString RotToString();
    // imu1s数据转QString
    QString Imu1sToString();
    // rot1s数据转QString
    QString Rot1sToString();

    //转台命令输出
    QByteArray rotSend(uint type,double data1,double data2);

signals:
    void sendImu(const double imu[7]);
    void sendRot(const double rot[4]);
};




#endif // MAINASSIST_H
