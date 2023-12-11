# ifndef INS_h
# define INS_h

# include <Eigen/Eigen>
# include <QFile>

// 文件概述 2021年3月23日
// 本文件包含了INS的相关类和函数，包括：
// 1. glvs类，定义了全局变量
// 2. earth类，定义了地球参数
// 3. INSerror类，定义了INS误差模型
// 4. Rot类，定义了转台参数
// 5. INS类，定义了惯导解算类
using namespace std;
using namespace Eigen;

#define PI M_PI
#define DEG (M_PI/180.0)

// 定义类
class glvs;
class earth;
class INSerror;
class Rot;
class INS;

// 定义全局变量
extern glvs glv;
extern const Vector3d O31;
extern INSerror INSerr;
extern Rot rot;
extern INS nav;

// 定义函数
Vector3d m2att(const Matrix3d Cnb);// 旋转矩阵转姿态
Vector3d q2att(const Quaterniond qnb);// 四元数转姿态
Matrix3d q2mat(const Quaterniond qnb);// 四元数转旋转矩阵
Matrix3d a2mat(const Vector3d att);// 姿态转旋转矩阵
Matrix3d rv2mat(const Vector3d rv);// 旋转向量转旋转矩阵
Vector3d qmulv(const Quaterniond qnb,const Vector3d fb);// 四元数旋转向量，qnb为四元数，fb为向量
Vector3d rotv(const Vector3d rv,const Vector3d vi);// 旋转向量转向量，rv为旋转向量，vi为向量
Quaterniond qupdt2(const Quaterniond qnb,const Vector3d rv_ib,const Vector3d rv_in);// 四元数更新，qnb为四元数，rv_ib为陀螺仪数据，rv_in为地球转速
MatrixXd wm2wtcoef(double ts, int n);// 多子样系数


// 全局变量类
class glvs{
public:
    double f;// 不是采用频率，而是和地球有关的参数
    double ts;// 采样周期
    double t0;// 初始时间，由接收到的第一帧数据的时间戳决定
    double Re;// 地球半径
    //double e2;
    double wie,meru;// 地球自转角速度
    double g0;// 重力加速度
    double tscale;// 时间尺度，=1为s
    Vector3d pos0;// 初始位置
    Quaterniond qnb0;// 初始四元数
    Vector3d att0;//初始姿态
    Vector3d vn0,v0,wm_1,vm_1;// 初始速度，初始速度，上一时刻陀螺仪数据，上一时刻加速度计数据
    MatrixXd cs;//划桨误差补偿系数
    int csmax;// 最大划桨误差补偿子样数

    double e, e2, ep, ep2, Rp;// 地球参数
    double mg, ug, deg, min, sec, hur, ppm, ppmpsh;// commonly used units
    double dps, dph, dpsh, dphpsh, dph2, dphpg, ugpsh, ugpsHz, ugpg2, mpsh, mpspsh, secpsh,\
        mGal,uGal,ws,mas,rps,dpss,Hz,dphpsHz,dphpg2,mil,nm,kn;

    glvs(double Re=6378137.0,double f=(1.0/298.257),double wie=15.0411/3600*DEG,double g0=9.7803267714);
};



class Rot{
public:
    double rx,rz,dth;// 转台参数
    Matrix3d Cnb;// 转台姿态
    Rot(double rx=0,double rz=0,double dth=0);// 构造函数
    void Setdth(double dth);// 设置转台非正交角
    void update(double rx,double rz);// 更新转台姿态
    //Matrix3d getMatrix(void);
};

class INSerror{
public:
    Vector3d Ka, Kg, da, dg;// 加速度计标度因数，陀螺仪标度因数，加速度计零偏，陀螺仪零偏
    Matrix3d Ea, Eg;// 加速度计安装误差，陀螺仪安装误差
    double dth;// 转台非正交角
    INSerror(void);
    void loadData(QString filepath);
};

// 定义地球参数
class earth{
public:

    double a,b;
    double f,e,e2;
    double wie;
    double Re;
    double g0;
    double g;


    double sl, sl2, sl4, cl, tl, RMh, RNh, clRNh, f_RMh, f_RNh, f_clRNh;
    Vector3d pos, vn, wnie, wnen, wnin,wnien, gn, gcc, *pgn;

    earth();

    void update(const Vector3d &pos,const Vector3d &vn=O31);

};

class INS{
public:
    earth eth;
    Rot turn;
    INSerror err;
    double ts;
    uint nn;
    double nts;
    Vector3d wib;

    Vector3d web;
    Vector3d wnb;

    Vector3d fn;
    Vector3d fb;
    Vector3d an;
    Quaterniond qnb;

    Vector3d vn;
    Matrix3d Mpv;
    Vector3d Mpvvn;
    Vector3d pos;

    Matrix3d Cnb0;
    Matrix3d Cnb;
    Vector3d att;

    VectorXd avp;

    INS();
    void init();
    void alignsb(const MatrixXd imu);
    void insupdate(MatrixXd imu);
};


#endif
