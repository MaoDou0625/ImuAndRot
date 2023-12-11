#include "INS.h"
#include <Eigen/Eigen>
# include <iostream>
//#include <QString>
//#include <QFile>
#include <QDebug>

using namespace std;
using namespace Eigen;

// 定义全局变量
glvs glv;
const Vector3d O31(0,0,0);
INSerror INSerr;
Rot rot;
INS nav;


// 定义函数
Vector3d m2att(const Matrix3d Cnb){
    // 旋转矩阵转换为欧拉角，旋转顺序为3-1-2
    Vector3d tmp=Cnb.eulerAngles(2,0,1);
    Vector3d att;
    att(0)=tmp(1);att(1)=tmp(2);att(2)=tmp(0);
    return att;
}
Vector3d q2att(const Quaterniond qnb){
    // 四元数转换为欧拉角，旋转顺序为3-1-2
    Vector3d tmp=qnb.toRotationMatrix().eulerAngles(2,0,1);
    Vector3d att;
    att(0)=tmp(1);att(1)=tmp(2);att(2)=tmp(0);
    return att;
}
Matrix3d q2mat(const Quaterniond qnb){
    // 四元数转换为旋转矩阵
    Matrix3d Cnb=qnb.toRotationMatrix();
    return Cnb;
}
Matrix3d a2mat(const Vector3d att){
    // 欧拉角转换为旋转矩阵，旋转顺序为3-1-2
    double si=sin(att[0]); double sj=sin(att[1]); double sk=sin(att[2]);
    double ci=cos(att[0]); double cj=cos(att[1]); double ck=cos(att[2]);
    Matrix3d Cnb;
    Cnb<<cj*ck-si*sj*sk,-ci*sk,sj*ck+si*cj*sk,\
                                                                            cj*sk+si*sj*ck,ci*ck,sj*sk-si*cj*ck,\
                                            -ci*sj,si,cj*ci;
    return Cnb;
}

Matrix3d rv2mat(const Vector3d rv){
    // 旋转矢量转换为旋转矩阵
    AngleAxisd rvtmp(rv.norm(),rv.normalized());
    Matrix3d Cnb=rvtmp.toRotationMatrix();
    return Cnb;
}

Vector3d qmulv(const Quaterniond qnb,const Vector3d fb){
    // 四元数乘向量
    Vector3d fn=qnb.toRotationMatrix()*fb;
    return fn;
}

Vector3d rotv(const Vector3d rv,const Vector3d vi){
    // 旋转向量
    Vector3d vo=rv2mat(rv)*vi;
    return vo;
}

Quaterniond qupdt2(const Quaterniond qnb, const Vector3d rv_ib, const Vector3d rv_in){
    // 四元数更新,qnb为四元数，rv_ib为陀螺仪数据，rv_in为地球转速
    // -qnbin*qnb*qnbib;
    AngleAxisd rvibtmp(rv_ib.norm(),rv_ib.normalized());
    Quaterniond qnb_ib(rvibtmp);
    AngleAxisd rvintmp(rv_in.norm(),-rv_in.normalized());
    Quaterniond qnb_in(rvintmp);
    Quaterniond qnb1=qnb_in*qnb*qnb_ib;
    return qnb1;
}


glvs::glvs(double Re,double f,double wie,double g0){
    this->Re=Re;
    this->f=f;
    this->wie=wie;
    this->g0=g0;
    this->ts=1.0/1000;


    this->pos0<<30.68665190913114*DEG,120.66426478617899*DEG,0;
    this->qnb0=a2mat(Vector3d(0,0,0));
    this->att0=q2att(this->qnb0);
    this->vn0<<0,0,0;
    this->Rp=(1-this->f)*this->Re;
    this->e=sqrt(2*this->f-this->f*this->f);
    this->e2=this->e*this->e;
    this->ep=sqrt(this->Re*this->Re-this->Rp*this->Rp)/this->Rp;
    this->ep2=this->ep*this->ep;
    this->meru=this->wie/1000;
    this->mg=1.0e-3*this->g0;
    this->ug=1.0e-6*this->g0;
    this->mGal=1.0e-3*0.01;
    this->uGal=this->mGal/1000;
    this->ugpg2=this->ug/this->g0/this->g0;
    this->ws=1/sqrt(this->Re/this->g0);
    this->ppm=1.0e-6;
    this->deg=PI/180;
    this->min=this->deg/60;
    this->sec=this->min/60;
    this->mas=this->sec/1000;
    this->hur=3600;
    this->dps=this->deg/1;
    this->rps=360*this->dps;
    this->dph=this->deg/this->hur;
    this->dpss=this->deg/sqrt(1);
    this->dpsh=this->deg/sqrt(this->hur);
    this->dphpsh=this->dph/sqrt(this->hur);
    this->dph2=this->dph/this->hur;
    this->Hz=1/1;
    this->dphpsHz=this->dph/this->Hz;
    this->dphpg=this->dph/this->g0;
    this->dphpg2=this->dphpg/this->g0;
    this->ugpsHz=this->ug/sqrt(this->Hz);
    this->ugpsh=this->ug/sqrt(this->hur);
    this->mpsh=1/sqrt(this->hur);
    this->mpspsh=1/1/sqrt(this->hur);
    this->ppmpsh=this->ppm/sqrt(this->hur);
    this->mil=2*PI/6000;
    this->nm=1853;
    this->kn=this->nm/this->hur;
    this->wm_1<<0,0,0;
    this->vm_1<<0,0,0;
    this->cs.resize(5,5);
    this->cs<<2/3,0/3,0/3,0/3,0/3,\
                                                      9/20,27/20,0/20,0/20,0/20,\
                                            54/105,92/105,214/105,0/105,0/105,\
                                                  250/504,525/504,650/504,1375/504,0/504,\
                                                      2315/4620,4558/4620,7296/4620,7834/4620,15797/4620;
    this->csmax=this->cs.rows()+1;
    this->v0<<0,0,0;
    this->t0=0;
    this->tscale=1;
}

Rot::Rot(double rx,double rz,double dth){
    this->rx=rx;
    this->rz=rz;
    this->dth=dth;
    this->Cnb.setIdentity();
}

void Rot::Setdth(double dth){
    this->dth=dth;
}

void Rot::update(double rz,double rx){
    this->rx=rx;
    this->rz=rz;

    // 旋转矢量
    Vector3d rvx;rvx<<cos(this->dth),0,-sin(this->dth);
    Vector3d rvz;rvz<<0,0,1;

    // 旋转向量
    AngleAxisd rxv(this->rx,rvx);
    AngleAxisd rzv(this->rz,rvz);

    // 旋转矩阵
    this->Cnb=rxv*rzv;
}

INSerror::INSerror(void){
    // 将Ea，Eg初始化为单位矩阵
    this->Eg.setIdentity();
    this->Ea.setIdentity();
    // 将Ka，Kg初始化为单位向量
    this->Ka<<1,1,1;
    this->Kg<<1,1,1;
    // 将da，dg初始化为0
    this->da.setZero();
    this->dg.setZero();
}

void INSerror::loadData(QString filepath){
    // 读取文件
    QFile file(filepath);
    if(!file.open(QIODevice::ReadOnly | QIODevice::Text)){
        qDebug()<<"INSerror file open error!";
        return;
    }
    QTextStream in(&file);

    QList<QList<double>> data;

    while(!in.atEnd()){
        QString line = in.readLine();
        QStringList list=line.split(" ",Qt::SkipEmptyParts);
        QList<double> datatmp;
        for(int i=0;i<list.size();i++){
            datatmp.append(list[i].toDouble());
        }
        data.append(datatmp);
    }
    // 储存误差中
    for(int i=0;i<3;i++){
        this->Ea(i,0)=data[i][0];
        this->Ea(i,1)=data[i][1];
        this->Ea(i,2)=data[i][2];
        this->Eg(i,0)=data[i+3][0];
        this->Eg(i,1)=data[i+3][1];
        this->Eg(i,2)=data[i+3][2];
        this->Ka(i)=data[i+6][0];
        this->Kg(i)=data[i+9][0];
        this->da(i)=data[i+12][0];
        this->dg(i)=data[i+15][0];
    }
    this->dth=data[18][0];
    qDebug()<<"error";
    cout<<Ea<<endl<<Eg<<endl<<Ka<<endl<<Kg<<endl<<da<<endl<<dg<<endl<<"dth: "<<dth<<endl;
}

earth::earth(){
    this->Re=glv.Re;this->e2=glv.e2;this->wie=glv.wie;this->g0=glv.g0;
    this->update(glv.pos0,glv.vn0);
}

void earth::update(const Vector3d &pos, const Vector3d &vn){
    this->pos=pos;
    this->vn=vn;
    double sl=sin(pos[0]); double sl2=sl*sl; double sl4=sl2*sl2;
    double cl=cos(pos[0]);    double tl=sl/cl;
    double sq=1-this->e2*sl*sl;   double sq2=sqrt(sq);
    double Rn=this->Re/sq2;
    this->RNh=Rn+pos[2];
    this->clRNh=cl*this->RNh;
    this->RMh=Rn*(1-this->e2)/sq+pos[2];
    this->wnie<<0,this->wie*cl,this->wie*sl;
    this->wnen<<-vn[1]/this->RMh,vn[0]/this->RNh,vn[0]/this->RNh*tl;
    this->wnin=this->wnie+this->wnen;
    this->wnien=this->wnie+this->wnin;
    this->g=this->g0*(1+5.27094e-3*sl2+2.32718e-5*sl4)-3.086e-6*pos[2];
    this->gn<<0,0,-this->g;
    this->gcc=this->gn+this->wnien.cross(vn);
}

INS::INS(){
    this->init();
}

void INS::init(){
    this->ts=glv.ts;
    this->nn=4;
    this->nts=nn*glv.ts;
    this->qnb=glv.qnb0;
    this->vn=glv.vn0;
    this->pos=glv.pos0;
    this->att=q2att(this->qnb);
    this->Cnb=q2mat(this->qnb);
    this->avp.resize(9);
    this->avp<<this->att,this->vn,this->pos;

    this->wib=this->Cnb.inverse()*this->eth.wnin;
    this->fn=-this->eth.gn;
    this->fb=this->Cnb.inverse()*this->fn;

    this->wnb=Vector3d::Zero();
    this->web=Vector3d::Zero();
    this->an=Vector3d::Zero();

    this->Mpv<<0,1.0/this->eth.RMh,0,\
                                               1.0/this->eth.clRNh,0,0,\
        0,0,1;
    this->Mpvvn=this->Mpv*this->vn;
}

void INS::alignsb(const MatrixXd imu){
    // 对准,以对准结束时间作为初始时间
    VectorXd tmp=imu.colwise().sum()/glv.ts/imu.rows();
    Vector3d vn1=this->eth.gn;
    Vector3d vn2=this->eth.wnie;

    Vector3d vb1=-tmp.block(3,0,3,1);
    Vector3d vb2=tmp.block(0,0,3,1);

    Vector3d vntmp1=vn1.cross(vn2);
    Vector3d vntmp2=vntmp1.cross(vn1);
    Vector3d vbtmp1=vb1.cross(vb2);
    Vector3d vbtmp2=vbtmp1.cross(vb1);

    Matrix3d tmp1;
    tmp1<<vn1.normalized(),vntmp1.normalized(),vntmp2.normalized();
    Matrix3d tmp2;
    tmp2<<vb1.normalized(),vbtmp1.normalized(),vbtmp2.normalized();
    Matrix3d Cnb=tmp1*tmp2.transpose();
    Quaterniond qnb(Cnb);qnb.normalize();
    this->qnb=qnb;
    this->att=q2att(this->qnb);
    this->Cnb=q2mat(this->qnb);
    this->avp.resize(9);
    this->avp<<this->att,this->vn,this->pos;

    glv.att0=this->att;
    glv.qnb0=this->qnb;
    glv.t0=imu(imu.rows()-1,imu.cols()-1);

    this->wib=this->Cnb.inverse()*this->eth.wnin;
    this->fn=-this->eth.gn;
    this->fb=this->Cnb.inverse()*this->fn;

    this->wnb=Vector3d::Zero();
    this->web=Vector3d::Zero();
    this->an=Vector3d::Zero();

    this->Mpv<<0,1.0/this->eth.RMh,0,\
                                               1.0/this->eth.clRNh,0,0,\
        0,0,1;
    this->Mpvvn=this->Mpv*this->vn;
}

void INS::insupdate(MatrixXd imu){
    int nn=imu.rows();
    double nts=this->ts*nn;
    //划桨误差补偿

    //直接求和
    VectorXd tmp=imu.colwise().sum();

    Vector3d phim=tmp.block(0,0,3,1);
    Vector3d dvbm=tmp.block(3,0,3,1);

    // 更新地速
    Vector3d vn01=this->vn+this->an*nts/2;
    Vector3d pos01=this->pos+this->Mpv*vn01*nts/2;
    this->eth.update(pos01,vn01);
    this->wib=phim/nts;
    this->fb=dvbm/nts;
    this->web=this->wib-this->Cnb.transpose()*this->eth.wnie;
    this->wnb=this->wib-(this->Cnb*rv2mat(phim/2)).transpose()*this->eth.wnin;

    // 更新速度
    this->fn=qmulv(this->qnb,this->fb);
    this->an=rotv(-this->eth.wnin*nts/2,this->fn)+this->eth.gcc;

    Vector3d vn1=this->vn+this->an*nts;

    //更新位置
    this->Mpv(0,1)=1/this->eth.RMh;
    this->Mpv(1,0)=1/this->eth.clRNh;
    this->Mpvvn=this->Mpv*(this->vn+vn1)/2;
    this->pos=this->pos+this->Mpvvn*nts;
    this->vn=vn1;
    //this->an0=this->an;

    //姿态更新
    //MatrixXd coef=wm2wtcoef(this->ts,nn);四字样算法
    this->qnb=qupdt2(this->qnb,phim,this->eth.wnin*nts);
    this->att=q2att(this->qnb);
    this->Cnb=q2mat(this->qnb);
    this->avp<<this->att,this->vn,this->pos;
}
