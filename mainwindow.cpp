#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QMessageBox>
#include <QChartView>
#include <Eigen/Geometry>
#include <iostream>
#include <QLCDNumber>

#include "portthread.h"
#include "INS.h"

using namespace std;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->ReadPort();
    P_Read_Con=false;
    P_Write_Con=false;
    Saving=false;
    Naving=false;
    Init();
    connect(ui->ButtonSearchPort,&QPushButton::clicked,\
            this,&MainWindow::ReadPort);//刷新端口
    connect(ui->ButtonReadCon,&QPushButton::clicked,\
            this,&MainWindow::SetPortRead);//连接端口
    connect(ui->ButtonWriteCon,&QPushButton::clicked,\
            this,&MainWindow::SetPortWrite);//连接端口
    connect(ui->ButtonSaveStart,&QPushButton::clicked,\
            this,&MainWindow::StartWork);//开始保存
    connect(ui->ButtonNavigation,&QPushButton::clicked,this,&MainWindow::StartNav);//开始导航

    connect(&readThread,&PortThread::state,this,&MainWindow::SetReadState);//连接状态
    connect(&writeThread,&PortThread::state,this,&MainWindow::SetWriteState);//连接状态
    connect(&readThread,&PortThread::dataReceived,this,&MainWindow::recieve);//接收数据
    connect(ui->ButtonHandCon,&QPushButton::clicked,this,&MainWindow::shakehand);//握手
    connect(ui->ButtonHandCon_2,&QPushButton::clicked,this,&MainWindow::shakehand2);//握手

    //将sendImu信号与ShowImu槽函数连接
    connect(&datain,&Data::sendImu,this,&MainWindow::ShowImu);
    //将sendRot信号与ShowRot槽函数连接
    connect(&datain,&Data::sendRot,this,&MainWindow::ShowRot);
    connect(&navthread,&Navigation::sendavp,this,&MainWindow::ShowNav);

    connect(ui->outRSend,&QPushButton::clicked,this,&MainWindow::RotSet);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::Init(){
    //ind_resolve={};
    dataTemp.clear();
    lostDataNum=0;
    dataImuNum=0;
    dataRotNum=0;

    //图表初始化
    groPlot.init();
    accPlot.init();
    //各图表各增加3条曲线
    groPlot.addLine("gx",Qt::red,-10,10);
    groPlot.addLine("gy",Qt::green,-10,10);
    groPlot.addLine("gz",Qt::blue,-10,10);
    accPlot.addLine("ax",Qt::red,-10,10);
    accPlot.addLine("ay",Qt::green,-10,10);
    accPlot.addLine("az",Qt::blue,-10,10);

    //图表显示初始化
    ui->groShow->setChart(groPlot.Chart);
    ui->accShow->setChart(accPlot.Chart);

    //lcd初始化
    ui->lcdfogx->setDigitCount(10);
    ui->lcdfogy->setDigitCount(10);
    ui->lcdfogz->setDigitCount(10);
    ui->lcdax->setDigitCount(10);
    ui->lcday->setDigitCount(10);
    ui->lcdaz->setDigitCount(10);
    ui->lcdrotx->setDigitCount(10);
    ui->lcdrotz->setDigitCount(10);
    ui->lcdtime->setDigitCount(10);
}

void MainWindow::ReadPort(){
    //如果没连接成果，则刷新端口
    if(!P_Read_Con){
        ui->comReadName->clear();
        foreach (portInfo, QSerialPortInfo::availablePorts()) {
            ui->comReadName->addItem(portInfo.portName());
        }
    }
    if(!P_Write_Con){
        ui->comWriteName->clear();
        foreach (portInfo, QSerialPortInfo::availablePorts()) {
            ui->comWriteName->addItem(portInfo.portName());
        }
    }
}

void MainWindow::SetPortRead(){
    bool ok;
    auto waitTime=ui->LineReadReadTime->text().toInt(&ok);
    if(ok){
        if(!P_Read_Con){//还未打开串口,初始化后打开串口
            //ui->textDebug->clear();
            Init();
            readThread.setPortName(ui->comReadName->currentText());
            readThread.setBaudRate(ui->comReadBaud->currentText().toInt());
            readThread.setReadFrequency(waitTime);
            readThread.openSerialPort();
        }else{
            readThread.closeSerialPort();
        }

    }else{
        QMessageBox::warning(this,tr("error"),"频率格式错误");
    }
}
void MainWindow::SetPortWrite(){
    if(!P_Write_Con){//还未打开串口,初始化后打开串口
        writeThread.setPortName(ui->comWriteName->currentText());
        writeThread.setBaudRate(ui->comWriteBaud->currentText().toInt());
        writeThread.openSerialPort();
    }else {
        writeThread.closeSerialPort();
    }
 }

void MainWindow::RotSet(){
    QByteArray data;
    data.resize(14);
    int axis=1;
    switch (axis) {
    case 1:
        break;
    case 2:
        break;
    case 3:
        break;
    default:
        break;
    }
    data[0]=0xaa;        data[1]=0xa5;
    data[2]=0x55;        data[3]=0x12;
    data[4]=0xa0;        data[5]=0xbb;
    data[6]=0x0d;        data[7]=0x00;
    data[8]=0x00;        data[9]=0x40;
    data[10]=0x9c;        data[11]=0x00;
    data[12]=0x00;        data[13]=0xfa;
    //write data
    writeThread.writeData(data);
}

void MainWindow::SetReadState(bool isOpen){
    P_Read_Con=isOpen;
    this->SetButton();
}
void MainWindow::SetWriteState(bool yes){
    P_Write_Con=yes;
    this->SetButton();
}

void MainWindow::SetButton(){
    if(P_Read_Con){
        ui->ButtonReadCon->setText("关闭串口");
        ui->comReadName->setDisabled(true);
        ui->comReadBaud->setDisabled(true);
        ui->LineReadReadTime->setDisabled(true);
    }else{
        ui->ButtonReadCon->setText("打开串口");
        ui->comReadName->setDisabled(false);
        ui->comReadBaud->setDisabled(false);
        ui->LineReadReadTime->setDisabled(false);
    }
    if(P_Write_Con){
        ui->ButtonWriteCon->setText("关闭串口");
        ui->comWriteName->setDisabled(true);
        ui->comWriteBaud->setDisabled(true);
    }else{
        ui->ButtonWriteCon->setText("打开串口");
        ui->comWriteName->setDisabled(false);
        ui->comWriteBaud->setDisabled(false);
    }
}
void MainWindow::shakehand(){
    //发送握手信号
    QByteArray data;
    data.append(0x55);
    data.append(0xaa);
    data.append(0x06);
    data.append(0x80);
    data.append(0x02);
    data.append(0x82);
    //write data
    readThread.writeData(data);
}
void MainWindow::shakehand2(){
    //发送握手信号
    QByteArray data;
    data.resize(14);
    data[0]=0xaa;        data[1]=0xa5;
    data[2]=0x55;        data[3]=0x0f;
    data[4]=0x00;        data[5]=0x00;
    data[6]=0x00;        data[7]=0x00;
    data[8]=0x00;        data[9]=0x00;
    data[10]=0x00;        data[11]=0x00;
    data[12]=0x00;        data[13]=0xb3;
    //write data
    writeThread.writeData(data);
}

void MainWindow::recieve(const QByteArray &data){
    QString dataimusp;//imu数据
    QString datarotsp;//rot数据
    auto s=dataTemp+data;// 将数据与之前的留下的帧头合并
    dataTemp.clear();// 清空之前的帧头
    dataimusp.clear();// 清空imu数据
    datarotsp.clear();// 清空rot数据
    //测试
    //ui->textDebug->append(s.toHex(' '));

    //开始位置
    int pos=s.indexOf(datain.inframe.start,1);
    //最大长度
    int lenmax=max(datain.inframe.lens1,datain.inframe.lens2);
    //(datain.inframe.lens1>datain.inframe.lens2)?datain.inframe.lens1:datain.inframe.lens2;


    // 解帧
    while(pos>=0){
        // 有一帧的长度
        if(pos+lenmax-1<s.size()){   //判断是否为帧头
            if((uint)(uchar)s[pos+1]==datain.inframe.type1\
                &&(uint)(uchar)s[pos+datain.inframe.lens1-1]==datain.inframe.end){
                //是imu帧头
                dataImuNum++;
                //解析imu数据，帧头2位，递增数1位，时间4位，陀螺数据3*4位，温度3*2位，加速度数据3*4位
                //截取一帧imu数据
                QByteArray dataimu=s.mid(pos+3,datain.inframe.lens1-4);
                //imu数据解帧
                datain.updateImu(dataimu);

                // 保存数据
                if(Saving){
                    // imu数据转为字符串，空格分隔
                    dataimusp=datain.ImuToString();
                    fileimu.write(dataimusp.toUtf8());
                }
                if(Naving){
                    // 导航
                    navthread.receiveData(datain.imu);
                }
                // 寻找下一个帧头
                pos=s.indexOf(datain.inframe.start,pos+datain.inframe.lens1);

            }else if((uint)(uchar)s[pos+1]==datain.inframe.type2\
                       &&(uint)(uchar)s[pos+datain.inframe.lens2-1]==datain.inframe.end){
                // 是rot帧头2
                dataRotNum++;
                //解析rot数据
                QByteArray datarot=s.mid(pos+3,datain.inframe.lens2-4);
                //rot数据解帧
                datain.updateRot(datarot);
                if(Saving){
                    // rot数据转为字符串，空格分隔
                    datarotsp=datain.RotToString();
                    filerot.write(datarotsp.toUtf8());
                }
                // 寻找下一个帧头
                pos=s.indexOf(datain.inframe.start,pos+datain.inframe.lens2);
            }else{
                //不是帧头，寻找下一个帧头
                pos=s.indexOf(datain.inframe.start,pos+1);
            }
        }else{
            //不够长度直接保存
            auto lens=s.size()-pos;
            dataTemp=s.right(lens);
            break;
        }
    }
}

//imu数据解帧
void MainWindow::decode(QByteArray datain,int type,Data &dataout){
    QDataStream in(datain);
    in.setByteOrder(QDataStream::ByteOrder::LittleEndian);

    //imuFrame imu;
    //double imusp[7];
    //double rotsp[4];
    /*rotFrame rot;
    switch (type) {
        case 1:
            in>>imu.time>>imu.wx>>imu.tx>>imu.wy>>imu.ty>>imu.wz>>imu.tz>>imu.ax1>>imu.ax2>>imu.ay1>>imu.ay2>>imu.az1>>imu.az2;
            imusp[0]=imu.wx;imusp[1]=imu.wy;imusp[2]=imu.wz;
            imusp[3]=imu.ax1-imu.ax2;imusp[4]=imu.ay1-imu.ay2;imusp[5]=imu.az1-imu.az2;
            imusp[6]=imu.time;
            dataout.updateImu(imusp);
            break;
        case 2:
            in>>rot.time>>rot.inn>>rot.tmp3>>rot.mid>>rot.tmp2>>rot.out>>rot.tmp1;
            rotsp[0]=rot.inn;rotsp[1]=rot.mid;rotsp[2]=rot.out;
            rotsp[3]=rot.time;
            dataout.updateRot(rotsp);
            break;
    }*/
}

void MainWindow::ShowImu(const double imu[7]){
    // add data to the graph
    for(int i=0;i<3;i++){
        groPlot.updateLine(i,imu[6],imu[i]);
        accPlot.updateLine(i,imu[6],imu[i+3]);
    }

    // lcd显示数据，如果数据太大，显示不全
    ui->lcdfogx->display(imu[0]);
    ui->lcdfogy->display(imu[1]);
    ui->lcdfogz->display(imu[2]);
    ui->lcdax->display(imu[3]);
    ui->lcday->display(imu[4]);
    ui->lcdaz->display(imu[5]);
    ui->lcdtime->display(imu[6]);

    //cout
    //cout<<imu[0]<<' '<<imu[1]<<' '<<imu[2]<<' '<<imu[3]<<' '<<imu[4]<<' '<<imu[5]<<' '<<imu[6]<<endl;

}
void MainWindow::ShowRot(const double rot[4]){
    ui->lcdrotx->display(rot[0]);
    ui->lcdrotz->display(rot[1]);
    ui->lcdtime->display(rot[3]);
}
void MainWindow::ShowNav(const VectorXd avp){
    VectorXd avp0(10);
    avp0<<glv.att0,glv.vn0,glv.pos0,0;
    VectorXd tmp=avp-avp0;
    ui->lcdattx->display(tmp(0)/DEG*60);
    ui->lcdatty->display(tmp(1)/DEG*60);
    ui->lcdattz->display(tmp(2)/DEG*60);
    ui->lcdvx->display(tmp(3));
    ui->lcdvy->display(tmp(4));
    ui->lcdvz->display(tmp(5));
    ui->lcddx->display(tmp(6)/DEG*60);
    ui->lcddy->display(tmp(7)/DEG*60);
    ui->lcddz->display(tmp(8)/DEG*60);
}
void MainWindow::sendPort(){

}

void MainWindow::StartWork(){
    if(Saving){
        Saving=false;
        ui->ButtonSaveStart->setText("开始保存");
        fileimu.close();
        filerot.close();
    }else{
        QDateTime time=QDateTime::currentDateTime();
        QString str_time=time.toString("yyyy-MM-dd-hh-mm-ss");
        QString path0=QDir::currentPath();
        QString dirpath=path0+"\\"+str_time;
        if(!dir.exists(dirpath)){
            bool ok=dir.mkdir(dirpath);
            if(!ok){
                QMessageBox::warning(this,tr("建立目录"),tr("新建目录失败"));
            }
        }else{
            QMessageBox::warning(this,tr("建立目录"),tr("目录已存在"));
        }
        //new file
        QString filenameimu="imu"+time.toString("MM_dd_hh_mm_ss")+".txt";
        //QString filename="diag.txt";
        QString filepathimu=dirpath+"\\"+filenameimu;
        fileimu.setFileName(filepathimu);
        if(!fileimu.open(QIODevice::ReadWrite)){
            QMessageBox::warning(this,tr("创建文件"),tr("新建文件失败"));
        }

        //new file
        QString filenamerot="rot"+time.toString("MM_dd_hh_mm_ss")+".txt";
        //QString filename="diag.txt";
        QString filepathrot=dirpath+"\\"+filenamerot;
        filerot.setFileName(filepathrot);
        if(!filerot.open(QIODevice::ReadWrite)){
            QMessageBox::warning(this,tr("创建文件"),tr("新建文件失败"));
        }

        Saving=true;
        ui->ButtonSaveStart->setText("停止保存");
    }
}

void MainWindow::StartNav(){
    // 若没有导航，则初始化后开始导航
    if(!Naving){
        // 初始化导航
        auto aligntime=ui->AlignTime->value();
        navthread.setParameters(aligntime,1000);
        // 导入误差
        INSerr.loadData("D:\\qtCode\\qtloadtxt\\caltest1.txt");
        // 开始导航
        navthread.startProcessing();
        Naving=true;
        ui->ButtonNavigation->setText("停止导航");
    }else{
        navthread.stopProcessing();
        Naving=false;
        ui->ButtonNavigation->setText("开始导航");
    }
}
