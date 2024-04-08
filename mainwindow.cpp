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

    //将sendImu信号与ShowImu槽函数连接
    connect(&tmpdata,&Data::sendImu,this,&MainWindow::ShowImu);
    //将sendRot信号与ShowRot槽函数连接
    connect(&tmpdata,&Data::sendRot,this,&MainWindow::ShowRot);
    connect(&navthread,&Navigation::sendavp,this,&MainWindow::ShowNav);

    connect(ui->outRSend,&QPushButton::clicked,this,&MainWindow::outRxisSet);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::Init(){
    //帧头 帧位 不定长度26或16字节
    ind_frame={0xfa,0x69,26,0x72,16,0xee};
    //imu数据
    ind_resolve={};
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
            ui->textDebug->clear();
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

        //发送握手信号
        QByteArray data;
        data.resize(14);
        data[0]=0x55;        data[1]=0xa5;
        data[2]=0x55;        data[3]=0x0f;
        data[4]=0x00;        data[5]=0x00;
        data[6]=0x00;        data[7]=0x00;
        data[8]=0x00;        data[9]=0x00;
        data[10]=0x00;        data[11]=0x00;
        data[12]=0x00;        data[13]=0xb3;
        //write data
        writeThread.writeData(data);
    }else {
        writeThread.closeSerialPort();
    }
 }

void MainWindow::outRxisSet(){
    QByteArray data;
    data.resize(14);
    data[0]=0x55;        data[1]=0xa5;
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

void MainWindow::recieve(const QByteArray &data){
    //QStringList str_resolve;//解析后的数据
    //QString datashow;
    QString dataimusp;//imu数据
    QString datarotsp;//rot数据
    auto s=dataTemp+data;// 将数据与之前的留下的帧头合并
    dataTemp.clear();// 清空之前的帧头
    dataimusp.clear();//清空imu数据
    datarotsp.clear();//清空rot数据
    //测试
    //ui->textDebug->append(s.toHex(' '));

    //开始位置
    int pos=s.indexOf(ind_frame.start,1);
    //最大长度
    int lenmax=(ind_frame.lens1>ind_frame.lens2)?ind_frame.lens1:ind_frame.lens2;


    // 解帧
    while(pos>=0){
        // 有一帧的长度
        if(pos+lenmax-1<s.size()){   //判断是否为帧头
            if((uint)(uchar)s[pos+1]==ind_frame.type1\
                &&(uint)(uchar)s[pos+ind_frame.lens1-1]==ind_frame.end){
                //是imu帧头
                dataImuNum++;
                imusp[0]=(uchar)s[pos+10]<<24|(uchar)s[pos+9]<<16|(uchar)s[pos+8]<<8|(uchar)s[pos+7];
                imusp[1]=(uchar)s[pos+14]<<24|(uchar)s[pos+13]<<16|(uchar)s[pos+12]<<8|(uchar)s[pos+11];
                imusp[2]=(uchar)s[pos+18]<<24|(uchar)s[pos+17]<<16|(uchar)s[pos+16]<<8|(uchar)s[pos+15];
                imusp[3]=(uchar)s[pos+19]-(uchar)s[pos+20];
                imusp[4]=(uchar)s[pos+21]-(uchar)s[pos+22];
                imusp[5]=(uchar)s[pos+23]-(uchar)s[pos+24];
                imusp[6]=((uchar)s[pos+3]<<24|(uchar)s[pos+4]<<16|(uchar)s[pos+5]<<8|(uchar)s[pos+6])*1.0/\
                           10000.0;

                //更新tmpdata
                tmpdata.updateImu(imusp);
                // 保存数据
                if(Saving){
                    fileimu.write(dataimusp.toUtf8());
                }
                if(Naving){
                    // 导航
                    navthread.receiveData(imusp);
                }

                // 寻找下一个帧头
                pos=s.indexOf(ind_frame.start,pos+ind_frame.lens1);

            }else if((uint)(uchar)s[pos+1]==ind_frame.type2\
                       &&(uint)(uchar)s[pos+ind_frame.lens2-1]==ind_frame.end){
                // 是rot帧头2
                dataRotNum++;
                rot[0]=((uchar)s[pos+10]<<24|(uchar)s[pos+9]<<16|(uchar)s[pos+8]<<8|(uchar)s[pos+7])*1.0/\
                         0x400000*360;
                rot[1]=((uchar)s[pos+14]<<24|(uchar)s[pos+13]<<16|(uchar)s[pos+12]<<8|(uchar)s[pos+11])*1.0/\
                         0x400000*360;
                rot[2]=((uchar)s[pos+3]<<24|(uchar)s[pos+4]<<16|(uchar)s[pos+5]<<8|(uchar)s[pos+6])*1.0/10000;

                //更新data
                tmpdata.updateRot(rot);
                /*
                if(tmpdata.updateRot(rot)){
                    // 如果数据超过1s，更新图表
                    ShowRot(tmpdata.rot);
                }*/
                if(Saving){
                    filerot.write(datarotsp.toUtf8());
                }
                // 寻找下一个帧头
                pos=s.indexOf(ind_frame.start,pos+ind_frame.lens2);
            }else{
                //不是帧头，寻找下一个帧头
                pos=s.indexOf(ind_frame.start,pos+1);
            }
        }else{
            //不够长度直接保存
            auto lens=s.size()-pos;
            dataTemp=s.right(lens);
            break;
        }
    }
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
void MainWindow::ShowRot(const double rot[3]){
    ui->lcdrotx->display(rot[0]);
    ui->lcdrotz->display(rot[1]);
    ui->lcdtime->display(rot[2]);
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


Data::Data(){
    this->init();
}
void Data::init(){
    // imu数据初始化
    this->imu[0]=0;
    this->imu[1]=0;
    this->imu[2]=0;
    this->imu[3]=0;
    this->imu[4]=0;
    this->imu[5]=0;
    this->imu[6]=0;
    // rot数据初始化
    this->rot[0]=0;
    this->rot[1]=0;
    this->rot[2]=0;
    // imu1s数据初始化
    this->imu1s[0]=0;
    this->imu1s[1]=0;
    this->imu1s[2]=0;
    this->imu1s[3]=0;
    this->imu1s[4]=0;
    this->imu1s[5]=0;
    this->imu1s[6]=0;
    this->imu1sNum=0;
    this->rot1sNum=0;
}

bool Data::updateImu(double imu[7]){
    // imu数据更新
    this->imu[0]=imu[0];
    this->imu[1]=imu[1];
    this->imu[2]=imu[2];
    this->imu[3]=imu[3];
    this->imu[4]=imu[4];
    this->imu[5]=imu[5];
    this->imu[6]=imu[6];

    // imu1s数据更新
    this->imu1s[0]+=imu[0];
    this->imu1s[1]+=imu[1];
    this->imu1s[2]+=imu[2];
    this->imu1s[3]+=imu[3];
    this->imu1s[4]+=imu[4];
    this->imu1s[5]+=imu[5];
    this->imu1s[6]=imu[6];
    this->imu1sNum++;

    // 若imu1s数据超过1s，返回true
    if(this->imu1sNum>=1000){
        this->imu1sNum=0;
        emit sendImu(this->imu1s);
        this->init();
        return true;
    }else {
        return false;
    }
}

bool Data::updateRot(double rot[3]){
    // rot数据更新
    this->rot[0]=rot[0];
    this->rot[1]=rot[1];
    this->rot[2]=rot[2];

    this->rot1sNum++;

    // 若rot数据超过1s，返回true
    if(this->rot1sNum>=1000){
        this->rot1sNum=0;
        emit sendRot(this->rot);
        return true;
    }else {
        return false;
    }
}


void Plot::init(){
    // 图标初始化
    this->Chart=new QChart();
    // 坐标轴初始化
    this->xaxis=new QValueAxis();
    // 坐标轴范围
    this->xmin=0;
    this->xmax=60;
    // 坐标设置
    this->xaxis->setMin(this->xmin);
    this->xaxis->setMax(this->xmax);
    this->xaxis->setLabelFormat("%.1f");
    this->Chart->addAxis(this->xaxis,Qt::AlignBottom);

    // 曲线数量
    this->num=0;

    // 曲线列表清空
    this->line.clear();
    // 曲线颜色列表清空
    this->color.clear();
    // 曲线名称列表清空
    this->name.clear();
    // 曲线最小值列表清空
    this->ymin.clear();
    // 曲线最大值列表清空
    this->ymax.clear();
    // 纵坐标轴列表清空
    this->yaxis.clear();
}

void Plot::addLine(QString name,QColor color,double ymin,double ymax){
    // 曲线数量
    this->num++;
    // 纵坐标轴列表增加一个
    this->yaxis.append(new QValueAxis());
    // 曲线列表增加一个
    this->line.append(new QLineSeries());
    // 曲线颜色列表增加一个
    this->color.append(color);
    // 曲线名称列表增加一个
    this->name.append(name);
    // 曲线最小值列表增加一个
    this->ymin.append(ymin);
    // 曲线最大值列表增加一个
    this->ymax.append(ymax);

    // 坐标设置
    this->yaxis[this->num-1]->setMin(this->ymin[this->num-1]);
    this->yaxis[this->num-1]->setMax(this->ymax[this->num-1]);
    this->yaxis[this->num-1]->setLabelFormat("%.1f");

    // 坐标轴添加到图表中
    this->Chart->addAxis(this->yaxis[this->num-1],Qt::AlignLeft);

    // 曲线添加到图表中
    this->Chart->addSeries(this->line[this->num-1]);

    // 曲线附加到坐标轴
    this->line[this->num-1]->attachAxis(this->xaxis);
    this->line[this->num-1]->attachAxis(this->yaxis[this->num-1]);

    // 曲线颜色
    this->line[this->num-1]->setColor(this->color[this->num-1]);

    // 曲线名称
    this->line[this->num-1]->setName(this->name[this->num-1]);

}

void Plot::delLine(int index){
    // 曲线数量
    this->num--;
    // 曲线列表删除一个
    this->line.removeAt(index);
    // 曲线颜色列表删除一个
    this->color.removeAt(index);
    // 曲线名称列表删除一个
    this->name.removeAt(index);
    // 曲线最小值列表删除一个
    this->ymin.removeAt(index);
    // 曲线最大值列表删除一个
    this->ymax.removeAt(index);
    // 纵坐标轴列表删除一个
    this->yaxis.removeAt(index);
    // 曲线删除
    this->Chart->removeSeries(this->line[index]);
    // 坐标轴删除
    this->Chart->removeAxis(this->yaxis[index]);
}

void Plot::updateLine(int index,double x,double y){
    // 曲线添加数据
    this->line[index]->append(QPointF(x,y));

    //如果是第一个数据，围绕数据绝对值的0.1倍设置纵坐标轴范围
    if(this->line[index]->count()==1){
        this->ymin[index]=y-0.1*abs(y);
        this->ymax[index]=y+0.1*abs(y);
        this->yaxis[index]->setMin(this->ymin[index]);
        this->yaxis[index]->setMax(this->ymax[index]);
    }
    // 如果数据超出范围，重新设置坐标轴范围
    if(y>this->ymax[index]){
        this->ymax[index]=y;
        this->yaxis[index]->setMax(this->ymax[index]);
    }
    if(y<this->ymin[index]){
        this->ymin[index]=y;
        this->yaxis[index]->setMin(this->ymin[index]);
    }
    // 如果时间超出范围，重新设置坐标轴范围
    if(x>this->xmax){
        this->xmin=floor(x/xmax)*xmax;
        this->xmax=this->xmin+60;
        this->xaxis->setMin(this->xmin);
        this->xaxis->setMax(this->xmax);
        this->line[index]->clear();
    }
}
