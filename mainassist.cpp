#include "mainassist.h"


Data::Data(){
    this->init();
}
void Data::init(){
    // 数据初始化
    //imu,rot数组清零
    //memset(this->imu,0,sizeof(this->imu));
    //memset(this->rot,0,sizeof(this->rot));
    memset(this->imu1s,0,sizeof(this->imu1s));
    memset(this->rot1s,0,sizeof(this->rot1s));
    //imu1sNum,rot1sNum清零
    this->imu1sNum=0;
    this->rot1sNum=0;
    this->inframe={0xfa,0x69,32,0x72,32,0xee};

    // 对准数据
    this->alignnum=0;
    //this->alignData.resize(0,0);
    this->freq1=400;
    this->freq2=100;
}

bool Data::updateImu(QByteArray data){
    //data解帧
    QDataStream in(data);
    in.setByteOrder(QDataStream::ByteOrder::BigEndian);

    in>>this->imutmp.time>>this->imutmp.wx>>this->imutmp.tx>>\
    this->imutmp.wy>>this->imutmp.ty>>this->imutmp.wz>>\
    this->imutmp.tz>>this->imutmp.ax1>>this->imutmp.ax2>>\
    this->imutmp.ay1>>this->imutmp.ay2>>this->imutmp.az1>>this->imutmp.az2;

    //imu数据更新
    this->imu[0]=this->imutmp.wx;
    this->imu[1]=this->imutmp.wy;
    this->imu[2]=this->imutmp.wz;
    this->imu[3]=this->imutmp.ax1-this->imutmp.ax2;
    this->imu[4]=this->imutmp.ay1-this->imutmp.ay2;
    this->imu[5]=this->imutmp.az1-this->imutmp.az2;
    this->imu[6]=this->imutmp.time*0.0001;

    //imu1s数据更新
    this->imu1s[0]+=this->imu[0];
    this->imu1s[1]+=this->imu[1];
    this->imu1s[2]+=this->imu[2];
    this->imu1s[3]+=this->imu[3];
    this->imu1s[4]+=this->imu[4];
    this->imu1s[5]+=this->imu[5];
    this->imu1s[6]=this->imu[6];
    this->imu1sNum++;

    // 若imu1s数据超过1s，返回true
    if(this->imu1sNum>=this->freq1){
        this->imu1sNum=0;
        emit sendImu(this->imu1s);
        this->init();
        return true;
    }else {
        return false;
    }
}

bool Data::updateRot(QByteArray data){
    //data解帧
    QDataStream in(data);
    in.setByteOrder(QDataStream::ByteOrder::BigEndian);
    in>>this->rottmp.time;
    in.setByteOrder(QDataStream::ByteOrder::LittleEndian);

    in>>this->rottmp.inn>>this->rottmp.tmp3>>this->rottmp.mid\
        >>this->rottmp.tmp2>>this->rottmp.out>>this->rottmp.tmp1;

    //rot数据更新
    this->rot[0]=this->rottmp.out*0.0001;
    this->rot[1]=this->rottmp.mid*0.0001;
    this->rot[2]=this->rottmp.inn*0.0001;
    this->rot[3]=this->rottmp.time*0.0001;

    this->rot1sNum++;

    // 若rot数据超过1s，返回true
    if(this->rot1sNum>=freq2){
        this->rot1sNum=0;
        emit sendRot(this->rot);
        return true;
    }else {
        return false;
    }
}

QString Data::ImuToString(){
    QString str;
    //imu数据不用科学计数法转QString
    str=QString::number(this->imu[0],'g')+" "+QString::number(this->imu[1],'g')+" "+QString::number(this->imu[2],'g')+" "\
          +QString::number(this->imu[3],'g')+" "+QString::number(this->imu[4],'g')+" "+QString::number(this->imu[5],'g')+" "\
          +QString::number(this->imu[6],'f')+"\n";
    return str;
}

QString Data::RotToString(){
    QString str;
    //rot数据不用科学计数法转QString
    str=QString::number(this->rot[0],'f')+" "+QString::number(this->rot[1],'f')+" "+QString::number(this->rot[2],'f')+" "\
          +QString::number(this->rot[3],'f')+"\n";

    return str;
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
