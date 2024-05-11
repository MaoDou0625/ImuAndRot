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

    this->outframe.len=14;
    this->outframe.start1=0xaa;
    this->outframe.start2=0xa5;
    this->outframe.start3=0x55;
    this->outframe.innpos=0x11;     this->outframe.midpos=0x21;     this->outframe.outpos=0x31;
    this->outframe.innrelapos=0x12; this->outframe.midrelapos=0x22; this->outframe.outrelapos=0x32;
    this->outframe.innv=0x13;       this->outframe.midv=0x23;       this->outframe.outv=0x33;
    this->outframe.innsway=0x1a;    this->outframe.midsway=0x2a;    this->outframe.outsway=0x3a;
    this->outframe.innenable=0x1e;  this->outframe.midenable=0x2e;  this->outframe.outenable=0x3e;
    this->outframe.innstop=0x1f;    this->outframe.midstop=0x2f;    this->outframe.outstop=0x3f;
    this->outframe.ccwdirect=0x80;  this->outframe.cwdirect=0x00;   this->outframe.shortdirect=0xa0;
    this->outframe.enable=0x00;this->outframe.disenable=0x80;

    this->outframe.inn0=312.533;
    this->outframe.mid0=0;
    this->outframe.out0=70;

    this->imushakehand.resize(6);
    this->imushakehand[0]=0x55;    this->imushakehand[1]=0xaa;
    this->imushakehand[2]=0x06;    this->imushakehand[3]=0x80;
    this->imushakehand[4]=0x02;    this->imushakehand[5]=0x82;

    this->rotshakehand.resize(14);
    this->rotshakehand[0]=0xaa;     this->rotshakehand[1]=0xa5;
    this->rotshakehand[2]=0x55;     this->rotshakehand[3]=0x0f;
    this->rotshakehand[4]=0x00;     this->rotshakehand[5]=0x00;
    this->rotshakehand[6]=0x00;     this->rotshakehand[7]=0x00;
    this->rotshakehand[8]=0x00;     this->rotshakehand[9]=0x00;
    this->rotshakehand[10]=0x00;    this->rotshakehand[11]=0x00;
    this->rotshakehand[12]=0x00;    this->rotshakehand[13]=0xb3;

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

QByteArray Data::rotSend(uint type,double data1,double data2){
    QByteArray data;
    data.resize(this->outframe.len);
    data.fill(0);
    QDataStream out(&data,QIODevice::WriteOnly);
    out.setByteOrder(QDataStream::ByteOrder::LittleEndian);
    out<<this->outframe.start1<<this->outframe.start2<<this->outframe.start3;
    this->outframe.para1=qRound(qAbs(data1*10000));
    this->outframe.para2=qRound(qAbs(data2*10000));
    switch (type) {
        case 11:
            out<<this->outframe.innpos<<this->outframe.para1;
            //if (data1<=0) out<<this->outframe.cwdirect;
            //else out<<this->outframe.ccwdirect;
            out<<this->outframe.shortdirect;
            out<<this->outframe.para2;
            break;
        case 12:
            out<<this->outframe.innrelapos<<this->outframe.para1;
            if (data1<=0) out<<this->outframe.cwdirect;
            else out<<this->outframe.ccwdirect;
            out<<this->outframe.para2;
            break;
        case 13:
            out<<this->outframe.innv<<this->outframe.para1;
            if (data1<=0) out<<this->outframe.cwdirect;
            else out<<this->outframe.ccwdirect;
            out<<this->outframe.para2;
            break;
        case 14:
            out<<this->outframe.innsway<<this->outframe.para1;
            out<<this->outframe.cwdirect;
            out<<this->outframe.para2;
            break;
        case 15:
            out<<this->outframe.innenable<<this->outframe.para1;
            if(data1==0) out<<this->outframe.disenable;
            else out<<this->outframe.enable;
            out<<this->outframe.para2;
            break;
        case 16:
            out<<this->outframe.innstop<<this->outframe.para1;
            out<<this->outframe.disenable;
            out<<this->outframe.para2;
            break;
        case 21:
            out<<this->outframe.midpos<<this->outframe.para1;
            out<<this->outframe.shortdirect;
            out<<this->outframe.para2;
            break;
        case 22:
            out<<this->outframe.midrelapos<<this->outframe.para1;
            if (data1<=0) out<<this->outframe.cwdirect;
            else out<<this->outframe.ccwdirect;
            out<<this->outframe.para2;
            break;
        case 23:
            out<<this->outframe.midv<<this->outframe.para1;
            if (data1<=0) out<<this->outframe.cwdirect;
            else out<<this->outframe.ccwdirect;
            out<<this->outframe.para2;
            break;
        case 24:
            out<<this->outframe.midsway<<this->outframe.para1;
            out<<this->outframe.cwdirect;
            out<<this->outframe.para2;
            break;
        case 25:
            out<<this->outframe.midenable<<this->outframe.para1;
            if(data1==0) out<<this->outframe.disenable;
            else out<<this->outframe.enable;
            out<<this->outframe.para2;
            break;
        case 26:
            out<<this->outframe.midstop<<this->outframe.para1;
            out<<this->outframe.disenable;
            out<<this->outframe.para2;
            break;
        case 31:
            out<<this->outframe.outpos<<this->outframe.para1;
            out<<this->outframe.shortdirect;
            out<<this->outframe.para2;
            break;
        case 32:
            out<<this->outframe.outrelapos<<this->outframe.para1;
            if (data1<=0) out<<this->outframe.cwdirect;
            else out<<this->outframe.ccwdirect;
            out<<this->outframe.para2;
            break;  
        case 33:
            out<<this->outframe.outv<<this->outframe.para1;
            if (data1<=0) out<<this->outframe.cwdirect;
            else out<<this->outframe.ccwdirect;
            out<<this->outframe.para2;
            break;
        case 34:
            out<<this->outframe.outsway<<this->outframe.para1;
            out<<this->outframe.cwdirect;
            out<<this->outframe.para2;
            break;
        case 35:
            out<<this->outframe.outenable<<this->outframe.para1;
            if(data1==0) out<<this->outframe.disenable;
            else out<<this->outframe.enable;
            out<<this->outframe.para2;
            break;
        case 36:
            out<<this->outframe.outstop<<this->outframe.para1;
            out<<this->outframe.disenable;
            out<<this->outframe.para2;
            break;
    }
    // 和校验
    data[data.size()-1]=0;
    for(int i=0;i<data.size()-1;i++){
        data[data.size()-1]+=data[i];
    }
    return data;
}

void Plot::init(){
    // 图标初始化
    this->Chart=new QChart();
    // 坐标轴初始化
    this->xaxis=new QValueAxis();
    // 坐标轴范围
    this->xmin=0;
    this->xlen=10;
    this->xmax=this->xmin+this->xlen;
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

        this->xmin=floor(x/this->xlen)*this->xlen;
        this->xmax=this->xmin+this->xlen;
        this->xaxis->setMin(this->xmin);
        this->xaxis->setMax(this->xmax);
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
        //this->xmin==floor(x/xmax)*xmax;
        this->xmax=this->xmax+this->xlen;
        //this->xaxis->setMin(this->xmin);
        // 如果此时没有处于缩放状态，设置横坐标轴范围
        if(this->xaxis->min()==this->xmin){
            this->xaxis->setMax(this->xmax);
        }
        //this->xaxis->setMax(this->xmax);
        //this->line[index]->clear();
    }
}
