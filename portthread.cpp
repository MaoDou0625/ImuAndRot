#include "PortThread.h"
#include <QDebug>

PortThread::PortThread(QObject *parent)
    : QObject(parent)
{
    //serialPort.moveToThread(&workerThread);
    connect(&workerThread, &QThread::finished, &serialPort, &QSerialPort::deleteLater);
    connect(this, &PortThread::startReading, this, &PortThread::readSerialPort);
    connect(this, &PortThread::startWriting, this, &PortThread::writeSerialPort);
    workerThread.start();
}

PortThread::~PortThread()
{
    if (workerThread.isRunning())
    {
        workerThread.quit();
        workerThread.wait();
    }
}

void PortThread::setPortName(const QString &portName)
{
    serialPort.setPortName(portName);
}

void PortThread::setBaudRate(qint32 baudRate)
{
    serialPort.setBaudRate(baudRate);
}

void PortThread::setDataBits(QSerialPort::DataBits dataBits)
{
    serialPort.setDataBits(dataBits);
}

void PortThread::setParity(QSerialPort::Parity parity)
{
    serialPort.setParity(parity);
}

void PortThread::setStopBits(QSerialPort::StopBits stopBits)
{
    serialPort.setStopBits(stopBits);
}

void PortThread::setFlowControl(QSerialPort::FlowControl flowControl)
{
    serialPort.setFlowControl(flowControl);
}

void PortThread::setReadFrequency(int frequency)
{
    readFrequency = frequency;
}

QByteArray PortThread::getReceivedData() const
{
    return dataBuffer;
}

void PortThread::openSerialPort()
{
    if (!serialPort.isOpen())
    {
        if (serialPort.open(QIODevice::ReadWrite))
        {
            emit startReading();
            emit state(true); // 发送串口打开状态
        }
        else
        {
            qWarning() << "Failed to open serial port:" << serialPort.errorString();
            emit state(false);
        }
    }
}

void PortThread::closeSerialPort()
{
    if (serialPort.isOpen())
    {
        serialPort.close();
        emit state(false); // 发送串口关闭状态
    }
}

void PortThread::writeData(const QByteArray &data)
{
    if (serialPort.isOpen())
    {
        emit startWriting(data);
    }
}

void PortThread::readSerialPort()
{
    if (serialPort.isOpen())
    {
        QByteArray receivedData = serialPort.readAll();
        if (!receivedData.isEmpty())
        {
            dataBuffer.append(receivedData);
            emit dataReceived(dataBuffer);
            dataBuffer.clear();
        }
        QTimer::singleShot(readFrequency, this, &PortThread::readSerialPort);
    }
}

void PortThread::writeSerialPort(const QByteArray &data)
{
    if (serialPort.isOpen())
    {
        serialPort.write(data);
    }
}
