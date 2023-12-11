#ifndef PORTTHREAD_H
#define PORTTHREAD_H

#include <QObject>
#include <QThread>
#include <QSerialPort>
#include <QByteArray>
#include <QTimer>

class PortThread : public QObject
{
    Q_OBJECT

public:
    explicit PortThread(QObject *parent = nullptr);
    ~PortThread();

    void setPortName(const QString &portName);
    void setBaudRate(qint32 baudRate);
    void setDataBits(QSerialPort::DataBits dataBits);
    void setParity(QSerialPort::Parity parity);
    void setStopBits(QSerialPort::StopBits stopBits);
    void setFlowControl(QSerialPort::FlowControl flowControl);
    void setReadFrequency(int frequency);
    QByteArray getReceivedData() const;

signals:
    void startReading();
    void state(bool isopen);
    void startWriting(const QByteArray &data);
    void dataReceived(const QByteArray& data);


public slots:
    void openSerialPort();
    void closeSerialPort();
    void writeData(const QByteArray &data);

private slots:
    void readSerialPort();
    void writeSerialPort(const QByteArray &data);

private:
    QThread workerThread;
    QSerialPort serialPort;
    QByteArray dataBuffer;
    int readFrequency = 1000;  // Default read frequency is 1000 milliseconds
};

#endif // PORTTHREAD_H
