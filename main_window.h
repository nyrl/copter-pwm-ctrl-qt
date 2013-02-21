#ifndef MAIN_WINDOW_H_
#define MAIN_WINDOW_H_

#include <QtGui/QApplication>
#include <QObject>
#include <QMainWindow>
#include <QSharedPointer>
#include <QPointer>
#include <QSocketNotifier>
#include <QtNetwork/QTcpServer>
#include <QtNetwork/QTcpSocket>
#include <QLCDNumber>
#include <QString>
#include <QVector>
#include <QFile>
#include "ui_main_window.h"




class CopterMotor : public QObject
{
    Q_OBJECT
  public:
    CopterMotor(const QString& _ctrlPath, QLCDNumber* _lcd);
    ~CopterMotor();

    double factor() const { return m_factor; }
    void factor(double _factor);

    void setPower(unsigned _power);

  protected:
    QLCDNumber* m_lcd;
    QFile       m_ctrlFile;
    double      m_factor;

    void invoke_open();
    void invoke_close();
    void invoke(int _power);
};

class CopterAxis : public QObject
{
    Q_OBJECT
  public:
    CopterAxis(const QSharedPointer<CopterMotor>& _motor1,
               const QSharedPointer<CopterMotor>& _motor2);

    double tilt() const; // -1.0 .. +1.0
    void tilt(double _tilt) const;

    void setPower(unsigned _power) { m_motor1->setPower(_power); m_motor2->setPower(_power); }

  protected:
    QSharedPointer<CopterMotor> m_motor1;
    QSharedPointer<CopterMotor> m_motor2;
};

class CopterCtrl : public QObject
{
    Q_OBJECT
  public:
    CopterCtrl(const QSharedPointer<CopterAxis>& _axisX,
               const QSharedPointer<CopterAxis>& _axisY,
               QLCDNumber* _lcd);

    double tiltX() const { return m_axisX->tilt(); }
    double tiltY() const { return m_axisY->tilt(); }
    void tiltX(double _tilt) const { m_axisX->tilt(_tilt); m_axisX->setPower(m_power); }
    void tiltY(double _tilt) const { m_axisY->tilt(_tilt); m_axisY->setPower(m_power); }
    void adjustTilt(double _tiltX, double _tiltY) const;

    void adjustPower(int _incr);
  protected:
    QLCDNumber* m_lcd;
    int m_power;
    QSharedPointer<CopterAxis> m_axisX;
    QSharedPointer<CopterAxis> m_axisY;

  signals:
    void lcdUpdate(int);
};




class MainWindow : public QMainWindow
{
    Q_OBJECT
  private:
    Ui::MainWindow* m_ui;

  public:
    MainWindow(QWidget* _parent = 0);

  protected:
    QPointer<CopterCtrl> m_copterCtrl;
    QTcpServer           m_tcpServer;
    QPointer<QTcpSocket> m_tcpConnection;
    int                  m_accelerometerInputFd;
    QPointer<QSocketNotifier> m_accelerometerInputNotifier;

    void handleTilt();

    static double updateHistory(QVector<double>& _history, double _newValue);

    double m_tiltX;
    double m_tiltY;
    double m_tiltZ;

    double m_lastAvgX;
    double m_lastAvgY;

    QVector<double> m_historyX;
    QVector<double> m_historyY;

  protected slots:
    void onConnection();
    void onDisconnected();
    void onNetworkRead();
    void onAccelerometerRead();

};


#endif // !MAIN_WINDOW_H_
