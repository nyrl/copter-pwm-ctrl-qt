#ifndef MAIN_WINDOW_H_
#define MAIN_WINDOW_H_

#include <QtGui/QApplication>
#include <QMainWindow>
#include <QSharedPointer>
#include <QLCDNumber>
#include <QString>
#include "ui_main_window.h"


class CopterMotor
{
  public:
    CopterMotor(const QString& _ctrlPath, const QString& _ctrlArg, QLCDNumber* _lcd);
    ~CopterMotor();

    double factor() const { return m_factor; }
    void factor(double _factor);

    void setPower(unsigned _power);

  protected:
    QString m_ctrlPath;
    QString m_ctrlArg;
    QLCDNumber* m_lcd;
    double m_factor;

    void invoke(const QStringList& _args);
};

class CopterAxis
{
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

class CopterCtrl
{
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
};




class MainWindow : public QMainWindow
{
    Q_OBJECT
  public:
    MainWindow(QWidget* _parent = 0);
    ~MainWindow();

    void applyCopterPower();

  protected:
    QSharedPointer<CopterCtrl> m_copterCtrl;

  private:
    Ui::MainWindow* m_ui;
};


#endif // !MAIN_WINDOW_H_