#include "main_window.h"

#include <QtGlobal>
#include <QStringList>
#include <QIODevice>
#include <QPalette>
#include <QColor>

#include <cmath>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <linux/input.h>
#include <linux/ioctl.h>


#define MMAIO				0xA1
#define MMA7660FC_IOCTL_S_POLL_DELAY	_IOW(MMAIO, 0x03, int)


static QString s_ctrl_path_x1("/sys/devices/platform/ehrpwm.0/pwm/ehrpwm.0:0/duty_percent");
static QString s_ctrl_path_x2("/sys/devices/platform/ehrpwm.0/pwm/ehrpwm.0:1/duty_percent");
static QString s_ctrl_path_y1("/sys/devices/platform/ehrpwm.1/pwm/ehrpwm.1:0/duty_percent");
static QString s_ctrl_path_y2("/sys/devices/platform/ehrpwm.1/pwm/ehrpwm.1:1/duty_percent");
static QString s_accel_input_path("/dev/copter_accel");

static unsigned s_tcp_port = 4000;

static double s_tilt_step = 0.02;
static double s_power_step1 = 1;
static double s_power_step2 = 5;
static int s_power_min = 0;
static int s_power_max = 100;
static int s_motor_min = 48;
static int s_motor_max = 72;

static double s_accel_linear = -0.02;
static double s_accel_derivative = -0.005;

static bool s_report_accel_data = false;




void CopterMotor::invoke_open()
{
  invoke(0);
}

void CopterMotor::invoke_close()
{
  invoke(0);
}

void CopterMotor::invoke(int _power)
{
  QString s;
  m_ctrlFile.open(QIODevice::WriteOnly|QIODevice::Truncate|QIODevice::Unbuffered|QIODevice::Text);
  double powerFactor = static_cast<double>(s_motor_max - s_motor_min) / static_cast<double>(s_power_max - s_power_min);
  double power = s_motor_min + static_cast<double>(_power - s_power_min) * powerFactor;
  s.sprintf("%d\n", static_cast<int>(power));
  m_ctrlFile.write(s.toAscii().data());
  m_ctrlFile.close();
}

CopterMotor::CopterMotor(const QString& _ctrlPath, QLCDNumber* _lcd)
 :m_lcd(_lcd),
  m_ctrlFile(_ctrlPath),
  m_factor(1.0)
{
  invoke_open();
}

CopterMotor::~CopterMotor()
{
  invoke_close();
}

void CopterMotor::factor(double _factor)
{
  m_factor = qMax(qMin(_factor, 1.0), 0.0);
}

void CopterMotor::setPower(unsigned _power)
{
  int pwr = round(m_factor * (double)_power);

  QPalette palette = m_lcd->palette();
  QColor bg = palette.color(QPalette::Disabled, m_lcd->backgroundRole());
  double pwrSat = 1.0 - static_cast<double>(_power-s_power_min)/(s_power_max-s_power_min);
  double ftrSat = m_factor;
  bg.setBlue( bg.blue() *pwrSat);
  bg.setGreen(bg.green()*pwrSat + 0xff*(1.0-pwrSat)*ftrSat);
  bg.setRed(  bg.red()  *pwrSat + 0xff*(1.0-pwrSat)*(1-ftrSat));
  palette.setColor(QPalette::Normal, m_lcd->backgroundRole(), bg);
  palette.setColor(QPalette::Active, m_lcd->backgroundRole(), bg);
  palette.setColor(QPalette::Inactive, m_lcd->backgroundRole(), bg);
  m_lcd->setPalette(palette);
  m_lcd->display(pwr);

  invoke(pwr);
}




CopterAxis::CopterAxis(const QSharedPointer<CopterMotor>& _motor1,
                       const QSharedPointer<CopterMotor>& _motor2)
 :m_motor1(_motor1),
  m_motor2(_motor2)
{
}

double CopterAxis::tilt() const
{
  return m_motor1->factor() - m_motor2->factor();
}

void CopterAxis::tilt(double _tilt) const
{
  if (_tilt == 0)
  {
    m_motor1->factor(1.0);
    m_motor2->factor(1.0);
  }
  else if (_tilt < 0)
  {
    m_motor1->factor(1.0+_tilt);
    m_motor2->factor(1.0);
  }
  else if (_tilt > 0)
  {
    m_motor1->factor(1.0);
    m_motor2->factor(1.0-_tilt);
  }
}




CopterCtrl::CopterCtrl(const QSharedPointer<CopterAxis>& _axisX,
                       const QSharedPointer<CopterAxis>& _axisY,
                       QLCDNumber* _lcd)
 :m_lcd(_lcd),
  m_power(0),
  m_axisX(_axisX),
  m_axisY(_axisY)
{
}

void CopterCtrl::adjustTilt(double _tiltX, double _tiltY) const
{
  m_axisX->tilt(m_axisX->tilt() + _tiltX);
  m_axisY->tilt(m_axisY->tilt() + _tiltY);
  m_axisX->setPower(m_power);
  m_axisY->setPower(m_power);
}

void CopterCtrl::adjustPower(int _incr)
{
  m_power += _incr;
  m_power = qMax(qMin(m_power, s_power_max), s_power_min);

  QPalette palette = m_lcd->palette();
  QColor bg = palette.color(QPalette::Disabled, m_lcd->backgroundRole());
  double pwrSat = 1.0 - static_cast<double>(m_power-s_power_min)/(s_power_max-s_power_min);
  bg.setBlue( bg.blue() *pwrSat);
  bg.setGreen(bg.green()*pwrSat + 0xff*(1.0-pwrSat));
  bg.setRed(  bg.red()  *pwrSat);
  palette.setColor(QPalette::Normal, m_lcd->backgroundRole(), bg);
  palette.setColor(QPalette::Active, m_lcd->backgroundRole(), bg);
  palette.setColor(QPalette::Inactive, m_lcd->backgroundRole(), bg);
  m_lcd->setPalette(palette);
  m_lcd->display(m_power);

  m_axisX->setPower(m_power);
  m_axisY->setPower(m_power);
}




MainWindow::MainWindow(QWidget* _parent)
 :QMainWindow(_parent),
  m_ui(new Ui::MainWindow),
  m_copterCtrl(),
  m_tcpServer(),
  m_tcpConnection(),
  m_accelerometerInputFd(-1),
  m_accelerometerInputNotifier(0),
  m_tiltX(0),
  m_tiltY(0),
  m_tiltZ(0),
  m_lastAvgX(0),
  m_lastAvgY(0),
  m_normalAvgX(0.0),
  m_normalAvgY(0.0),
  m_historyX(10, 0.0),
  m_historyY(10, 0.0)
{
  m_ui->setupUi(this);

  QSharedPointer<CopterMotor> mx1(new CopterMotor(s_ctrl_path_x1, m_ui->motor_x1));
  QSharedPointer<CopterMotor> mx2(new CopterMotor(s_ctrl_path_x2, m_ui->motor_x2));
  QSharedPointer<CopterMotor> my1(new CopterMotor(s_ctrl_path_y1, m_ui->motor_y1));
  QSharedPointer<CopterMotor> my2(new CopterMotor(s_ctrl_path_y2, m_ui->motor_y2));

  QSharedPointer<CopterAxis>  m_axisX(new CopterAxis(mx2, mx1));
  QSharedPointer<CopterAxis>  m_axisY(new CopterAxis(my2, my1));
  m_copterCtrl = new CopterCtrl(m_axisX, m_axisY, m_ui->motor_all);

  m_tcpServer.listen(QHostAddress::Any, s_tcp_port);
  connect(&m_tcpServer, SIGNAL(newConnection()), this, SLOT(onConnection()));

  m_accelerometerInputFd = open(s_accel_input_path.toAscii().data(), O_SYNC, O_RDONLY);
  if (m_accelerometerInputFd == -1)
    qDebug() << "Cannot open accelerometer input file " << s_accel_input_path << ", reason: " << errno;

  m_accelerometerInputNotifier = new QSocketNotifier(m_accelerometerInputFd, QSocketNotifier::Read, this);
  connect(m_accelerometerInputNotifier, SIGNAL(activated(int)), this, SLOT(onAccelerometerRead()));
  m_accelerometerInputNotifier->setEnabled(true);

  m_copterCtrl->adjustPower(0);

  showFullScreen();
  showMaximized();
}

void MainWindow::onConnection()
{
  if (!m_tcpConnection.isNull())
    qDebug() << "Replacing existing connection";
  m_tcpConnection = m_tcpServer.nextPendingConnection();
  qDebug() << "Accepted new connection";
  m_tcpConnection->setSocketOption(QAbstractSocket::LowDelayOption, 1);
  connect(m_tcpConnection, SIGNAL(disconnected()), this, SLOT(onDisconnected()));
  connect(m_tcpConnection, SIGNAL(readyRead()), this, SLOT(onNetworkRead()));
}

void MainWindow::onDisconnected()
{
  qDebug() << "Existing connection disconnected";
  m_tcpConnection = 0;
}

void MainWindow::onNetworkRead()
{
  if (m_tcpConnection.isNull())
    return;

  while (m_tcpConnection->isReadable())
  {
    char c;
    if (!m_tcpConnection->getChar(&c))
      break;
    switch (c)
    {
      case '1': m_copterCtrl->adjustTilt(-s_tilt_step, -s_tilt_step); break;
      case '2': m_copterCtrl->adjustTilt(0,            -s_tilt_step); break;
      case '3': m_copterCtrl->adjustTilt(+s_tilt_step, -s_tilt_step); break;
      case '4': m_copterCtrl->adjustTilt(-s_tilt_step, 0); break;
      case '5': m_copterCtrl->tiltX(0); m_copterCtrl->tiltY(0); break;
      case '6': m_copterCtrl->adjustTilt(+s_tilt_step, 0); break;
      case '7': m_copterCtrl->adjustTilt(-s_tilt_step, +s_tilt_step); break;
      case '8': m_copterCtrl->adjustTilt(0,            +s_tilt_step); break;
      case '9': m_copterCtrl->adjustTilt(+s_tilt_step, +s_tilt_step); break;
      case 'Z': m_copterCtrl->adjustPower(-s_power_max); break;
      case 'z': m_copterCtrl->adjustPower(-s_power_step2); break;
      case 'x': m_copterCtrl->adjustPower(-s_power_step1); break;
      case 'c': m_copterCtrl->adjustPower(+s_power_step1); break;
      case 'v': m_copterCtrl->adjustPower(+s_power_step2); break;
      case 'V': m_copterCtrl->adjustPower(+s_power_max); break;

      case '(': s_tilt_step *= 0.9; break;
      case ')': s_tilt_step /= 0.9; break;
      case '[': s_accel_linear *= 0.9; break;
      case ']': s_accel_linear /= 0.9; break;
      case '{': s_accel_derivative *= 0.9; break;
      case '}': s_accel_derivative /= 0.9; break;

      case '<':
                if (m_historyX.size() > 1)
                  m_historyX.pop_front();
                if (m_historyY.size() > 1)
                  m_historyY.pop_front();
                break;
      case '>':
                m_historyX.push_front(0.0);
                m_historyY.push_front(0.0);
                break;

      case '_':
                m_normalAvgX = m_lastAvgX;
                m_normalAvgY = m_lastAvgY;
                break;

      case 'I': s_report_accel_data = true; break;
      case 'i': s_report_accel_data = false; break;
    }

    QString buf;
    buf.sprintf("accel linear %f, derivative %f, tilt step %f, filter size %i/%i\r\n"
                "last average x %f, y %f; normal x %f, y %f\r\n",
                static_cast<double>(s_accel_linear),
                static_cast<double>(s_accel_derivative),
                static_cast<double>(s_tilt_step),
                static_cast<int>(m_historyX.size()), static_cast<int>(m_historyY.size()),
                static_cast<double>(m_lastAvgX),
                static_cast<double>(m_lastAvgY),
                static_cast<double>(m_normalAvgX),
                static_cast<double>(m_normalAvgY));
    m_tcpConnection->write(buf.toAscii());
  }
}

void MainWindow::onAccelerometerRead()
{
  struct input_event evt;

  if (read(m_accelerometerInputFd, reinterpret_cast<char*>(&evt), sizeof(evt)) != sizeof(evt))
  {
    qDebug() << "Incomplete accelerometer data read";
    return;
  }

  switch (evt.type)
  {
    case EV_ABS:
      switch (evt.code)
      {
        case ABS_X: m_tiltX = evt.value; break;
        case ABS_Y: m_tiltY = evt.value; break;
        case ABS_Z: m_tiltZ = evt.value; break;
      }
      break;

    case EV_SYN:
      handleTilt();

      if (s_report_accel_data && !m_tcpConnection.isNull())
      {
        QString buf;
        buf.sprintf("ACCEL %i %i %i AVG %i %i\r\n",
                    static_cast<int>(m_tiltX),
                    static_cast<int>(m_tiltY),
                    static_cast<int>(m_tiltZ),
                    static_cast<int>(m_lastAvgX),
                    static_cast<int>(m_lastAvgY));
        m_tcpConnection->write(buf.toAscii());
      }

      break;
  }
}


double MainWindow::updateHistory(QVector<double>& _history, double _newValue)
{
  _history.erase(_history.begin());
  _history.push_back(_newValue);

  double sum = 0;
  for (QVector<double>::const_iterator it(_history.begin()); it != _history.end(); ++it)
    sum += *it;

  return sum / _history.size();
}

void MainWindow::handleTilt()
{
  double nextAvgX = updateHistory(m_historyX, m_tiltX);
  double nextAvgY = updateHistory(m_historyX, m_tiltX);

  const double adjX = s_accel_linear*(nextAvgX - m_normalAvgX) + s_accel_derivative*(nextAvgX - m_lastAvgX);
  const double adjY = s_accel_linear*(nextAvgY - m_normalAvgY) + s_accel_derivative*(nextAvgY - m_lastAvgY);

  m_copterCtrl->adjustTilt(adjX, adjY);

  m_lastAvgX = nextAvgX;
  m_lastAvgY = nextAvgY;
}


