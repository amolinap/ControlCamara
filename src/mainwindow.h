#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QGridLayout>
#include <QDockWidget>
#include <QKeyEvent>
#include <QTimer>

//#include "HUD.h"
//#include "QGCLogPlayer.h"
//#include "QGCFlight.h"
//#include "GoogleEarth/map3D/QGCGoogleEarthView.h"
//#include "SerialLink.h"
#include "mavlink.h"
#include "MG.h"
#include "UASManager.h"

namespace Ui {
    class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void keyPressEvent(QKeyEvent *event);

public slots:
    void miSetFocus();
    void addLink(LinkInterface *link);
    void UASCreated(SlugsMAV* mav);
    void setActiveUAS(SlugsMAV* mav);
    void setAlertHeartbeat();
    void setAlertHeartbeatTimeout();
    void refreshTimeOut();
    void aplicarValores();
    void sendMessageStatus(QString status);
    void sendDireccion(int value);
    void sendMovimiento(int value);
    void sendVelocidad(int value);
    void sendMessage();

private:
    Ui::MainWindow *ui;
    MAVLinkProtocol *mavlink;
    //QPointer<HUD> hudWidget;
    //QPointer<QGCLogPlayer> playerWidget;
    QWidget* wgDatos;
    QGridLayout* glDatos;
    QDockWidget* dwDatos;
    double heartbeat;
    bool timeOut;
    int timeOutGPS;
    QTimer *timer;
    int typeMessage;

    SlugsMAV *activeMav;

    mavlink_motor_position_t mlMotorPosition;
    mavlink_move_motor_t mlMotorMove;

signals:
    void emitKeyPress(QKeyEvent *event);
};

#endif // MAINWINDOW_H
