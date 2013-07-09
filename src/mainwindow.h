#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QGridLayout>
#include <QDockWidget>
#include <QKeyEvent>
#include <QTimer>

#include "HUD.h"
//#include "QGCLogPlayer.h"
#include "QGCFlight.h"
//#include "GoogleEarth/map3D/QGCGoogleEarthView.h"
#include "SerialLink.h"

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

private:
    Ui::MainWindow *ui;
    MAVLinkProtocol *mavlink;
        QPointer<HUD> hudWidget;
        //QPointer<QGCLogPlayer> playerWidget;
        QWidget* wgDatos;
        QGridLayout* glDatos;
        QDockWidget* dwDatos;
        double heartbeat;
        bool timeOut;
        int timeOutGPS;
        QTimer *timer;

signals:
        void emitKeyPress(QKeyEvent *event);
};

#endif // MAINWINDOW_H
