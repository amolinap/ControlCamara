#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    mavlink = new MAVLinkProtocol();
    mavlink->setSystemId(127);

    connect(ui->cxHeartBeat, SIGNAL(toggled(bool)), mavlink, SLOT(enableHeartbeats(bool)));

    QList<LinkInterface*> links = LinkManager::instance()->getLinks();
    foreach(LinkInterface* link, links)
    {
        this->addLink(link);
    }

    timer = new QTimer(this);
    timer->setInterval(250);
    connect(timer, SIGNAL(timeout()), this, SLOT(refreshTimeOut()));
    timer->start();

    //connect(UASManager::instance(), SIGNAL(UASCreated(SlugsMAV*)), this, SLOT(UASCreated(SlugsMAV*)));
    connect(UASManager::instance(), SIGNAL(activeUASSet(SlugsMAV*)), this, SLOT(setActiveUAS(SlugsMAV*)));

    connect(ui->dlDireccion, SIGNAL(valueChanged(int)), ui->lbDireccion, SLOT(setNum(int)));
    connect(ui->dlMovimiento, SIGNAL(valueChanged(int)), ui->lbMovimiento, SLOT(setNum(int)));
    connect(ui->dlVelocidad, SIGNAL(valueChanged(int)), ui->lbVelocidad, SLOT(setNum(int)));
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::keyPressEvent(QKeyEvent *event)
{
    emit emitKeyPress(event);
}

void MainWindow::miSetFocus()
{
    this->setFocus();
}


void MainWindow::addLink(LinkInterface *link)
{
    // IMPORTANT! KEEP THESE TWO LINES
    // THEY MAKE SURE THE LINK IS PROPERLY REGISTERED
    // BEFORE LINKING THE UI AGAINST IT
    // Register (does nothing if already registered)
    LinkManager::instance()->add(link);
    LinkManager::instance()->addProtocol(link, mavlink);
}

void MainWindow::UASCreated(SlugsMAV *mav)
{}

void MainWindow::setActiveUAS(SlugsMAV *mav)
{
    if (mav != NULL)
    {
        //this->activeUAS = uas;

        connect(mav, SIGNAL(emitHeartBeat()), this, SLOT(setAlertHeartbeat()));
        connect(mav, SIGNAL(emitHeartBeatTimeOut()), this, SLOT(setAlertHeartbeatTimeout()));
    }
}


void MainWindow::setAlertHeartbeat()
{
    this->heartbeat = 1;
    timeOut = false;
}

void MainWindow::setAlertHeartbeatTimeout()
{
    this->heartbeat = -1;
    timeOut = true;

    if(timeOutGPS <= 5)
    {
        if(timeOutGPS == 5)
        {

        }

        timeOutGPS++;
    }
}

void MainWindow::refreshTimeOut()
{
//    iAlertHeartbeat->setValue(heartbeat);
//    alertHeartbeat = iAlertHeartbeat->setStyleAlert();

    if(heartbeat == -1)
    {
        ui->wgHeartBeat->setStyleSheet("background-color: #FF3333;");
    }
    else
    {
        ui->wgHeartBeat->setStyleSheet("background-color: #66FF33;");
    }
}
