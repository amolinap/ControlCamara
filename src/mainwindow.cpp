#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
        QMainWindow(parent),
        ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    mavlink = new MAVLinkProtocol();
    mavlink->setSystemId(127);
    typeMessage = 0;

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

    connect(ui->cbDireccion, SIGNAL(valueChanged(int)), ui->dlDireccion, SLOT(setValue(int)));
    connect(ui->cbMovimiento, SIGNAL(valueChanged(int)), ui->dlMovimiento, SLOT(setValue(int)));
    connect(ui->cbVelocidad, SIGNAL(valueChanged(int)), ui->dlVelocidad, SLOT(setValue(int)));

    connect(ui->dlDireccion, SIGNAL(valueChanged(int)), this, SLOT(sendDireccion(int)));
    connect(ui->dlMovimiento, SIGNAL(valueChanged(int)), this, SLOT(sendMovimiento(int)));
    connect(ui->dlVelocidad, SIGNAL(valueChanged(int)), this, SLOT(sendVelocidad(int)));

    connect(ui->btAplicar, SIGNAL(clicked()), this, SLOT(aplicarValores()));
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
{
    Q_UNUSED(mav);
}

void MainWindow::setActiveUAS(SlugsMAV *mav)
{
    if (mav != NULL)
    {
        this->activeMav = mav;

        connect(this->activeMav, SIGNAL(emitHeartBeat()), this, SLOT(setAlertHeartbeat()));
        connect(this->activeMav, SIGNAL(emitHeartBeatTimeOut()), this, SLOT(setAlertHeartbeatTimeout()));
        connect(this->activeMav, SIGNAL(emitSendMessage(QString)), this, SLOT(sendMessageStatus(QString)));
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
            //codigo para realizar una actividad si se pierde el pulso!!!
        }

        timeOutGPS++;
    }
}

void MainWindow::refreshTimeOut()
{
    if(heartbeat == -1)
    {
        ui->gbAlertas->setStyleSheet("background-color: #FF3333;");
    }
    else
    {
        ui->gbAlertas->setStyleSheet("background-color: #66FF33;");
    }
}

void MainWindow::aplicarValores()
{
    sendMessage();
}

void MainWindow::sendDireccion(int value)
{
    ui->cbDireccion->setValue(value);

    mlMotorMove.dir = ui->cbDireccion->value();

    sendMessage();
}

void MainWindow::sendMovimiento(int value)
{
    ui->cbMovimiento->setValue(value);

    mlMotorMove.amount = ui->cbMovimiento->value();

    sendMessage();
}

void MainWindow::sendVelocidad(int value)
{
    ui->cbVelocidad->setValue(value);

    mlMotorMove.speed = ui->cbVelocidad->value();

    sendMessage();
}

void MainWindow::sendMessageStatus(QString status)
{
    ui->tbMessages->appendPlainText(status);
}

void MainWindow::sendMessage()
{
    mavlink_message_t msg;

    mavlink_msg_move_motor_encode(MG::SYSTEM::ID, MG::SYSTEM::COMPID, &msg, &(mlMotorMove));

    //this->activeMav->sendMessage(msg);

    qDebug()<<"Direccion: "<<mlMotorMove.dir<<" Velocidad: "<<mlMotorMove.speed<<" Movimiento: "<<mlMotorMove.amount;
}
