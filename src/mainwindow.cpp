#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    mavlink = new MAVLinkProtocol();

    //SerialLink* link = new SerialLink();
    // TODO This should be only done in the dialog itself

//    LinkManager::instance()->add(link);
//    LinkManager::instance()->addProtocol(link, mavlink);
    QList<LinkInterface*> links = LinkManager::instance()->getLinks();
    foreach(LinkInterface* link, links)
    {
        this->addLink(link);
    }

//    dwDatos= new QDockWidget("DATOS");
//    dwDatos->setMinimumWidth(350);
//    dwDatos->setMaximumWidth(350);
//    wgDatos = new QWidget();
//    glDatos = new QGridLayout(wgDatos);
//    glDatos->setContentsMargins(5,5,5,5);
//    glDatos->addWidget(new QGCLogPlayer(mavlink));

//    QGCFlight* flight = new QGCFlight();
//    QGCGoogleEarthView * google = new QGCGoogleEarthView();

//    connect(flight, SIGNAL(emitParametrosPosicion(double,double,double)), google, SLOT(setParametrosPosicion(double,double,double)));
//    connect(flight, SIGNAL(emitParametrosVuelo(double,double,double)), google, SLOT(setParametrosVuelo(double,double,double)));
//    connect(this, SIGNAL(emitKeyPress(QKeyEvent*)), google, SLOT(keyPressChange(QKeyEvent*)));
//    connect(google, SIGNAL(emitFocus()), this, SLOT(miSetFocus()));

//    glDatos->addWidget(flight);

//    hudWidget = new HUD(400, 400, wgDatos);
//    hudWidget->show();
//    glDatos->addWidget(hudWidget);
//    dwDatos->setWidget(wgDatos);

//    addDockWidget(Qt::RightDockWidgetArea, dwDatos);

//    setCentralWidget(google);
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
