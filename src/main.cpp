#include <QtGui/QApplication>
#include "mainwindow.h"
#include "UDPLink.h"
//#include "QGCLogPlayer.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    //MAVLinkProtocol* mavlink = new MAVLinkProtocol();
    //QGCLogPlayer w(mavlink);
    UDPLink* udpLink = new UDPLink(QHostAddress::Any, 14550);

    // Check if link could be connected
    if (!udpLink->connect())
    {

    }

    QFile* styleSheet = new QFile(":/images/style-mission.css");

    if (styleSheet->open(QIODevice::ReadOnly | QIODevice::Text))
    {
        QString style = QString(styleSheet->readAll());
        style.replace("ICONDIR", QCoreApplication::applicationDirPath()+ "/images/");
        qApp->setStyleSheet(style);
    }

    MainWindow w;
    w.show();

    return a.exec();
}
