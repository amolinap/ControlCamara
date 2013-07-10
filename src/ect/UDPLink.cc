/*=====================================================================

QGroundControl Open Source Ground Control Station

(c) 2009, 2010 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>

This file is part of the QGROUNDCONTROL project

    QGROUNDCONTROL is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    QGROUNDCONTROL is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with QGROUNDCONTROL. If not, see <http://www.gnu.org/licenses/>.

======================================================================*/

/**
 * @file
 *   @brief Definition of UDP connection (server) for unmanned vehicles
 *   @author Lorenz Meier <mavteam@student.ethz.ch>
 *
 */

#include <QTimer>
#include <QList>
#include <QDebug>
#include <QMutexLocker>
#include <iostream>
#include "UDPLink.h"
#include "LinkManager.h"
#include "QGC.h"
//#include <netinet/in.h>

UDPLink::UDPLink(QHostAddress host, quint16 port)
{
    this->host = host;
    this->port = port;
    this->connectState = false;
    this->hosts = new QList<QHostAddress>();
    //this->ports = new QMap<QHostAddress, quint16>();
    this->ports = new QList<quint16>();

    // Set unique ID and add link to the list of links
    this->id = getNextLinkId();
    this->name = tr("UDP Link (port:%1)").arg(14550);
    LinkManager::instance()->add(this);

    saveData = false;
    saveFile = "";
}

UDPLink::~UDPLink()
{
    if(!saveFile.isEmpty())
    {
        QFile fileData(saveFile);
        fileData.close();
        qDebug()<<"Close file for writing";
    }

    disconnect();
}

/**
 * @brief Runs the thread
 *
 **/
void UDPLink::run()
{
    //    forever
    //    {
    //        QGC::SLEEP::msleep(5000);
    //    }
    exec();
}

void UDPLink::setAddress(QString address)
{
    Q_UNUSED(address);
    // FIXME TODO Implement address
    //socket->setLocalAddress(QHostAddress(address));
}

void UDPLink::setPort(int port)
{
    this->port = port;
    disconnect();
    connect();
}

/**
 * @param host Hostname in standard formatting, e.g. localhost:14551 or 192.168.1.1:14551
 */
void UDPLink::addHost(const QString& host)
{
    if (host.contains(":"))
    {
        // Add host
        hosts->append(QHostAddress(host.split(":").first()));
        // Set port according to user input
        ports->append(host.split(":").last().toInt());
    }
    else
    {
        // Add host
        hosts->append(QHostAddress(host));
        // Set port according to default (this port)
        ports->append(port);
    }
}


void UDPLink::writeBytes(const char* data, qint64 size)
{
    // Broadcast to all connected systems
    //QList<QHostAddress>::iterator h;
    // for (h = hosts->begin(); h != hosts->end(); ++h)

    for (int h = 0; h < hosts->size(); h++)
    {
        QHostAddress currentHost = hosts->at(h);
        quint16 currentPort = ports->at(h);

        for (int i=0; i<size; i++)
        {
            unsigned char v =data[i];
            qDebug("%02x ", v);
        }
        qDebug() <<"Sent to " << currentHost.toString() << ":" << currentPort;

        socket->writeDatagram(data, size, currentHost, currentPort);
    }
}

/**
 * @brief Read a number of bytes from the interface.
 *
 * @param data Pointer to the data byte array to write the bytes to
 * @param maxLength The maximum number of bytes to write
 **/
void UDPLink::readBytes()
{
    const qint64 maxLength = 2048;
    char data[maxLength];
    QHostAddress sender;
    quint16 senderPort;

    unsigned int s = socket->pendingDatagramSize();
    if (s > maxLength)
        std::cerr << __FILE__ << __LINE__ << " UDP datagram overflow, allowed to read less bytes than datagram size" << std::endl;

    socket->readDatagram(data, maxLength, &sender, &senderPort);

    // FIXME TODO Check if this method is better than retrieving the data by individual processes
    QByteArray b(data, s);
    emit bytesReceived(this, b);
    //file->write(b.fromHex());

    if(saveData)
    {
        file->write(b);
    }

    //    for (int i=0; i<s; i++)
    //    {
    //        unsigned int v=data[i];

    //        QString binary = QString::number(v, 2);
    //        //file->write(binary.toLocal8Bit());
    //        qDebug("%02x ", v);
    //    }


    //    // Echo data for debugging purposes
    //    std::cerr << __FILE__ << __LINE__ << "Received datagram:" << std::endl;
    //    int i;
    //    for (i=0; i<s; i++)
    //    {
    //        unsigned int v=data[i];
    //        fprintf(stderr,"%02x ", v);
    //    }
    //    std::cerr << std::endl;


    // Add host to broadcast list if not yet present
    if (!hosts->contains(sender))
    {
        hosts->append(sender);
        ports->append(senderPort);
        //        ports->insert(sender, senderPort);
    }
    else
    {
        int index = hosts->indexOf(sender);
        ports->replace(index, senderPort);
    }

}


/**
 * @brief Get the number of bytes to read.
 *
 * @return The number of bytes to read
 **/
qint64 UDPLink::bytesAvailable() {
    return socket->pendingDatagramSize();
}

/**
 * @brief Disconnect the connection.
 *
 * @return True if connection has been disconnected, false if connection couldn't be disconnected.
 **/
bool UDPLink::disconnect()
{
    delete socket;
    socket = NULL;

    connectState = false;

    emit disconnected();
    emit connected(false);
    return !connectState;
}

/**
 * @brief Connect the connection.
 *
 * @return True if connection has been established, false if connection couldn't be established.
 **/
bool UDPLink::connect()
{
    socket = new QUdpSocket(this);

    //Check if we are using a multicast-address
    //    bool multicast = false;
    //    if (host.isInSubnet(QHostAddress("224.0.0.0"),4))
    //    {
    //        multicast = true;
    //        connectState = socket->bind(port, QUdpSocket::ShareAddress);
    //    }
    //    else
    //    {
    connectState = socket->bind(host, port);
    //    }

    //Provides Multicast functionality to UdpSocket
    /* not working yet
    if (multicast)
    {
        int sendingFd = socket->socketDescriptor();

        if (sendingFd != -1)
        {
            // set up destination address
            struct sockaddr_in sendAddr;
            memset(&sendAddr,0,sizeof(sendAddr));
            sendAddr.sin_family=AF_INET;
            sendAddr.sin_addr.s_addr=inet_addr(HELLO_GROUP);
            sendAddr.sin_port=htons(port);

            // set TTL
            unsigned int ttl = 1; // restricted to the same subnet
            if (setsockopt(sendingFd, IPPROTO_IP, IP_MULTICAST_TTL, (unsigned int*)&ttl, sizeof(ttl) ) < 0)
            {
                std::cout << "TTL failed\n";
            }
        }
    }
    */

    //QObject::connect(socket, SIGNAL(readyRead()), this, SLOT(readPendingDatagrams()));
    QObject::connect(socket, SIGNAL(readyRead()), this, SLOT(readBytes()));

    emit connected(connectState);
    if (connectState)
    {
        emit connected();
        connectionStartTime = QGC::groundTimeUsecs()/1000;
    }

    start(HighPriority);
    return connectState;
}

/**
 * @brief Check if connection is active.
 *
 * @return True if link is connected, false otherwise.
 **/
bool UDPLink::isConnected() {
    return connectState;
}

int UDPLink::getId()
{
    return id;
}

QString UDPLink::getName()
{
    return name;
}

void UDPLink::setName(QString name)
{
    this->name = name;
    emit nameChanged(this->name);
}


qint64 UDPLink::getNominalDataRate() {
    return 54000000; // 54 Mbit
}

qint64 UDPLink::getTotalUpstream() {
    statisticsMutex.lock();
    qint64 totalUpstream = bitsSentTotal / ((QGC::groundTimeUsecs()/1000 - connectionStartTime) / 1000);
    statisticsMutex.unlock();
    return totalUpstream;
}

qint64 UDPLink::getCurrentUpstream() {
    return 0; // TODO
}

qint64 UDPLink::getMaxUpstream() {
    return 0; // TODO
}

qint64 UDPLink::getBitsSent() {
    return bitsSentTotal;
}

qint64 UDPLink::getBitsReceived() {
    return bitsReceivedTotal;
}

qint64 UDPLink::getTotalDownstream() {
    statisticsMutex.lock();
    qint64 totalDownstream = bitsReceivedTotal / ((QGC::groundTimeUsecs()/1000 - connectionStartTime) / 1000);
    statisticsMutex.unlock();
    return totalDownstream;
}

qint64 UDPLink::getCurrentDownstream() {
    return 0; // TODO
}

qint64 UDPLink::getMaxDownstream() {
    return 0; // TODO
}

bool UDPLink::isFullDuplex() {
    return true;
}

int UDPLink::getLinkQuality() {
    /* This feature is not supported with this interface */
    return -1;
}

void UDPLink::saveDataToFile(const QString &path)//saveBinaryData
{
    //    QString tempFile(QFileInfo(path).baseName());
    //    QString tempPath(QFileInfo(path).dir().path()+"/");

    //    if(saveFile != tempPath + "/" + tempFile + ".bin")
    //    {
    //        filePath = tempPath;
    //        fileData = tempFile;
    //        createFile();
    //    }

    //    this->saveData = save;
    qDebug()<<"Archivo"<<path;
    QString tempFile(QFileInfo(path).baseName());
    QString tempPath(QFileInfo(path).dir().path());
    qDebug()<<"Nombre del Archivo"<<tempFile;

    bool value = false;
    if(saveFile == tempPath + "/" + tempFile + ".bin")
    {
        saveData = false;
        file->close();

        if(file->remove())
        {
            value = true;
            qDebug()<<"Removiendo Archivo"<<saveFile;
        }
    }
    else
    {
        value = true;
    }

    if(value)
    {
        filePath = tempPath;
        fileData = tempFile;
        createFile();
    }
}

void UDPLink::createFile()
{    
    saveFile = filePath+"/"+fileData+".bin";

    if(QFileInfo(saveFile).exists())
    {
        file = new QFile(saveFile);

        if(file->remove())
        {
            qDebug()<<"Removiendo Archivo Existente"<<saveFile;
        }
    }

    if(!QFileInfo(saveFile).exists())
    {
        file = new QFile(saveFile);
        file->open(QIODevice::WriteOnly | QIODevice::Append);
        this->saveData = true;

        emit writeInFile(true);
        qDebug()<<"Creando Archivo"<<saveFile;
    }
}

void UDPLink::stopWrite()
{
    if(saveData)
    {
        saveData = false;
        file->close();
        emit writeInFile(false);
        qDebug()<<"Deteniendo Archivo"<<saveFile;
    }
}
