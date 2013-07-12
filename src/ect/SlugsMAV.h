/*=====================================================================
======================================================================*/

#ifndef SLUGSMAV_H
#define SLUGSMAV_H

#include <QTimer>
#include <QMap>
#include <QString>
#include <QApplication>
#include <QDebug>

//#include "UASManager.h"
#include "MAVLinkProtocol.h"
#include "mavlink.h"
#include "LinkManager.h"
//#include "QGCConfiguration.h"

/**
* @brief Received messages defined for slugs
*
* The SlugsMAV received messages defined for SLUGS specified in protocol MAVLINK.
*
**/
class SlugsMAV : public QObject
{
    Q_OBJECT
    //Q_INTERFACES(UASInterface)

public:
    /** @brief This is the class constructor.
     *
     * @param mavlink  The protocol communication
     * @param id    The id for UAV
     **/
    SlugsMAV(MAVLinkProtocol* mavlink, int id = 0);
    ~SlugsMAV();
    void setSelected();

public slots:
    /**
     * @brief This function is called by MAVLink once a complete, uncorrupted (CRC check valid)
     * mavlink packet is received.
     *
     * @param link Hardware link the message came from (e.g. /dev/ttyUSB0 or UDP port).
     *             messages can be sent back to the system via this link
     * @param message MAVLink message, as received from the MAVLink protocol stack
     */
    void receiveMessage(LinkInterface* link, mavlink_message_t message);
    void setSystemType(int systemType);
    void updateState();
    int getUASID();
    void sendMessage(mavlink_message_t message);
    void sendMessage(LinkInterface* link, mavlink_message_t message);
    void addLink(LinkInterface* link);
    void removeLink(QObject* object);

signals:
    void emitHeartBeat();
    void emitHeartBeatTimeOut();
    void emitSendMessage(QString);
    void emitMotorPosition(mavlink_motor_position_t motorPosition);

protected:
    static const unsigned int timeoutIntervalHeartbeat = 2000 * 1000; ///< Heartbeat timeout is 1.5 seconds
    int type;
    QList<LinkInterface*>* links;
    MAVLinkProtocol* mavlink;

private:
    int uasId;    
    //float hCommand, uCommand, rCommand;
    //int countTimeGPS, minuteGPS, secondGPS, fromWP, countPTZ;
    //bool outPTZ;
    //UAVType typeUAV;
    mavlink_heartbeat_t mlHeartBeat;
    mavlink_motor_position_t mlMotorPosition;
    quint64 lastHeartbeat;      ///< Time of the last heartbeat message
};

#endif // SLUGSMAV_H
