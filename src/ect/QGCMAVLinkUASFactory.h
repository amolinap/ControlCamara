#ifndef QGCMAVLINKUASFACTORY_H
#define QGCMAVLINKUASFACTORY_H

#include <QObject>

#include "QGCMAVLink.h"
#include "MAVLinkProtocol.h"
//#include "UASInterface.h"
#include "LinkInterface.h"

// INCLUDE ALL MAV/UAS CLASSES USING MAVLINK
//#include "UAS.h"
#include "SlugsMAV.h"
//#include "PxQuadMAV.h"
//#include "ArduPilotMegaMAV.h"

class QGCMAVLinkUASFactory : public QObject
{
    Q_OBJECT
public:
    explicit QGCMAVLinkUASFactory(QObject *parent = 0);

    /** @brief Create a new UAS object using MAVLink as protocol */
    static void createUAS(MAVLinkProtocol* mavlink, LinkInterface* link, int sysid, mavlink_heartbeat_t* heartbeat, QObject* parent=NULL);

signals:

public slots:

};

#endif // QGCMAVLINKUASFACTORY_H
