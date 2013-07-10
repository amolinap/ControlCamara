#include "SlugsMAV.h"
#include "MAVLinkProtocol.h"
#include "UASManager.h"

SlugsMAV::SlugsMAV(MAVLinkProtocol* mavlink, int id)// :                UAS(mavlink, id)
{
    Q_UNUSED(mavlink);

    hCommand = 0;
    uCommand = 0;
    u_m = 0;

    uasId = id;
    countTimeGPS = 0;
    minuteGPS = 0;
    secondGPS = 0;
    fromWP = 0;
    countPTZ = 0;
    outPTZ = false;
    typeUAV = UAV;
}

SlugsMAV::~SlugsMAV()
{

}

void SlugsMAV::receiveMessage(LinkInterface* link, mavlink_message_t message)
{
    Q_UNUSED(link);

    if (message.sysid == uasId)
    {
        switch (message.msgid)
        {
        case MAVLINK_MSG_ID_HEARTBEAT://MESSAGE ISSUED PERIODICALLY
            lastHeartbeat = QGC::groundTimeUsecs();
            emit emitHeartBeat();
            {
                mavlink_msg_heartbeat_decode(&message,&mlHeartBeat);                

                qDebug()<<mlHeartBeat.autopilot<<mlHeartBeat.mavlink_version<<mlHeartBeat.type;
            }
            break;        

            default:            
            break;
        }
    }
}

void SlugsMAV::setTypeUAV(UAVType type)
{
    this->typeUAV = type;
}

void SlugsMAV::setSystemType(int systemType)
{
    type = systemType;
//    // If the airframe is still generic, change it to a close default type
//    if (airframe == 0)
//    {
//        switch (systemType)
//        {
//        case MAV_FIXED_WING:
//            airframe = QGC_AIRFRAME_EASYSTAR;
//            break;
//        case MAV_QUADROTOR:
//            airframe = QGC_AIRFRAME_MIKROKOPTER;
//            break;
//        }
//    }

//    emit systemSpecsChanged(uasId);
}


void SlugsMAV::updateState()
{
    // Check if heartbeat timed out
    quint64 heartbeatInterval = QGC::groundTimeUsecs() - lastHeartbeat;
    if (heartbeatInterval > timeoutIntervalHeartbeat)
    {
        //emit heartbeatTimeout(heartbeatInterval);
        emit emitHeartBeatTimeOut();
    }
}

int SlugsMAV::getUASID()
{
    return type;
}

void SlugsMAV::setSelected()
{
    if (UASManager::instance()->getActiveUAS() != this)
    {
        UASManager::instance()->setActiveUAS(this);
    }
}
