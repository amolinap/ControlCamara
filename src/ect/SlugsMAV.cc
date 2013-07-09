#include "SlugsMAV.h"

SlugsMAV::SlugsMAV(MAVLinkProtocol* mavlink, int id)// :                UAS(mavlink, id)
{
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
    //UAS::receiveMessage(link, message);// Let UAS handle the default message set

    int a = 5;

    if (message.sysid == uasId)
    {
        switch (message.msgid)
        {
        case MAVLINK_MSG_ID_HEARTBEAT://MESSAGE ISSUED PERIODICALLY
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
//    type = systemType;
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
