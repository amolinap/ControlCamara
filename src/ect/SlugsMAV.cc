#include "SlugsMAV.h"
#include "MAVLinkProtocol.h"
#include "UASManager.h"

SlugsMAV::SlugsMAV(MAVLinkProtocol* protocol, int id)
{
    links = new QList<LinkInterface*>();
    mavlink = protocol;

    uasId = id;
}

SlugsMAV::~SlugsMAV()
{

}

void SlugsMAV::receiveMessage(LinkInterface* link, mavlink_message_t message)
{
    if (!link) return;

    if (!links->contains(link))
    {
        addLink(link);
    }

    if (message.sysid == uasId)
    {
        switch (message.msgid)
        {
        case MAVLINK_MSG_ID_HEARTBEAT://MESSAGE ISSUED PERIODICALLY
            lastHeartbeat = QGC::groundTimeUsecs();
            emit emitHeartBeat();
            {
                mavlink_msg_heartbeat_decode(&message,&mlHeartBeat);                

                //qDebug()<<mlHeartBeat.autopilot<<mlHeartBeat.mavlink_version<<mlHeartBeat.type;
            }
            break;

        case MAVLINK_MSG_ID_MOTOR_POSITION://MESSAGE ISSUED PERIODICALLY
            {
                mavlink_msg_motor_position_decode(&message,&mlMotorPosition);

                emit emitMotorPosition(mlMotorPosition);
                //qDebug()<<mlHeartBeat.autopilot<<mlHeartBeat.mavlink_version<<mlHeartBeat.type;
            }
            break;

            default:            
            break;
        }
    }
}

void SlugsMAV::addLink(LinkInterface* link)
{
    if (!links->contains(link))
    {
        links->append(link);
        connect(link, SIGNAL(destroyed(QObject*)), this, SLOT(removeLink(QObject*)));
    }
}

void SlugsMAV::removeLink(QObject* object)
{
    LinkInterface* link = dynamic_cast<LinkInterface*>(object);
    if (link)
    {
        links->removeAt(links->indexOf(link));
    }
}

void SlugsMAV::sendMessage(mavlink_message_t message)
{
    if(links!= NULL)
        return;

    // Emit message on all links that are currently connected
    foreach (LinkInterface* link, *links)
    {
        if (link)
        {
            sendMessage(link, message);
        }
        else
        {
            // Remove from list
            links->removeAt(links->indexOf(link));
        }
    }
}

void SlugsMAV::sendMessage(LinkInterface* link, mavlink_message_t message)
{
    if(!link) return;
    // Create buffer
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    // Write message into buffer, prepending start sign
    int len = mavlink_msg_to_send_buffer(buffer, &message);
    mavlink_finalize_message_chan(&message, mavlink->getSystemId(), mavlink->getComponentId(), link->getId(), message.len);
    // If link is connected
    if (link->isConnected())
    {
        // Send the portion of the buffer now occupied by the message
        link->writeBytes((const char*)buffer, len);

        emit emitSendMessage("Mensaje enviado");
    }
}

void SlugsMAV::setSystemType(int systemType)
{
    type = systemType;

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
