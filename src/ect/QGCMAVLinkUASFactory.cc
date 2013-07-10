#include "QGCMAVLinkUASFactory.h"
#include "UASManager.h"

QGCMAVLinkUASFactory::QGCMAVLinkUASFactory(QObject *parent) :
        QObject(parent)
{
}

void QGCMAVLinkUASFactory::createUAS(MAVLinkProtocol* mavlink, LinkInterface* link, int sysid, mavlink_heartbeat_t* heartbeat, QObject* parent)
{
    Q_UNUSED(link);
    Q_UNUSED(parent);

    SlugsMAV* mav;

    switch (heartbeat->autopilot)
    {
    case MAV_AUTOPILOT_GENERIC:
        {
            mav = new SlugsMAV(mavlink, sysid);
            // Set the system type
            mav->setSystemType((int)heartbeat->type);
            // Connect this robot to the UAS object
            connect(mavlink, SIGNAL(messageReceived(LinkInterface*, mavlink_message_t)), mav, SLOT(receiveMessage(LinkInterface*, mavlink_message_t)));
            //uas = mav;
        }
        break;

    case MAV_AUTOPILOT_SLUGS:
        {
            mav = new SlugsMAV(mavlink, sysid);
            // Set the system type
            mav->setSystemType((int)heartbeat->type);
            // Connect this robot to the UAS object
            // it is IMPORTANT here to use the right object type,
            // else the slot of the parent object is called (and thus the special
            // packets never reach their goal)
            connect(mavlink, SIGNAL(messageReceived(LinkInterface*, mavlink_message_t)), mav, SLOT(receiveMessage(LinkInterface*, mavlink_message_t)));
            //uas = mav;
        }
        break;

    default:
        {
            mav = new SlugsMAV(mavlink, sysid);
            mav->setSystemType((int)heartbeat->type);
            // Connect this robot to the UAS object
            // it is IMPORTANT here to use the right object type,
            // else the slot of the parent object is called (and thus the special
            // packets never reach their goal)
            connect(mavlink, SIGNAL(messageReceived(LinkInterface*, mavlink_message_t)), mav, SLOT(receiveMessage(LinkInterface*, mavlink_message_t)));
            //uas = mav;
        }
        break;
    }

    //uas->setAutopilotType((int)heartbeat->autopilot);
    //uas->addLink(link);
    UASManager::instance()->addUAS(mav);    
}
