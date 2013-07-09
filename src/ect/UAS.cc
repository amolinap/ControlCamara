/*===================================================================
======================================================================*/

/**
 * @file
 *   @brief Represents one unmanned aerial vehicle
 *
 *   @author Lorenz Meier <mavteam@student.ethz.ch>
 *
 */

#include <QList>
#include <QMessageBox>
#include <QTimer>
#include <QSettings>
#include <iostream>
#include <QDebug>
#include <cmath>
#include <qmath.h>
#include "UAS.h"
#include "LinkInterface.h"
#include "UASManager.h"
#include "QGC.h"
#include "MAVLinkProtocol.h"
#include "QGCMAVLink.h"
#include "LinkManager.h"
#include "SerialLink.h"

UAS::UAS(MAVLinkProtocol* protocol, int id) : UASInterface(),
uasId(id),
startTime(QGC::groundTimeMilliseconds()),
commStatus(COMM_DISCONNECTED),
name(""),
autopilot(-1),
links(new QList<LinkInterface*>()),
unknownPackets(),
mavlink(protocol),
//waypointManager(*this),
mode(-1),
status(-1),
navMode(-1),
onboardTimeOffset(0),
receiveDropRate(0),
sendDropRate(0),
positionLock(false),
localX(0.0),
localY(0.0),
localZ(0.0),
latitude(0.0),
longitude(0.0),
altitude(0.0),
roll(0.0),
pitch(0.0),
yaw(0.0),
statusTimeout(new QTimer(this)),
paramsOnceRequested(false),
airframe(0)
//paramManager(NULL)
{
    color = UASInterface::getNextColor();    
    connect(statusTimeout, SIGNAL(timeout()), this, SLOT(updateState()));
    connect(this, SIGNAL(systemSpecsChanged(int)), this, SLOT(writeSettings()));
    statusTimeout->start(500);
    readSettings();
}

UAS::~UAS()
{
    writeSettings();
    delete links;
    links=NULL;
}

void UAS::writeSettings()
{
    QSettings settings;
    settings.beginGroup(QString("MAV%1").arg(uasId));
    settings.setValue("NAME", this->name);
    settings.setValue("AIRFRAME", this->airframe);
    settings.setValue("AP_TYPE", this->autopilot);    
    settings.endGroup();
    settings.sync();
}

void UAS::readSettings()
{
    QSettings settings;
    settings.beginGroup(QString("MAV%1").arg(uasId));
    this->name = settings.value("NAME", this->name).toString();
    this->airframe = settings.value("AIRFRAME", this->airframe).toInt();
    this->autopilot = settings.value("AP_TYPE", this->autopilot).toInt();
    settings.endGroup();
}

int UAS::getUASID() const
{
    return uasId;
}

void UAS::updateState()
{
    // Check if heartbeat timed out
    quint64 heartbeatInterval = QGC::groundTimeUsecs() - lastHeartbeat;
    if (heartbeatInterval > timeoutIntervalHeartbeat)
    {
        emit heartbeatTimeout(heartbeatInterval);
        emit heartbeatTimeout();
    }

    // Position lock is set by the MAVLink message handler
    // if no position lock is available, indicate an error
    if (positionLock)
    {
        positionLock = false;
    }
    else
    {
        if (mode > (uint8_t)MAV_MODE_LOCKED && positionLock)
        {
            //GAudioOutput::instance()->notifyNegative();
        }
    }
}

void UAS::setSelected()
{
    if (UASManager::instance()->getActiveUAS() != this)
    {
        UASManager::instance()->setActiveUAS(this);        
    }
}

bool UAS::getSelected() const
{
    return (UASManager::instance()->getActiveUAS() == this);
}

void UAS::receiveMessage(LinkInterface* link, mavlink_message_t message)
{
    if (!link) return;

    if (!links->contains(link))
    {
        addLink(link);        
    }

    if (message.sysid == uasId)
    {
        QString uasState;
        QString stateDescription;        

        switch (message.msgid)
        {
        case MAVLINK_MSG_ID_HEARTBEAT:
            lastHeartbeat = QGC::groundTimeUsecs();
            emit heartbeat(this);
            // Set new type if it has changed
            if (this->type != mavlink_msg_heartbeat_get_type(&message))
            {
                this->type = mavlink_msg_heartbeat_get_type(&message);
                if (airframe == 0)
                {
                    switch (type)
                    {
                    case MAV_FIXED_WING:
                        setAirframe(UASInterface::QGC_AIRFRAME_EASYSTAR);
                        break;
                    case MAV_QUADROTOR:
                        setAirframe(UASInterface::QGC_AIRFRAME_CHEETAH);
                        break;
                    default:
                        // Do nothing
                        break;
                    }
                }

                this->autopilot = mavlink_msg_heartbeat_get_autopilot(&message);                
            }

            break;

        case MAVLINK_MSG_ID_BOOT:
            getStatusForCode((int)MAV_STATE_BOOT, uasState, stateDescription);
            emit statusChanged(this, uasState, stateDescription);
            onboardTimeOffset = 0; // Reset offset measurement
            break;

        case MAVLINK_MSG_ID_SYS_STATUS://MESSAGE ISSUED PERIODICALLY
            {
                mavlink_sys_status_t state;
                mavlink_msg_sys_status_decode(&message, &state);

                emit emitMessageSysStatus(uasId, state);

                //bool statechanged = false;
                //bool modechanged = false;

                if (state.status != this->status)
                {
                    //statechanged = true;
                    this->status = state.status;
                    getStatusForCode((int)state.status, uasState, stateDescription);
                    emit statusChanged(this, uasState, stateDescription);
                    emit statusChanged(this->status);                    
                }

                if (navMode != state.nav_mode)
                {
                    emit navModeChanged(uasId, state.nav_mode, getNavModeText(state.nav_mode));
                    navMode = state.nav_mode;
                }

                emit loadChanged(this,state.load/10.0f);                

                if (this->mode != static_cast<int>(state.mode))
                {
                    //modechanged = true;
                    this->mode = static_cast<int>(state.mode);
                    QString mode;

                    switch (state.mode)
                    {
                    case (uint8_t)MAV_MODE_LOCKED:
                        mode = "LOCKED MODE";
                        break;
                    case (uint8_t)MAV_MODE_MANUAL:
                        mode = "MODO MANUAL";
                        break;
                    case (uint8_t)MAV_MODE_AUTO:
                        mode = "AUTO MODE";
                        break;
                    case (uint8_t)MAV_MODE_GUIDED:
                        mode = "MODO GUIADO";
                        break;

                    case (uint8_t)MAV_MODE_TEST1:
                        mode = "TEST1 MODE";
                        break;
                    case (uint8_t)MAV_MODE_TEST2:
                        mode = "TEST2 MODE";
                        break;
                    case (uint8_t)MAV_MODE_READY:
                        mode = "READY MODE";
                        break;

                    case (uint8_t)MAV_MODE_TEST3:
                        mode = "TEST3 MODE";
                        break;

                    case (uint8_t)MAV_MODE_RC_TRAINING:
                        mode = "RC TRAINING MODE";
                        break;
                    default:
                        mode = "UNINIT MODE";
                        break;
                    }

                    emit modeChanged(this->getUASID(), mode, state.mode);                    
                }

                emit dropRateChanged(this->getUASID(), state.packet_drop/1000.0f);                                

                if (state.status == MAV_STATE_POWEROFF)
                {
                    emit systemRemoved(this);
                    emit systemRemoved();
                }
            }
            break;

        case MAVLINK_MSG_ID_RAW_IMU://MESSAGE ISSUED PERIODICALLY
            {
                mavlink_raw_imu_t raw;
                mavlink_msg_raw_imu_decode(&message, &raw);                

                emit emitMessageScaledRaw(uasId, raw);
            }
            break;

         case MAVLINK_MSG_ID_SCALED_IMU://MESSAGE ISSUED PERIODICALLY
            {
                mavlink_scaled_imu_t scaled;
                mavlink_msg_scaled_imu_decode(&message, &scaled);                

                emit emitMessageScaledImu(uasId, scaled);
            }
            break;

        case MAVLINK_MSG_ID_ATTITUDE://MESSAGE ISSUED PERIODICALLY
            {
                mavlink_attitude_t attitude;
                mavlink_msg_attitude_decode(&message, &attitude);
                quint64 time = getUnixTime(0);//attitude.usec);
                roll = QGC::limitAngleToPMPIf(attitude.roll);
                pitch = QGC::limitAngleToPMPIf(attitude.pitch);
                yaw = QGC::limitAngleToPMPIf(attitude.yaw);

                emit valueChangedPlot(uasId, "roll", "rad", roll, time);
                emit valueChangedPlot(uasId, "pitch", "rad", pitch, time);
                emit valueChangedPlot(uasId, "yaw", "rad", yaw, time);
                emit valueChangedPlot(uasId, "rollspeed", "rad/s", attitude.rollspeed, time);
                emit valueChangedPlot(uasId, "pitchspeed", "rad/s", attitude.pitchspeed, time);
                emit valueChangedPlot(uasId, "yawspeed", "rad/s", attitude.yawspeed, time);
                emit valueChangedPlot(uasId, "usecatt", "s/n", attitude.usec, time);

                emit emitMessageAttitudeRad(uasId, attitude);

                uint64_t tempUsec = attitude.usec;

                if(tempUsec != usec)
                {
                    emit saveData();
                    usec = tempUsec;
                }                

                emit attitudeChanged(this, roll, pitch, yaw, time);                
            }
            break;

        case MAVLINK_MSG_ID_LOCAL_POSITION://MESSAGE ISSUED PERIODICALLY
            {
                mavlink_local_position_t pos;
                mavlink_msg_local_position_decode(&message, &pos);
                quint64 time = getUnixTime(pos.usec);
                localX = pos.x;
                localY = pos.y;
                localZ = pos.z;
                emit valueChangedPlot(uasId, "z", "m", pos.z, time);
                emit valueChangedPlot(uasId, "y speed", "m/s", pos.vy, time);
                emit emitMessageLocalPosition(uasId, pos);                
                emit emitValueAltitude(pos.z);

                // Set internal state
                if (!positionLock)
                {
                    // If position was not locked before, notify positive
                    //GAudioOutput::instance()->notifyPositive();
                }
                positionLock = true;
            }
            break;

        case MAVLINK_MSG_ID_GPS_RAW://MESSAGE ISSUED PERIODICALLY
            {
                mavlink_gps_raw_t pos;
                mavlink_msg_gps_raw_decode(&message, &pos);
                quint64 time = getUnixTime();                

                emit emitMessageGPSRaw(uasId, pos);                

                if (pos.fix_type > 0)
                {
                    emit globalPositionChanged(this, pos.lat, pos.lon, pos.alt, time);

                    latitude = pos.lat;
                    longitude = pos.lon;
                    altitude = pos.alt;
                    positionLock = true;

                    // Check for NaN
                    int alt = pos.alt;
                    if (alt != alt)
                    {
                        alt = 0;
                        emit textMessageReceived(uasId, message.compid, 255, "GCS ERROR: RECEIVED NaN FOR ALTITUDE");
                    }

                    // Smaller than threshold and not NaN
                    if (pos.v < 1000000 && pos.v == pos.v)
                    {
                        //emit valueChanged(uasId, "speed", "m/s", pos.v, time);
                        //qDebug() << "GOT GPS RAW";
                        // emit speedChanged(this, (double)pos.v, 0.0, 0.0, time);
                    }
                    else
                    {
                        emit textMessageReceived(uasId, message.compid, 255, QString("GCS ERROR: RECEIVED INVALID SPEED OF %1 m/s").arg(pos.v));
                    }
                }
            }
            break;

            case MAVLINK_MSG_ID_RAW_PRESSURE://MESSAGE ISSUED PERIODICALLY
            {
                mavlink_raw_pressure_t pressure;
                mavlink_msg_raw_pressure_decode(&message, &pressure);                

                emit emitMessageRawPressure(uasId, pressure);
            }
            break;

            case MAVLINK_MSG_ID_SCALED_PRESSURE://MESSAGE ISSUED PERIODICALLY
            {
                mavlink_scaled_pressure_t pressure;
                mavlink_msg_scaled_pressure_decode(&message, &pressure);                

                emit emitMessageScaledPressure(uasId, pressure);
            }
            break;

        case MAVLINK_MSG_ID_RC_CHANNELS_RAW://MESSAGE ISSUED PERIODICALLY
            {
                mavlink_rc_channels_raw_t channels;
                mavlink_msg_rc_channels_raw_decode(&message, &channels);

                emit emitMessageChannelsRaw(uasId, channels);
                emit thrustChanged(this, ((channels.chan1_raw-1000)*0.1));
            }
            break;

        case MAVLINK_MSG_ID_PARAM_VALUE://MESSAGE ISSUED GET PARAMETERS TO UAV
            {
                mavlink_param_value_t value;
                mavlink_msg_param_value_decode(&message, &value);
                QByteArray bytes((char*)value.param_id, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
                QString parameterName = QString(bytes);                
                float val = value.param_value;

                emit parameterChanged(uasId, message.compid, parameterName, val);
                emit parameterChanged(uasId, message.compid, value.param_count, value.param_index, parameterName, val);
            }
            break;

        case MAVLINK_MSG_ID_ACTION_ACK:
            mavlink_action_ack_t ack;
            mavlink_msg_action_ack_decode(&message, &ack);
            if (ack.result == 1)
            {
                emit textMessageReceived(uasId, message.compid, 0, tr("SUCCESS: Executed action: %1").arg(ack.action));
            }
            else
            {
                emit textMessageReceived(uasId, message.compid, 0, tr("FAILURE: Rejected action: %1").arg(ack.action));
            }
            break;

        case MAVLINK_MSG_ID_WAYPOINT_COUNT:
            {
                mavlink_waypoint_count_t wpc;
                mavlink_msg_waypoint_count_decode(&message, &wpc);
                if (wpc.target_system == mavlink->getSystemId() && wpc.target_component == mavlink->getComponentId())
                {
                    //waypointManager.handleWaypointCount(message.sysid, message.compid, wpc.count);
                }
            }
            break;

        case MAVLINK_MSG_ID_WAYPOINT:
            {
                mavlink_waypoint_t wp;
                mavlink_msg_waypoint_decode(&message, &wp);

                if(wp.target_system == mavlink->getSystemId() && wp.target_component == mavlink->getComponentId())
                {
                    //waypointManager.handleWaypoint(message.sysid, message.compid, &wp);
                }
            }
            break;

        case MAVLINK_MSG_ID_WAYPOINT_ACK:
            {
                mavlink_waypoint_ack_t wpa;
                mavlink_msg_waypoint_ack_decode(&message, &wpa);

                if(wpa.target_system == mavlink->getSystemId() && wpa.target_component == mavlink->getComponentId())
                {
                    //waypointManager.handleWaypointAck(message.sysid, message.compid, &wpa);
                }
            }
            break;

        case MAVLINK_MSG_ID_WAYPOINT_REQUEST:
            {
                mavlink_waypoint_request_t wpr;
                mavlink_msg_waypoint_request_decode(&message, &wpr);

                if(wpr.target_system == mavlink->getSystemId() && wpr.target_component == mavlink->getComponentId())
                {
                    //waypointManager.handleWaypointRequest(message.sysid, message.compid, &wpr);
                }
            }
            break;

        case MAVLINK_MSG_ID_WAYPOINT_REACHED:
            {
                mavlink_waypoint_reached_t wpr;
                mavlink_msg_waypoint_reached_decode(&message, &wpr);
                //waypointManager.handleWaypointReached(message.sysid, message.compid, &wpr);
                QString text = QString("System %1 reached waypoint %2").arg(getUASName()).arg(wpr.seq);

                emit textMessageReceived(message.sysid, message.compid, 0, text);
            }
            break;

        case MAVLINK_MSG_ID_WAYPOINT_CURRENT:
            {
                mavlink_waypoint_current_t wpc;
                mavlink_msg_waypoint_current_decode(&message, &wpc);
                //waypointManager.handleWaypointCurrent(message.sysid, message.compid, &wpc);
            }
            break;

        case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW://MESSAGE ISSUED PERIODICALLY
            {
                mavlink_servo_output_raw_t servos;
                mavlink_msg_servo_output_raw_decode(&message, &servos);                

                emit emitMessageServoOutputRaw(uasId, servos);
            }
            break;

        case MAVLINK_MSG_ID_STATUSTEXT:
            {
                QByteArray b;
                b.resize(MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN);
                mavlink_msg_statustext_get_text(&message, (int8_t*)b.data());

                QString text = QString(b);
                int severity = mavlink_msg_statustext_get_severity(&message);

                emit textMessageReceived(uasId, message.compid, severity, text);
            }
            break;        

            // Messages to ignore
            case MAVLINK_MSG_ID_LOCAL_POSITION_SETPOINT_SET:            
            case MAVLINK_MSG_ID_DIAGNOSTIC:
            case MAVLINK_MSG_ID_SLUGS_NAVIGATION:
            case MAVLINK_MSG_ID_DATA_LOG:
            case MAVLINK_MSG_ID_GPS_DATE_TIME:
            case MAVLINK_MSG_ID_MID_LVL_CMDS:
            case MAVLINK_MSG_ID_CPU_LOAD:
            case MAVLINK_MSG_ID_SLUGS_ACTION:
            case MAVLINK_MSG_ID_SENSOR_BIAS:
            case MAVLINK_MSG_ID_SENSOR_DIAG:
            case MAVLINK_MSG_ID_NOVATEL_DIAG:
            case MAVLINK_MSG_ID_PTZ_STATUS:
            case MAVLINK_MSG_ID_VOLT_SENSOR:
            case MAVLINK_MSG_ID_ISR_LOCATION:
            break;
        default:
            {
                if (!unknownPackets.contains(message.msgid))
                {
                    unknownPackets.append(message.msgid);
                    QString errString = tr("ERROR AL DECODIFICAR EL MENSAJE NUMERO %1").arg(message.msgid);

                    emit textMessageReceived(uasId, message.compid, 255, errString);                    
                }
            }
            break;
        }
    }
}

void UAS::setHomePosition(double lat, double lon, double alt)
{
    // Send new home position to UAS
    mavlink_gps_set_global_origin_t home;
    home.target_system = uasId;
    home.target_component = 0; // ALL components
    home.latitude = lat*1E7;
    home.longitude = lon*1E7;
    home.altitude = alt*1000;

    mavlink_message_t msg;
    mavlink_msg_gps_set_global_origin_encode(mavlink->getSystemId(), mavlink->getComponentId(), &msg, &home);
    sendMessage(msg);
}

quint64 UAS::getUnixTime(quint64 time)
{
    if (time == 0)
    {
        return MG::TIME::getGroundTimeNow();
    }
    // Check if time is smaller than 40 years,
    // assuming no system without Unix timestamp
    // runs longer than 40 years continuously without
    // reboot. In worst case this will add/subtract the
    // communication delay between GCS and MAV,
    // it will never alter the timestamp in a safety
    // critical way.
    //
    // Calculation:
    // 40 years
    // 365 days
    // 24 hours
    // 60 minutes
    // 60 seconds
    // 1000 milliseconds
    // 1000 microseconds
#ifndef _MSC_VER
    else if (time < 1261440000000000LLU)
#else
        else if (time < 1261440000000000)
#endif
        {
        if (onboardTimeOffset == 0)
        {
            onboardTimeOffset = MG::TIME::getGroundTimeNow() - time/1000;
        }
        return time/1000 + onboardTimeOffset;
    }
    else
    {
        // Time is not zero and larger than 40 years -> has to be
        // a Unix epoch timestamp. Do nothing.
        return time/1000;
    }
}

void UAS::sendMessage(mavlink_message_t message)
{
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

void UAS::sendMessage(LinkInterface* link, mavlink_message_t message)
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
    }
}

QString UAS::getNavModeText(int mode)
{
    switch (mode)
    {
    case MAV_NAV_GROUNDED:
        return QString("GROUNDED");
        break;
    case MAV_NAV_PASSTHROUGH:
        return QString("PASSTHROUGH");
        break;
    case MAV_NAV_LANDING:
        return QString("LANDING");
        break;
    case MAV_NAV_LIFTOFF:
        return QString("LIFTOFF");
        break;
    case MAV_NAV_SEL_PT:
        return QString("SELECT_PT");
        break;
    case MAV_NAV_LOST:
        return QString("LOST");
        break;
    case MAV_NAV_RETURNING:
        return QString("RETURNING");
        break;
    case MAV_NAV_MID_LEVEL:
        return QString("MID_LEVEL");
        break;
    case MAV_NAV_WAYPOINT:
        return QString("WAYPOINT");
        break;
    case MAV_NAV_ISR:
        return QString("ISR");
        break;
    case MAV_NAV_LINE_PATROL:
        return QString("LINE_PATROL");
        break;
    default:
        return QString("UNKNOWN");
    }
}

void UAS::getStatusForCode(int statusCode, QString& uasState, QString& stateDescription)
{
    switch (statusCode)
    {
    case MAV_STATE_UNINIT:
        uasState = tr("UNINIT");
        stateDescription = tr("Unitialized, booting up.");
        break;
    case MAV_STATE_BOOT:
        uasState = tr("BOOT");
        stateDescription = tr("Booting system, please wait.");
        break;
    case MAV_STATE_CALIBRATING:
        uasState = tr("CALIBRATING");
        stateDescription = tr("Calibrating sensors, please wait.");
        break;
    case MAV_STATE_ACTIVE:
        uasState = tr("ACTIVO");
        stateDescription = tr("Active, normal operation.");
        break;
    case MAV_STATE_STANDBY:
        uasState = tr("STANDBY");
        stateDescription = tr("Standby mode, ready for liftoff.");
        break;
    case MAV_STATE_CRITICAL:
        uasState = tr("CRITICAL");
        stateDescription = tr("FAILURE: Continuing operation.");
        break;
    case MAV_STATE_EMERGENCY:
        uasState = tr("EMERGENCY");
        stateDescription = tr("EMERGENCY: Land Immediately!");
        break;
    case MAV_STATE_HILSIM:
        uasState = tr("HIL SIM");
        stateDescription = tr("HIL Simulation, Sensors read from SIM");
        break;

    case MAV_STATE_POWEROFF:
        uasState = tr("SHUTDOWN");
        stateDescription = tr("Powering off system.");
        break;

    case MAV_STATE_RETURNING:
        uasState = tr("RETORNANDO");
        stateDescription = tr("Returning of system.");
        break;

    default:
        uasState = tr("DESCONOCIDO");
        stateDescription = tr("Unknown system state");
        break;
    }
}

quint64 UAS::getUptime() const
{
    if(startTime == 0) {
        return 0;
    } else {
        return MG::TIME::getGroundTimeNow() - startTime;
    }
}

int UAS::getCommunicationStatus() const
{
    return commStatus;
}

void UAS::requestParameters()
{
    mavlink_message_t msg;
    mavlink_msg_param_request_list_pack(mavlink->getSystemId(), mavlink->getComponentId(), &msg, this->getUASID(), 25);
    // Send message twice to increase chance of reception
    sendMessage(msg);
}

void UAS::writeParametersToStorage()
{
    mavlink_message_t msg;
    // TODO Replace MG System ID with static function call and allow to change ID in GUI
    mavlink_msg_action_pack(MG::SYSTEM::ID, MG::SYSTEM::COMPID, &msg, this->getUASID(),MAV_COMP_ID_IMU, (uint8_t)MAV_ACTION_STORAGE_WRITE);
    //mavlink_msg_action_pack(MG::SYSTEM::ID, MG::SYSTEM::COMPID, &msg, this->getUASID(),(uint8_t)MAV_ACTION_STORAGE_WRITE);
    sendMessage(msg);
}

void UAS::readParametersFromStorage()
{
    mavlink_message_t msg;
    // TODO Replace MG System ID with static function call and allow to change ID in GUI
    mavlink_msg_action_pack(MG::SYSTEM::ID, MG::SYSTEM::COMPID, &msg, this->getUASID(), MAV_COMP_ID_IMU,(uint8_t)MAV_ACTION_STORAGE_READ);
    sendMessage(msg);
}

void UAS::setParameter(const int component, const QString& id, const float value)
{
    if (!id.isNull())
    {
        mavlink_message_t msg;
        mavlink_param_set_t p;
        p.param_value = value;
        p.target_system = (uint8_t)uasId;
        p.target_component = (uint8_t)component;

        // Copy string into buffer, ensuring not to exceed the buffer size
        for (unsigned int i = 0; i < sizeof(p.param_id); i++)
        {            
            if ((int)i < id.length() && i < (sizeof(p.param_id) - 1))
            {
                p.param_id[i] = id.toAscii()[i];
            }            
            else
            {
                p.param_id[i] = 0;
            }
        }
        mavlink_msg_param_set_encode(mavlink->getSystemId(), mavlink->getComponentId(), &msg, &p);
        sendMessage(msg);
    }
}

void UAS::requestParameter(int component, int parameter)
{
    mavlink_message_t msg;
    mavlink_param_request_read_t read;
    read.param_index = parameter;
    read.target_system = uasId;
    read.target_component = component;
    mavlink_msg_param_request_read_encode(mavlink->getSystemId(), mavlink->getComponentId(), &msg, &read);
    sendMessage(msg);    
}

void UAS::setSystemType(int systemType)
{
    type = systemType;
    // If the airframe is still generic, change it to a close default type
    if (airframe == 0)
    {
        switch (systemType)
        {
        case MAV_FIXED_WING:
            airframe = QGC_AIRFRAME_EASYSTAR;
            break;
        case MAV_QUADROTOR:
            airframe = QGC_AIRFRAME_MIKROKOPTER;
            break;
        }
    }

    emit systemSpecsChanged(uasId);
}

void UAS::setUASName(const QString& name)
{
    this->name = name;
    writeSettings();
    emit nameChanged(name);
    emit systemSpecsChanged(uasId);
}

int UAS::getSystemType()
{
    return this->type;
}

void UAS::startHil(){

    mavlink_message_t msg;
    // TODO Replace MG System ID with static function call and allow to change ID in GUI
    mavlink_msg_action_pack(MG::SYSTEM::ID, MG::SYSTEM::COMPID, &msg, this->getUASID(), MAV_COMP_ID_IMU,(int)MAV_ACTION_START_HILSIM);
    // Send message twice to increase chance of reception
    sendMessage(msg);
    sendMessage(msg);
}

void UAS::stopHil(){

    mavlink_message_t msg;
    // TODO Replace MG System ID with static function call and allow to change ID in GUI
    mavlink_msg_action_pack(MG::SYSTEM::ID, MG::SYSTEM::COMPID, &msg, this->getUASID(), MAV_COMP_ID_IMU,(int)MAV_ACTION_STOP_HILSIM);
    // Send message twice to increase chance of reception
    sendMessage(msg);
    sendMessage(msg);
}

QString UAS::getUASName(void) const
{
    QString result;
    if (name == "")
    {
        result = tr("VANT ") + result.sprintf("%03d", getUASID());
    }
    else
    {
        result = name;
    }
    return result;
}

void UAS::addLink(LinkInterface* link)
{
    if (!links->contains(link))
    {
        links->append(link);
        connect(link, SIGNAL(destroyed(QObject*)), this, SLOT(removeLink(QObject*)));
    }
}

void UAS::removeLink(QObject* object)
{
    LinkInterface* link = dynamic_cast<LinkInterface*>(object);
    if (link)
    {
        links->removeAt(links->indexOf(link));
    }
}

QList<LinkInterface*>* UAS::getLinks()
{
    return links;
}
