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
 *   @brief Abstract interface, represents one unmanned aerial vehicle
 *
 *   @author Lorenz Meier <mavteam@student.ethz.ch>
 *
 */

#ifndef _UASINTERFACE_H_
#define _UASINTERFACE_H_

#include <QObject>
#include <QList>
#include <QAction>
#include <QColor>
#include <QPointer>
#include "LinkInterface.h"
#include "ProtocolInterface.h"
//#include "UASWaypointManager.h"
//#include "QGCUASParamManager.h"

/**
 * @brief Interface for all robots.
 *
 * This interface is abstract and thus cannot be instantiated. It serves only as type definition.
 * It represents an unmanned aerial vehicle, e.g. a micro air vehicle.
 **/
class UASInterface : public QObject
{
    Q_OBJECT
public:
    virtual ~UASInterface() {}
    /** @brief Enum for status communication */
    enum CommStatus
    {
        COMM_DISCONNECTED = 0,/**< Unit is disconnected, no failure state reached so far */
        COMM_CONNECTING = 1,/** The communication is established **/
        COMM_CONNECTED = 2,/** The communication link is up **/
        COMM_DISCONNECTING = 3,/** The connection is closed **/
        COMM_FAIL = 4,
        COMM_TIMEDOUT = 5///< Communication link failed
    };

    enum Airframe {
        QGC_AIRFRAME_GENERIC = 0,
        QGC_AIRFRAME_EASYSTAR,
        QGC_AIRFRAME_TWINSTAR,
        QGC_AIRFRAME_MERLIN,
        QGC_AIRFRAME_CHEETAH,
        QGC_AIRFRAME_MIKROKOPTER,
        QGC_AIRFRAME_REAPER,
        QGC_AIRFRAME_PREDATOR,
        QGC_AIRFRAME_COAXIAL,
        QGC_AIRFRAME_PTERYX
    };

    /** @brief Get the name of the UAV
    *
    * @return Name of UAV
    **/
    virtual QString getUASName() const = 0;    
    /** @brief Get the ID of the connected UAV
    *
    * @return ID of UAV
    **/
    virtual int getUASID() const = 0;
    /** @brief The time interval the robot is switched on
    *
    * @return Time interval UAV
    **/
    virtual quint64 getUptime() const = 0;
    /** @brief Get the status flag for the communication
    *
    * @return Status of the communication
    **/
    virtual int getCommunicationStatus() const = 0;   
    /** @brief Get the local position on axis X
    *
    * @return Position on axis X
    **/
    virtual double getLocalX() const = 0;
    /** @brief Get the local position on axis Y
    *
    * @return Position on axis Y
    **/
    virtual double getLocalY() const = 0;
    /** @brief Get the local position on axis Z
    *
    * @return Position on axis Z
    **/
    virtual double getLocalZ() const = 0;
    /** @brief Get the latitude of GPS
    *
    * @return Latitude of GPS
    **/
    virtual double getLatitude() const = 0;
    /** @brief Get the longitude of GPS
    *
    * @return Longitude of GPS
    **/
    virtual double getLongitude() const = 0;
    /** @brief Get the altitude of GPS
    *
    * @return Altitude of GPS
    **/
    virtual double getAltitude() const = 0;
    /** @brief Get the roll of UAV on flight
    *
    * @return Roll of UAV
    **/
    virtual double getRoll() const = 0;
    /** @brief Get the pitch of UAV on flight
    *
    * @return Pitch of UAV
    **/
    virtual double getPitch() const = 0;
    /** @brief Get the yaw of UAV on flight
    *
    * @return Yaw of UAV
    **/
    virtual double getYaw() const = 0;
    /** @brief Get if UAV is current selected
    *
    * @return UAV is selected
    **/
    virtual bool getSelected() const = 0;
    /** @brief Get the airframe of MAV
    *
    * @return Airframe of UAV
    **/
    virtual int getAirframe() const = 0;
    /** @brief Get reference to the waypoint manager
    *
    * @return Waypoint manager pointer
    **/
    //virtual UASWaypointManager* getWaypointManager(void) = 0;
    /** @brief Get reference to the parameters manager
    *
    * @return Parameters manager pointer
    **/
    //virtual QGCUASParamManager* getParamManager() const = 0;
    /** @brief Set reference to the param manager **/
    //virtual void setParamManager(QGCUASParamManager* manager) = 0;
    /** @brief Get the links associated with this robot
    *
    * @return List with all links this robot is associated with. Association is usually
    *         based on the fact that a message for this robot has been received through that
    *         interface. The LinkInterface can support multiple protocols.
    **/
    virtual QList<LinkInterface*>* getLinks() = 0;
    /**
    * @brief Get the color for this UAS
    *
    * This static function holds a color map that allows to draw a new color for each robot
    *
    * @return The next color in the color map. The map holds 20 colors and starts from the beginning
    *         if the colors are exceeded.
    */
    static QColor getNextColor()
    {        
        static QList<QColor> colors = QList<QColor>();
        static int nextColor = -1;

        if (nextColor == -1)
        {
            colors.append(QColor(231,72,28));
            colors.append(QColor(104,64,240));
            colors.append(QColor(203,254,121));
            colors.append(QColor(161,252,116));
            colors.append(QColor(232,33,47));
            colors.append(QColor(116,251,110));
            colors.append(QColor(234,38,107));
            colors.append(QColor(104,250,138));
            colors.append(QColor(235,43,165));
            colors.append(QColor(98,248,176));
            colors.append(QColor(236,48,221));
            colors.append(QColor(92,247,217));
            colors.append(QColor(200,54,238));
            colors.append(QColor(87,231,246));
            colors.append(QColor(151,59,239));
            colors.append(QColor(81,183,244));
            colors.append(QColor(75,133,243));
            colors.append(QColor(242,255,128));
            colors.append(QColor(230,126,23));
            nextColor = 0;
        }
        return colors[nextColor++];
    }

    /** @brief Get the type of the system (airplane, quadrotor, helicopter,..)*/
    virtual int getSystemType() = 0;
    QColor getColor()
    {
        return color;
    }
    virtual int getAutopilotType() = 0;
    virtual void setAutopilotType(int apType)= 0;

public slots:
    /** @brief Set a new name for the system */
    virtual void setUASName(const QString& name) = 0;
    /** @brief Selects the airframe */
    virtual void setAirframe(int airframe) = 0;
    /** @brief Set world frame origin / home position at this GPS position */
    virtual void setHomePosition(double lat, double lon, double alt) = 0;
    /** @brief Request all onboard parameters of all components */
    virtual void requestParameters() = 0;
    /** @brief Request one specific onboard parameter */
    virtual void requestParameter(int component, int parameter) = 0;
    /** @brief Write parameter to permanent storage */
    virtual void writeParametersToStorage() = 0;
    /** @brief Read parameter from permanent storage */
    virtual void readParametersFromStorage() = 0;
    /** @brief Set a system parameter
    *
    * @param component ID of the system component to write the parameter to
    * @param id String identifying the parameter
    * @param value Value of the parameter, IEEE 754 single precision floating point
    */
    virtual void setParameter(const int component, const QString& id, const float value) = 0;
    /**
    * @brief Add a link to the list of current links
    *
    * Adding the link to the robot's internal link list will allow him so send its own messages
    * over all registered links. Usually a link is added once a message for this particular robot
    * has been received over a link, but adding could also be done manually.
    * @warning Not all links should be added to all robots by default, because else all robots will
    *          attempt to send over all links, typically choking wireless serial links.
    */
    virtual void addLink(LinkInterface* link) = 0;
    /**
    * @brief Set the current robot as focused in the user interface
    */
    virtual void setSelected() = 0;

protected:
    QColor color;

signals:
    /** @brief The robot state has changed **/
    void statusChanged(int stateFlag);
    /** @brief The robot state has changed
     *
     * @param uas this robot
     * @param status short description of status, e.g. "connected"
     * @param description longer textual description. Should be however limited to a short text, e.g. 200 chars.
     */
    void statusChanged(UASInterface* uas, QString status, QString description);
    /** @brief System has been removed / disconnected / shutdown cleanly, remove */
    void systemRemoved(UASInterface* uas);
    void systemRemoved();
    /** @brief A text message from the system has been received */
    void textMessageReceived(int uasid, int componentid, int severity, QString text);
    void navModeChanged(int uasid, int mode, const QString& text);
    /**
     * @brief Drop rate of communication link updated
     *
     * @param systemId id of the air system
     * @param receiveDrop drop rate of packets this MAV receives (sent from GCS or other MAVs)
     */
    void dropRateChanged(int systemId,  float receiveDrop);
    /** @brief Robot mode has changed */
    void modeChanged(int sysId, QString status, int mode);
    /** @brief A value of the robot has changed.
      *
      * Typically this is used to send lowlevel information like the battery voltage to the plotting facilities of
      * the groundstation
      *
      * @param uasId ID of this system
      * @param name name of the value, e.g. "battery voltage"
      * @param value the value that changed
      * @param msec the timestamp of the message, in milliseconds
      */
    void valueChangedPlot(const int uasId, const QString& name, const QString& unit, const double value, const quint64 msec);
    void valueChangedPlot(const int uasId, const QString& name, const QString& unit, const quint64 value, const quint64 msec);
    void valueChangedPlot(const int uasId, const QString& name, const QString& unit, const int value, const quint64 msec);
    void emitSignalGPS(bool);
    /** @brief The main/battery voltage has changed/was updated */
    void voltageAvionics(int uasId, double voltage);
    void voltageCritical(double battery);
    void statusPTZ(int systemId, bool outPTZ, double pan, double tilt);
    void parameterChanged(int uas, int component, QString parameterName, float value);
    void parameterChanged(int uas, int component, int parameterCount, int parameterId, QString parameterName, float value);
    void statusChanged(UASInterface* uas, QString status);    
    void thrustChanged(UASInterface*, double thrust);
    void heartbeat(UASInterface* uas);
    void attitudeChanged(UASInterface*, double roll, double pitch, double yaw, quint64 usec);
    void globalPositionChanged(UASInterface*, double lat, double lon, double alt, quint64 usec);
    /** @brief Heartbeat timed out */
    void heartbeatTimeout();
    /** @brief Heartbeat timed out */
    void heartbeatTimeout(unsigned int ms);
    /** @brief Name of system changed */
    void nameChanged(QString newName);
    /** @brief Core specifications have changed */
    void systemSpecsChanged(int uasId);    
    void emitValueAltitude(double altitude);
    void emitValueAirSpeed(double airSpeed);

protected:    
    static const unsigned int timeoutIntervalHeartbeat = 2000 * 1000; ///< Heartbeat timeout is 1.5 seconds
};

Q_DECLARE_INTERFACE(UASInterface, "org.qgroundcontrol/1.0");

#endif // _UASINTERFACE_H_
