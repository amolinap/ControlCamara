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
 *   @brief Definition of Unmanned Aerial Vehicle object
 *
 *   @author Lorenz Meier <mavteam@student.ethz.ch>
 *
 */

#ifndef _UAS_H_
#define _UAS_H_

#include "UASInterface.h"
#include "MG.h"
#include "MAVLinkProtocol.h"
#include "QGCMAVLink.h"
//#include "Coordinate.h"
//#include "Geography.h"

/**
 * @brief A generic MAVLINK-connected MAV/UAV
 *
 * This class represents one vehicle. It can be used like the real vehicle, e.g. a call to halt()
 * will automatically send the appropriate messages to the vehicle. The vehicle state will also be
 * automatically updated by the comm architecture, so when writing code to e.g. control the vehicle
 * no knowledge of the communication infrastructure is needed.
 */
class UAS : public UASInterface
{
    friend class UASWaypointManager;
    Q_OBJECT
public:
    /** @brief This is the class constructor.
    *
    * @param protocol       The MAVLinkProtocol
    * @param id    The UAS ID
    **/
    UAS(MAVLinkProtocol* protocol, int id = 0);
    ~UAS();

    /** @brief Get the name of the UAV
    *
    * @return Name of UAV
    **/
    QString getUASName(void) const;
    /** @brief Get the ID of the connected UAV
    *
    * @return ID of UAV
    **/
    int getUASID() const;
    /** @brief Get the airframe of MAV
    *
    * @return Airframe of UAV
    **/
    int getAirframe() const { return airframe; }
    /** @brief The time interval the robot is switched on
    *
    * @return Time interval UAV
    **/
    quint64 getUptime() const;
    /** @brief Get the status flag for the communication
    *
    * @return Status of the communication
    **/
    int getCommunicationStatus() const;
    /** @brief Get the links associated with this robot
    *
    * @return List with all links this robot is associated with. Association is usually
    *         based on the fact that a message for this robot has been received through that
    *         interface. The LinkInterface can support multiple protocols.
    **/
    QList<LinkInterface*>* getLinks();
    /** @brief Get the local position on axis X
    *
    * @return Position on axis X
    **/
    double getLocalX() const { return localX; }
    /** @brief Get the local position on axis Y
    *
    * @return Position on axis Y
    **/
    double getLocalY() const { return localY; }
    /** @brief Get the local position on axis Z
    *
    * @return Position on axis Z
    **/
    double getLocalZ() const { return localZ; }
    /** @brief Get the latitude of GPS
    *
    * @return Latitude of GPS
    **/
    double getLatitude() const { return latitude; }
    /** @brief Get the longitude of GPS
    *
    * @return Longitude of GPS
    **/
    double getLongitude() const { return longitude; }
    /** @brief Get the altitude of GPS
    *
    * @return Altitude of GPS
    **/
    double getAltitude() const { return altitude; }
    /** @brief Get the roll of UAV on flight
    *
    * @return Roll of UAV
    **/
    double getRoll() const { return roll; }
    /** @brief Get the pitch of UAV on flight
    *
    * @return Pitch of UAV
    **/
    double getPitch() const { return pitch; }
    /** @brief Get the yaw of UAV on flight
    *
    * @return Yaw of UAV
    **/
    double getYaw() const { return yaw; }
    /** @brief Get if UAV is current selected
    *
    * @return UAV is selected
    **/
    bool getSelected() const;
    /** @brief Get the UAS status
    *
    * @param statusCode Item of enum MAV_STATE
    * @param uasState Status of navigation UAV
    * @param stateDescription Description for status UAV
    */
    void getStatusForCode(int statusCode, QString& uasState, QString& stateDescription);
    /** @brief Get navigation mode
    *
    * @param mode Item of enum MAV_NAV
    */
    QString getNavModeText(int mode);
    /** @brief Check if vehicle is in autonomous mode */
    bool isAuto();

    //UASWaypointManager* getWaypointManager() { return &waypointManager; }
    /** @brief Get reference to the param manager **/
    //QGCUASParamManager* getParamManager() const { return paramManager; }
    /** @brief Set reference to the param manager **/
    //void setParamManager(QGCUASParamManager* manager) { paramManager = manager; }
    int getSystemType();
    int getAutopilotType() {return autopilot;}
    uint64_t usec;
    void resetUpTime() {startTime=QGC::groundTimeMilliseconds();}

protected:
    int uasId;                    ///< Unique system ID
    unsigned char type;           ///< UAS type (from type enum)
    quint64 startTime;            ///< The time the UAS was switched on
    CommStatus commStatus;        ///< Communication status
    QString name;                 ///< Human-friendly name of the vehicle, e.g. bravo
    int autopilot;                ///< Type of the Autopilot: -1: None, 0: Generic, 1: PIXHAWK, 2: SLUGS, 3: Ardupilot (up to 15 types), defined in MAV_AUTOPILOT_TYPE ENUM
    QList<LinkInterface*>* links; ///< List of links this UAS can be reached by
    QList<int> unknownPackets;    ///< Packet IDs which are unknown and have been received
    MAVLinkProtocol* mavlink;     ///< Reference to the MAVLink instance
    //UASWaypointManager waypointManager;
    int mode;                   ///< The current mode of the MAV
    int status;                 ///< The current status of the MAV
    int navMode;                ///< The current navigation mode of the MAV
    quint64 onboardTimeOffset;
    float receiveDropRate;      ///< Percentage of packets that were dropped on the MAV's receiving link (from GCS and other MAVs)
    float sendDropRate;         ///< Percentage of packets that were not received from the MAV by the GCS
    bool positionLock;          ///< Status if position information is available or not
    double localX;
    double localY;
    double localZ;
    double latitude;
    double longitude;
    double altitude;
    double speedX;              ///< True speed in X axis
    double speedY;              ///< True speed in Y axis
    double speedZ;              ///< True speed in Z axis
    double roll;
    double pitch;
    double yaw;
    quint64 lastHeartbeat;      ///< Time of the last heartbeat message
    QTimer* statusTimeout;      ///< Timer for various status timeouts
    bool paramsOnceRequested;   ///< If the parameter list has been read at least once
    int airframe;               ///< The airframe type    
    //QGCUASParamManager* paramManager; ///< Parameter manager class
    /** @brief Get the UNIX timestamp in milliseconds */
    quint64 getUnixTime(quint64 time=0);

public slots:
    /** @brief Set the autopilot type */
    void setAutopilotType(int apType) { autopilot = apType; emit systemSpecsChanged(uasId); }
    /** @brief Set the type of airframe */
    void setSystemType(int systemType);
    /** @brief Set the specific airframe type */
    void setAirframe(int airframe) { this->airframe = airframe; emit systemSpecsChanged(uasId); }
    /** @brief Set a new name **/
    void setUASName(const QString& name);
    /** @brief Places the UAV in Hardware-in-the-Loop simulation status **/
    void startHil();
    /** @brief Stops the UAV's Hardware-in-the-Loop simulation status **/
    void stopHil();
    /** @brief Add a link associated with this robot */
    void addLink(LinkInterface* link);
    /** @brief Remove a link associated with this robot */
    void removeLink(QObject* object);
    /** @brief Receive a message from one of the communication links. */
    virtual void receiveMessage(LinkInterface* link, mavlink_message_t message);
    /** @brief Send a message over this link (to this or to all UAS on this link) */
    void sendMessage(LinkInterface* link, mavlink_message_t message);
    /** @brief Send a message over all links this UAS can be reached with (!= all links) */
    void sendMessage(mavlink_message_t message);
    /** @brief Set this UAS as the system currently in focus, e.g. in the main display widgets */
    void setSelected();
    /** @brief Request all parameters */
    void requestParameters();
    /** @brief Request a single parameter by index */
    void requestParameter(int component, int parameter);
    /** @brief Set a system parameter
    *
    * @param component ID of the system component to write the parameter to
    * @param id String identifying the parameter
    * @param value Value of the parameter, IEEE 754 single precision floating point
    */
    void setParameter(const int component, const QString& id, const float value);
    /** @brief Write parameters to permanent storage */
    void writeParametersToStorage();
    /** @brief Read parameters from permanent storage */
    void readParametersFromStorage();
    /** @brief Update the system state */
    void updateState();
    /** @brief Set world frame origin / home position at this GPS position */
    void setHomePosition(double lat, double lon, double alt);

signals:
    /** @brief The system load (MCU/CPU usage) changed */
    void loadChanged(UASInterface* uas, double load);
    /** @brief Propagate a heartbeat received from the system */
    void heartbeat(UASInterface* uas);
    /** @brief Propagate signal for save data telemetry */
    void saveData();
    /** @brief Emit pack a scaled pressure message */


protected slots:
    /** @brief Write settings to disk */
    void writeSettings();
    /** @brief Read settings from disk */
    void readSettings();    
};


#endif // _UAS_H_
