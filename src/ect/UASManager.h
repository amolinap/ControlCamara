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
 *   @brief Definition of class UASManager
 *   @author Lorenz Meier <mavteam@student.ethz.ch>
 *
 */

#ifndef _UASMANAGER_H_
#define _UASMANAGER_H_

#include <QThread>
#include <QList>
#include <QMutex>
#include "SlugsMAV.h"

/**
 * @brief Central manager for all connected aerial vehicles
 *
 * This class keeps a list of all connected / configured UASs. It also stores which
 * UAS is currently select with respect to user input or manual controls.
 **/
class UASManager : public QThread
{
    Q_OBJECT

public:
    static UASManager* instance();
    ~UASManager();

    void run();
    /**
     * @brief Get the currently selected UAS
     *
     * @return NULL pointer if no UAS exists, active UAS else
     **/
    SlugsMAV* getActiveUAS();
    /**
     * @brief Get the UAS with this id
     *
     * Although not enforced by this implementation, the IDs are constrained to be
     * in the range of 1 - 127 by the MAVLINK protocol.
     *
     * @param id unique system / aircraft id
     * @return UAS with the given ID, NULL pointer else
     **/
    SlugsMAV* getUASForId(int id);
    QList<SlugsMAV*> getUASList();
    /** @brief Get home position latitude */
    double getHomeLatitude() const { return homeLat; }
    /** @brief Get home position longitude */
    double getHomeLongitude() const { return homeLon; }
    /** @brief Get home position altitude */
    double getHomeAltitude() const { return homeAlt; }


public slots:
    /**
     * @brief Add a new UAS to the list
     *
     * This command will only be executed if this UAS does not yet exist.
     * @param UAS unmanned air system to add
     **/
    void addUAS(SlugsMAV* UAS);

    /** @brief Remove a system from the list */
    void removeUAS(QObject* uas);
    /**
      * @brief Set a UAS as currently selected
      *
      * @param UAS Unmanned Air System to set
      **/
    void setActiveUAS(SlugsMAV* UAS);
    /** @brief Load settings */
    void loadSettings();
    /** @brief Store settings */
    void storeSettings();

protected:
    UASManager();
    QList<SlugsMAV*> systems;
    SlugsMAV* activeUAS;
    QMutex activeUASMutex;
    double homeLat;
    double homeLon;
    double homeAlt;

signals:
    void UASCreated(SlugsMAV* UAS);
    /** @brief The UAS currently under main operator control changed */
    void activeUASSet(SlugsMAV* UAS);
    /** @brief The UAS currently under main operator control changed */
    void activeUASSet(int systemId);
    /** @brief The UAS currently under main operator control changed */
    void activeUASSetListIndex(int listIndex);
    /** @brief The UAS currently under main operator control changed */
    //void activeUASStatusChanged(UASInterface* UAS, bool active);
    /** @brief The UAS currently under main operator control changed */
    void activeUASStatusChanged(int systemId, bool active);
    /** @brief Current home position changed */
    void homePositionChanged(double lat, double lon, double alt);
};

#endif // _UASMANAGER_H_
