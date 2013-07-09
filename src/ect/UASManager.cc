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
 *   @brief Implementation of class UASManager
 *   @author Lorenz Meier <mavteam@student.ethz.ch>
 *
 */

#include <QList>
#include <QApplication>
#include <QMessageBox>
#include <QTimer>
#include <QSettings>
#include "SlugsMAV.h"
//#include "UASInterface.h"
#include "UASManager.h"
#include "QGC.h"

UASManager* UASManager::instance()
{
    static UASManager* _instance = 0;
    if(_instance == 0) {
        _instance = new UASManager();

        // Set the application as parent to ensure that this object
        // will be destroyed when the main application exits
        _instance->setParent(qApp);
    }
    return _instance;
}

void UASManager::storeSettings()
{
    QSettings settings;
    settings.beginGroup("QGC_UASMANAGER");
    settings.setValue("HOMELAT", homeLat);
    settings.setValue("HOMELON", homeLon);
    settings.setValue("HOMEALT", homeAlt);
    settings.endGroup();
    settings.sync();
}

void UASManager::loadSettings()
{
    QSettings settings;
    settings.sync();
    settings.beginGroup("QGC_UASMANAGER");
//    setHomePosition(settings.value("HOMELAT", homeLat).toDouble(),
//                    settings.value("HOMELON", homeLon).toDouble(),
//                    settings.value("HOMEALT", homeAlt).toDouble());
    settings.endGroup();
}

void UASManager::setHomePosition(double lat, double lon, double alt)
{
    // Checking for NaN and infitiny
//    if (lat == lat && lon == lon && alt == alt && !std::isinf(lat) && !std::isinf(lon) && !std::isinf(alt))
//    {
//        bool changed = false;
//        if (homeLat != lat) changed = true;
//        if (homeLon != lon) changed = true;
//        if (homeAlt != alt) changed = true;

//        homeLat = lat;
//        homeLon = lon;
//        homeAlt = alt;

//        if (changed) emit homePositionChanged(homeLat, homeLon, homeAlt);
//    }
}

/**
 * @brief Private singleton constructor
 *
 * This class implements the singleton design pattern and has therefore only a private constructor.
 **/
UASManager::UASManager() :
        activeUAS(NULL),
        homeLat(47.3769),
        homeLon(8.549444),
        homeAlt(470.0)
{
    start(QThread::LowPriority);
    loadSettings();
}

UASManager::~UASManager()
{
    storeSettings();

    foreach (SlugsMAV* mav, systems)
    {
        delete mav;
    }
}


void UASManager::run()
{
//    forever
//    {
//        QGC::SLEEP::msleep(5000);
//    }
    exec();
}

void UASManager::addUAS(SlugsMAV* uas)
{
    // WARNING: The active uas is set here
    // and then announced below. This is necessary
    // to make sure the getActiveUAS() function
    // returns the UAS once the UASCreated() signal
    // is emitted. The code is thus NOT redundant.
    bool firstUAS = false;
    if (activeUAS == NULL)
    {
        firstUAS = true;
        activeUAS = uas;
    }

    // Only execute if there is no UAS at this index
    if (!systems.contains(uas))
    {
        systems.append(uas);
        connect(uas, SIGNAL(destroyed(QObject*)), this, SLOT(removeUAS(QObject*)));
        //connect(this, SIGNAL(homePositionChanged(double,double,double)), uas, SLOT(setHomePosition(double,double,double)));
        emit UASCreated(uas);
    }

    // If there is no active UAS yet, set the first one as the active UAS
    if (firstUAS)
    {
        setActiveUAS(uas);
    }
}

void UASManager::removeUAS(QObject* uas)
{
    SlugsMAV* mav = qobject_cast<SlugsMAV*>(uas);

    if (mav)
    {
        int listindex = systems.indexOf(mav);

        if (mav == activeUAS)
        {
            if (systems.count() > 1)
            {
                // We only set a new UAS if more than one is present
                if (listindex != 0)
                {
                    // The system to be removed is not at position 1
                    // set position one as new active system
                    setActiveUAS(systems.first());
                }
                else
                {
                    // The system to be removed is at position 1,
                    // select the next system
                    setActiveUAS(systems.at(1));
                }
            }
            else
            {
                // TODO send a null pointer if no UAS is present any more
                // This has to be proberly tested however, since it might
                // crash code parts not handling null pointers correctly.
            }
        }
        systems.removeAt(listindex);
    }
}

QList<SlugsMAV*> UASManager::getUASList()
{
    return systems;
}

SlugsMAV* UASManager::getActiveUAS()
{
    return activeUAS; ///< Return zero pointer if no UAS has been loaded
}

SlugsMAV* UASManager::getUASForId(int id)
{
    SlugsMAV* system = NULL;

    foreach(SlugsMAV* sys, systems)
    {
        if (sys->getUASID() == id)
        {
            system = sys;
        }
    }

    return system;
}

void UASManager::setActiveUAS(SlugsMAV* uas)
{
    if (uas != NULL)
    {
//        activeUASMutex.lock();
//        if (activeUAS != NULL)
//        {
//            emit activeUASStatusChanged(activeUAS, false);
//            emit activeUASStatusChanged(activeUAS->getUASID(), false);
//        }
        activeUAS = uas;
//        activeUASMutex.unlock();

        activeUAS->setSelected();
        emit activeUASSet(uas);
//        emit activeUASSet(uas->getUASID());
//        emit activeUASSetListIndex(systems.indexOf(uas));
//        emit activeUASStatusChanged(uas, true);
//        emit activeUASStatusChanged(uas->getUASID(), true);
    }
}
