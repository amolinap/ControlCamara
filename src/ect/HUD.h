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
 *   @brief Definition of Head up display
 *
 *   @author Lorenz Meier <mavteam@student.ethz.ch>
 *
 */

#ifndef HUD_H
#define HUD_H

#include <QImage>
#include <QGLWidget>
#include <QPainter>
#include <QFontDatabase>
#include <QTimer>
#include <QShowEvent>
#include <QContextMenuEvent>
#include <QMenu>
#include <QDesktopServices>
#include <QFileDialog>

#include <QDebug>
#include <cmath>
#include <qmath.h>
#include <limits>

#include "UASInterface.h"
//#include "UASWaypointManager.h"
//#include "Geography.h"
#include "SlugsMAV.h"
#include "UASManager.h"
#include "UAS.h"

#include "MG.h"
#include "QGC.h"

/**
 * @brief Displays a Head Up Display (HUD)
 *
 * This class represents a head up display (HUD) and draws this HUD in an OpenGL widget (QGLWidget).
 * It can superimpose the HUD over the current live image stream (any arriving image stream will be auto-
 * matically used as background), or it draws the classic blue-brown background known from instruments.
 */
class HUD : public QGLWidget
{
    Q_OBJECT
public:
    /**
     * @warning The HUD widget will not start painting its content automatically
     *          to update the view, start the auto-update by calling HUD::start().
     *
     * @param width
     * @param height
     * @param parent
     */
    HUD(int width = 640, int height = 480, QWidget* parent = NULL);
    ~HUD();

    /** @brief Resize graphics to new size
      *
      * @param w New width window
      * @param h New height window
    */
    void resizeGL(int w, int h);

public slots:
    /** @brief Initialize the Graphics Library */
    void initializeGL();    
    /** @brief Set the currently monitored UAS
     *
     * @param uas the UAS/MAV to monitor/display with the HUD
     */
    void setActiveUAS(UASInterface* uas);
    /** @brief Receive update attitude UAV on flight
     * @param roll Gyre on axis X
     * @param pitch Gyre on axis Y
     * @param yaw Gyre on axis Z
     * @param usec The UNIX timestamp in milliseconds
     */
    void updateAttitude(UASInterface* uas, double roll, double pitch, double yaw, quint64 timestamp);
    /** @brief Receive update the altitude UAS
      *
      * @param z New altitude in plane z
    */
    void updateAltitude(double z);
    /** @brief Receive update the new position GPS global
      *
      * @param uas  The active UAS
      * @param lat  Latitude GPS
      * @param lon  Longitude GPS
      * @param alt  Altitude GPS
      * @param usec The UNIX timestamp in milliseconds
    */
    void updateGlobalPosition(UASInterface*,double,double,double,quint64);
    /**
      * @brief Receive updating airspeed
      *
      * @param airSpeed Airspeed flight
    */
    void updateSpeed(double airSpeed);
    /** @brief Receive update the status of flight
      *
      * @param uas        The id of active UAS
      * @param status   The status of flight
      * @param description  Description of status flight
    */
    void updateState(UASInterface* uas,QString status,QString description);
    /** @brief Receive update the mode of flight
      *
      * @param sysId        The id of active UAS
      * @param status       The mode flight
      * @param mode  Enum for the mode flight
    */
    void updateMode(int id,QString status, int mode);
    /** @brief Receive update the mode of navigation used in flight
      *
      * @param uasid    The id of active UAS
      * @param mode     The mode navigation
      * @param text     Description of status mode navigation
    */
    void updateModeNavigation(int uasid, int mode, const QString &text);    
    /** @brief Enable the paint HUD instruments
      *
      * @param enabled If enable paint HUD instruments
    */
    void enableHUDInstruments(bool enabled);
    /**
      * @brief Receive updating battery critical
      *
      * @param received Voltage of battery critical
    */
    void setAlertCritical(double value);
    /**
      * @brief Receive updating battery voltage
      *
      * @param id       The id of active UAS
      * @param battery  Voltage of battery
    */
    void setAlertBattery(int id, double battery);

protected slots:
    /**
     * @brief This functions works in the OpenGL view, which is already translated by the x and y center offsets.
     *
     */
    void paintCenterBackground(float roll, float pitch, float yaw);
    /**
     * @param pitch pitch angle in degrees (-180 to 180)
     */
    void paintPitchLines(float pitch, QPainter* painter);
    /**
     * Paint text on top of the image and OpenGL drawings
     *
     * @param text chars to write
     * @param color text color
     * @param fontSize text size in mm
     * @param refX position in reference units (mm of the real instrument). This is relative to the measurement unit position, NOT in pixels.
     * @param refY position in reference units (mm of the real instrument). This is relative to the measurement unit position, NOT in pixels.
     */
    void paintText(QString text, QColor color, float fontSize, float refX, float refY, QPainter* painter);
    /** @brief Setup the OpenGL view for drawing a sub-component of the HUD
     * @param referencePositionX horizontal position in the reference mm-unit space
     * @param referencePositionY horizontal position in the reference mm-unit space
     * @param referenceWidth width in the reference mm-unit space
     * @param referenceHeight width in the reference mm-unit space
     */
    void setupGLView(float referencePositionX, float referencePositionY, float referenceWidth, float referenceHeight);
    void paintHUD();
    void paintPitchLinePos(QString text, float refPosX, float refPosY, QPainter* painter);
    void paintPitchLineNeg(QString text, float refPosX, float refPosY, QPainter* painter);
    void drawLine(float refX1, float refY1, float refX2, float refY2, float width, const QColor& color, QPainter* painter);
    void drawEllipse(float refX, float refY, float radiusX, float radiusY, float startDeg, float endDeg, float lineWidth, const QColor& color, QPainter* painter);
    void drawCircle(float refX, float refY, float radius, float startDeg, float endDeg, float lineWidth, const QColor& color, QPainter* painter);
    void drawChangeIndicatorGauge(float xRef, float yRef, float radius, float expectedMaxChange, float value, const QColor& color, QPainter* painter, bool solid=true);
    void drawPolygon(QPolygonF refPolygon, QPainter* painter);

private:
    /**
     * @param y coordinate in pixels to be converted to reference mm units
     * @return the screen coordinate relative to the QGLWindow origin
     */
    float refToScreenX(float x);
    /**
     * @param x coordinate in pixels to be converted to reference mm units
     * @return the screen coordinate relative to the QGLWindow origin
     */
    float refToScreenY(float y);
    /** @brief Convert mm line widths to QPen line widths */
    float refLineWidthToPen(float line);
    /** @brief Rotate a polygon around a point clockwise
     * Rotate a polygon around a point
     *
     * @param p polygon to rotate
     * @param origin the rotation center
     * @param angle rotation angle, in radians
     * @return p Polygon p rotated by angle around the origin point
     */
    void rotatePolygonClockWiseRad(QPolygonF& p, float angle, QPointF origin);
    /** @brief Preferred Size */
    QSize sizeHint() const;
    /** @brief Start updating widget */
    void showEvent(QShowEvent* event);
    /** @brief Stop updating widget */
    void hideEvent(QHideEvent* event);

    static const int updateInterval = 40;

    UASInterface* uas; ///< The uas currently monitored
    float yawInt; ///< The yaw integral. Used to damp the yaw indication.
    QString mode; ///< The current vehicle mode
    QString modeNavigation; ///< The current vehicle mode
    QString state; ///< The current vehicle state    
    double scalingFactor; ///< Factor used to scale all absolute values to screen coordinates
    float xCenterOffset, yCenterOffset; ///< Offset from center of window in mm coordinates
    float vwidth; ///< Virtual width of this window, 200 mm per default. This allows to hardcode positions and aspect ratios. This virtual image plane is then scaled to the window size.
    float vheight; ///< Virtual height of this window, 150 mm per default
    float vGaugeSpacing; ///< Virtual spacing of the gauges from the center, 50 mm per default
    float vPitchPerDeg; ///< Virtual pitch to mm conversion. Currently one degree is 3 mm up/down in the pitch markings
    int xCenter; ///< Center of the HUD instrument in pixel coordinates. Allows to off-center the whole instrument in its OpenGL window, e.g. to fit another instrument
    int yCenter; ///< Center of the HUD instrument in pixel coordinates. Allows to off-center the whole instrument in its OpenGL window, e.g. to fit another instrument
    // HUD colors
    QColor defaultColor;       ///< Color for most HUD elements, e.g. pitch lines, center cross, change rate gauges
    QColor infoColor;          ///< Color for normal/default messages
    QTimer* refreshTimer;      ///< The main timer, controls the update rate    
    QFont font;                ///< The HUD font, per default the free Bitstream Vera SANS, which is very close to actual HUD fonts
    float roll;
    float pitch;
    float yaw;
    float rollLP;
    float pitchLP;
    float yawLP;
    double yawDiff;
    double zPos;
    double xSpeed;
    double zSpeed;
    double airSpeed;
    double lat;
    double lon;
    double battery;
    double critical;
    //float load;
    bool hudInstrumentsEnabled;
    void paintEvent(QPaintEvent *event);
};

#endif // HUD_H
