/*===================================================================
======================================================================*/

/**
 * @file
 *   @brief Head Up Display (HUD)
 *
 *   @author Lorenz Meier <mavteam@student.ethz.ch>
 *
 */

#include "HUD.h"

// Fix for some platforms, e.g. windows
#ifndef GL_MULTISAMPLE
#define GL_MULTISAMPLE  0x809D
#endif

template<typename T>
inline bool isnan(T value)
{
    return value != value;

}

// requires #include <limits>
template<typename T>
inline bool isinf(T value)
{
    return std::numeric_limits<T>::has_infinity && (value == std::numeric_limits<T>::infinity() || (-1*value) == std::numeric_limits<T>::infinity());
}

HUD::HUD(int width, int height, QWidget* parent)
    : QGLWidget(QGLFormat(QGL::SampleBuffers), parent),
    uas(NULL),
    yawInt(0.0f),
    mode(tr("UNKNOWN MODE")),
    modeNavigation(tr("UNKNOWN NAVIGATION")),
    state(tr("UNKNOWN STATE")),    
    xCenterOffset(0.0f),
    yCenterOffset(0.0f),
    vwidth(200.0f),
    vheight(150.0f),
    vGaugeSpacing(60.0f),
    vPitchPerDeg(6.0f), ///< 4 mm y translation per degree)
    defaultColor(QColor(70, 200, 70)),
    infoColor(QColor(20, 200, 20)),
    refreshTimer(new QTimer(this)),
    roll(0.0f),
    pitch(0.0f),
    yaw(0.0f),
    rollLP(0.0f),
    pitchLP(0.0f),
    yawLP(0.0f),
    yawDiff(0.0f),
    zPos(0.0f),
    xSpeed(0.0),
    zSpeed(0.0),
    lat(0.0),
    lon(0.0),
    battery(0.0f),
    critical(0.0f),
    hudInstrumentsEnabled(true)    
{    
    setAutoFillBackground(false);
    setMinimumSize(80, 60);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    QImage fill = QImage(width, height, QImage::Format_Indexed8);
    fill.setNumColors(3);
    fill.setColor(0, qRgb(0, 0, 0));
    fill.setColor(1, qRgb(0, 0, 0));
    fill.setColor(2, qRgb(0, 0, 0));
    fill.fill(0);

    refreshTimer->setInterval(updateInterval);
    connect(refreshTimer, SIGNAL(timeout()), this, SLOT(paintHUD()));
    QFontDatabase fontDatabase = QFontDatabase();
    const QString fontFileName = ":/general/vera.ttf"; ///< Font file is part of the QRC file and compiled into the app
    const QString fontFamilyName = "Bitstream Vera Sans";
    if(!QFile::exists(fontFileName)) qDebug() << "ERROR! font file: " << fontFileName << " DOES NOT EXIST!";

    fontDatabase.addApplicationFont(fontFileName);
    font = fontDatabase.font(fontFamilyName, "Roman", qMax(5,(int)(10.0f*scalingFactor*1.2f+0.5f)));
    QFont* fontPtr = &font;
    if (!fontPtr)
    {
        qDebug() << "ERROR! FONT NOT LOADED!";
    }
    else
    {
        if (font.family() != fontFamilyName) qDebug() << "ERROR! WRONG FONT LOADED: " << fontFamilyName;
    }

    connect(UASManager::instance(), SIGNAL(activeUASSet(UASInterface*)), this, SLOT(setActiveUAS(UASInterface*)));

    if (UASManager::instance()->getActiveUAS() != NULL) setActiveUAS(UASManager::instance()->getActiveUAS());

    setVisible(false);    
}

HUD::~HUD()
{
    refreshTimer->stop();
}

QSize HUD::sizeHint() const
{
    return QSize(width(), (width()*3.0f)/4);
}

void HUD::showEvent(QShowEvent* event)
{
    // React only to internal (pre-display)
    // events
    Q_UNUSED(event)
    refreshTimer->start(updateInterval);
}

void HUD::hideEvent(QHideEvent* event)
{
    // React only to internal (pre-display)
    // events
    Q_UNUSED(event);
    refreshTimer->stop();
}

void HUD::setActiveUAS(UASInterface* uas)
{
    if (this->uas != NULL)
    {
        disconnect(this->uas, SIGNAL(attitudeChanged(UASInterface*,double,double,double,quint64)), this, SLOT(updateAttitude(UASInterface*, double, double, double, quint64)));        
        disconnect(this->uas, SIGNAL(statusChanged(UASInterface*,QString,QString)), this, SLOT(updateState(UASInterface*,QString,QString)));
        disconnect(this->uas, SIGNAL(modeChanged(int,QString,int)), this, SLOT(updateMode(int,QString,int)));
        disconnect(this->uas, SIGNAL(navModeChanged(int,int,QString)), this, SLOT(updateModeNavigation(int,int,QString)));
        disconnect(this->uas, SIGNAL(heartbeat(UASInterface*)), this, SLOT(receiveHeartbeat(UASInterface*)));

        disconnect(this->uas, SIGNAL(emitValueAltitude(double)), this, SLOT(updateAltitude(double)));
        disconnect(this->uas, SIGNAL(globalPositionChanged(UASInterface*,double,double,double,quint64)), this, SLOT(updateGlobalPosition(UASInterface*,double,double,double,quint64)));
        disconnect(this->uas, SIGNAL(emitValueAirSpeed(double)), this, SLOT(updateSpeed(double)));
        disconnect(this->uas, SIGNAL(voltageAvionics(int,double)), this, SLOT(setAlertBattery(int,double)));
        disconnect(this->uas, SIGNAL(voltageCritical(double)), this, SLOT(setAlertCritical(double)));
    }

    if (uas)
    {
        connect(uas, SIGNAL(attitudeChanged(UASInterface*,double,double,double,quint64)), this, SLOT(updateAttitude(UASInterface*, double, double, double, quint64)));
        connect(uas, SIGNAL(statusChanged(UASInterface*,QString,QString)), this, SLOT(updateState(UASInterface*,QString,QString)));
        connect(uas, SIGNAL(modeChanged(int,QString,int)), this, SLOT(updateMode(int,QString,int)));
        connect(uas, SIGNAL(navModeChanged(int,int,QString)), this, SLOT(updateModeNavigation(int,int,QString)));
        connect(uas, SIGNAL(heartbeat(UASInterface*)), this, SLOT(receiveHeartbeat(UASInterface*)));

        connect(uas, SIGNAL(emitValueAltitude(double)), this, SLOT(updateAltitude(double)));
        connect(uas, SIGNAL(globalPositionChanged(UASInterface*,double,double,double,quint64)), this, SLOT(updateGlobalPosition(UASInterface*,double,double,double,quint64)));
        connect(uas, SIGNAL(emitValueAirSpeed(double)), this, SLOT(updateSpeed(double)));
        connect(uas, SIGNAL(voltageAvionics(int,double)), this, SLOT(setAlertBattery(int,double)));
        connect(uas, SIGNAL(voltageCritical(double)), this, SLOT(setAlertCritical(double)));

        this->uas = uas;
    }
}

void HUD::updateAttitude(UASInterface* uas, double roll, double pitch, double yaw, quint64 timestamp)
{
    if(isVisible())
    {
        Q_UNUSED(uas);
        Q_UNUSED(timestamp);

        this->roll = -1 * roll;
        this->pitch = pitch;
        this->yaw = yaw;
    }
}

void HUD::updateAltitude(double z)
{
    this->zPos = z;
}

void HUD::updateGlobalPosition(UASInterface* uas,double lat, double lon, double altitude, quint64 timestamp)
{
    if(isVisible())
    {
        Q_UNUSED(uas);
        Q_UNUSED(timestamp);
        Q_UNUSED(altitude);
        this->lat = lat;
        this->lon = lon;        
    }
}

void HUD::updateSpeed(double airSpeed)
{
    if(isVisible())
    {        
        this->airSpeed = airSpeed;
    }
}

void HUD::updateState(UASInterface *uas, QString status, QString description)
{
    Q_UNUSED(uas);
    Q_UNUSED(description);

    this->state = status;
}

void HUD::updateMode(int id, QString status, int mode)
{    
    Q_UNUSED(id);
    Q_UNUSED(mode);
    this->mode = status;
}

void HUD::updateModeNavigation(int uasid, int mode, const QString &text)
{
    Q_UNUSED(uasid);
    Q_UNUSED(mode);
    this->modeNavigation = text;
}

float HUD::refToScreenX(float x)
{
    return (scalingFactor * x);
}

float HUD::refToScreenY(float y)
{    
    return (scalingFactor * y);
}

void HUD::paintCenterBackground(float roll, float pitch, float yaw)
{
    // Center indicator is 100 mm wide
    float referenceWidth = 70.0;
    float referenceHeight = 70.0;

    // HUD is assumed to be 200 x 150 mm
    // so that positions can be hardcoded
    // but can of course be scaled.

    double referencePositionX = vwidth / 2.0 - referenceWidth/2.0;
    double referencePositionY = vheight / 2.0 - referenceHeight/2.0;

    //this->width()/2.0+(xCenterOffset*scalingFactor), this->height()/2.0+(yCenterOffset*scalingFactor);

    setupGLView(referencePositionX, referencePositionY, referenceWidth, referenceHeight);

    // Store current position in the model view
    // the position will be restored after drawing
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();

    // Move to the center of the window
    glTranslatef(referenceWidth/2.0f,referenceHeight/2.0f,0);

    // Move based on the yaw difference
    glTranslatef(yaw, 0.0f, 0.0f);

    // Rotate based on the bank
    glRotatef((roll/M_PI)*180.0f, 0.0f, 0.0f, 1.0f);

    // Translate in the direction of the rotation based
    // on the pitch. On the 777, a pitch of 1 degree = 2 mm
    //glTranslatef(0, ((-pitch/M_PI)*180.0f * vPitchPerDeg), 0);
    glTranslatef(0.0f, (-pitch * vPitchPerDeg * 16.5f), 0.0f);

    // Ground
    glColor3ub(179,102,0);

    glBegin(GL_POLYGON);
    glVertex2f(-300,-300);
    glVertex2f(-300,0);
    glVertex2f(300,0);
    glVertex2f(300,-300);
    glVertex2f(-300,-300);
    glEnd();

    // Sky
    glColor3ub(0,153,204);

    glBegin(GL_POLYGON);
    glVertex2f(-300,0);
    glVertex2f(-300,300);
    glVertex2f(300,300);
    glVertex2f(300,0);
    glVertex2f(-300,0);

    glEnd();
}

void HUD::paintText(QString text, QColor color, float fontSize, float refX, float refY, QPainter* painter)
{
    QPen prevPen = painter->pen();
    float pPositionX = refToScreenX(refX) - (fontSize*scalingFactor*0.072f);
    float pPositionY = refToScreenY(refY) - (fontSize*scalingFactor*0.212f);

    QFont font("Bitstream Vera Sans");
    // Enforce minimum font size of 5 pixels
    int fSize = qMax(5, (int)(fontSize*scalingFactor*1.26f));
    font.setPixelSize(fSize);

    QFontMetrics metrics = QFontMetrics(font);
    int border = qMax(4, metrics.leading());
    QRect rect = metrics.boundingRect(0, 0, width() - 2*border, int(height()*0.125),
                                      Qt::AlignLeft | Qt::TextWordWrap, text);
    painter->setPen(color);
    painter->setFont(font);
    painter->setRenderHint(QPainter::TextAntialiasing);
    painter->drawText(pPositionX, pPositionY,
                      rect.width(), rect.height(),
                      Qt::AlignCenter | Qt::TextWordWrap, text);
    painter->setPen(prevPen);
}

void HUD::initializeGL()
{
    bool antialiasing = true;

    // Antialiasing setup
    if(antialiasing)
    {
        glEnable(GL_MULTISAMPLE);
        glEnable(GL_BLEND);

        glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

        glEnable(GL_POINT_SMOOTH);
        glEnable(GL_LINE_SMOOTH);

        glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
        glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    }
    else
    {
        glDisable(GL_BLEND);
        glDisable(GL_POINT_SMOOTH);
        glDisable(GL_LINE_SMOOTH);
    }
}

void HUD::setupGLView(float referencePositionX, float referencePositionY, float referenceWidth, float referenceHeight)
{
    int pixelWidth  = (int)(referenceWidth * scalingFactor);
    int pixelHeight = (int)(referenceHeight * scalingFactor);
    // Translate and scale the GL view in the virtual reference coordinate units on the screen
    int pixelPositionX = (int)((referencePositionX * scalingFactor) + xCenterOffset);
    int pixelPositionY = this->height() - (referencePositionY * scalingFactor) + yCenterOffset - pixelHeight;

    //The viewport is established at the correct pixel position and clips everything
    // out of the desired instrument location
    glViewport(pixelPositionX, pixelPositionY, pixelWidth, pixelHeight);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    // The ortho projection is setup in a way that so that the drawing is done in the
    // reference coordinate space
    glOrtho(0, referenceWidth, 0, referenceHeight, -1, 1);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    //glScalef(scaleX, scaleY, 1.0f);
}

void HUD::paintEvent(QPaintEvent *event)
{
    // Event is not needed
    // the event is ignored as this widget
    // is refreshed automatically
    Q_UNUSED(event);
}

void HUD::paintHUD()
{
    if (isVisible())
    {
        // Read out most important values to limit hash table lookups
        // Low-pass roll, pitch and yaw
        rollLP = rollLP * 0.2f + 0.8f * roll;
        pitchLP = pitchLP * 0.2f + 0.8f * pitch;
        yawLP = yawLP * 0.2f + 0.8f * yaw;

        // Translate for yaw
        const float maxYawTrans = 60.0f;

        float newYawDiff = yawDiff;
        if (isinf(newYawDiff)) newYawDiff = yawDiff;
        if (newYawDiff > M_PI) newYawDiff = newYawDiff - M_PI;

        if (newYawDiff < -M_PI) newYawDiff = newYawDiff + M_PI;

        newYawDiff = yawDiff * 0.8 + newYawDiff * 0.2;

        yawDiff = newYawDiff;

        yawInt += newYawDiff;

        if (yawInt > M_PI) yawInt = (float)M_PI;
        if (yawInt < -M_PI) yawInt = (float)-M_PI;

        float yawTrans = yawInt * (float)maxYawTrans;
        yawInt *= 0.6f;

        if ((yawTrans < 5.0) && (yawTrans > -5.0)) yawTrans = 0;

        // Negate to correct direction
        yawTrans = -yawTrans;

        //qDebug() << "yaw translation" << yawTrans << "integral" << yawInt << "difference" << yawDiff << "yaw" << yaw;

        // Update scaling factor
        // adjust scaling to fit both horizontally and vertically
        scalingFactor = this->width()/vwidth;
        double scalingFactorH = this->height()/vheight;
        if (scalingFactorH < scalingFactor) scalingFactor = scalingFactorH;

        // OPEN GL PAINTING
        // Store model view matrix to be able to reset it to the previous state
        makeCurrent();
        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        paintCenterBackground(roll, pitch, yawTrans);

        glMatrixMode(GL_MODELVIEW);
        glPopMatrix();

        // END OF OPENGL PAINTING

        if (hudInstrumentsEnabled)
        {
            QPainter painter;
            painter.begin(this);
            painter.setRenderHint(QPainter::Antialiasing, true);
            painter.setRenderHint(QPainter::HighQualityAntialiasing, true);
            painter.translate((this->vwidth/2.0+xCenterOffset)*scalingFactor, (this->vheight/2.0+yCenterOffset)*scalingFactor);

            paintText(mode, infoColor, 5.0f, (-vwidth/2.0) + 10, -vheight/2.0 + 10, &painter);
            paintText(modeNavigation, infoColor, 5.0f, (-vwidth/2.0) + 10, -vheight/2.0 + 15, &painter);
            paintText(state, infoColor, 5.0f, (-vwidth/2.0) + 10, -vheight/2.0 + 20, &painter);

            QString voltageCritical("Criticos %1 v");
            paintText(voltageCritical.arg(critical, 4, 'f', 2, '0'), infoColor, 5.0f, (-vwidth/2.0) + 140, -vheight/2.0 + 10, &painter);

            QString voltageAvionics("Avionica %1 v");
            paintText(voltageAvionics.arg(battery, 4, 'f', 2, '0'), infoColor, 5.0f, (-vwidth/2.0) + 140, -vheight/2.0 + 15, &painter);

            double tempLat = UASManager::instance()->getHomeLatitude();
            double tempLon = UASManager::instance()->getHomeLongitude();

            //Coordinate* home = new Coordinate(tempLat, tempLon);
            //Coordinate* position = new Coordinate(lat, lon);

            double distance = 10;//Geography::DistanceCoordinate(home, position, Geography::KM);
            QString distanceText("Distancia %1 Km");
            paintText(distanceText.arg(distance, 4, 'f', 2, '0'), infoColor, 5.0f, (-vwidth/2.0) + 140, -vheight/2.0 + 20, &painter);

            // Waypoint
            //paintText(waypointName, defaultColor, 4.0f, (-vwidth/3.0) + 10, +vheight/3.0 + 15, &painter);

            // YAW INDICATOR
            //
            //      .
            //    .   .
            //   .......
            //
            const float yawIndicatorWidth = 4.0f;
            const float yawIndicatorY = vheight/2.0f - 10.0f;
            QPolygon yawIndicator(4);
            yawIndicator.setPoint(0, QPoint(refToScreenX(0.0f), refToScreenY(yawIndicatorY)));
            yawIndicator.setPoint(1, QPoint(refToScreenX(yawIndicatorWidth/2.0f), refToScreenY(yawIndicatorY+yawIndicatorWidth)));
            yawIndicator.setPoint(2, QPoint(refToScreenX(-yawIndicatorWidth/2.0f), refToScreenY(yawIndicatorY+yawIndicatorWidth)));
            yawIndicator.setPoint(3, QPoint(refToScreenX(0.0f), refToScreenY(yawIndicatorY)));
            painter.setPen(defaultColor);
            painter.drawPolyline(yawIndicator);

            // CENTER

            // HEADING INDICATOR
            //
            //    __      __
            //       \/\/
            //
            const float hIndicatorWidth = 7.0f;
            const float hIndicatorY = -25.0f;
            const float hIndicatorYLow = hIndicatorY + hIndicatorWidth / 6.0f;
            const float hIndicatorSegmentWidth = hIndicatorWidth / 7.0f;
            QPolygon hIndicator(7);
            hIndicator.setPoint(0, QPoint(refToScreenX(0.0f-hIndicatorWidth/2.0f), refToScreenY(hIndicatorY)));
            hIndicator.setPoint(1, QPoint(refToScreenX(0.0f-hIndicatorWidth/2.0f+hIndicatorSegmentWidth*1.75f), refToScreenY(hIndicatorY)));
            hIndicator.setPoint(2, QPoint(refToScreenX(0.0f-hIndicatorSegmentWidth*1.0f), refToScreenY(hIndicatorYLow)));
            hIndicator.setPoint(3, QPoint(refToScreenX(0.0f), refToScreenY(hIndicatorY)));
            hIndicator.setPoint(4, QPoint(refToScreenX(0.0f+hIndicatorSegmentWidth*1.0f), refToScreenY(hIndicatorYLow)));
            hIndicator.setPoint(5, QPoint(refToScreenX(0.0f+hIndicatorWidth/2.0f-hIndicatorSegmentWidth*1.75f), refToScreenY(hIndicatorY)));
            hIndicator.setPoint(6, QPoint(refToScreenX(0.0f+hIndicatorWidth/2.0f), refToScreenY(hIndicatorY)));
            painter.setPen(defaultColor);
            painter.drawPolyline(hIndicator);

            // SETPOINT
            const float centerWidth = 4.0f;
            painter.setPen(defaultColor);
            painter.setBrush(Qt::NoBrush);

            const float centerCrossWidth = 10.0f;
            // left
            painter.drawLine(QPointF(refToScreenX(-centerWidth / 2.0f), refToScreenY(0.0f)), QPointF(refToScreenX(-centerCrossWidth / 2.0f), refToScreenY(0.0f)));
            // right
            painter.drawLine(QPointF(refToScreenX(centerWidth / 2.0f), refToScreenY(0.0f)), QPointF(refToScreenX(centerCrossWidth / 2.0f), refToScreenY(0.0f)));
            // top
            painter.drawLine(QPointF(refToScreenX(0.0f), refToScreenY(-centerWidth / 2.0f)), QPointF(refToScreenX(0.0f), refToScreenY(-centerCrossWidth / 2.0f)));

            // COMPASS
            const float compassY = -vheight/2.0f + 30.0f;
            QRectF compassRect(QPointF(refToScreenX(-5.0f), refToScreenY(compassY)), QSizeF(refToScreenX(10.0f), refToScreenY(5.0f)));
            painter.setBrush(Qt::NoBrush);
            painter.setPen(Qt::SolidLine);
            painter.setPen(defaultColor);
            painter.drawRoundedRect(compassRect, 2, 2);
            QString yawAngle;

            //    const float yawDeg = ((values.value("yaw", 0.0f)/M_PI)*180.0f)+180.f;

            // YAW is in compass-human readable format, so 0 - 360deg. This is normal in aviation, not -180 - +180.
            const float yawDeg = ((yawLP/M_PI)*180.0f)+180.0f+180.0f;
            int yawCompass = static_cast<int>(yawDeg) % 360;
            yawAngle.sprintf("%03d", yawCompass);
            paintText(yawAngle, Qt::yellow, 3.5f, -4.3f, compassY+ 0.97f, &painter);

            // GAUGES
            drawChangeIndicatorGauge(-vGaugeSpacing, -15.0f, 15.0f, 2.0f, this->zPos, defaultColor, &painter, false);

            // Right speed gauge
            drawChangeIndicatorGauge(vGaugeSpacing, -15.0f, 15.0f, 5.0f, this->airSpeed, defaultColor, &painter, false);

            painter.translate(refToScreenX(yawTrans), 0);

            // Rotate view and draw all roll-dependent indicators
            painter.rotate((rollLP/M_PI)* -180.0f);

            painter.translate(0, (-pitchLP/(float)M_PI)* -180.0f * refToScreenY(1.8f));

            paintPitchLines(pitchLP, &painter);
            painter.end();
        }
        else
        {
            QPainter painter;
            painter.begin(this);
            painter.end();
        }
    }
}

void HUD::paintPitchLines(float pitch, QPainter* painter)
{
    QString label;

    const float yDeg = vPitchPerDeg;
    const float lineDistance = 5.0f; ///< One pitch line every 10 degrees
    const float posIncrement = yDeg * lineDistance;
    float posY = posIncrement;
    const float posLimit = sqrt(pow(vwidth, 2.0f) + pow(vheight, 2.0f));

    const float offsetAbs = pitch * yDeg;

    float offset = pitch;
    if (offset < 0) offset = -offset;
    int offsetCount = 0;
    while (offset > lineDistance)
    {
        offset -= lineDistance;
        offsetCount++;
    }

    int iPos = (int)(0.5f + lineDistance); ///< The first line
    int iNeg = (int)(-0.5f - lineDistance); ///< The first line

    offset *= yDeg;


    painter->setPen(defaultColor);

    posY = -offsetAbs + posIncrement; //+ 100;// + lineDistance;

    while (posY < posLimit)
    {
        paintPitchLinePos(label.sprintf("%3d", iPos), 0.0f, -posY, painter);
        posY += posIncrement;
        iPos += (int)lineDistance;
    }



    // HORIZON
    //
    //    ------------    ------------
    //
    const float pitchWidth = 30.0f;
    const float pitchGap = pitchWidth / 2.5f;
    const QColor horizonColor = defaultColor;
    const float diagonal = sqrt(pow(vwidth, 2.0f) + pow(vheight, 2.0f));
    const float lineWidth = refLineWidthToPen(0.5f);

    // Left horizon
    drawLine(0.0f-diagonal, offsetAbs, 0.0f-pitchGap/2.0f, offsetAbs, lineWidth, horizonColor, painter);

    // Right horizon
    drawLine(0.0f+pitchGap/2.0f, offsetAbs, 0.0f+diagonal, offsetAbs, lineWidth, horizonColor, painter);



    label.clear();

    posY = offsetAbs  + posIncrement;


    while (posY < posLimit)
    {
        paintPitchLineNeg(label.sprintf("%3d", iNeg), 0.0f, posY, painter);
        posY += posIncrement;
        iNeg -= (int)lineDistance;
    }
}

void HUD::paintPitchLinePos(QString text, float refPosX, float refPosY, QPainter* painter)
{
    //painter->setPen(QPen(QBrush, normalStrokeWidth));

    const float pitchWidth = 30.0f;
    const float pitchGap = pitchWidth / 2.5f;
    const float pitchHeight = pitchWidth / 12.0f;
    const float textSize = pitchHeight * 1.1f;
    const float lineWidth = 0.5f;

    // Positive pitch indicator:
    //
    //      _______      _______
    //     |10                  |
    //

    // Left vertical line
    drawLine(refPosX-pitchWidth/2.0f, refPosY, refPosX-pitchWidth/2.0f, refPosY+pitchHeight, lineWidth, defaultColor, painter);
    // Left horizontal line
    drawLine(refPosX-pitchWidth/2.0f, refPosY, refPosX-pitchGap/2.0f, refPosY, lineWidth, defaultColor, painter);
    // Text left
    paintText(text, defaultColor, textSize, refPosX-pitchWidth/2.0 + 0.75f, refPosY + pitchHeight - 1.75f, painter);

    // Right vertical line
    drawLine(refPosX+pitchWidth/2.0f, refPosY, refPosX+pitchWidth/2.0f, refPosY+pitchHeight, lineWidth, defaultColor, painter);
    // Right horizontal line
    drawLine(refPosX+pitchWidth/2.0f, refPosY, refPosX+pitchGap/2.0f, refPosY, lineWidth, defaultColor, painter);
}

void HUD::paintPitchLineNeg(QString text, float refPosX, float refPosY, QPainter* painter)
{
    const float pitchWidth = 30.0f;
    const float pitchGap = pitchWidth / 2.5f;
    const float pitchHeight = pitchWidth / 12.0f;
    const float textSize = pitchHeight * 1.1f;
    const float segmentWidth = ((pitchWidth - pitchGap)/2.0f) / 7.0f; ///< Four lines and three gaps -> 7 segments

    const float lineWidth = 0.1f;

    // Negative pitch indicator:
    //
    //      -10
    //     _ _ _ _|     |_ _ _ _
    //
    //

    // Left vertical line
    drawLine(refPosX-pitchGap/2.0, refPosY, refPosX-pitchGap/2.0, refPosY-pitchHeight, lineWidth, defaultColor, painter);
    // Left horizontal line with four segments
    for (int i = 0; i < 7; i+=2)
    {
        drawLine(refPosX-pitchWidth/2.0+(i*segmentWidth), refPosY, refPosX-pitchWidth/2.0+(i*segmentWidth)+segmentWidth, refPosY, lineWidth, defaultColor, painter);
    }
    // Text left
    paintText(text, defaultColor, textSize, refPosX-pitchWidth/2.0f + 0.75f, refPosY + pitchHeight - 1.75f, painter);

    // Right vertical line
    drawLine(refPosX+pitchGap/2.0, refPosY, refPosX+pitchGap/2.0, refPosY-pitchHeight, lineWidth, defaultColor, painter);
    // Right horizontal line with four segments
    for (int i = 0; i < 7; i+=2)
    {
        drawLine(refPosX+pitchWidth/2.0f-(i*segmentWidth), refPosY, refPosX+pitchWidth/2.0f-(i*segmentWidth)-segmentWidth, refPosY, lineWidth, defaultColor, painter);
    }
}

void rotatePointClockWise(QPointF& p, float angle)
{
    // Standard 2x2 rotation matrix, counter-clockwise
    //
    //   |  cos(phi)   sin(phi) |
    //   | -sin(phi)   cos(phi) |
    //

    //p.setX(cos(angle) * p.x() + sin(angle) * p.y());
    //p.setY(-sin(angle) * p.x() + cos(angle) * p.y());


    p.setX(cos(angle) * p.x() + sin(angle)* p.y());
    p.setY((-1.0f * sin(angle) * p.x()) + cos(angle) * p.y());
}

float HUD::refLineWidthToPen(float line)
{
    return line * 2.50f;
}

void HUD::rotatePolygonClockWiseRad(QPolygonF& p, float angle, QPointF origin)
{
    // Standard 2x2 rotation matrix, counter-clockwise
    //
    //   |  cos(phi)   sin(phi) |
    //   | -sin(phi)   cos(phi) |
    //
    for (int i = 0; i < p.size(); i++)
    {
        QPointF curr = p.at(i);

        const float x = curr.x();
        const float y = curr.y();

        curr.setX(((cos(angle) * (x-origin.x())) + (-sin(angle) * (y-origin.y()))) + origin.x());
        curr.setY(((sin(angle) * (x-origin.x())) + (cos(angle) * (y-origin.y()))) + origin.y());
        p.replace(i, curr);
    }
}

void HUD::drawPolygon(QPolygonF refPolygon, QPainter* painter)
{
    // Scale coordinates
    QPolygonF draw(refPolygon.size());
    for (int i = 0; i < refPolygon.size(); i++)
    {
        QPointF curr;
        curr.setX(refToScreenX(refPolygon.at(i).x()));
        curr.setY(refToScreenY(refPolygon.at(i).y()));
        draw.replace(i, curr);
    }
    painter->drawPolygon(draw);
}

void HUD::drawChangeIndicatorGauge(float xRef, float yRef, float radius, float expectedMaxChange, float value, const QColor& color, QPainter* painter, bool solid)
{
    // Draw the circle
    QPen circlePen(Qt::SolidLine);
    if (!solid) circlePen.setStyle(Qt::DotLine);
    circlePen.setWidth(refLineWidthToPen(0.5f));
    circlePen.setColor(defaultColor);
    painter->setBrush(Qt::NoBrush);
    painter->setPen(circlePen);
    drawCircle(xRef, yRef, radius, 200.0f, 170.0f, 1.0f, color, painter);

    QString label;
    label.sprintf("%05.1f", value);

    // Draw the value
    paintText(label, color, 5.0f, xRef-7.5f, yRef-2.0f, painter);

    // Draw the needle
    // Scale the rotation so that the gauge does one revolution
    // per max. change
    const float rangeScale = (2.0f * M_PI) / expectedMaxChange;
    const float maxWidth = radius / 10.0f;
    const float minWidth = maxWidth * 0.3f;

    QPolygonF p(6);

    p.replace(0, QPointF(xRef-maxWidth/2.0f, yRef-radius * 0.5f));
    p.replace(1, QPointF(xRef-minWidth/2.0f, yRef-radius * 0.9f));
    p.replace(2, QPointF(xRef+minWidth/2.0f, yRef-radius * 0.9f));
    p.replace(3, QPointF(xRef+maxWidth/2.0f, yRef-radius * 0.5f));
    p.replace(4, QPointF(xRef,               yRef-radius * 0.46f));
    p.replace(5, QPointF(xRef-maxWidth/2.0f, yRef-radius * 0.5f));

    rotatePolygonClockWiseRad(p, value*rangeScale, QPointF(xRef, yRef));

    QBrush indexBrush;
    indexBrush.setColor(defaultColor);
    indexBrush.setStyle(Qt::SolidPattern);
    painter->setPen(Qt::SolidLine);
    painter->setPen(defaultColor);
    painter->setBrush(indexBrush);
    drawPolygon(p, painter);
}

void HUD::drawLine(float refX1, float refY1, float refX2, float refY2, float width, const QColor& color, QPainter* painter)
{
    QPen pen(Qt::SolidLine);
    pen.setWidth(refLineWidthToPen(width));
    pen.setColor(color);
    painter->setPen(pen);
    painter->drawLine(QPoint(refToScreenX(refX1), refToScreenY(refY1)), QPoint(refToScreenX(refX2), refToScreenY(refY2)));
}

void HUD::drawEllipse(float refX, float refY, float radiusX, float radiusY, float startDeg, float endDeg, float lineWidth, const QColor& color, QPainter* painter)
{
    Q_UNUSED(startDeg);
    Q_UNUSED(endDeg);
    QPen pen(painter->pen().style());
    pen.setWidth(refLineWidthToPen(lineWidth));
    pen.setColor(color);
    painter->setPen(pen);
    painter->drawEllipse(QPointF(refToScreenX(refX), refToScreenY(refY)), refToScreenX(radiusX), refToScreenY(radiusY));
}

void HUD::drawCircle(float refX, float refY, float radius, float startDeg, float endDeg, float lineWidth, const QColor& color, QPainter* painter)
{
    drawEllipse(refX, refY, radius, radius, startDeg, endDeg, lineWidth, color, painter);
}

void HUD::resizeGL(int w, int h)
{
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, w, 0, h, -1, 1);
    glMatrixMode(GL_MODELVIEW);
    glPolygonMode(GL_FRONT, GL_FILL);
    //FIXME
    paintHUD();
}

void HUD::enableHUDInstruments(bool enabled)
{
    hudInstrumentsEnabled = enabled;
}

void HUD::setAlertBattery(int id, double battery)
{
    Q_UNUSED(id);

    this->battery = battery;
}

void HUD::setAlertCritical(double value)
{
    this->critical = value;
}
