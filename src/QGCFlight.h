#ifndef QGCFLIGHT_H
#define QGCFLIGHT_H

#include <QWidget>
#include <QTimer>
#include <qmath.h>

#include "UASManager.h"

namespace Ui {
    class QGCFlight;
}

class QGCFlight : public QWidget
{
    Q_OBJECT

public:
    explicit QGCFlight(QWidget *parent = 0);
    ~QGCFlight();

private:
    Ui::QGCFlight *ui;
    UASInterface* uas; ///< The uas currently monitored
    float roll;
    float pitch;
    float yaw;
    float lat;
    float lon;
    float alt;
    QTimer* refreshTimer;      ///< The main timer, controls the update rate
    float yawLP;
    float pitchLP;
    float rollLP;

public slots:
    void setActiveUAS(UASInterface* uas);
    void updateAttitude(UASInterface* uas, double roll, double pitch, double yaw, quint64 timestamp);
    void globalPositionChanged(UASInterface* uas, double lat, double lon, double alt, quint64 usec);
    void refreshData();

signals:
    void emitParametrosPosicion(double lat, double lon, double alt);
    void emitParametrosVuelo(double roll, double pitch, double yaw);
};

#endif // QGCFLIGHT_H
