#include "QGCFlight.h"
#include "ui_QGCFlight.h"

QGCFlight::QGCFlight(QWidget *parent) :
        QWidget(parent),
        uas(NULL),
        yawLP(0.0f),
        pitchLP(0.0f),
        refreshTimer(new QTimer()),
        ui(new Ui::QGCFlight)
{
    ui->setupUi(this);

    connect(UASManager::instance(), SIGNAL(activeUASSet(UASInterface*)), this, SLOT(setActiveUAS(UASInterface*)));

    refreshTimer->setInterval(100);
    connect(refreshTimer, SIGNAL(timeout()), this, SLOT(refreshData()));
    refreshTimer->start();
}

QGCFlight::~QGCFlight()
{
    delete ui;
}

void QGCFlight::setActiveUAS(UASInterface* uas)
{
    if (this->uas != NULL)
    {
        disconnect(this->uas, SIGNAL(attitudeChanged(UASInterface*,double,double,double,quint64)), this, SLOT(updateAttitude(UASInterface*, double, double, double, quint64)));
        disconnect(uas, SIGNAL(globalPositionChanged(UASInterface*,double,double,double,quint64)), this, SLOT(globalPositionChanged(UASInterface*,double,double,double,quint64)));
    }

    if (uas)
    {
        connect(uas, SIGNAL(attitudeChanged(UASInterface*,double,double,double,quint64)), this, SLOT(updateAttitude(UASInterface*, double, double, double, quint64)));
        connect(uas, SIGNAL(globalPositionChanged(UASInterface*,double,double,double,quint64)), this, SLOT(globalPositionChanged(UASInterface*,double,double,double,quint64)));

        this->uas = uas;
    }
}

void QGCFlight::updateAttitude(UASInterface* uas, double roll, double pitch, double yaw, quint64 timestamp)
{
    if(isVisible())
    {
        Q_UNUSED(uas);
        Q_UNUSED(timestamp);

        this->roll = -1 * roll;
        this->pitch = pitch;
        this->yaw = yaw;
//        float diff = fabs(yaw - this->yaw);

//        while (diff > (float)M_PI)
//        {
//            diff -= (float)M_PI;
//        }

//        if (diff > 0.1f)
//        {
//            this->yaw = yaw;
//            //drawIcon(mypen);
//        }
    }
}

void QGCFlight::refreshData()
{
    ui->tbLatitud->setText(QString::number(lat));
    ui->tbLongitud->setText(QString::number(lon));
    ui->tbAltura->setText(QString::number(alt));

    ui->tbPitch->setText(QString::number(pitch));
    ui->tbRoll->setText(QString::number(roll));
    ui->tbYaw->setText(QString::number(yaw));

    //    planeOrient.setRoll(((roll/M_PI)+1.0)*360.0);
    //    planeOrient.setTilt(((pitch/M_PI)+1.0)*360.0);
    //    planeOrient.setHeading(((yaw/M_PI)+1.0)*360.0);


        //const float yawDeg = ((yaw/M_PI)*180.0f)+180.0f+180.0f;
    rollLP = rollLP * 0.2f + 0.8f * roll;
    pitchLP = pitchLP * 0.2f + 0.8f * pitch;
    yawLP = yawLP * 0.2f + 0.8f * yaw;

    const float yawDeg = ((yawLP/M_PI)*180.0f)+180.0f+180.0f;
    int yawCompass = static_cast<int>(yawDeg) % 360;
    //float pitchD = (-pitchLP/(float)M_PI)* -180.0f;
    double gPitch = (180*pitch)/M_PI;

    //ui->tbPitchDeg->setText(QString::number(((pitch/M_PI)+1.0)*360.0));
    ui->tbPitchDeg->setText(QString::number(90+gPitch));
    //ui->tbRollDeg->setText(QString::number(((roll/M_PI)+1.0)*360.0));
    ui->tbRollDeg->setText(QString::number((rollLP/M_PI)* -180.0f));
    //ui->tbYawDeg->setText(QString::number(((yaw/M_PI)+1.0)*360.0));
    ui->tbYawDeg->setText(QString::number(yawCompass));

    emit emitParametrosPosicion(lat, lon, alt);
    emit emitParametrosVuelo(((rollLP/M_PI)* -180.0f)*-1, 90+gPitch, yawCompass);
}

void QGCFlight::globalPositionChanged(UASInterface *uas, double lat, double lon, double alt, quint64 usec)
{
    Q_UNUSED(uas);
    Q_UNUSED(usec);

    this->lat = lat;
    this->lon = lon;
    this->alt = alt;
}
