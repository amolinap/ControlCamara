#ifndef QGCLOGPLAYER_H
#define QGCLOGPLAYER_H

#include <QWidget>
#include <QFile>
#include <QFileDialog>
#include <QMessageBox>
#include <QDesktopServices>
#include <QDebug>

#include "MAVLinkProtocol.h"
#include "LinkInterface.h"
//#include "MAVLinkSimulationLink.h"
#include "QGC.h"
#include "MainWindow.h"
#include "LinkManager.h"

namespace Ui {
    class QGCLogPlayer;
}

class QGCLogPlayer : public QWidget
{
    Q_OBJECT

public:
    explicit QGCLogPlayer(MAVLinkProtocol* mavlink, QWidget *parent = 0);
    ~QGCLogPlayer();

public slots:
    /** @brief Replay the logfile */
    void play();
    /** @brief Pause the logfile */
    void pause();
    /** @brief Reset the logfile */
    bool reset(int packetIndex=0);
    /** @brief Select logfile */
    void selectLogFile();
    /** @brief Load log file */
    void loadLogFile(const QString& file);
    /** @brief Jumps to the current percentage of the position slider */
    void jumpToSliderVal(int slidervalue);
    /** @brief This function is the "mainloop" of the log player, reading one line
     * and adjusting the mainloop timer to read the next line in time.
     * It might not perfectly match the timing of the log file,
     * but it will never induce a static drift into the log file replay.
     * For scientific logging, the use of onboard timestamps and the log
     * functionality of the line chart plot is recommended.
     */
    void logLoop();
    /** @brief Set acceleration factor in percent
     * @param factor 0: 0.01X, 50: 1.0X, 100: 100.0X
     */
    void setAccelerationFactorInt(int factor);
    /**
     * @brief This method allows you to change the working path of the information read or stored
     *
     * @param path The new working path
     **/
    void changePATH(const QString& path);

protected:
    int lineCounter;
    int totalLines;
    quint64 startTime;
    quint64 endTime;
    quint64 currentStartTime;
    float accelerationFactor;
    MAVLinkProtocol* mavlink;
    MAVLinkSimulationLink* logLink;
    QFile logFile;
    QTimer loopTimer;
    int loopCounter;
    bool mavlinkLogFormat;
    int binaryBaudRate;
    static const int packetLen = MAVLINK_MAX_PACKET_LEN;
    static const int timeLen = sizeof(quint64);
    void changeEvent(QEvent *e);

private:
    Ui::QGCLogPlayer *ui;
    QString pathSaveData;

signals:
    /** @brief Send ready bytes */
    void bytesReady(LinkInterface* link, const QByteArray& bytes);
};

#endif // QGCLOGPLAYER_H
