/*=====================================================================
======================================================================*/

#ifndef SLUGSMAV_H
#define SLUGSMAV_H

#include <QTimer>
#include <QMap>
#include <QString>
#include <QApplication>
#include <QDebug>

//#include "UASManager.h"
#include "MAVLinkProtocol.h"
#include "mavlink.h"
#include "LinkManager.h"
//#include "QGCConfiguration.h"

/**
* @brief Received messages defined for slugs
*
* The SlugsMAV received messages defined for SLUGS specified in protocol MAVLINK.
*
**/
class SlugsMAV : public QObject
{
    Q_OBJECT
    //Q_INTERFACES(UASInterface)

public:
    /** @brief This is the class constructor.
     *
     * @param mavlink  The protocol communication
     * @param id    The id for UAV
     **/
    SlugsMAV(MAVLinkProtocol* mavlink, int id = 0);
    ~SlugsMAV();
    void setSelected();

    /**
     * @brief Enumeration SLUGS data information
     */
    enum ValueType
    {
        None = 0,

        GPS_RAW_TIME = 1, /**< Timestamp (microseconds since UNIX epoch or microseconds since system boot) */
        GPS_RAW_FIX = 2,/**<  0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.*/
        GPS_RAW_LATITUDE = 3,/**< Latitude in degrees*/
        GPS_RAW_LONGITUDE = 4,/**< Longitude in degrees*/
        GPS_RAW_ALTITUDE = 5,/**< Altitude in meters*/
        GPS_RAW_EPH = 6,
        GPS_RAW_EPV = 7,
        GPS_RAW_SPEED = 8,/**< GPS ground speed*/
        GPS_RAW_COMPASS = 9,/**< Compass heading in degrees, 0..360 degrees*/

        GPS_DATE_YEAR = 10,/**< Year reported by Gps*/
        GPS_DATE_MONTH = 11,/**< Month reported by Gps*/
        GPS_DATE_DAY = 12,/**< Day reported by Gps*/
        GPS_DATE_HOUR = 13,/**< Hour reported by Gps*/
        GPS_DATE_MIN = 14,/**< Min reported by Gps*/
        GPS_DATE_SEC = 15,/**< Sec reported by Gps*/
        GPS_DATE_SAT = 16,/**< Visible sattelites reported by Gps*/
        GPS_DATE_CLOCKSAT = 17,/**< Visible sattelites reported by Gps*/
        GPS_DATE_USESAT = 18,/**< Visible sattelites reported by Gps*/
        GPS_DATE_GPPGL = 19,/**< Visible sattelites reported by Gps*/
        GPS_DATE_MASK = 20,/**< Visible sattelites reported by Gps*/
        GPS_DATE_PERCENT = 21,/**< Visible sattelites reported by Gps*/

        SCALED_PRESSURE_ABS = 22,/**< Absolute pressure (hectopascal)*/
        SCALED_PRESSURE_DIFF = 23,/**< Differential pressure 1 (hectopascal)*/
        SCALED_PRESSURE_TEMP = 24,/**< Temperature measurement (0.01 degrees celsius)*/

        DIAGNOSTIC_F1 = 25,
        DIAGNOSTIC_F2 = 26,
        DIAGNOSTIC_F3 = 27,
        DIAGNOSTIC_S1 = 28,
        DIAGNOSTIC_S2 = 29,
        DIAGNOSTIC_S3 = 30,

        DATA_LOG_1 = 31,
        DATA_LOG_2 = 32,
        DATA_LOG_3 = 33,
        DATA_LOG_4 = 34,
        DATA_LOG_5 = 35,
        DATA_LOG_6 = 36,

        NAVIGATION_AIR_SPEED = 37,
        NAVIGATION_ROLL = 38,
        NAVIGATION_PITCH = 39,
        NAVIGATION_TURN = 40,
        NAVIGATION_ACCELERATION = 41,
        NAVIGATION_DISTANCE = 42,
        NAVIGATION_REMAINING = 43,
        NAVIGATION_WP1 = 44,
        NAVIGATION_WP2 = 45,

        CPU_LOAD_BATTERY = 46,
        CPU_LOAD_SENSOR = 47,
        CPU_LOAD_CONTROL = 48,

        RAW_IMU_ACC_X = 49,
        RAW_IMU_ACC_Y = 50,
        RAW_IMU_ACC_Z = 51,
        RAW_IMU_GYR_X = 52,
        RAW_IMU_GYR_Y = 53,
        RAW_IMU_GYR_Z = 54,
        RAW_IMU_MAG_X = 55,
        RAW_IMU_MAG_Y = 56,
        RAW_IMU_MAG_Z = 57,

        LOCAL_POSITION_X = 58,
        LOCAL_POSITION_Y = 59,
        LOCAL_POSITION_Z = 60,
        LOCAL_POSITION_SPEED_X = 61,
        LOCAL_POSITION_SPEED_Y = 62,
        LOCAL_POSITION_SPEED_Z = 63,

        SYS_STATUS_MODE = 64,
        SYS_STATUS_NAV_MODE = 65,
        SYS_STATUS_STATUS = 66,
        SYS_STATUS_LOAD = 67,
        SYS_STATUS_BAT_V = 68,
        SYS_STATUS_BAT_R = 69,
        SYS_STATUS_PAC_D = 70,

        SERVO_OUTPUT_RAW_1 = 71,
        SERVO_OUTPUT_RAW_2 = 72,
        SERVO_OUTPUT_RAW_3 = 73,
        SERVO_OUTPUT_RAW_4 = 74,
        SERVO_OUTPUT_RAW_5 = 75,
        SERVO_OUTPUT_RAW_6 = 76,
        SERVO_OUTPUT_RAW_7 = 77,
        SERVO_OUTPUT_RAW_8 = 78,

        RC_CHANNELS_RAW_1 = 79,
        RC_CHANNELS_RAW_2 = 80,
        RC_CHANNELS_RAW_3 = 81,
        RC_CHANNELS_RAW_4 = 82,

        SCALED_IMU_ACC_X = 83,
        SCALED_IMU_ACC_Y = 84,
        SCALED_IMU_ACC_Z = 85,
        SCALED_IMU_GYR_X = 86,
        SCALED_IMU_GYR_Y = 87,
        SCALED_IMU_GYR_Z = 88,
        SCALED_IMU_MAG_X = 89,
        SCALED_IMU_MAG_Y = 90,
        SCALED_IMU_MAG_Z = 91,

        ATTITUDE_ROLL = 92,
        ATTITUDE_PITCH = 93,
        ATTITUDE_YAW = 94,
        ATTITUDE_ROLL_SPEED = 95,
        ATTITUDE_PITCH_SPEED = 96,
        ATTITUDE_YAW_SPEED = 97,
        ATTITUDE_USEC = 98,

        RAW_PRESSURE_ABS = 99,
        RAW_PRESSURE_DIF_1 = 100,
        RAW_PRESSURE_DIF_2 = 101,
        RAW_PRESSURE_TEM = 102,

        NAVIGATION_H_C = 103,

        SENSOR_FLOAT1 = 104,
        SENSOR_FLOAT2 = 105,
        SENSOR_INT1 = 106,
        SENSOR_CHAR1 = 107,

        VOLT_TYPE = 108,
        VOLT_VOLTAGE = 109,
        VOLT_READING = 110,

        STATUS_GPS_FAILS = 111,
        STATUS_GPS_QUALITY = 112,
        STATUS_GPS_TYPE = 113,
        STATUS_GPS_STATUS = 114,
        STATUS_GPS_MVARIATION = 115,
        STATUS_GPS_MDIRECTION = 116,
        STATUS_GPS_MODE = 117,

        NOVATEL_TIME_STATUS = 118,
        NOVATEL_RECEIVER_STATUS = 119,
        NOVATEL_SOLUTION_STATUS = 120,
        NOVATEL_POSITION_TYPE = 121,
        NOVATEL_VELOCITY_TYPE = 122,
        NOVATEL_POSITION_AGE = 123,
        NOVATEL_FAILS = 124,

        PTZ_ZOOM = 125,
        PTZ_PAN = 126,
        PTZ_TILT = 127,

        RC_CHANNELS_RAW_5 = 128
                        };

    /**
     * @brief Enumeration SLUGS data information
     */
    enum UAVType
    {
        UAV = 0,
        SMALL_UAV = 1
                };

    /**
     * @brief This function return the airspeed.
     *
     * @return Airspeed
     */
    double getUM() const { return u_m; }
    /**
     * @brief This function return value m of equation line voltage.
     *
     * @return Value m voltage
     */
    static double getMV(){return mv;}
    /**
     * @brief This function return value b of equiation line voltage.
     *
     * @return Value b voltage
     */
    static double getBV(){return bv;}
    /**
     * @brief This function return value m of equiation line flow.
     *
     * @return Value m flow
     */
    static double getMC(){return mc;}
    /**
     * @brief This function return value b of equation line flow.
     *
     * @return Value b flow
     */
    static double getBC(){return bc;}

public slots:
    /**
     * @brief This function is called by MAVLink once a complete, uncorrupted (CRC check valid)
     * mavlink packet is received.
     *
     * @param link Hardware link the message came from (e.g. /dev/ttyUSB0 or UDP port).
     *             messages can be sent back to the system via this link
     * @param message MAVLink message, as received from the MAVLink protocol stack
     */
    void receiveMessage(LinkInterface* link, mavlink_message_t message);
    void setTypeUAV(UAVType type);
    float getUC(){return uCommand;}
    void setSystemType(int systemType);
    void updateState();
    int getUASID();

signals:
    void emitHeartBeat();
    void emitHeartBeatTimeOut();

protected:

    double u_m;
    //Equation line voltage UAV
    static const double mv = 0.0013021804;
    static const double bv = 8.738022002;

    //Equation line voltage Small UAV
    static const double smv = 0.0004182953;
    static const double sbv = 8.1086480834;

    //Equation line flow
    static const double mc = 0.0181553831;
    static const double bc = 90.4051033146;

    static const unsigned int timeoutIntervalHeartbeat = 2000 * 1000; ///< Heartbeat timeout is 1.5 seconds
    int type;

private:
    int uasId;    
    float hCommand, uCommand, rCommand;
    int countTimeGPS, minuteGPS, secondGPS, fromWP, countPTZ;
    bool outPTZ;
    UAVType typeUAV;
    mavlink_heartbeat_t mlHeartBeat;
    quint64 lastHeartbeat;      ///< Time of the last heartbeat message
};

#endif // SLUGSMAV_H
