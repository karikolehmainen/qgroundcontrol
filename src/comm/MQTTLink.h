#ifndef MQTTLINK_H
#define MQTTLINK_H

#include <QElapsedTimer>
#include <QGeoCoordinate>
#include <QLoggingCategory>
#include <QMap>
#include <QMutex>
#include <QMetaType>
#include <QTcpSocket>
#include <QHostAddress>
//#include <QtMqtt/QtMqtt>

//#include "MQTTLinkMissionItemHandler.h"
#include "MQTTLinkTCP.h"
#include "QGCMAVLink.h"
#include "QGCLoggingCategory.h"
#include "MAVLinkProtocol.h"

#define QGC_MQTT_PORT 1883

Q_DECLARE_LOGGING_CATEGORY(MQTTLinkLog)
Q_DECLARE_LOGGING_CATEGORY(MQTTLinkVerboseLog)



class LinkManager;

class MQTTConfiguration : public LinkConfiguration
{
    Q_OBJECT

public:
    MQTTConfiguration(const QString& name);
    MQTTConfiguration(MQTTConfiguration* source);

    Q_PROPERTY(quint16 port READ port WRITE setPort NOTIFY portChanged)
    Q_PROPERTY(QString host READ host WRITE setHost NOTIFY hostChanged)
    Q_PROPERTY(QString user READ user WRITE setUser NOTIFY userChanged)
    Q_PROPERTY(QString pass READ pass WRITE setPass NOTIFY passChanged)
    Q_PROPERTY(QString topic READ topic WRITE setTopic NOTIFY topicChanged)

    quint16             port        (void) const                    { return _port; }
    const QHostAddress& address     (void)                          { return _address; }
    const QString       host        (void)                          { return _address.toString(); }
    const QString       user        (void)                          { return _user; }
    const QString       pass        (void)                          { return _pass; }
    const QString       topic       (void)                          { return _topic; }
    void                setPort     (quint16 port);
    void                setAddress  (const QHostAddress& address);
    void                setHost     (const QString host);
    void                setUser     (const QString user);
    void                setPass     (const QString pass);
    void                setTopic    (const QString topic);

    // Overrides from LinkConfiguration
    LinkType    type            (void) override                                         { return LinkConfiguration::TypeMQTT; }
    void        copyFrom        (LinkConfiguration* source) override;
    void        loadSettings    (QSettings& settings, const QString& root) override;
    void        saveSettings    (QSettings& settings, const QString& root) override;
    QString     settingsURL     (void) override                                         { return "MQTTSettings.qml"; }
    QString     settingsTitle   (void) override                                         { return tr("MQTT Link Settings"); }

signals:
    void portChanged(void);
    void hostChanged(void);
    void userChanged(void);
    void passChanged(void);
    void topicChanged(void);

private:
    QHostAddress    _address;
    quint16         _port;
    QString         _user;
    QString         _pass;
    QString         _topic;
};

class MQTTLink : public LinkInterface
{
    Q_OBJECT

public:
    MQTTLink(SharedLinkConfigurationPtr& config);
    virtual ~MQTTLink();


    //MQTTLinkTCP* mqttLinkTCP(void) { return _mqttLinkTCP; }

    // Overrides from LinkInterface
    bool isConnected(void) const override { return _connected; }
    void disconnect (void) override;

    // Kari: REmove these MissionItemHandlers as they maybe are for testing only
    /// Sets a failure mode for unit testingqgcm
    ///     @param failureMode Type of failure to simulate
    ///     @param failureAckResult Error to send if one the ack error modes
    //void setMissionItemFailureMode(MQTTLinkMissionItemHandler::FailureMode_t failureMode, MAV_MISSION_RESULT failureAckResult);

    /// Called to send a MISSION_ACK message while the MissionManager is in idle state
    //void sendUnexpectedMissionAck(MAV_MISSION_RESULT ackType) { _missionItemHandler.sendUnexpectedMissionAck(ackType); }

    /// Called to send a MISSION_ITEM message while the MissionManager is in idle state
    //void sendUnexpectedMissionItem(void) { _missionItemHandler.sendUnexpectedMissionItem(); }

    /// Called to send a MISSION_REQUEST message while the MissionManager is in idle state
    //void sendUnexpectedMissionRequest(void) { _missionItemHandler.sendUnexpectedMissionRequest(); }

    //void sendUnexpectedCommandAck(MAV_CMD command, MAV_RESULT ackResult);

    /// Reset the state of the MissionItemHandler to no items, no transactions in progress.
    //void resetMissionItemHandler(void) { _missionItemHandler.reset(); }

    /// Returns the filename for the simulated log file. Only available after a download is requested.
    //QString logDownloadFile(void) { return _logDownloadFilename; }

    // Special commands for testing COMMAND_LONG handlers. By default all commands except for MAV_CMD_MOCKLINK_NO_RESPONSE_NO_RETRY should retry.
    static constexpr MAV_CMD MAV_CMD_MQTTLINK_ALWAYS_RESULT_ACCEPTED            = MAV_CMD_USER_1;
    static constexpr MAV_CMD MAV_CMD_MQTTLINK_ALWAYS_RESULT_FAILED              = MAV_CMD_USER_2;
    static constexpr MAV_CMD MAV_CMD_MQTTLINK_SECOND_ATTEMPT_RESULT_ACCEPTED    = MAV_CMD_USER_3;
    static constexpr MAV_CMD MAV_CMD_MQTTLINK_SECOND_ATTEMPT_RESULT_FAILED      = MAV_CMD_USER_4;
    static constexpr MAV_CMD MAV_CMD_MQTTLINK_NO_RESPONSE                       = MAV_CMD_USER_5;
    static constexpr MAV_CMD MAV_CMD_MQTTLINK_NO_RESPONSE_NO_RETRY              = static_cast<MAV_CMD>(MAV_CMD_USER_5 + 1);

    void clearSendMavCommandCounts(void) { _sendMavCommandCountMap.clear(); }
    int sendMavCommandCount(MAV_CMD command) { return _sendMavCommandCountMap[command]; }

    typedef enum {
        FailRequestMessageNone,
        FailRequestMessageCommandAcceptedMsgNotSent,
        FailRequestMessageCommandUnsupported,
        FailRequestMessageCommandNoResponse,
    } RequestMessageFailureMode_t;
    void setRequestMessageFailureMode(RequestMessageFailureMode_t failureMode) { _requestMessageFailureMode = failureMode; }

signals:
    void writeBytesQueuedSignal                 (const QByteArray bytes);
    void highLatencyTransmissionEnabledChanged  (bool highLatencyTransmissionEnabled);

private slots:
    void _readBytes     (void);

private slots:
    // LinkInterface overrides
    void _writeBytes(const QByteArray bytes) final;

    void _writeBytesQueued(const QByteArray bytes);

private:
    // LinkInterface overrides
    bool _connect                       (void) override;
    bool _allocateMavlinkChannel        () override;
    void _freeMavlinkChannel            () override;
    uint8_t mavlinkAuxChannel           (void) const;
    bool mavlinkAuxChannelIsSet         (void) const;

    // QThread override
    void run(void) final;

    // MockLink methods
    void _sendHeartBeat                 (void);
    void _sendHighLatency2              (void);
    void _handleIncomingNSHBytes        (const char* bytes, int cBytes);
    void _handleIncomingMavlinkBytes    (const uint8_t* bytes, int cBytes);
    void _handleIncomingMavlinkMsg      (const mavlink_message_t& msg);
    void _loadParams                    (void);
    void _handleHeartBeat               (const mavlink_message_t& msg);
    void _handleSetMode                 (const mavlink_message_t& msg);
    void _handleParamRequestList        (const mavlink_message_t& msg);
    void _handleParamSet                (const mavlink_message_t& msg);
    void _handleParamRequestRead        (const mavlink_message_t& msg);
    void _handleFTP                     (const mavlink_message_t& msg);
    void _handleCommandLong             (const mavlink_message_t& msg);
    void _handleManualControl           (const mavlink_message_t& msg);
    void _handlePreFlightCalibration    (const mavlink_command_long_t& request);
    void _handleLogRequestList          (const mavlink_message_t& msg);
    void _handleLogRequestData          (const mavlink_message_t& msg);
    void _handleParamMapRC              (const mavlink_message_t& msg);
    bool _handleRequestMessage          (const mavlink_command_long_t& request, bool& noAck);
    float _floatUnionForParam           (int componentId, const QString& paramName);
    void _setParamFloatUnionIntoMap     (int componentId, const QString& paramName, float paramFloat);
    void _sendHomePosition              (void);
    void _sendGpsRawInt                 (void);
    void _sendVibration                 (void);
    void _sendSysStatus                 (void);
    void _sendBatteryStatus             (void);
    void _sendStatusTextMessages        (void);
    void _sendChunkedStatusText         (uint16_t chunkId, bool missingChunks);
    void _respondWithAutopilotVersion   (void);
    void _sendRCChannels                (void);
    void _paramRequestListWorker        (void);
    void _logDownloadWorker             (void);
    void _sendADSBVehicles              (void);
    void _moveADSBVehicle               (void);
    void _sendGeneralMetaData           (void);

    static MQTTLink* _startMQTTLinkWorker(QString configName, MAV_AUTOPILOT firmwareType, MAV_TYPE vehicleType, bool sendStatusText);
    static MQTTLink* _startMQTTLink(MQTTConfiguration* mockConfig);

    uint8_t                     _mavlinkAuxChannel              = std::numeric_limits<uint8_t>::max();
    QMutex                      _mavlinkAuxMutex;

    QString                     _host;
    //MQTTLinkMissionItemHandler  _missionItemHandler;
    MQTTConfiguration* _mqttConfig;
    QTcpSocket*       _socket;
    bool              _socketIsConnected;


    QString           _name;
    QString           _user;
    QString           _pass;
    quint16           _port;
    QString           _topic;
    bool              _connected;
    bool              _inNSH;

/*    uint8_t                     _vehicleSystemId;
    uint8_t                     _vehicleComponentId             = MAV_COMP_ID_AUTOPILOT1;

    bool                        _inNSH;
    bool                        _mavlinkStarted;

    uint8_t                     _mavBaseMode;
    uint32_t                    _mavCustomMode;
    uint8_t                     _mavState;

    QElapsedTimer               _runningTime;
    static const int32_t        _batteryMaxTimeRemaining        = 15 * 60;
    int8_t                      _battery1PctRemaining           = 100;
    int32_t                     _battery1TimeRemaining          = _batteryMaxTimeRemaining;
    MAV_BATTERY_CHARGE_STATE    _battery1ChargeState            = MAV_BATTERY_CHARGE_STATE_OK;
    int8_t                      _battery2PctRemaining           = 100;
    int32_t                     _battery2TimeRemaining          = _batteryMaxTimeRemaining;
    MAV_BATTERY_CHARGE_STATE    _battery2ChargeState            = MAV_BATTERY_CHARGE_STATE_OK;

    MAV_AUTOPILOT               _firmwareType;
    MAV_TYPE                    _vehicleType;
    double                      _vehicleLatitude;
    double                      _vehicleLongitude;
    double                      _vehicleAltitude;
    bool                        _commLost                       = false;
    bool                        _highLatencyTransmissionEnabled = true;

    // These are just set for reporting the fields in _respondWithAutopilotVersion()
    // and ensuring that the Vehicle reports the fields in Vehicle::firmwareBoardVendorId etc.
    // They do not control any mock simulation (and it is up to the Custom build to do that).
    uint16_t                    _boardVendorId      = 0;
    uint16_t                    _boardProductId     = 0;

    MQTTLinkTCP* _mqttLinkTCP = nullptr;

    bool _sendStatusText;
    bool _apmSendHomePositionOnEmptyList;
*/

    int _sendHomePositionDelayCount;
    int _sendGPSPositionDelayCount;

    int _currentParamRequestListComponentIndex; // Current component index for param request list workflow, -1 for no request in progress
    int _currentParamRequestListParamIndex;     // Current parameter index for param request list workflow

    //static const uint16_t _logDownloadLogId = 0;        ///< Id of siumulated log file
    //static const uint32_t _logDownloadFileSize = 1000;  ///< Size of simulated log file

    QString     _logDownloadFilename;       ///< Filename for log download which is in progress
    uint32_t    _logDownloadCurrentOffset;  ///< Current offset we are sending from
    uint32_t    _logDownloadBytesRemaining; ///< Number of bytes still to send, 0 = send inactive

    QGeoCoordinate  _adsbVehicleCoordinate;
    double          _adsbAngle;

    RequestMessageFailureMode_t _requestMessageFailureMode = FailRequestMessageNone;

    QMap<MAV_CMD, int>  _sendMavCommandCountMap;
    QMap<int, QMap<QString, QVariant>>          _mapParamName2Value;
    QMap<int, QMap<QString, MAV_PARAM_TYPE>>    _mapParamName2MavParamType;

    //static double       _defaultVehicleLatitude;
    //static double       _defaultVehicleLongitude;
    //static double       _defaultVehicleAltitude;
    //static int          _nextVehicleSystemId;
    static const char*  _failParam;
};
#endif // MQTTLINK_H
