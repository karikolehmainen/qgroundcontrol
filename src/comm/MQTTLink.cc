#include "MQTTLink.h"
#include "QGCLoggingCategory.h"
#include "QGCApplication.h"
#include "LinkManager.h"

#include <QDebug>
#include <QFile>
#include <QMutexLocker>
#include <QTimer>

#include <string.h>

#include "FirmwarePlugin/PX4/px4_custom_mode.h"

QGC_LOGGING_CATEGORY(MQTTLinkLog, "MQTTLinkLog")
QGC_LOGGING_CATEGORY(MQTTLinkVerboseLog, "MQTTLinkVerboseLog")

// The LinkManager is only forward declared in the header, so a static_assert is here instead to ensure we update if the value changes.
static_assert(LinkManager::invalidMavlinkChannel() == std::numeric_limits<uint8_t>::max(), "update MQTTLink::_mavlinkAuxChannel");

MQTTLink::MQTTLink(SharedLinkConfigurationPtr& config)
    : LinkInterface                         (config)
//    , _name                                 ("MQTTLink")
//    , _connected                            (false)
    , _mqttConfig(qobject_cast<MQTTConfiguration*>(config.get()))
    , _socket(nullptr)
    , _socketIsConnected(false)
{
    qCDebug(MQTTLinkLog) << "MQTTLink" << this;

    _host     = _mqttConfig->host();
    _port     = _mqttConfig->port();
    _user     = _mqttConfig->user();
    _pass     = _mqttConfig->pass();
    _topic    = _mqttConfig->topic();

    //QObject::connect(this, &MQTTLink::writeBytesQueuedSignal, this, &MQTTLink::_writeBytesQueued, Qt::QueuedConnection);
    _socket = new QTcpSocket();
    QObject::connect(_socket, &QIODevice::readyRead, this, &MQTTLink::_readBytes);

    moveToThread(this);

    _loadParams();

    //_runningTime.start();
}

MQTTLink::~MQTTLink(void)
{
    disconnect();
    if (!_logDownloadFilename.isEmpty()) {
        QFile::remove(_logDownloadFilename);
    }
    qCDebug(MQTTLinkLog) << "~MQTTLink" << this;
}

bool MQTTLink::_connect(void)
{
    if (!_connected) {
        qCDebug(MQTTLinkLog) << "MQTTLink::_connect not yet connected";
        _connected = true;
        // MQTTLinks use Mavlink 2.0
        mavlink_status_t* mavlinkStatus = mavlink_get_channel_status(mavlinkChannel());
        mavlinkStatus->flags &= ~MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
        mavlink_status_t* auxStatus = mavlink_get_channel_status(mavlinkAuxChannel());
        auxStatus->flags &= ~MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
        start();
        qCDebug(MQTTLinkLog) << "MQTTLink::_connect now connected";
        emit connected();
    }

    return true;
}

bool MQTTLink::_allocateMavlinkChannel()
{
    // should only be called by the LinkManager during setup
    Q_ASSERT(!mavlinkAuxChannelIsSet());
    Q_ASSERT(!mavlinkChannelIsSet());

    if (!LinkInterface::_allocateMavlinkChannel()) {
        qCWarning(MQTTLinkLog) << "LinkInterface::_allocateMavlinkChannel failed";
        return false;
    }

    auto mgr = qgcApp()->toolbox()->linkManager();
    _mavlinkAuxChannel = mgr->allocateMavlinkChannel();
    if (!mavlinkAuxChannelIsSet()) {
        qCWarning(MQTTLinkLog) << "_allocateMavlinkChannel failed";
        LinkInterface::_freeMavlinkChannel();
        return false;
    }
    qCDebug(MQTTLinkLog) << "_allocateMavlinkChannel" << _mavlinkAuxChannel;
    return true;
}

void MQTTLink::_freeMavlinkChannel()
{
    qCDebug(MQTTLinkLog) << "_freeMavlinkChannel" << _mavlinkAuxChannel;
    if (!mavlinkAuxChannelIsSet()) {
        Q_ASSERT(!mavlinkChannelIsSet());
        return;
    }

    auto mgr = qgcApp()->toolbox()->linkManager();
    mgr->freeMavlinkChannel(_mavlinkAuxChannel);
    LinkInterface::_freeMavlinkChannel();
}

uint8_t MQTTLink::mavlinkAuxChannel(void) const
{
    return _mavlinkAuxChannel;
}

bool MQTTLink::mavlinkAuxChannelIsSet(void) const
{
    return (LinkManager::invalidMavlinkChannel() != _mavlinkAuxChannel);
}

void MQTTLink::disconnect(void)
{
    if (_connected) {
        _connected = false;
        quit();
        wait();
        emit disconnected();
    }
}

void MQTTLink::run(void)
{
    ;
}

void MQTTLink::_loadParams(void)
{
    QFile paramFile;

}

void MQTTLink::_readBytes()
{
    if (_socket) {
        qint64 byteCount = _socket->bytesAvailable();
        if (byteCount)
        {
            QByteArray buffer;
            buffer.resize(byteCount);
            _socket->read(buffer.data(), buffer.size());
            emit bytesReceived(this, buffer);
        }
    }
}

void MQTTLink::_sendHeartBeat(void)
{
    ;
}

void MQTTLink::_sendHighLatency2(void)
{
    ;
}

void MQTTLink::_sendSysStatus(void)
{
    ;
}

void MQTTLink::_sendBatteryStatus(void)
{
    ;
}

void MQTTLink::_sendVibration(void)
{
    ;
}

/// @brief Called when QGC wants to write bytes to the MAV
void MQTTLink::_writeBytes(const QByteArray bytes)
{
    // This prevents the responses to mavlink messages from being sent until the _writeBytes returns.
    emit writeBytesQueuedSignal(bytes);
}

void MQTTLink::_writeBytesQueued(const QByteArray bytes)
{
    if (_inNSH) {
        _handleIncomingNSHBytes(bytes.constData(), bytes.count());
    } else {
        if (bytes.startsWith(QByteArray("\r\r\r"))) {
            _inNSH  = true;
            _handleIncomingNSHBytes(&bytes.constData()[3], bytes.count() - 3);
        }

        _handleIncomingMavlinkBytes((uint8_t *)bytes.constData(), bytes.count());
    }
}

/// @brief Handle incoming bytes which are meant to be interpreted by the NuttX shell
void MQTTLink::_handleIncomingNSHBytes(const char* bytes, int cBytes)
{
}

static bool is_ip(const QString& address)
{
    int a,b,c,d;
    if (sscanf(address.toStdString().c_str(), "%d.%d.%d.%d", &a, &b, &c, &d) != 4
            && strcmp("::1", address.toStdString().c_str())) {
        return false;
    } else {
        return true;
    }
}

static QString get_ip_address(const QString& address)
{
    if(is_ip(address))
        return address;
    // Need to look it up
    QHostInfo info = QHostInfo::fromName(address);
    if (info.error() == QHostInfo::NoError)
    {
        QList<QHostAddress> hostAddresses = info.addresses();
        for (int i = 0; i < hostAddresses.size(); i++)
        {
            // Exclude all IPv6 addresses
            if (!hostAddresses.at(i).toString().contains(":"))
            {
                return hostAddresses.at(i).toString();
            }
        }
    }
    return {};
}

/// @brief Handle incoming bytes which are meant to be handled by the mavlink protocol
void MQTTLink::_handleIncomingMavlinkBytes(const uint8_t* bytes, int cBytes)
{
    QMutexLocker lock{&_mavlinkAuxMutex};
    mavlink_message_t msg;
    mavlink_status_t comm;
    memset(&comm, 0, sizeof(comm));
    int parsed = 0;

    for (qint64 i=0; i<cBytes; i++)
    {
        parsed = mavlink_parse_char(mavlinkAuxChannel(), bytes[i], &msg, &comm);
        if (!parsed) {
            continue;
        }
        lock.unlock();
        //_handleIncomingMavlinkMsg(msg);
        lock.relock();
    }
}

MQTTConfiguration::MQTTConfiguration(const QString& name)
    : LinkConfiguration(name)
{
    _port    = QGC_MQTT_PORT;
    _address = QHostAddress::Any;
    _user          = "";
    _pass          = "";
    _topic          = "";
}

MQTTConfiguration::MQTTConfiguration(MQTTConfiguration* source)
    : LinkConfiguration(source)
{
    _address       = source->_address;
    _port          = source->_port;
    _user          = source->_user;
    _pass          = source->_pass;
    _topic         = source->_topic;
}

void MQTTConfiguration::copyFrom(LinkConfiguration *source)
{
    LinkConfiguration::copyFrom(source);
    auto* usource = qobject_cast<MQTTConfiguration*>(source);

    if (!usource) {
        qCWarning(MQTTLinkLog) << "dynamic_cast failed" << source << usource;
        return;
    }

    _address       = usource->_address;
    _port          = usource->_port;
    _user          = usource->_user;
    _pass          = usource->_pass;
    _topic          = usource->_topic;
}

void MQTTConfiguration::saveSettings(QSettings& settings, const QString& root)
{
    settings.beginGroup(root);
    settings.setValue("port", (int)_port);
    settings.setValue("host", address().toString());
    settings.setValue("user", _user);
    settings.setValue("pass", _pass);
    settings.setValue("topic", _topic);
    settings.sync();
    settings.endGroup();
}

void MQTTConfiguration::loadSettings(QSettings& settings, const QString& root)
{
    settings.beginGroup(root);
    _port = (quint16)settings.value("port", QGC_MQTT_PORT).toUInt();
    QString address = settings.value("host", _address.toString()).toString();
    _address = QHostAddress(address);
    _user = settings.value("user", "").toString();
    _pass = settings.value("pass", "").toString();
    _topic = settings.value("topic", "").toString();
    settings.endGroup();
}
void MQTTConfiguration::setPort(quint16 port)
{
    _port = port;
}

void MQTTConfiguration::setAddress(const QHostAddress& address)
{
    _address = address;
}

void MQTTConfiguration::setHost(const QString host)
{
    QString ipAdd = get_ip_address(host);
    if(ipAdd.isEmpty()) {
        qWarning() << "TCP:" << "Could not resolve host:" << host;
    } else {
        _address = QHostAddress(ipAdd);
    }
}

void MQTTConfiguration::setUser(const QString user)
{
    _user = user;
}

void MQTTConfiguration::setPass(const QString pass)
{
    _pass = pass;
}

void MQTTConfiguration::setTopic(const QString topic)
{
    _topic = topic;
}
