/****************************************************************************
 *
 * (c) 2009-2020 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory. MQTT setting UI layout
 *
 ****************************************************************************/

import QtQuick          2.3
import QtQuick.Controls 1.2
import QtQuick.Layouts  1.2

import QGroundControl               1.0
import QGroundControl.Controls      1.0
import QGroundControl.ScreenTools   1.0
import QGroundControl.Palette       1.0

GridLayout {
    columns:        2
    rowSpacing:     _rowSpacing
    columnSpacing:  _colSpacing

    function saveSettings() {
        subEditConfig.host = hostField.text
        subEditConfig.port = portField.text
        subEditConfig.user = userField.text
        subEditConfig.pass = passField.text
        subEditConfig.topic = topicField.text
    }

    QGCLabel { text: qsTr("Server Address") }
    QGCTextField {
        id:                     hostField
        Layout.preferredWidth:  _secondColumnWidth
        text:                   subEditConfig.host
    }

    QGCLabel { text: qsTr("Port") }
    QGCTextField {
        id:                     portField
        Layout.preferredWidth:  _secondColumnWidth
        text:                   subEditConfig.port
        inputMethodHints:       Qt.ImhFormattedNumbersOnly
    }

    QGCLabel { text: qsTr("Username") }
    QGCTextField {
        id:                     userField
        Layout.preferredWidth:  _secondColumnWidth
        text:                   subEditConfig.user
    }

    QGCLabel { text: qsTr("Password") }
    QGCTextField {
        id:                     passField
        Layout.preferredWidth:  _secondColumnWidth
        text:                   subEditConfig.pass
    }
    
    QGCLabel { text: qsTr("Fleet ID") }
    QGCTextField {
        id:                     topicField
        Layout.preferredWidth:  _secondColumnWidth
        text:                   subEditConfig.topic
    }
}
