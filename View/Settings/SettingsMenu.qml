import QtQuick 2.7
import QtQuick.Controls 2.1
import "../Custom"
import "../../Model/Robot"

Page {

    id: page

    anchors.fill: parent

    property Robots robotModel
    property real batteryWarningThreshold

    signal closeMenu()

    MenuHeader {
        id: settingsMenuHeader
        txt: "Settings"
        onCloseMenu: page.closeMenu()
    }

    SettingsMenuContent {

        id: settings

        robotModel: page.robotModel
        oriBatteryWarningThreshold: page.batteryWarningThreshold

        anchors {
            left: parent.left
            top: settingsMenuHeader.bottom
            right: parent.right
            bottom: parent.bottom
        }

        onClose: page.closeMenu()
    }
}

