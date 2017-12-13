import QtQuick 2.7
import QtQuick.Controls 2.1
import "../Custom"
import "../../Model/Robot"
import "../../Model/Tutorial/"

Page {

    id: page

    anchors.fill: parent

    property Robots robotModel
    property Tutorial tutorial
    property real batteryWarningThreshold
    property string langue

    signal closeMenu()

    MenuHeader {
        langue: page.langue
        id: settingsMenuHeader
        txt: langue == "English" ? "设置" : "Settings"
        onCloseMenu: page.closeMenu()
    }

    SettingsMenuContent {

        id: settings

        robotModel: page.robotModel
        tutorial: page.tutorial
        langue: page.langue

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

