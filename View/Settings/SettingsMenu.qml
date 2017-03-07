import QtQuick 2.7
import QtQuick.Controls 2.0
import "../Custom"

Page {
    id: page
    readonly property int index: 4
    anchors.fill: parent

    MenuHeader {
        id: settingsMenuHeader
        objectName: "settingsMenuHeader"
        txt: "Settings"
    }

    SettingsMenuContent {
        anchors {
            left: parent.left
            top: settingsMenuHeader.bottom
            right: parent.right
            bottom: parent.bottom
        }
    }
}

