import QtQuick 2.7
import QtQuick.Controls 2.1
import "../Custom"

Page {
    id: page
    anchors.fill: parent
    signal closeMenu()

    MenuHeader {
        id: settingsMenuHeader
        txt: "Settings"
        onCloseMenu: page.closeMenu()
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

