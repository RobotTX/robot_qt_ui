import QtQuick 2.7
import QtQuick.Controls 2.1
import "../Custom"

Page {
    id: page
    anchors.fill: parent

    signal closeMenu()
    signal saveState()

    signal saveMap(string file_name)

    MenuHeader {
        id: mapMenuHeader
        objectName: "mapMenuHeader"
        txt: "Map"
        onCloseMenu: page.closeMenu()
    }

    MapMenuContent {
        anchors {
            left: parent.left
            top: mapMenuHeader.bottom
            right: parent.right
            bottom: parent.bottom    
        }

        onSaveState: page.saveState()
        onSaveMap: page.saveMap(file_name)
    }
}

