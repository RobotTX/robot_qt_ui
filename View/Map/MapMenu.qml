import QtQuick 2.7
import QtQuick.Controls 2.1
import "../Custom" // MenuHeader

Page {

    id: page
    anchors.fill: parent

    signal closeMenu()
    signal savePosition()

    signal saveMap(string file_name)

    MenuHeader {
        id: mapMenuHeader
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

        onSavePosition: page.savePosition()
        // originally comes from saveButton being clicked
        onSaveMap: {
            console.log("saving map " + file_name)
            page.saveMap(file_name)
        }
    }
}

