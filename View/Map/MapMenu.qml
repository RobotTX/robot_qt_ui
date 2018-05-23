import QtQuick 2.7
import QtQuick.Controls 2.1
import "../Custom" // MenuHeader

Page {

    id: page
    anchors.fill: parent

    property string langue
    signal closeMenu()
    signal savePosition()

    signal saveMap(string file_name)

    MenuHeader {
        langue: page.langue
        id: mapMenuHeader
        txt: langue == "English" ? "地图" :  "Map"
        onCloseMenu: page.closeMenu()
    }

    MapMenuContent {
        anchors {
            left: parent.left
            top: mapMenuHeader.bottom
            right: parent.right
            bottom: parent.bottom    
        }
        langue: page.langue
        onSavePosition: page.savePosition()
        // originally comes from saveButton being clicked
        onSaveMap: {
//            console.log("saving map " + file_name)
            page.saveMap(file_name)
        }
    }
}

