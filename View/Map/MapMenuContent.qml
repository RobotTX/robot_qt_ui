import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQuick.Dialogs 1.0
import "../../Helper/style.js" as Style

Frame {
    id: mapMenuFrame
    objectName: "mapMenuFrame"

    signal saveState()
    signal loadState()

    signal saveMap(string file_name)

    padding: 0

    background: Rectangle {
        color: Style.lightGreyBackground
        border.color: Style.lightGreyBorder
        border.width: 1
    }

    MapMenuButton {
        id: scanButton
        txt: "Scan a Map"
        imgSrc: "qrc:/icons/scan_map"
        anchors.top: parent.top
        anchors.topMargin: 12
    }

    MapMenuButton {
        id: saveMapButton
        txt: "Save the Map"
        imgSrc: "qrc:/icons/save_map"
        anchors.top: scanButton.bottom
        onClicked: fileDialog.open()
    }

    MapMenuButton {
        id: loadButton
        txt: "Load a Map"
        imgSrc: "qrc:/icons/load_map"
        anchors.top: saveMapButton.bottom
    }

    MapMenuButton {
        id: editButton
        txt: "Edit the Map"
        imgSrc: "qrc:/icons/edit_map"
        anchors.top: loadButton.bottom
    }

    MapMenuButton {
        id: mergeButton
        txt: "Merge Maps"
        imgSrc: "qrc:/icons/merge_map"
        anchors.top: editButton.bottom
    }

    MapMenuButton {
        id: saveStateButton
        txt: "Save the State of the map"
        imgSrc: "qrc:/icons/saveState"
        anchors.top: mergeButton.bottom
        onClicked: mapMenuFrame.saveState()
    }

    MapMenuButton {
        id: loadStateButton
        txt: "Load the State of the map"
        imgSrc: "qrc:/icons/loadState"
        anchors.top: saveStateButton.bottom
        // the signal needs to be relayed in order to call the function that is in mapView
        onClicked: mapMenuFrame.loadState()
    }

    FileDialog {
        id: fileDialog
        // won't let you choose a file name if selectExisting is true
        selectExisting: false
        title: "Please choose a location for your map"
        folder: "/home/joan/Gobot/build-Gobot-Desktop_Qt_5_8_0_GCC_64bit-Debug/Maps/"

        onAccepted: {
            mapMenuFrame.saveMap(fileDialog.fileUrls)
        }

        onRejected: {
            console.log("Canceled the save of a map")
        }
    }
}

