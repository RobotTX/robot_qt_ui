import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQuick.Dialogs 1.2
import QtQuick.Controls.Styles 1.4
import QtQuick.Window 2.0
import "../../Helper/style.js" as Style


Frame {

    id: mapMenuFrame
    objectName: "mapMenuFrame"

    // two cases when a save file dialog is opened :
    // 1 - we just want to save the map
    // 2 - we actually want to import a map but we save this one first
    property bool haveToUploadAfterSaveMap: false

    signal savePosition()
    signal loadPosition()

    signal saveMap(string file_name)
    signal importMap(string file_name)

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
        txt: "Export the current Map"
        imgSrc: "qrc:/icons/save_map"
        anchors.top: scanButton.bottom
        onClicked: saveFileDialog.open()
    }

    MapMenuButton {
        id: loadButton
        txt: "Import an existing Map"
        imgSrc: "qrc:/icons/load_map"
        anchors.top: saveMapButton.bottom
        onClicked: messageDialog.open()
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
        id: savePositionButton
        txt: "Save the position of the map"
        imgSrc: "qrc:/icons/saveState"
        anchors.top: mergeButton.bottom
        onClicked: mapMenuFrame.savePosition()
    }

    MapMenuButton {
        id: loadPositionButton
        txt: "Reset the position of the map"
        imgSrc: "qrc:/icons/loadState"
        anchors.top: savePositionButton.bottom
        // the signal needs to be relayed in order to call the function that is in mapView
        onClicked: mapMenuFrame.loadPosition()
    }

    FileDialog {
        id: saveFileDialog
        // format of files is pgm
        nameFilters: "*.pgm"
        // won't let you choose a file name if selectExisting is true
        selectExisting: false
        title: "Please choose a location for your map"
        // to start directly with that folder selected
        folder: "/home/joan/Gobot/build-Gobot-Desktop_Qt_5_8_0_GCC_64bit-Debug/mapConfigs/"

        onAccepted: {
            // if an already existing file is selected we only send the url, if a file is being created we add the extension .pgm
            (fileUrl.toString().lastIndexOf(".pgm") == -1) ? mapMenuFrame.saveMap(fileUrl + ".pgm") : mapMenuFrame.saveMap(fileUrl);
            // depending on whether we try to upload a map or not we open the corresponding dialog
            (mapMenuFrame.haveToUploadAfterSaveMap) ? loadFileDialog.open() : console.log("NO need to open load dialog");
        }

        onRejected: {
            console.log("Canceled the save of a map")
        }
    }

    FileDialog {
        id: loadFileDialog
        // allow only pgm files to be selected
        nameFilters: "*.pgm"
        title: "Import a map"
        folder: "/home/joan/Gobot/build-Gobot-Desktop_Qt_5_8_0_GCC_64bit-Debug/mapConfigs/"
        selectMultiple: false
        onRejected: {
            console.log("Canceled the save of a map")
        }
        onAccepted: {
            console.log("gonna send file " << fileUrl);
            mapMenuFrame.importMap(fileUrl);
        }
    }

    MessageDialog {
        id: messageDialog
        text: "Do you wish to save the current map before importing a new one ?"
        informativeText: "          If you don't save the current map, your changes will be discarded"
        icon: StandardIcon.Question
        standardButtons: StandardButton.Cancel | StandardButton.Ignore | StandardButton.Yes
        onRejected: console.log("You canceled the import of a map");
        onAccepted: {
            loadFileDialog.open()
            console.log("You clicked ignore");
        }
        onYes: {
            console.log("You clicked yes");
            // to make sure the load file dialog is openened after the current has been saved
            mapMenuFrame.haveToUploadAfterSaveMap = true
            saveFileDialog.open()
        }
    }
}

