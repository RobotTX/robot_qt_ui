import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQuick.Dialogs 1.2
import QtQuick.Controls.Styles 1.4
import QtQuick.Window 2.0
import "../../Helper/style.js" as Style
import "../Custom"

Frame {

    id: mapMenuFrame
    objectName: "mapMenuFrame"

    // two cases when a save file dialog is opened :
    // 1 - we just want to save the map
    // 2 - we actually want to import a map but we save this one first
    property bool haveToUploadAfterSaveMap: false

    signal savePosition()
    signal loadPosition()
    signal centerMap()

    signal saveMap(string file_name)
    signal importMap(string file_name)

    padding: 0

    background: Rectangle {
        color: Style.lightGreyBackground
        border.color: Style.lightGreyBorder
        border.width: 1
    }

    NormalButton {
        id: scanButton
        txt: "Scan a Map"
        imgSrc: "qrc:/icons/scan_map"
        anchors.top: parent.top
        anchors.topMargin: 12
        onClicked: scanMap.show();
    }

    NormalButton {
        id: saveMapButton
        txt: "Export the current Map"
        imgSrc: "qrc:/icons/save_map"
        anchors.top: scanButton.bottom
        onClicked: saveFileDialog.open()
    }

    NormalButton {
        id: loadButton
        txt: "Import an existing Map"
        imgSrc: "qrc:/icons/load_map"
        anchors.top: saveMapButton.bottom
        onClicked: messageDialog.open()
    }

    NormalButton {
        id: editButton
        txt: "Edit the Map"
        imgSrc: "qrc:/icons/edit_map"
        anchors.top: loadButton.bottom
        onClicked: editMap.show();

    }

    NormalButton {
        id: mergeButton
        txt: "Merge Maps"
        imgSrc: "qrc:/icons/merge_map"
        anchors.top: editButton.bottom
        onClicked: mergeMap.show();
    }

    NormalButton {
        id: savePositionButton
        txt: "Save the position of the map"
        imgSrc: "qrc:/icons/saveState"
        anchors.top: mergeButton.bottom
        onClicked: mapMenuFrame.savePosition()
    }

    NormalButton {
        id: loadPositionButton
        txt: "Reset the position of the map"
        imgSrc: "qrc:/icons/loadState"
        anchors.top: savePositionButton.bottom
        // the signal needs to be relayed in order to call the function that is in mapView
        onClicked: mapMenuFrame.loadPosition()
    }

    NormalButton {
        id: recenterButton
        txt: "Center map"
        imgSrc: "qrc:/icons/centerMap"
        anchors.top: loadPositionButton.bottom
        onClicked: mapMenuFrame.centerMap()
    }

    FileDialog {
        id: saveFileDialog
        // format of files is pgm
        nameFilters: "*.pgm"
        // won't let you choose a file name if selectExisting is true
        selectExisting: false
        title: "Please choose a location for your map"
        // to start directly with that folder selected
        /// TODO pk tu fais des trucs comme ca
        //folder: Qt.resolvedUrl(".").substring(7)
        folder: "/home/joan/Gobot/build-Gobot-Desktop_Qt_5_8_0_GCC_64bit-Debug/mapConfigs/"

        onAccepted: {
            var fileStr = fileUrl.toString();
            console.log("Accepted the save of a map " + fileStr + " " + fileStr.indexOf("file://"));
            if(fileStr.indexOf("file://") === 0)
                fileStr = fileStr.slice(7);

            // if an already existing file is selected we only send the url, if a file is being created we add the extension .pgm
            if(fileStr.lastIndexOf(".pgm") == -1)
                mapMenuFrame.saveMap(fileStr + ".pgm")
            else
                mapMenuFrame.saveMap(fileStr);

            // depending on whether we try to upload a map or not we open the corresponding dialog
            if(mapMenuFrame.haveToUploadAfterSaveMap)
                loadFileDialog.open()
            else
                console.log("NO need to open load dialog");
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
        /// TODO pk tu fais des trucs comme ca
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

    CustomDialog {
        id: messageDialog
        title: "Importing an existing map"
        message: "Do you wish to save the current map before importing a new one ?\n\n\tIf you don't save the current map, your changes will be discarded"
        rejectMessage: "Cancel"
        acceptMessage: "Load"
        yesMessage: "Save & Load"
        height: 170

        x: applicationWindow.width / 2 - width / 2 - Style.mainMenuWidth
        y: applicationWindow.height / 2 - height / 2 - Style.menuHeaderHeight

        onRejected: console.log("You canceled the import of a map");
        onAccepted: {
            loadFileDialog.open()
        }
        onYes: {
            // to make sure the load file dialog is openened after the current has been saved
            mapMenuFrame.haveToUploadAfterSaveMap = true
            saveFileDialog.open()
        }
    }
}
