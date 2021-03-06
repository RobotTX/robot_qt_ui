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
    property string langue

    signal savePosition()
    signal loadPosition()
    signal centerMap()

    signal saveMap(string file_name)
    signal importMap(string file_name)

    /// test mp3
    signal importMP3(string file_name)

    padding: 0

    background: Rectangle {
        color: Style.lightGreyBackground
        border.color: Style.lightGreyBorder
        border.width: 1
    }

    NormalButton {
        id: scanButton
        txt: langue == "English" ? "Scan New Map" : "扫描新地图"
        imgSrc: "qrc:/icons/scan_map"
        anchors.top: parent.top
        anchors.topMargin: 12
        onClicked: scanMap.show();
    }

    NormalButton {
        id: saveMapButton
        txt: langue == "English" ? "Export Map" : "导出地图到本地"
        imgSrc: "qrc:/icons/save_map"
        anchors.top: scanButton.bottom
        onClicked: saveFileDialog.open()
    }

    NormalButton {
        id: loadButton
        txt: langue == "English" ? "Import Map" : "从本地导入地图"
        imgSrc: "qrc:/icons/load_map"
        anchors.top: saveMapButton.bottom
        onClicked: messageDialog.open()
    }

    NormalButton {
        id: editButton
        txt: langue == "English" ? "Edit Current Map" : "编辑当前地图"
        imgSrc: "qrc:/icons/edit_map"
        anchors.top: loadButton.bottom
        onClicked: editMap.show();

    }

    NormalButton {
        id: savePositionButton
        txt: langue == "English" ? "Save Map Current Position" : "保存地图当前位置"
        imgSrc: "qrc:/icons/saveState"
        anchors.top: editButton.bottom
        onClicked: mapMenuFrame.savePosition()
    }

    NormalButton {
        id: loadPositionButton
        txt: langue == "English" ? "Locate Map Saved Position" : "定位地图保存位置"
        imgSrc: "qrc:/icons/loadState"
        anchors.top: savePositionButton.bottom
        // the signal needs to be relayed in order to call the function that is in mapView
        onClicked: mapMenuFrame.loadPosition()
    }

    NormalButton {
        id: recenterButton
        txt: langue == "English" ? "Map Center Position" : "地图中心位置"
        imgSrc: "qrc:/icons/centerMap"
        anchors.top: loadPositionButton.bottom
        // if the map has become unreachable for the user because of a false manipulation
        // (typically because he dragged the map out of the frame and saved this Positioner
        // then this button can be used to recenter the map
        onClicked: mapMenuFrame.centerMap()
    }

//    NormalButton {
//        id: loadMp3
//        txt: langue == "English" ? "地图中心位置" : "Load Audio"
//        imgSrc: "qrc:/icons/centerMap"
//        anchors.top: recenterButton.bottom
//        // if the map has become unreachable for the user because of a false manipulation
//        // (typically because he dragged the map out of the frame and saved this Positioner
//        // then this button can be used to recenter the map
//        onClicked: mapMenuFrame.importMP3("/home/oogyvai/Downloads/shape_of_you.mp3");
//    }

    FileDialog {
        id: saveFileDialog
        // format of files is pgm
        nameFilters: "*.pgm"
        // won't let you choose a file name if selectExisting is true
        selectExisting: false
        title: langue == "English" ? "Please choose a location for your map" : "请选择一个路径"

        onAccepted: {
            var fileStr = fileUrl.toString();
            if (fileStr.indexOf(" ") >= 0) {
                console.log("space in the name not authorized");
                warningDialog.open()
            } else {
            // TODO need to check for mac
                /// file:// for linux, file: for windows
                if(fileStr.indexOf("file://") === 0) {
                    fileStr = fileStr.slice(7);
                } else if(fileStr.indexOf("file:") === 0) {
                    fileStr = fileStr.slice(5);
                }

                console.log("fileStr in mapMenuContent while saving map = " + fileStr);

                // if an already existing file is selected we only send the url, if a file is being created we add the extension .pgm
                if(fileStr.lastIndexOf(".pgm") == -1)
                    mapMenuFrame.saveMap(fileStr + ".pgm")
                else
                    mapMenuFrame.saveMap(fileStr);

                // depending on whether we try to upload a map or not we open the corresponding dialog
                if(mapMenuFrame.haveToUploadAfterSaveMap)
                    loadFileDialog.open()
                else {

                }
            }
        }
        onRejected: {
            console.log("Canceled the save of a map")
        }
    }

    CustomDialog {
        id: warningDialog
        parent: ApplicationWindow.overlay
        x: (mapMenuFrame.width - width) / 2
        y: (mapMenuFrame.height - height) / 2
        height: 60

        title: langue == "English" ? "Warning dialog" : "警告窗口"
        acceptMessage: langue == "English" ? "\nSpace are not allowed" : "\nn地图名称不能包含空格"

    }

    FileDialog {
        id: loadFileDialog
        // allow only pgm files to be selected
        nameFilters: "*.pgm"
        title: langue == "English" ? "Import a map" : "导入地图"
//        selectMultiple: false
        onRejected: {
//            console.log("Canceled the save of a map")
        }
        onAccepted: {
            console.log("gonna send file " + fileUrl);
            mapMenuFrame.importMap(fileUrl);
        }
    }

    CustomDialog {
        id: messageDialog
        font.pointSize: Style.ubuntuSubHeadingSize
        title: langue == "English" ? "Importing an existing map" : "导入已经存在的地图"
        message: langue == "English" ? "\nDo you wish to save the current map before importing a new one ?" : "\n在读取新地图之前，是否保存当前地图 ?"
        rejectMessage: langue == "English" ? "Cancel" : "取消"
        acceptMessage: langue == "English" ? "Load" : "读取"
        yesMessage: langue == "English" ? "Save & Load" : "保存&读取"
        height: 170

//        x: mapMenuFrame.width
//        y: mapMenuFrame.height / 2 - height / 2;// - Style.menuHeaderHeight

        parent: ApplicationWindow.overlay
        x: (parent.width - width) / 2
        y: (parent.height - height) / 2

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
