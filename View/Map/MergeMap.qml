import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQuick.Window 2.2
import QtQuick.Layouts 1.3
import QtQuick.Dialogs 1.2
import "../../Helper/style.js" as Style
import "../../Model/Robot/"
import "../Robot/"

Window {

    id: window
    title: "Merge maps"
    objectName: "mergeMapWindow"

    // no need to do it twice, on the hideEvent or the showEvent, both would work, here we clean the map on the hideEvent
    onVisibleChanged: {
        _mapsList.clear();
        if(!visible) resetWidget();
    }

    width: 1000
    minimumWidth: 800
    height: 700
    minimumHeight: 600

    signal importMap(string file)
    signal exportMap(string file)
    signal resetWidget()
    signal getMapFromRobot(string ip)

    property Robots robotModel

    Frame {

        id: toolbar

        background: Rectangle {
            anchors.fill: parent
            color: "black"
            opacity: 0.05
        }

        // to place the toolbar on top of the map
        z: 2

        height: 44
        padding: 0

        anchors {
            left: parent.left
            top: parent.top
            right: parent.right
        }

        // to load a map from your the computer file system
        MergeMapButton {
            id: importButton
            src: "qrc:/icons/load_map"
            txt: "Add Map From File"
            width: 175
            anchors {
                top: parent.top
                topMargin: 12
                bottom: parent.bottom
                bottomMargin: 12
                left: parent.left
                leftMargin: 20
            }
            onClicked: loadFileDialog.open()
        }

        // the window that actually opens to choose the file
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
                window.importMap(fileUrl.toString().substring(7));
                _mapsList.addRobot(fileUrl.toString().substring(7), _mapsList.count+1)
            }
        }

        Rectangle {
            id: separation1
            anchors {
                left: importButton.right
                verticalCenter: parent.verticalCenter
            }
            color: Style.mergeMapGrey
            height: 20
            width: 2
        }

        MergeMapButton {
            id: fromRobotButton
            src: "qrc:/icons/small_robot"
            txt: "Add Map From a Robot"
            width: 200
            anchors {
                top: parent.top
                topMargin: 12
                bottom: parent.bottom
                bottomMargin: 12
                left: separation1.right
                leftMargin: 20
            }
            // the list of robots from which we can choose a map
            RobotListInPopup {
                id: robotsList
                robotModel: window.robotModel
                robotMapsList: _mapsList
                y: fromRobotButton.height + 12
                onRobotSelected: {
                    console.log("adding robot " + name + " " + ip)
                    _mapsList.addRobot(name, ip)
                    window.getMapFromRobot(ip)
                }
            }
            onClicked: {
                console.log("click import map from robot button")
                robotsList.open()
            }
        }

        Rectangle {
            id: separation2
            anchors {
                left: fromRobotButton.right
                verticalCenter: parent.verticalCenter
            }
            color: Style.mergeMapGrey
            height: 20
            width: 2
        }

        Button {

            id: resetButton

            padding: 0
            width: 50

            anchors {
                top: parent.top
                topMargin: 12
                bottom: parent.bottom
                bottomMargin: 12
                left: separation2.right
                leftMargin: 20
            }

            background: Rectangle {
                color: "transparent"
                anchors.fill: parent
                anchors.margins: 5
                radius: 8
            }

            Image {
                id: resetImg
                width: 20
                height: 20
                source: "qrc:/icons/reset"
                fillMode: Image.PreserveAspectFit
                anchors.verticalCenter: parent.verticalCenter
            }

            onClicked: {
                _mapsList.clear();
                window.resetWidget();
            }
        }

        Button {

            id: saveButton

            padding: 0
            width: 20

            anchors {
                top: parent.top
                topMargin: 12
                bottom: parent.bottom
                bottomMargin: 12
                right: parent.right
                rightMargin: 13
            }

            background: Rectangle {
                // careful the color "transparent" cannot be used as it induces a bug which hides the image of the button
                color: Style.lightGreyBackground
                anchors.fill: parent
                anchors.margins: 5
                radius: 8
            }

            Image {
                id: saveImg
                anchors.fill: parent
                source: "qrc:/icons/save"
                fillMode: Image.PreserveAspectFit
                anchors.verticalCenter: parent.verticalCenter
            }

            onClicked: saveFileDialog.open()
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

            onAccepted: window.exportMap(fileUrl.toString())
        }

        Button {

            id: closeButton
            width: 20

            anchors {
                top: parent.top
                bottom: parent.bottom
                right: saveButton.left
                rightMargin: 22
            }

            background: Rectangle {
                // careful the color "transparent" cannot be used as it induces a bug which hides the image of the button
                color: Style.lightGreyBackground
                anchors.fill: parent
                anchors.margins: 5
                radius: 8
            }

            Image {
                id: closeImg
                anchors.fill: parent
                source: "qrc:/icons/closeBtn"
                fillMode: Image.PreserveAspectFit
                anchors.verticalCenter: parent.verticalCenter
            }

            onClicked: window.hide();
        }
    }

    // to store the robots whose map has been imported for merging (subset of the robotModel and only the name and ip attributes are used
    ListModel {
        id: _mapsList

        function addRobot(name, ip){
            append({
                "name": name,
                "ip": ip
            });
        }

        function removeRobot(id){
            remove(id)
        }

        function contains(ip){
            for(var i = 0; i < count; i++)
                if(get(i).ip === ip)
                    return true;
            return false;
        }
    }

    // the menu of the left from where u can rotate maps
    MergeMapsLeftMenu {
        z: 1
        id: leftMenu
        mapsList: _mapsList
        anchors {
            top: toolbar.bottom
            bottom: parent.bottom
            left: parent.left
        }
        onExportMap: saveFileDialog.open()
        onCloseWidget: window.hide()
    }

    Rectangle {

        clip: true

        anchors {
            left: leftMenu.right
            right: parent.right
            top: toolbar.bottom
            bottom: parent.bottom
        }

        Rectangle {

            id: mergedMap
            clip: true
            objectName: "mergeMapsView"

            width: 2048
            height: 2048

            color: "#cdcdcd"

            MouseArea {
                anchors.fill: parent
                clip: true
                acceptedButtons: Qt.LeftButton
                drag.target: parent

                onClicked: console.log(mouseX + " " + mouseY + " width " + width + " height " + height)

                onWheel: {
                    var newScale = mergedMap.scale + mergedMap.scale * wheel.angleDelta.y / 120 / 10;
                    if(newScale > 0.20 && newScale < Style.maxZoom)
                        mergedMap.scale = newScale;
                }
            }
        }
    }

    function grabMergedMap(_fileName){
        console.log("grabbed called " + _fileName.substring(7) + ".pgm");
        if(_fileName.toString().lastIndexOf(".pgm") === -1){
            console.log("you");
            mergedMap.grabToImage(function(result) {
                result.saveToFile(_fileName.substring(7) + ".pgm");
                // important to call the hide function here as this call is asynchronous and if you call hide outside
                // you will most likely hide the window before you can grab it and will end up grabbing nothing
                window.close();});
        }

        else grabToImage(function(result) {
                                          result.saveToFile(_fileName.substring(7));});
    }

    function cancelImportMap(){
        console.log("Cancelling import map");
        _mapsList.removeRobot(_mapsList.count-1);
        mapImportErrorDialog.open();
    }

    Dialog {
        id: mapImportErrorDialog
        title: "Import impossible";
        standardButtons: Dialog.Ok
        Label {
            text: "Sorry, you can only merge maps of the same size";
        }
    }
}
