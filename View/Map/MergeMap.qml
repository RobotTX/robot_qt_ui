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

    width: 1000
    minimumWidth: 800
    height: 700
    minimumHeight: 600

    signal importMap(string file)

    property Robots robotsModel

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
            RobotListInPopup {
                id: robotsList
                robotsModel: window.robotsModel
                robotMapsList: _robotsList
                y: fromRobotButton.height + 12
                onRobotSelected: {
                    console.log("adding robot " + name + " " + ip)
                    _robotsList.addRobot(name, ip)
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

            onClicked: console.log("ok")
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

    ListModel {
        id: _robotsList

        function addRobot(name, ip){
            append({
                "name": name,
                "ip": ip
            });
        }

        function removeRobot(ip){
            for(var i = 0; i < count; i++)
                if(get(i).ip === ip)
                    remove(i);
        }

        function contains(ip){
            for(var i = 0; i < count; i++){
                console.log("curr ip " + get(i).ip + " compared to IP: " + ip)
                if(get(i).ip == ip){
                    console.log("contains " + ip);
                    return true;
                }
            }

            console.log("does not contain " + ip);
            return false;
        }
    }

    MergeMapsLeftMenu {
        id: leftMenu
        robotsList: _robotsList
        anchors {
            top: toolbar.bottom
            bottom: parent.bottom
            left: parent.left
        }
    }

    Canvas {
        anchors {
            left: leftMenu.right
            right: parent.right
            top: toolbar.bottom
            bottom: parent.bottom
        }
        onPaint: {
            var ctx = getContext("2d");
            ctx.fillStyle = Qt.rgba(1, 0, 0, 1);
            ctx.fillRect(0, 0, width, height);
        }
    }
}
