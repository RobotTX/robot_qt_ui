import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQuick.Window 2.2
import "../../Helper/style.js" as Style

Window {

    id: window
    objectName: "mergeMapWindow"

    width: 1000
    minimumWidth: 800
    height: 700
    minimumHeight: 600

    signal importMap(string file)

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
            anchors {
                top: parent.top
                topMargin: 12
                bottom: parent.bottom
                bottomMargin: 12
                left: parent.left
                leftMargin: 20
            }
            onClicked: window.importMap("lol")
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
            anchors {
                top: parent.top
                topMargin: 12
                bottom: parent.bottom
                bottomMargin: 12
                left: separation1.right
                leftMargin: 20
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

            onClicked: console.log("lol")
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
}
