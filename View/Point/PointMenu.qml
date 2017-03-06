import QtQuick 2.7
import QtQuick.Controls 2.0
import "../Custom"
import "../../Helper/style.js" as Style

Page {
    id: page
    anchors.fill: parent

    MenuHeader {
        id: pointMenuHeader
        objectName: "pointMenuHeader"
        txt: "Point"

        Button {
            anchors {
                top: parent.top
                bottom: parent.bottom
                right: parent.right
            }
            anchors.rightMargin: 22

            width: Style.smallBtnWidth
            height: Style.smallBtnHeight

            background: Rectangle {
                color: "transparent"
            }

            Image {
                source: "qrc:/icons/add"
                fillMode: Image.Pad // For not stretching image
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.verticalCenter: parent.verticalCenter
            }
            onClicked: console.log("Clicked to add a new point or group")
        }
    }

    PointMenuContent {
        anchors {
            left: parent.left
            top: pointMenuHeader.bottom
            right: parent.right
            bottom: parent.bottom
        }
    }
}


