import QtQuick 2.7
import QtQuick.Controls 2.0
import "../../Helper/style.js" as Style
import "../../Model/Point"
import "../Custom"

Page {
    id: page
    readonly property int index: 2
    anchors.fill: parent
    property Point _pointModel;

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
                asynchronous: true
                source: "qrc:/icons/add"
                fillMode: Image.Pad // For not stretching image
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.verticalCenter: parent.verticalCenter
            }
            onClicked: console.log("Clicked to add a new point or group")
        }
    }

    PointMenuContent {
        pointModel: _pointModel
        anchors {
            left: parent.left
            top: pointMenuHeader.bottom
            right: parent.right
            bottom: parent.bottom
        }
    }
}


