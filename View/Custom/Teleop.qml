import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQuick.Layouts 1.3
import "../../Helper/style.js" as Style

GridLayout {
    columns: 3
    columnSpacing: 8
    rowSpacing: 8

    ListModel {
        id: listBtn
        ListElement { src: "upLeft" }
        ListElement { src: "up" }
        ListElement { src: "upRight" }
        ListElement { src: "left" }
        ListElement { src: "play" }
        ListElement { src: "right" }
        ListElement { src: "downLeft" }
        ListElement { src: "down" }
        ListElement { src: "downRight" }
    }

    Repeater {
        model: listBtn
        delegate: Button {
            Layout.preferredWidth: 30
            Layout.preferredHeight: 30
            padding: 0

            background: Rectangle {
                color: "white"
                border.width: 1
                border.color: Style.lightGreyBorder
                radius: 2
            }

            contentItem: Image {
                asynchronous: true
                source: "qrc:/icons/" + src
                fillMode: Image.Pad // For not stretching image
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.verticalCenter: parent.verticalCenter
            }
            onClicked: console.log("Clicked on teleop " + index)
        }
    }
}
