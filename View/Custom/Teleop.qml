import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQuick.Layouts 1.3
import "../../Helper/style.js" as Style

GridLayout {

    columns: 3
    columnSpacing: 8
    rowSpacing: 8

    signal sendTeleop(int index)

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
            checked: checkedIndex === index

            background: Rectangle {
                color: parent.checked ? Style.lightBlue : "white"
                border.width: 1
                border.color: Style.lightGreyBorder
                radius: 2
            }

            contentItem: Image {
                asynchronous: true
                source: index === 4 && checkedIndex != -1 ? "qrc:/icons/pause" : "qrc:/icons/" + src
                fillMode: Image.Pad // For not stretching image
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.verticalCenter: parent.verticalCenter
            }
            onClicked: sendTeleop(index);
        }
    }
}
