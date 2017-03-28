import QtQuick 2.7

Rectangle {

    property string txt

    width: 16
    height: 22
    color: "transparent"

    Rectangle {
        id: rectangle
        anchors.top: parent.top
        anchors.horizontalCenter: parent.horizontalCenter
        color: "#979797"
        height: 7
        width: 2
    }

    Rectangle {
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.top: rectangle.bottom
        anchors.topMargin: 10
        Text {
            id: textId
            anchors.fill: parent
            verticalAlignment: Text.AlignVCenter
            horizontalAlignment: Text.AlignHCenter
            text: txt
            font.pointSize: 8
        }
    }
}

