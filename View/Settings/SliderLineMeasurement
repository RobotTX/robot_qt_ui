import QtQuick 2.7
import QtQuick.Controls 2.1

Rectangle {

    property string txt

    width: 16
    height: 22
    color: "transparent"

    Rectangle {

        id: rectangle
        anchors.top: parent.top
        anchors.horizontalCenter: parent.horizontalCenter
        // some grey
        color: "#979797"
        height: 7
        width: 2
    }

    Rectangle {

        anchors {
            horizontalCenter: parent.horizontalCenter
            top: rectangle.bottom
            topMargin: 10
        }

        Label {
            id: textId
            anchors.fill: parent
            verticalAlignment: Text.AlignVCenter
            horizontalAlignment: Text.AlignHCenter
            text: txt
            font.pointSize: 8
        }
    }
}
