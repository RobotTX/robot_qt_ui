import QtQuick 2.7

Rectangle {

    property string txt

    Rectangle {
        id: rectangle

        color: "#979797"
        height: 7
        width: 2
    }

    Rectangle {
        anchors.top: rectangle.bottom
        anchors.topMargin: 10
        Text {
            id: textId
            text: txt
            font.pointSize: 8
        }
    }
}

