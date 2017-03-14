import QtQuick 2.0
import QtQuick.Controls 2.1

Button {
    property string txt
    property string imgSrc

    height: 45

    anchors {
        left: parent.left
        right: parent.right
    }

    background: Rectangle {
        color: "transparent"
    }

    Image {
        id: icon
        source: imgSrc
        fillMode: Image.Pad // For not stretching image
        anchors.verticalCenter: parent.verticalCenter
        anchors.left: parent.left
        anchors.leftMargin: 20
    }

    Label {
        text: qsTr(txt)
        anchors.verticalCenter: parent.verticalCenter
        anchors.left: icon.right
        anchors.leftMargin: 11
    }
}

