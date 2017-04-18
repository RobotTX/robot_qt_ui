import QtQuick 2.0
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style

Button {

    property string src
    property string txt

    padding: 0

    background: Rectangle {
        color: hovered ? Style.lightGreyBorder : "transparent"
        anchors.fill: parent
        anchors.margins: 5
        radius: 8
    }

    contentItem: Item {

        Image {
            id: robotImg
            width: 20
            height: 20
            source: src
            fillMode: Image.PreserveAspectFit
            anchors.verticalCenter: parent.verticalCenter
            anchors.left: parent.left
            anchors.leftMargin: 5
        }

        Label {
            id: textId
            verticalAlignment: Text.AlignVCenter
            anchors.left: robotImg.right
            anchors.leftMargin: 8
            anchors.right: parent.right
            anchors.verticalCenter: parent.verticalCenter
            text: txt
        }
    }
}

