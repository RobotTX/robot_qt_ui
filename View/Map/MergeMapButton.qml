import QtQuick 2.0
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style

Button {

    property string src
    property string txt

    padding: 0

    background: Rectangle {
        color: "transparent"
        anchors.fill: parent
        anchors.margins: 5
        radius: 8
    }

    Image {
        id: robotImg
        width: 20
        height: 20
        source: src
        fillMode: Image.PreserveAspectFit
        anchors.verticalCenter: parent.verticalCenter
    }

    Label {
        id: textId
        verticalAlignment: Text.AlignVCenter
        anchors.left: robotImg.right
        anchors.leftMargin: 8
        anchors.right: parent.right
        text: txt
    }
}

