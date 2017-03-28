import QtQuick 2.0
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style

Button {

    property string src

    padding: 0
    width: 180



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

    Text {
        verticalAlignment: Text.AlignVCenter
        anchors.left: robotImg.right
        anchors.leftMargin: 8
        anchors.right: parent.right
        text: "Add Map From File"
    }

    onClicked: console.log("lol")
}
