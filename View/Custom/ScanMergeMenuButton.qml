import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style

Button {
    id: button
    property string txt
    property string src

    height: 40
    padding: 0

    background: Rectangle {
        color: button.hovered ? Style.selectedItemColor : "transparent"
    }

    Image {
        id: icon
        source: src
        fillMode: Image.Pad // to not stretch the image
        anchors {
            verticalCenter: parent.verticalCenter
            left: parent.left
            leftMargin: 20
        }
    }

    CustomLabel {
        text: qsTr(txt)
        color: "#262626"
        anchors{
            verticalCenter: parent.verticalCenter
            left: icon.right
            leftMargin: 11
        }
    }
}
