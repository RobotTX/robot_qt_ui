import QtQuick 2.0
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style

Button {

    property string txt
    property string imgSrc

    height: 40

    anchors {
        left: parent.left
        right: parent.right
    }

    background: Rectangle {
        // to change the color of the button when pressed
        color: pressed ? Style.selectedItemColor : "transparent"
    }

    Image {
        id: icon
        source: imgSrc
        fillMode: Image.Pad // to not stretch the image
        anchors{
            verticalCenter: parent.verticalCenter
            left: parent.left
            leftMargin: 20
        }
    }

    Label {
        text: qsTr(txt)
        anchors{
            verticalCenter: parent.verticalCenter
            left: icon.right
            leftMargin: 11
        }
    }
}

