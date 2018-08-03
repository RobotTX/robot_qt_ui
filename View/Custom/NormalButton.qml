import QtQuick 2.0
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style

Button {

    property string txt
    property string imgSrc
    property string sizeTxt: Style.ubuntuHeading2Size

    height: 40

    anchors {
        left: parent.left
        right: parent.right
    }

    background: Rectangle {
        color: hovered ? Style.selectedItemColor : "transparent"
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

    CustomLabel {
        text: qsTr(txt)
        color: "#262626"
        font.pointSize: sizeTxt
        anchors{
            verticalCenter: parent.verticalCenter
            left: icon.right
            leftMargin: 11
        }
    }
}

