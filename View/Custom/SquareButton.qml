import QtQuick 2.0
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style

Button {
    id: btn
    property string txt
    property string imgSrc
    property bool clickedButton

    height: 70
    width: 70

    anchors {
        left: parent.left
        right: parent.right
    }

    background: Rectangle {
//        border.color: Style.darkGrey
//        color: hovered ? Style.selectedItemColor : "transparent"
        color: btn.pressed ? Style.lightGreyBorder : btn.hovered ? Style.selectedItemColor : btn.checked ? Style.selectedItemColor : Style.lightGreyBackgroundHover
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
        anchors{
            verticalCenter: parent.verticalCenter
            left: icon.right
            leftMargin: 15
        }
    }
}

