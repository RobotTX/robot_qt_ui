import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style

Button {
    id: btn
    property string imgSrc
    property string backColor: "transparent"
    width: Style.smallBtnWidth
    height: Style.smallBtnHeight

    background: Rectangle {
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.verticalCenter: parent.verticalCenter
        width: Math.min(btn.width, btn.height)
        height: width
        color: pressed ? Style.lightGreyBorder : hovered ? Style.lightGreyBackgroundHover : backColor
        radius: btn.hovered ? width/2 : 0
    }

    Image {
        asynchronous: true
        source: imgSrc
        fillMode: Image.Pad
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.verticalCenter: parent.verticalCenter
    }
}
