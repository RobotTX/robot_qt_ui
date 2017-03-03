import QtQuick 2.7
import QtQuick.Controls 2.0
import "../../Helper/style.js" as Style

Button {
    property string imgSrc
    anchors.verticalCenter: parent.verticalCenter
    width: 20
    height: 20

    background: Rectangle {
        color: pressed ? Style.lightGreyBorder : "transparent"
        radius: pressed ? 10 : 0
    }

    Image {
        source: imgSrc
        fillMode: Image.Pad // For not stretching image
        anchors.horizontalCenter: parent.horizontalCenter // Centering text
        anchors.fill: parent
    }
}
