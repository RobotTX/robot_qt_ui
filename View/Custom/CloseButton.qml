import QtQuick 2.7
import QtQuick.Controls 2.0
import "../../Helper/style.js" as Style

Button {
    anchors.verticalCenter: parent.verticalCenter
    width: 20
    height: 20

    background: Rectangle {
        color: closeBtn.pressed ? Style.lightGreyBorder : "transparent"
    }

    Image {
        source: "qrc:/icons/closeBtn"
        fillMode: Image.Pad // For not stretching image
        anchors.horizontalCenter: parent.horizontalCenter // Centering text
        anchors.fill: parent
    }
}
