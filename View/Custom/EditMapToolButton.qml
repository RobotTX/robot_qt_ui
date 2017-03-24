import QtQuick 2.0
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style

Button {

    property string src

    height: 40
    width: 40

    background: Rectangle {
        // careful the color "transparent" cannot be used as it induces a bug which hides the image of the button
        color:  (hovered || checked) ? Style.lightBlue : Style.lightGreyBackground
        anchors.fill: parent
        anchors.margins: 5
        radius: 8
    }

    Image {
        source: src
        anchors.verticalCenter: parent.verticalCenter
        anchors.horizontalCenter: parent.horizontalCenter
    }
}
