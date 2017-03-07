import QtQuick 2.7
import QtQuick.Controls 2.0
import "../../Helper/style.js" as Style

Button {
    width: 66
    height: 76
    checkable: true
    property string imgSrc
    property string txt
    objectName: txt + "Button"

    // This is to change the background color of the button when selected
    background: Rectangle {
        color: checked ? "#66000000" : "transparent"

        // The blue border when the item is selected
        Rectangle {
            color: checked ? Style.darkSkyBlue : "transparent"
            width : 3
            anchors {
                left: parent.left
                top: parent.top
                bottom: parent.bottom
            }
        }
    }

    Image {
        id: image
        asynchronous: true
        y: (txt == "Settings") ? 0 : 15
        source: checked ? imgSrc + "_checked" : imgSrc
        fillMode: Image.Pad // For not stretching image
        anchors.horizontalCenter: parent.horizontalCenter // Centering text
        anchors.fill: if(txt == "Settings"){parent}
    }

    Label {
        color: checked ? Style.darkSkyBlue : Style.midGrey
        text: (txt == "Settings") ? qsTr("") : qsTr(txt)
        font.pointSize: 9
        anchors.top: image.bottom // Placing text in bottom
        anchors.topMargin: 4 // Leaving space between text and borders
        anchors.horizontalCenter: parent.horizontalCenter // Centering text
        renderType: Label.NativeRendering // Rendering type
    }
}
