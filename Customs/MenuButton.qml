import QtQuick 2.7
import QtQuick.Controls 2.0

Button {
    width: 66
    height: 76
    autoExclusive: true
    checkable: true
    property string imgSrc
    property string txt

    // This is to change the background color of the button when selected
    background: Rectangle {
        color: checked ? "#66000000" : "transparent"

        // The blue border when the item is selected
        Rectangle {
            color: checked ? "#4a8fe3" : "transparent"
            width : 3
            anchors {
                left: parent.left
                top: parent.top
                bottom: parent.bottom
            }
        }
    }

    Image {
        source: checked ? imgSrc + "_checked" : imgSrc
        fillMode: Image.Pad // For not stretching image (optional)
        anchors.fill: parent
        anchors.margins: 2 // Leaving space between image and borders (optional)
        anchors.bottomMargin: (txt == "Settings") ? 0 : 10 // Leaving space for text in bottom
    }

    Text {
        color: checked ? "#4a8fe3" : "#8f8f94"
        text: (txt == "Settings") ? qsTr("") : qsTr(txt)
        anchors.bottom: parent.bottom // Placing text in bottom
        anchors.margins: 2 // Leaving space between text and borders  (optional)
        anchors.horizontalCenter: parent.horizontalCenter // Centering text
        renderType: Text.NativeRendering // Rendering type (optional)
    }
}
