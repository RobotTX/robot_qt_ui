import QtQuick 2.7
import QtQuick.Controls 2.0

Button {
    width: 66
    height: 76
    autoExclusive: true
    checkable: true
    property string imgSrc
    property string txt

    background: Rectangle {
        //color: checked ? "#000000" : "#27313a"
        //opacity: checked ? 0.4 : 0.8
        color: "#27313a"
        opacity: 0.8
    }

    Image {
        source: imgSrc
        fillMode: Image.Pad // For not stretching image (optional)
        anchors.fill: parent
        anchors.margins: 2 // Leaving space between image and borders (optional)
        anchors.bottomMargin:10 // Leaving space for text in bottom
    }
    Text {
        text: txt
        anchors.bottom: parent.bottom // Placing text in bottom
        anchors.margins: 2 // Leaving space between text and borders  (optional)
        anchors.horizontalCenter: parent.horizontalCenter // Centering text
        renderType: Text.NativeRendering // Rendering type (optional)
    }
}
