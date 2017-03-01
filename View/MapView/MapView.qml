import QtQuick 2.7
import QtQuick.Controls 2.0
import QtGraphicalEffects 1.0

Frame {
    readonly property string txt: "You don’t have any map yet. Go to the ‘Map’ tab to scan a map or load a map from your computer."

    background: Rectangle {
        color: "#e4e4e4"
    }


    Image {
        id: mapImage
        source: "qrc:/icons/big_map"
        fillMode: Image.PreserveAspectFit // For not stretching image
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.margins: 2 // Leaving space between image and borders
        anchors.bottomMargin: 25 // Leaving space for text in bottom
        anchors.bottom: mapText.top
    }

    /// To change the color of the image
    ColorOverlay {
        anchors.fill: mapImage
        source: mapImage
        color: "#bdbdbd"
    }

    Text {
        id: mapText
        color: "#8f8e94"
        text: qsTr(txt)
        y: parent.height / 2 // Placing text in the middle
        width: 340
        wrapMode: Text.WordWrap
        anchors.margins: 2 // Leaving space between text and borders
        anchors.horizontalCenter: parent.horizontalCenter // Centering text
        renderType: Text.NativeRendering // Rendering type
    }
}
