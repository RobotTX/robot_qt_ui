import QtQuick 2.7
import QtQuick.Controls 2.0
import "../../Helper/style.js" as Style

Frame {
    readonly property string txt: "You don’t have any map yet. Go to the ‘Map’ tab to scan a map or load a map from your computer."

    background: Rectangle {
        color: Style.midGrey3
    }

    Image {
        id: mapImage
        asynchronous: true
        source: "qrc:/icons/big_map"
        fillMode: Image.PreserveAspectFit // For not stretching image
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.margins: 2 // Leaving space between image and borders
        anchors.bottomMargin: 25 // Leaving space for text in bottom
        anchors.bottom: mapText.top
    }

    Label {
        id: mapText
        color: Style.midGrey
        text: qsTr(txt)
        horizontalAlignment: Label.AlignHCenter
        y: parent.height / 2 // Placing text in the middle
        width: 340
        wrapMode: Label.WordWrap
        anchors.margins: 2 // Leaving space between text and borders
        anchors.horizontalCenter: parent.horizontalCenter
        renderType: Label.NativeRendering // Rendering type
    }
}
