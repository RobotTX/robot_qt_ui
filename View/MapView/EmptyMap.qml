import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style

Frame {
    property string langue
    readonly property string txt: langue == "English" ? "You don't have any map yet. Go to 'Map' tab to scan a map or load a map from your computer." : "你还没有地图。请前往'地图'项扫描地图，或者从电脑下载地图"

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
