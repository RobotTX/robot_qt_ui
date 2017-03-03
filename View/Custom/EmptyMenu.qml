import QtQuick 2.7
import QtQuick.Controls 2.0
import "../../Helper/style.js" as Style

Frame {
    property string imgSrc
    property string txt

    anchors.fill: parent

    background: Rectangle {
        color: Style.lightGreyBackground
    }

    Image {
        source: imgSrc
        fillMode: Image.PreserveAspectFit // For not stretching image
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.margins: 2 // Leaving space between image and borders
        anchors.bottomMargin: 25 // Leaving space for text in bottom
        anchors.bottom: text.top
    }

    Label {
        id: text
        color: Style.midGrey
        text: qsTr(txt)
        horizontalAlignment: Label.AlignHCenter
        y: parent.height / 2 - Style.menuHeaderHeight / 2 // Placing text in the middle
        width: Style.menuWidth - 40
        wrapMode: Label.WordWrap
        anchors.margins: 2 // Leaving space between text and borders
        anchors.horizontalCenter: parent.horizontalCenter // Centering text
        renderType: Label.NativeRendering // Rendering type
    }
}
