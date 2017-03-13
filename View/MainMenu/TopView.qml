import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style
import "../Custom"

Frame {
    id: topViewFrame
    property bool hasMap

    signal saveState()
    signal loadState()

    height: Style.menuHeaderHeight
    z: 1
    anchors {
        left: parent.left
        top: parent.top
        right: parent.right
    }

    background: Rectangle {
        color: Style.lightGreyBackground
        border.color: Style.lightGreyBorder
        border.width: 1
    }

    Label {
        color: Style.midGrey2
        text: qsTr("Soon some awesome messages here ")
    }

    /// The load state button
    SmallButton {
        id: loadStateButton
        imgSrc: "qrc:/icons/loadState"
        anchors.verticalCenter: parent.verticalCenter
        anchors.right: parent.right
        anchors.rightMargin: 10
        enabled: hasMap
        onClicked: topViewFrame.loadState()
    }

    /// The save state button
    SmallButton {
        id: saveStateButton
        imgSrc: "qrc:/icons/saveState"
        anchors.verticalCenter: parent.verticalCenter
        anchors.right: loadStateButton.left
        anchors.rightMargin: 14
        enabled: hasMap
        onClicked: topViewFrame.saveState()
    }
}
