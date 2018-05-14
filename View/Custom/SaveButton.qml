import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style

Button {
    id: btn
    property string langue
    property string txt: langue == "English"  ? "保存" : "Save"
    property string tooltip
    property bool canSave: true
    property bool timerTriggered: false
    height: 23

    CustomLabel {
        text:  qsTr(txt)
        color: "white"
        anchors.verticalCenter: parent.verticalCenter
        anchors.horizontalCenter: parent.horizontalCenter
    }

    background: Rectangle {
        radius: 3
        color: {
             canSave ? (pressed ? Style.darkSkyBlueBorder : Style.darkSkyBlue) : Style.disableSaveColor;

           }
        border.width: 1
//        border.color: canSave ? Style.darkSkyBlueBorder : Style.disableSaveBorder
        border.color: canSave ? (pressed ? Style.darkSkyBlueBorder : Style.darkSkyBlue) : Style.disableSaveColor;
    }

    MouseArea {
        id: mouseArea
        anchors.fill: parent
        hoverEnabled: true
        propagateComposedEvents: true
        onEntered: if(!btn.canSave && tooltip !== "") timer.start();
        onExited: if(!btn.canSave && tooltip !== "") timerTriggered = false;
        onClicked: mouse.accepted = false;
        onPressed: mouse.accepted = false;
        onReleased: mouse.accepted = false;
    }

    CustomToolTip {
        id: toolTip
        x: -6
        y: btn.height
        visible: !btn.canSave && btn.hovered && tooltip !== "" && timerTriggered
        text: qsTr(tooltip)
    }

    Timer {
        id: timer
        interval: 800
        onTriggered: {
            if(!btn.canSave && btn.hovered)
                timerTriggered = true;
        }
    }
}
