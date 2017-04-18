import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style

Button {

    id: btn

    property string imgSrc
    property string backColor: "transparent"
    property string tooltip
    property bool timerTriggered: false
    width: Style.smallBtnWidth
    height: Style.smallBtnHeight

    background: Rectangle {
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.verticalCenter: parent.verticalCenter
        width: Math.min(btn.width, btn.height)
        height: btn.width
        color: btn.pressed ? Style.lightGreyBorder : btn.hovered ? Style.lightGreyBackgroundHover : backColor
        radius: btn.hovered ? btn.width/2 : 0
    }

    contentItem : Item {
        Image {
            asynchronous: true
            source: imgSrc
            fillMode: Image.Pad
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.verticalCenter: parent.verticalCenter
        }
    }

    MouseArea {
        id: mouseArea
        anchors.fill: parent
        hoverEnabled: true
        onEntered: {
            if(tooltip !== ""){
                //hovered = true;
                timer.start();
            }
        }
        onExited: {
            if(tooltip !== ""){
                //hovered = false;
                timerTriggered = false;
            }
        }
        onClicked: mouse.accepted = false;
        onPressed: mouse.accepted = false;
        onReleased: mouse.accepted = false;
    }

    CustomToolTip {
        id: toolTip
        x: -6
        y: btn.height
        visible: btn.hovered && tooltip !== "" && timerTriggered
        text: tooltip
    }

    Timer {
        id: timer
        interval: 800
        onTriggered: {
            if(btn.hovered)
                timerTriggered = true;
        }
    }
}
