import QtQuick 2.7
import QtQuick.Controls 2.1

Label {

    id: label
//    property string wrapModeLabel: "WrapAtWordBoundaryOrAnywhere"

    maximumLineCount: 1
    elide: Text.ElideRight
//    wrapMode: wrapModeLabel

    property bool hovered: false
    property bool timerTriggered: false

    MouseArea {
        id: mouseArea
        anchors.fill: parent
        hoverEnabled: true
        onEntered: {
            if(label.truncated){
                hovered = true;
                timer.start();
            }
        }
        onExited: {
            if(label.truncated){
                hovered = false;
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
        y: label.height
        visible: hovered && label.truncated && timerTriggered
        text: label.text
    }

    Timer {
        id: timer
        interval: 800
        onTriggered: {
            if(hovered)
                timerTriggered = true;
        }
    }
}
