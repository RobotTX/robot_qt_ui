import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style

Label {
    id: label
    visible: text === "" || opacity == 0
    wrapMode: Text.WordWrap
    width: parent.width

    PropertyAnimation {
        id: anim
        target: label
        property: "opacity"
        to: 0
        duration: 500
    }

    onTextChanged: {
        if(text !== ""){
            label.opacity = 100;
            timer.start();
        }
    }

    Timer {
        id: timer
        interval: 10000
        onTriggered: {
            anim.running = true;
        }
    }
}
