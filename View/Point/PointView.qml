import QtQuick 2.7
import QtQuick.Controls 2.1

Image {
    id: image
    property string name
    property bool isVisible
    property string groupName

    source: "qrc:/icons/pointView"
    width: 18
    height: 24
    asynchronous: true
    smooth: false
    visible: ((groupName !== "") && isVisible)

    MouseArea {
        anchors.fill: parent
        onClicked: console.log("Clicked on " + name + " in group " + groupName + " " + isVisible)
    }
}
