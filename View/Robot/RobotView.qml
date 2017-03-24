import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/helper.js" as Helper

Image {
    property string _name
    property string _ip
    property double _orientation

    transform: Rotation {
        origin.x: width / 2
        origin.y: height / 2
        angle: orientation
    }

    source: "qrc:/icons/robotView"
    width: 12
    asynchronous: true
    smooth: false
    fillMode: Image.PreserveAspectFit

    MouseArea {
        anchors.fill: parent
        onClicked: {
            console.log("Clicked on " + _name + " at ip " + _ip + " with an orientation of " + orientation)
        }
    }
}
