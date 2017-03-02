import QtQuick 2.7
import QtQuick.Controls 2.0
import "../../Helper/style.js" as Style

Frame {
    padding: 0

    EmptyMap {
        id: emptyMap
        objectName: "emptyMap"
        anchors.fill: parent
        visible: false
    }

    Image {
        id: mapImage
        source: "qrc:/maps/map.pgm"
        fillMode: Image.PreserveAspectFit // For not stretching image

        smooth: false

        MouseArea {
            id: dragArea
            anchors.fill: parent

            drag.target: parent
            onWheel: {
                mapImage.scale += mapImage.scale * wheel.angleDelta.y / 120 / 10;
            }
        }
    }
}
