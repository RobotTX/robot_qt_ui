import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style

Slider {
    id: slider

    height: 14
    padding: 0

    handle: Rectangle {
        id: handle
        anchors {
            top: parent.top
        }
        radius: 10
        height: 14
        width: 14
        color: Style.darkSkyBlue
        x: slider.visualPosition * slider.availableWidth - width / 2
        y: slider.availableHeight / 2 - height / 2
    }

    background: Rectangle {
        x: slider.leftPadding
        y: slider.availableHeight / 2 - height / 2
        height: 2
        width: slider.availableWidth
        color: "#bdbebf"
        radius: 2

        Rectangle {
            width: handle.x
            height: parent.height
            color: Style.darkSkyBlue
            radius: 2
        }
    }
}
