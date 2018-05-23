import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style

Slider {

    id: slider
    property int cursor_width: img.width
    property int cursor_height: img.height

    padding: 0
    height: 15

    background: Rectangle {
        id: background
        x: slider.leftPadding
        y: slider.topPadding + slider.availableHeight / 2 - height / 2
        implicitWidth: 200
        implicitHeight: 6
        width: slider.availableWidth
        height: implicitHeight
        radius: 2
        color: "#bdbebf"


        Rectangle {
            width: img.x
            height: parent.height
            color: Style.darkSkyBlue
            radius: 2
        }
    }

    // have a custom image to drag instead of the native big sphere
    handle: Image {
        id: img
//        x: Math.max(0.1, Math.min(0.9, slider.visualPosition)) * (slider.availableWidth) - width/2
        x: Math.max(0, Math.min(0.9, slider.visualPosition)) * (slider.availableWidth) - width/2
        y: slider.topPadding + slider.availableHeight / 2 - height / 2
        source: "qrc:/icons/cursor"
    }

    function initializeBatteryThreshold(_value){
//        console.log("ini value to " + _value)
        slider.value = _value
    }
}

