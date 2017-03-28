import QtQuick 2.7
import QtQuick.Controls 2.1

Rectangle {

    property real threshold: slider.value

    Slider {
        id: slider
        snapMode: Slider.SnapAlways
        height: 5
        value: 0
        from: 0.1
        to: 0.5
    }

    function initializeBatteryThreshold(_value){
        console.log("ini value to " + _value)
        slider.value = _value
    }
}
