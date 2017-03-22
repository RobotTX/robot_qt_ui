import QtQuick 2.7
import EditMapItems 1.0

Item {
    width:300
    height:200

    EditMapPaintedItem {
        id: paintedItem
        anchors.centerIn: parent
        thickness: 10
        color: "blue"
        width: 100
        height: 100
    }

    Text {
        anchors { bottom: parent.bottom; horizontalCenter: parent.horizontalCenter; bottomMargin: 20 }
        text: "YO"
    }
}


