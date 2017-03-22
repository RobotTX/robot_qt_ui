import QtQuick 2.7
import EditMapItems 1.0
import QtQuick.Controls 2.1

Item {

    width:300
    height:200

    property  int shape: 0
    property string color: "green"
    property int thickness: 5

    EditMapPaintedItem {
        id: paintedItem
        objectName: "paintedItem"
        anchors.centerIn: parent
        width: 100
        height: 100
    }

}



