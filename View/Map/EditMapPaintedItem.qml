import QtQuick 2.7
import EditMapItems 1.0
import QtQuick.Controls 2.1

EditMapPaintedItem {

/*
    property  int shape: 0
    property string color: "green"
    property int thickness: 10
*/
    id: paintedItem

    width: 2048
    height: 2048
    smooth: false

    function test(str){
        console.log(parent.width + " " + parent.height + " " + x + " " + y);
    }

}


