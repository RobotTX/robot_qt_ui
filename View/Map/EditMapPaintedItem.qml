import QtQuick 2.7
import EditMapItems 1.0
import QtQuick.Controls 2.1

EditMapPaintedItem {

    id: paintedItem

    width: 2048
    height: 2048
    smooth: false

    function test(str){
        console.log(parent.width + " " + parent.height + " " + x + " " + y);
    }

}
