import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/helper.js" as Helper

Image {

    id: image
    property string name
    property bool isVisible
    property string groupName
    property int type: Helper.PointViewType.PERM
    property double originX
    property double originY


    source: imageSource()
    width: 18
    height: 24
    asynchronous: true
    smooth: false
    visible: ((groupName !== "") && isVisible)

    MouseArea {
        anchors.fill: parent
        onClicked: console.log("Clicked on " + name + " in group " + groupName + " " + isVisible + " " + type)
    }

    /// To change the source file of the pointView according to its type
    function imageSource(){
        var src = "qrc:/icons/pointView";
        switch(type){
            case Helper.PointViewType.PERM:
            break;
            case Helper.PointViewType.TEMP:
                src = "qrc:/icons/addPointView";
            break;
            default:
                console.log("The pointView \"" + name + "\" in group \"" + groupName + "\" is in an undefined status " + type);
            break;
        }
        return src;
    }
}
