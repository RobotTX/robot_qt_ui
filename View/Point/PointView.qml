import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/helper.js" as Helper

Image {
    property string _name
    property bool _isVisible
    property string _groupName
    property int type: Helper.PointViewType.PERM
    property double originX
    property double originY


    source: imageSource()
    width: 18
    asynchronous: true
    smooth: false
    visible: _isVisible
    fillMode: Image.PreserveAspectFit

    /// To change the source file of the pointView according to its type
    function imageSource(){
        var src = "qrc:/icons/pointView";
        switch(type){
            case Helper.PointViewType.PERM:
            break;
            case Helper.PointViewType.TEMP:
                src = "qrc:/icons/addPointView";
            break;
            case Helper.PointViewType.HOME:
                src = "qrc:/icons/homeView";
            break;
            case Helper.PointViewType.PATHPOINT:
                src = "qrc:/icons/pathPoint";
            break;
            case Helper.PointViewType.PATHPOINT_START:
                src = "qrc:/icons/pathPointStart";
            break;
            default:
                console.log("The pointView \"" + _name + "\" in group \"" + _groupName + "\" is in an undefined status " + type);
            break;
        }
        return src;
    }
}
