import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/helper.js" as Helper
import "../../Helper/style.js" as Style

Image {
    id: img
    property string _name
    property bool _isVisible
    property string _groupName
    property int type: Helper.PointViewType.PERM
    property double originX
    property double originY
    property string tooltipText

    source: imageSource()
    width: 18
    asynchronous: true
    smooth: false
    visible: _isVisible
    fillMode: Image.PreserveAspectFit

    ToolTip {
        id: tooltip
        font.pointSize: 10
        contentItem: Label {
            id: label
            width: label.paintedWidth + 30
            wrapMode: Text.WrapAnywhere
            text: tooltipText
        }

        // for tooltips the y is given relatively to the y of the associated item (here the image)
        // so we put it slightly above our image
        y: -20 - paintedHeight
        background: Rectangle {
            radius: 8
            anchors.fill: parent
            border.color: Style.darkSkyBlue
            color: "white"
        }
    }

    MouseArea {
        id: mArea
        hoverEnabled: true
        anchors.fill: parent
        onHoveredChanged: {
            //console.log("being hovered")
            if(_isVisible)
                tooltip.visible = !tooltip.visible
        }
    }

    /// To change the source file of the pointView according to its type
    function imageSource(){
        console.log("parent coords " + x + " " + y + " " + width + " " + height)
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
