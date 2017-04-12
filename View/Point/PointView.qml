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

    x: - width / 2
    y: - height

    source: imageSource()
    width: 18
    asynchronous: true
    smooth: false
    visible: _isVisible
    fillMode: Image.PreserveAspectFit

    Label {
        id: tooltip

        visible: false
        font.pointSize: 10
        text: tooltipText

        anchors {
            horizontalCenter: parent.horizontalCenter
            bottom: parent.top
            bottomMargin: 10
        }

        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter

        background: Rectangle {
            anchors.horizontalCenter: tooltip.horizontalCenter
            anchors.verticalCenter: tooltip.verticalCenter
            width: tooltip.paintedWidth + 8
            height: tooltip.paintedHeight + 8
            radius: 8
            border.color: Style.darkSkyBlue
            color: "white"
        }
    }

    MouseArea {

        id: mArea

        hoverEnabled: true
        anchors.fill: parent

        onHoveredChanged: {
            if(_isVisible)
              tooltip.visible = !tooltip.visible
        }
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

    function setPos(posX, posY){
        x = posX - width/2;
        y = posY - height;
    }
}
