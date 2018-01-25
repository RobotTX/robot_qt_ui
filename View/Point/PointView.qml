import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/helper.js" as Helper
import "../../Helper/style.js" as Style

Item {

    property string _name
    property bool _isVisible
    property string _groupName
    property int type: Helper.PointViewType.PERM
    property string tooltipText
    property int pointOrientation: 0
    property int mapOrientation: 0

    visible: _isVisible


    transform: Rotation {
        origin.x: 0
        origin.y: 0
//        angle: (type === Helper.PointViewType.HOME || type === Helper.PointViewType.HOME_TEMP) ? pointOrientation : mapOrientation
        angle: pointOrientation
    }

     property Image image: img

    Image {
        id: img
        source: imageSource()
        width: 27
        asynchronous: true
        smooth: false
        fillMode: Image.PreserveAspectFit
        x: -width / 2
        y: -height
    }

    Label {
        id: tooltip

        visible: img
        font.pointSize: 10
        text: tooltipText

        anchors {
            horizontalCenter: img.horizontalCenter
            bottom: img.top
            bottomMargin: 10
        }

        transform: Rotation {
            origin.x: tooltip.width / 2
            origin.y: tooltip.height + img.height / 2 + tooltip.anchors.bottomMargin
            angle: -pointOrientation + mapOrientation
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
        anchors.fill: img

        onHoveredChanged: if(_isVisible) tooltip.visible = !tooltip.visible /// desktop
//        onClicked: if(_isVisible) tooltip.visible = !tooltip.visible /// android
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
            case Helper.PointViewType.HOME_ORANGE:
                src = "qrc:/icons/homeView_orange";
            break;
            case Helper.PointViewType.PATHPOINT:
                src = "qrc:/icons/pathPoint";
            break;
            case Helper.PointViewType.PATHPOINT_START:
                src = "qrc:/icons/pathPointStart";
            break;
            case Helper.PointViewType.PATHPOINT_START_RED:
                src = "qrc:/icons/pathPointStartRed";
            break;
            case Helper.PointViewType.PATHPOINT_START_YELLOW:
                src = "qrc:/icons/pathPointStartYellow";
            break;
            case Helper.PointViewType.HOME_TEMP:
                src = "qrc:/icons/addHomeView";
            break;
            case Helper.PointViewType.PATHPOINT_END:
                src = "qrc:/icons/pathPointEnd";
            break;
            case Helper.PointViewType.PATHPOINT_NEXT:
                src = "qrc:/icons/pathPointNext";
            break;
            default:
                console.log("The pointView \"" + _name + "\" in group \"" + _groupName + "\" is in an undefined status " + type);
            break;
        }
        return src;
    }

    function setType(newType){
        type = newType;
    }

    function setOrientation(newOri){
        pointOrientation = newOri;
    }
}
