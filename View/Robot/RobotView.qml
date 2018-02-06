import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/helper.js" as Helper
import "../../Helper/style.js" as Style
import "../../Model/Robot"
import "../../Model/Path"
import "../../Model/Point"

Image {
    id: img
    objectName: "robotView"
    property string _name
    property string _ip
    property bool hover: false
    property int mapOrientation: 0
    property Robots robotModel
    property Paths pathModel
    property Points pointModel
    property string langue

    signal soundOn(string ip)
    signal soundOff(string ip)

    transform: Rotation {
        origin.x: width / 2
        origin.y: height / 2
        angle: orientation
    }

    source: "qrc:/icons/robotView"
    width: 18
    asynchronous: true
    smooth: false
    fillMode: Image.PreserveAspectFit

    Label {
        id: tooltip

//        visible: img.hover
        visible: img
        font.pointSize: 10
        text: _name
        color: "White"
        anchors {
            horizontalCenter: img.horizontalCenter
            bottom: img.top
            bottomMargin: 10
        }

        transform: Rotation {
            origin.x: tooltip.width / 2
            origin.y: tooltip.height + img.height / 2 + tooltip.anchors.bottomMargin

            angle: -orientation + mapOrientation
        }

        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter

        background: Rectangle {
            anchors.horizontalCenter: tooltip.horizontalCenter
            anchors.verticalCenter: tooltip.verticalCenter
            width: tooltip.paintedWidth + 8
            height: tooltip.paintedHeight + 8
            radius: 8
            border.color: Style.orangeColor //Style.darkSkyBlue
            color: Style.orangeColor
        }
    }

    MouseArea {

        anchors.fill: parent
        hoverEnabled: true
        acceptedButtons: Qt.LeftButton | Qt.RightButton
        onClicked: {
//           console.log("Clicked on robot " + _name + " at ip " + _ip + " with an orientation of "
//                       + orientation + " " + img.x + " " + img.y + " " + img.width + " " + img.height)
            if (mouse.button === Qt.RightButton) {
                console.log("right button on robot");
                robotPopupMenu.open();
            } else if (mouse.button === Qt.LeftButton) {
                console.log("Clicked on robot " + _name + " at ip " + _ip + " with an orientation of "
                            + orientation + " " + img.x + " " + img.y + " " + img.width + " " + img.height)
            }
        }

        onHoveredChanged: {
            hover = !hover
        }

        RobotPopupMapView {
            id: robotPopupMenu
            pointModel: img.pointModel
            pathModel: img.pathModel
            robotModel: img.robotModel
            langue: img.langue
            onPointSelected: {
                console.log("robotModel.newHomeSignal");robotModel.newHomeSignal(ip, _homeX, _homeY, orientation)
            }
            onPathSelected: robotModel.newPathSignal(ip, _groupName, _pathName)
            onDeletePath: robotModel.deletePathSignal(ip)
            onSaveCurrentPath: {
                pathModel.saveCurrentPath(pathName,pathPoints)
            }
            onSaveCurrentHome: {
                pointModel.saveCurrentHome("CS", homeX, homeY, homeOri);
                console.log( " home X = " + homeX + " home Y = " + homeY + " homeOri = " + homeOri);
            }
            onSoundOn: img.soundOn(ip)
            onSoundOff: img.soundOff(ip)
        }
    }
}
