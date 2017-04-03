import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/helper.js" as Helper
import "../../Helper/style.js" as Style

Image {

    property string _name
    property string _ip
    property double _orientation

    transform: Rotation {
        origin.x: width / 2
        origin.y: height / 2
        angle: orientation
    }

    source: "qrc:/icons/robotView"
    width: 12
    asynchronous: true
    smooth: false
    fillMode: Image.PreserveAspectFit

    Label {

        id: tooltip

        visible: false
        font.pointSize: 10
        text: _name

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

        anchors.fill: parent
        hoverEnabled: true

        onClicked: {
           console.log("Clicked on " + _name + " at ip " + _ip + " with an orientation of " + orientation)
        }

        onHoveredChanged: {
            console.log("robot hovered")
            tooltip.visible = !tooltip.visible
        }
    }
}
