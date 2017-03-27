import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQuick.Layouts 1.3
import "../../Helper/style.js" as Style
import "../Custom"
import "../../Model/Point"
import "../../Model/Path"
import "../../Model/Robot"

Frame {
    id: robotMenuFrame
    property Robots robotModel
    property Points pointModel
    property Paths pathModel

    background: Rectangle {
        color: Style.lightGreyBackground
        border.color: Style.lightGreyBorder
        border.width: 1
    }
    padding: 0

    EmptyMenu {
        visible: robotModel.count === 0
        txt: "No robot connected. Make sure that the robot and your computer are connected to the same WIFI network."
        imgSrc: "qrc:/icons/big_robot"
    }

    Component {
        id: delegate
        RobotListItem {
            pointModel: robotMenuFrame.pointModel
            pathModel: robotMenuFrame.pathModel
            robotModel: robotMenuFrame.robotModel
            width: flick.width
        }
    }

    Flickable {
        id: flick
        ScrollBar.vertical: ScrollBar { }
        contentHeight: contentItem.childrenRect.height
        anchors.fill: parent
        anchors.topMargin: 10

        Rectangle {
            anchors.fill: parent
            color: "transparent" //Style.lightGreyBackground
        }

        Column {
            /// The list containing both the graphical and model of the robots in the menu
            Repeater {
                model: robotModel
                delegate: delegate
            }
        }
    }
}
