import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style
import "../../Model/Point"
import "../Custom"

Frame {
    id: pointMenuFrame
    objectName: "pointMenuFrame"
    padding: 0
    property Points pointModel

    background: Rectangle {
        color: Style.lightGreyBackground
        border.color: Style.lightGreyBorder
        border.width: 1
    }

    /// This frame is displayed when there is no point
    EmptyMenu {
        visible: pointList.count == 0
        txt: "You have no points, click the '+' button or double click the map to create a point."
        imgSrc: "qrc:/icons/big_point"
    }

    /// This frame is displayed when we have points
    Component {
        id: delegate
        PointListItem {
            width: pointMenuFrame.width
            myList: pointList
        }
    }

    /// The list containing both the graphical and model of the points in the menu
    ListView {
        id: pointList
        objectName: "pointList"
        anchors.fill: parent
        model: pointModel
        delegate: delegate
        focus: true
        anchors.topMargin: 14

        signal deletePointOrGroup(string name, string groupName)
        signal hideShow(string name, string groupName, bool isVisible)

    }
}
