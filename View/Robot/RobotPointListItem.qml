import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQuick.Layouts 1.3
import "../../Helper/style.js" as Style
import "../../Helper/helper.js" as Helper
import "../../Model/Point"
import "../../Model/Robot"
import "../Custom"

Column {
    id: groupListItem
    objectName: "robotPointListItem"

    property Points pointModel
    property Column column
    property Robots robotModel
    property string langue
    property bool open: false

    signal renameGroup(string name)
    signal editPoint(string name, string groupName)
    signal createGroup(string name)

        /// The item displaying the name of the point/group
        Label {
            text: if (name !== "") {
                      pointModel.addGroup("Robot - " + name);
                      createGroup(name);
                  }
            visible: false
            anchors.verticalCenter: parent.verticalCenter
        }
}
