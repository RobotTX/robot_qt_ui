import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQuick.Layouts 1.3
import "../../Helper/style.js" as Style
import "../../Helper/helper.js" as Helper
import "../../Model/Point"
import "../../Model/Path"
import "../../Model/Robot"

Frame {
    id: frame
    property Points pointModel
    property Paths pathModel
    property Robots robotModel

    height: 105 + robotPathListItem.height

    background: Rectangle {
        anchors.fill: parent
        color: "transparent" //Style.lightGreyBackground
    }

    padding: 0

    Label {
        id: nameLabel
        text: qsTr(name)
        height: 20
        maximumLineCount: 1
        elide: Text.ElideRight
        font.pixelSize: 16

        anchors {
            top: parent.top
            left: parent.left
            right: rightButton.left
            leftMargin: 20
            topMargin: 15
        }
    }

    Item {
        visible: !nameLabel.visible
        anchors {
            top: parent.top
            left: parent.left
            right: rightButton.left
            leftMargin: 20
            topMargin: 11
        }
        height: 28
        TextField {
            id: nameField
            selectByMouse: true
            placeholderText: qsTr(name)
            height: 28
            anchors {
                verticalCenter: parent.verticalCenter
                left: parent.left
                right: cancelName.left
                rightMargin: 5
            }

            background: Rectangle {
                radius: 2
                border.color: parent.activeFocus ? Style.lightBlue : Style.lightGreyBorder
                border.width: parent.activeFocus ? 3 : 1
            }
        }

        Button {
            id: cancelName
            anchors {
                verticalCenter: nameField.verticalCenter
                right: saveName.left
                rightMargin: 5
            }

            background: Rectangle {
                color: parent.hovered ? Style.lightGreyBackgroundHover : "transparent"
                radius: parent.hovered ? Style.smallBtnWidth / 2 : 0
            }

            width: Style.smallBtnWidth
            height: Style.smallBtnHeight

            Image {
                asynchronous: true
                source: "qrc:/icons/closeBtn"
                fillMode: Image.Pad // For not stretching image
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.verticalCenter: parent.verticalCenter
            }
            onClicked: {
                nameLabel.visible = true;
                nameField.focus = false;
            }
        }

        Button {
            id: saveName
            anchors {
                verticalCenter: nameField.verticalCenter
                right: parent.right
                rightMargin: 5
            }

            background: Rectangle {
                color: parent.hovered ? Style.lightGreyBackgroundHover : "transparent"
                radius: parent.hovered ? Style.smallBtnWidth / 2 : 0
            }

            width: Style.smallBtnWidth
            height: Style.smallBtnHeight

            Image {
                asynchronous: true
                source: "qrc:/icons/save"
                fillMode: Image.Pad // For not stretching image
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.verticalCenter: parent.verticalCenter
            }
            onClicked: {
                var newName = Helper.formatName(nameField.text);
                if(newName !== ""){
                    nameLabel.visible = true;
                    nameField.focus = false;
                    robotModel.newNameSignal(ip, newName);
                }
            }
        }
    }

    Button {
        id: rightButton
        anchors {
            verticalCenter: nameLabel.verticalCenter
            right: parent.right
            rightMargin: 20
        }

        background: Rectangle {
            color: parent.hovered ? Style.lightGreyBackgroundHover : "transparent"
            radius: parent.hovered ? Style.smallBtnWidth / 2 : 0
        }

        width: Style.smallBtnWidth
        height: Style.smallBtnHeight

        Image {
            asynchronous: true
            source: "qrc:/icons/more"
            fillMode: Image.Pad // For not stretching image
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.verticalCenter: parent.verticalCenter
        }
        onClicked: robotPopupMenu.open()

        RobotPopupMenu {
            id: robotPopupMenu
            x: rightButton.width
            pointModel: frame.pointModel
            pathModel: frame.pathModel
            onPointSelected: robotModel.newHomeSignal(ip, _homeName, _homeX, _homeY)
            onPathSelected: robotModel.newPathSignal(ip, _groupName, _pathName)
            onRenameRobot: {
                nameLabel.visible = false;
                nameField.focus = true;
            }
            onDeletePath: robotModel.deletePathSignal(ip)
        }
    }

    ProgressBar {
        id: batteryLevel
        value: battery / 100
        anchors {
            top: nameLabel.bottom
            left: parent.left
            right: parent.right
            topMargin: 9
            leftMargin: 20
            rightMargin: 20
        }

        background: Rectangle {
            implicitWidth: parent.width
            implicitHeight: 4

            color: Style.lightGreyBorder
            radius: 3
        }

        contentItem: Item {
            implicitWidth: parent.width
            implicitHeight: 4

            Rectangle {
                width: batteryLevel.visualPosition * parent.width
                height: parent.height
                radius: 2
                color: Style.darkSkyBlue
            }
        }
    }

    Label {
        id: pathLabel
        text: pathName !== "" && pathPoints.count > 0 ? (playingPath ? qsTr("Heading to " + pathPoints.get(stage).pathPointName) : qsTr("Waiting at " + pathPoints.get(stage).pathPointName)) : qsTr("No Path Assigned")
        font.pixelSize: 14
        maximumLineCount: 1
        elide: Text.ElideRight
        color: pathName !== "" && pathPoints.count > 0 ? Style.darkSkyBlue : Style.midGrey2
        anchors {
            top: batteryLevel.bottom
            left: parent.left
            right: parent.right
            topMargin: 11
            leftMargin: 20
            rightMargin: 20
        }
        Component.onCompleted: console.log("nb pathpoints " + pathPoints.count)
    }

    RobotPathListItem {
        id: robotPathListItem
        robotModel: frame.robotModel
        pathModel: frame.pathModel
        anchors {
            top: pathLabel.bottom
            left: parent.left
            right: parent.right
            topMargin: 10
            leftMargin: 20
            rightMargin: 20
        }
        onPathSelected: robotModel.newPathSignal(ip, _groupName, _pathName)
    }

    Rectangle {
        anchors {
            bottom: parent.bottom
            left: parent.left
            right: parent.right
            leftMargin: 20
            rightMargin: 20
        }
        color: Style.lightGreyBorder
        height: 2
    }
}
