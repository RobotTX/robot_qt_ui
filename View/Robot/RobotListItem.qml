import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQuick.Layouts 1.3
import "../../Helper/style.js" as Style
import "../../Helper/helper.js" as Helper
import "../../Model/Point"
import "../../Model/Path"
import "../../Model/Robot"
import "../Custom"

Frame {
    id: frame
    property Points pointModel
    property Paths pathModel
    property Robots robotModel
    property real batteryWarningThreshold

    signal dockRobot(string ip)

    height: 105 + robotPathListItem.height
    enabled: !processingCmd

    background: Rectangle {
        color: "transparent"
    }

    padding: 0

    CustomLabel {
        id: nameLabel
        text: qsTr(name)
        height: 20
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
/*
            onEditingFinished: {
                if(saveName.enabled)
                    saveName.clicked()
            }
*/
            background: Rectangle {
                radius: 2

                border.color: nameField.text === "" ? Style.errorColor : nameField.activeFocus ? Style.lightBlue : Style.lightGreyBorder
                border.width: nameField.activeFocus || nameField.text === "" ? 3 : 1
            }
            onVisibleChanged: nameField.text = ""
        }

        SmallButton {
            id: cancelName
            tooltip: "Cancel"
            imgSrc: "qrc:/icons/closeBtn"
            anchors {
                verticalCenter: nameField.verticalCenter
                right: saveName.left
                rightMargin: 5
            }

            onClicked: {
                nameLabel.visible = true;
                nameField.focus = false;
            }
        }

        SmallButton {
            id: saveName
            tooltip: "Save"
            imgSrc: "qrc:/icons/save"
            anchors {
                verticalCenter: nameField.verticalCenter
                right: parent.right
                rightMargin: 5
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

    SmallButton {
        id: rightButton
        imgSrc: "qrc:/icons/more"
        anchors {
            verticalCenter: nameLabel.verticalCenter
            right: parent.right
            rightMargin: 20
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
            onLaserPressed: robotModel.activateLaser(ip, !laserActivated)
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
                color: battery < 50 * batteryWarningThreshold ? Style.errorColor2 : Style.darkSkyBlue
            }
        }
    }

    CustomLabel {
        id: pathLabel
        text: {
            if(pathName !== "" && pathPoints.count > 0){
                if(stage >= 0){
                    if(stage < pathPoints.count){
                        if(playingPath)
                            qsTr("Heading to " + pathPoints.get(stage).pathPointName);
                        else
                            qsTr("Waiting to go to " + pathPoints.get(stage).pathPointName);
                    } else
                        qsTr("Stage not in the pathpoint list");
                } else {
                    if(Math.abs(stage + 1) < pathPoints.count){
                        if(stage == -1)
                            qsTr("Stuck going to " + pathPoints.get(Math.abs(stage + 1)).pathPointName);
                        else
                            qsTr("Stuck going from " + pathPoints.get(Math.abs(stage  + 2)).pathPointName + " to " + pathPoints.get(Math.abs(stage + 1)).pathPointName);
                    } else
                        qsTr("Stage not in the pathpoint list");
                }
            } else
                qsTr("No Path Assigned");
        }
        font.pixelSize: 14
        color: {
            if(pathName !== "" && pathPoints.count > 0){
                if(stage >= 0)
                    Style.darkSkyBlue
                else
                    Style.errorColor2
            } else
                Style.midGrey2
        }
        anchors {
            top: batteryLevel.bottom
            left: parent.left
            right: parent.right
            topMargin: 11
            leftMargin: 20
            rightMargin: 20
        }
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
        onDockRobot: frame.dockRobot(ip)
    }

    ToolSeparator {
        orientation: Qt.Horizontal
        anchors {
            bottom: parent.bottom
            left: parent.left
            right: parent.right
            leftMargin: 20
            rightMargin: 20
        }
    }

    Item {
        visible: processingCmd
        Rectangle {
            radius: 3
            color: Style.lightGreyBackground
            opacity: 0.5
            anchors.fill: parent
        }

        CustomBusyIndicator {
            running: processingCmd
            anchors.fill: parent
        }
    }
}
