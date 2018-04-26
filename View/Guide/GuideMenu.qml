import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQuick.Layouts 1.3
import "../Custom"
import "../../Model/Path"
import "../../Model/Point"
import "../../Model/Robot"
import "../../Model/Speech"
import "../../Helper/helper.js" as Helper
import "../../Helper/style.js" as Style

Page {
    id: page
    anchors.fill: parent
    property Paths pathModel // Path file in qml model
    property Paths tmpPathModel
    property Points pointModel
    property Robots robotModel
    property Speechs speechModel
    property string langue
    property string groupSelected
    property variant pathsGuide: []
    property int menuIndex: 0
    signal useTmpPathModel(bool use)
    signal useRobotPathModel(bool use)
    signal closeMenu()
    signal setMessageTop(int status, string msg)

    onVisibleChanged: {
        if(visible){
            useRobotPathModel(false);
            pathModel.visiblePathChanged();
            menuIndex = 0;
        }
    }

    /// frame to display groups
    Frame {
        id: groupMenuFrame
        visible: menuIndex === 0
        anchors.fill: parent
        padding: 0

        CustomDialogGolden {
            id: authentification
            parent: ApplicationWindow.overlay
            x: (page.width - width) / 2
            y: (page.height - height) / 2
            height: 170
            title: langue == "English" ? "警告"  : "Authentification"
            message: "Please enter your id and your password";
            colorBackground: Style.backgroundColorItemGuide
            textColor: Style.goldenColor
            colorBorder: Style.goldenColor

            Column {
                Text {
                    id: txtId
                    text: "Id : "
                    height: 30
                    anchors {
                        top: parent.top
                        topMargin: 50
                        left: parent.left
                        leftMargin: 10
                    }
                    color: Style.goldenColor
                }

                TextField {
                    id: idTextField
                    placeholderText: "Enter id"
                    text: "admin"
                    height: 30
                    anchors {
                        bottom: txtId.bottom
                        left: pwdTextField.left
                    }
                }

                Text {
                    id: txtPwd
                    text: "Password : "
                    height: 30
                    anchors {
                        top: txtId.bottom
                        left: txtId.left
                        topMargin: 5
                    }
                    color: Style.goldenColor
                }

                TextField {
                    id: pwdTextField
                    placeholderText: "Enter passsword"
                    text: "admin"
                    echoMode: TextInput.Password
                    height: 30
                    anchors {
                        bottom: txtPwd.bottom
                        left: txtPwd.right
                        leftMargin: 5
                    }
                }
            }

            acceptMessage: langue == "English" ? "Yes" : "Login"
            rejectMessage: "Cancel"
            onAccepted: {
                if (idTextField.text === "admin" && pwdTextField.text === "admin") {
                    page.closeMenu();
                } else {
                    wrongPwd.open();
                }
            }
            onRejected: console.log("Cancel");
        }

        CustomDialogGolden {
            id: wrongPwd
            parent: ApplicationWindow.overlay
            x: (page.width - width) / 2
            y: (page.height - height) / 2
            height: 130
            title: langue == "English" ? "警告"  : "Warning"
            message: "The id or the password is wrong. Please try again."
            acceptMessage: langue == "English" ? "Yes" : "Ok"
            colorBackground: Style.backgroundColorItemGuide
            textColor: Style.goldenColor
            colorBorder: Style.goldenColor
            onAccepted: {
                authentification.open();
            }
        }

        GuideMenuContent {
            pathModel: page.pathModel
            robotModel: page.robotModel
            langue: page.langue
            anchors {
                left: parent.left
                top: parent.top
                right: parent.right
                bottom: parent.bottom
            }
            onGroupSelected: {
                page.groupSelected = groupName;
                pathModel.hideShowGroup(groupName)
                menuIndex = 1;
            }
            onCloseGuideMenu: {
                authentification.open();
            }
        }
    }

    /// frame to display paths
    Frame {
        id: pathMenuFrame
        visible: menuIndex === 1
        anchors.fill: parent
        padding: 0

        Frame {
            id: frameContent
            height: parent.height
            padding: 0
            anchors {
                top: parent.top
                left: parent.left
                right: parent.right
            }

            background: Rectangle {
                color: Style.backgroundColorItemGuide
                border.color: Style.backgroundColorDarkGuide
                border.width: 1
            }

            Flickable {
                ScrollBar.vertical: ScrollBar { }
                contentHeight: contentItem.childrenRect.height
                anchors.fill: parent
                anchors.topMargin: 10

                Column {
                    id: columnId
                    /// The list containing both the graphical and model of the paths in the menu
                    Grid {
                        columns: 4
                        spacing: 20

                        Repeater {
                            model: pathModel
                            delegate: delegate
                        }
                    }
                }
            }

            Component {
                id: delegate

                Column {
                    anchors {
                        left: parent.left
                        leftMargin: 10
                        rightMargin: 10
                    }
                    Grid {
                        columns: 4
                        spacing: 20

                        Button {
                            id: btnBacl
                            height: 230
                            width: 230

                            background: Rectangle {
                                color: "#3b358e"
                                border.width: 1
                            }

                            Image {
                                id: icon
                                source: "qrc:/icons/back_128128" //imgSrc
                                fillMode: Image.Pad // to not stretch the image
                                anchors{
                                    left: parent.left
                                    leftMargin: 50
                                    bottom: parent.bottom
                                    bottomMargin: 50
                                }
                            }

                            onClicked: {
                                page.menuIndex = 0;
                                pathModel.hideShowGroup(groupName);
                            }
                        }

                        Repeater {
                            model: paths
                            delegate: del
                        }
                    }

                    Component {
                        id: del

                        Button {
                            id: btnGroup
                            visible: groupName === Helper.noGroup ? !groupIsOpen : groupIsOpen
                            height: 230
                            width: 230

                            background: Rectangle {
                                color: {
                                    if (index === 0) {
                                        "#4e82c7"
                                    } else if (index === 1) {
                                        "#64aaea"
                                    } else if (index === 2) {
                                        "#5fa5bc"
                                    } else if (index === 3) {
                                        "#c12b35"
                                    } else if (index === 4) {
                                        "#c1285b"
                                    } else if (index === 5) {
                                        "#c3237e"
                                    } else if (index === 6) {
                                        "#ff6666"
                                    } else if (index === 7) {
                                        "#3b358e"
                                    }
                                }
                            }

                            CustomLabel {
                                text: qsTr(pathName)
                                color: "white"
                                anchors{
                                    verticalCenter: parent.verticalCenter
                                    horizontalCenter: parent.horizontalCenter
                                    top: parent.top
                                    topMargin: 100
                                }
                                font.pointSize: 20
                                font.bold: true
                            }

                            onClicked: {
                                sendPathToRobot.open();
                            }

                            CustomDialogGolden {
                                id: sendPathToRobot
                                parent: ApplicationWindow.overlay
                                x: (page.width - width) / 2
                                y: (page.height - height) / 2
                                height: 130
                                colorBackground: Style.backgroundColorItemGuide
//                                title: langue == "English" ? "警告"  : "Warning"
                                message: "Do you want the robot to perform path \"" + pathName +  "\" ?"
                                textColor: Style.goldenColor
                                colorBorder: Style.goldenColor
                                font.bold: true
                                acceptMessage: langue == "English" ? "Hao" : "Yes"
                                rejectMessage: "Cancel"
                                onAccepted: {
                                        robotModel.newPathSignal(robotModel.ipRobot, groupName, pathName);
                                        robotModel.playPathSignal(robotModel.ipRobot);
                                        pauseRobotInPath.open();
                                }
                                onRejected: console.log("Cancel");
                            }

                            CustomStopDialog {
                                id: pauseRobotInPath
                                parent: ApplicationWindow.overlay
                                x: (page.width - width) / 2
                                y: (page.height - height) / 2

                                height: 500
                                width: 800
                                message: "You can : \n\t> pause the robot by clicking on the \"PAUSE\" button \n\t> stop the robot by clicking on the \"STOP\" button ";
                                acceptMessage: langue == "English" ? "Yes" : "STOP"
                                yesMessage: "PAUSE"
                                imgPausePlay: "qrc:/icons/pauseDialog300"
                                colorPlayPause: Style.darkSkyBlueBorder
                                onAccepted: {
                                    robotModel.stopPathSignal(robotModel.ipRobot);
                                }
                                onYes: {
                                    robotModel.pausePathSignal(robotModel.ipRobot);
                                    playRobotInPath.open();
                                    pauseRobotInPath.close();
                                }
                            }

                            CustomStopDialog {
                                id: playRobotInPath
                                parent: ApplicationWindow.overlay
                                x: (page.width - width) / 2
                                y: (page.height - height) / 2

                                height: 500
                                width: 800
                                message: "You can : \n\t> play the robot by clicking on the \"PLAY\" button \n\t> stop the robot by clicking on the \"STOP\" button ";
                                acceptMessage: langue == "English" ? "Yes" : "STOP"
                                yesMessage: "PLAY"
                                imgPausePlay: "qrc:/icons/playDialog300"
                                colorPlayPause: Style.greenPlay
                                onAccepted: {
                                    robotModel.stopPathSignal(robotModel.ipRobot);
                                }
                                onYes: {
                                    robotModel.playPathSignal(robotModel.ipRobot);
                                    pauseRobotInPath.open();
                                    playRobotInPath.close();
                                }
                            }

                        }

                    }
                }
            }
        }
    }
}


