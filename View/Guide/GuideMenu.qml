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
    property int currentMenu: 0
    signal useTmpPathModel(bool use)
    signal useRobotPathModel(bool use)
    signal closeMenu()
    signal setMessageTop(int status, string msg)

    onVisibleChanged: {
        if(visible){
            useRobotPathModel(false);
            pathModel.visiblePathChanged();
            pathModel.hideShowGroupAll();
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
                    pathModel.showGroupDefault();
                } else {
                    wrongPwd.open();
                }
            }
//            onRejected: console.log("Cancel");
            onRejected: {}
        }

        CustomDialogGolden {
            id: wrongPwd
            parent: ApplicationWindow.overlay
            x: (page.width - width) / 2
            y: (page.height - height) / 2
            height: 130
            title: langue == "English" ? "警告"  : "Warning"
            message: "The id or the password is wrong. Please try again."
            acceptMessage: langue == "English" ? "是" : "Ok"
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
                pathModel.hideShowGroupAll();
                pathModel.hideShowGroup(groupName);
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
                                color: "transparent"
//                                border.width: 1
                            }

                            Image {
                                id: icon
                                source: "qrc:/icons/back_gold_128128" //imgSrc
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
                                pathModel.hideShowGroupAll();
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
                            visible: groupIsOpen
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

                            /// this is a "fake label", used mainly to get feedback of path completed and robot stuck from robot feedback
                            CustomLabel {
                                id: fakeLabel
                                text: {
                                    if (robotModel.robotStuck === true && currentMenu === 5 && pauseRobotInPath.visible) { /// robot is stuck in its path ; make sure that window does not appear when we are in RobotView
                                        pauseRobotInPath.close();
                                        playRobotInPath.open();
                                    } else if (robotModel.pathCompleted === true) { /// robot has completed its path
                                        pauseRobotInPath.close();
                                        page.menuIndex = 0;
                                    }
                                    qsTr("");
                                }
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
                                message: "Do you want the robot to perform path \"" + pathName +  "\" ?"
                                textColor: Style.goldenColor
                                colorBorder: Style.goldenColor
                                font.bold: true
                                acceptMessage: langue == "English" ? "确认" : "Yes"
                                rejectMessage: "Cancel"
                                onAccepted: {
                                        robotModel.newPathSignal(robotModel.ipRobot, groupName, pathName); /// send path to robot
                                        robotModel.setLooping(robotModel.ipRobot, false); /// set looping to be false so the robot won t loop
                                        robotModel.playPathSignal(robotModel.ipRobot); /// tell the robot to play the path
                                        pauseRobotInPath.open();
                                }
//                                onRejected: console.log("Cancel");
                                onRejected: {}
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
                                    pauseRobotInPath.close();
                                }

                                onYes: {
                                    robotModel.pausePathSignal(robotModel.ipRobot);
                                    pauseRobotInPath.close();
                                    playRobotInPath.open();
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
                                    pauseRobotInPath.close();
                                }
                                onYes: {
                                    robotModel.playPathSignal(robotModel.ipRobot);
                                    playRobotInPath.close();
                                    pauseRobotInPath.open();
                                }
                            }

                        }

                    }
                }
            }
        }
    }
}


