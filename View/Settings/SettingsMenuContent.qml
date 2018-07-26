import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQml 2.2
import "../../Helper/style.js" as Style
import "../Custom"
import "../../Model/Robot"
import "../../View/Custom/"
import "../../Model/Path"
import "../../Model/Tutorial"
import "../Robot"

Frame {

    id: settingsPage
    objectName: "settings"

    signal close()
    signal saveSettingsSignal(int mapChoice, double _batteryThreshold, int languageChoice)
    signal saveWifiSignal(string ip_wifi, string wifi, string pwd_wifi)
//    signal changeLanguage(string language)
    signal openCreatePointMenu()
    signal saveVelocitySignal(string ipRobotWifi, double linearVelocity, double angularVelocity)
    signal saveBatterySignal(string ipRobotWifi, double battery)

    property string inputNameWifi: ""
    property string inputPwdWifi: ""
    property string ipRobotWifi: ""
    property int currentMenuIndex: -1
    property variant ssidWifi: []

    property real batteryWarningThreshold
    property int mapChoice: 1
    property int languageChoice: 0
    property string langue
    property string nameRobot

    property real oriBatteryWarningThreshold
    property int oriMapChoice
    property int oriLanguageChoice

    property Robots robotModel
    property Tutorial tutorial
    property Paths pathModel

    background: Rectangle {
        anchors.fill: parent
        color: Style.lightGreyBackground
        border.color: Style.lightGreyBorder
        border.width: 1
    }

    onVisibleChanged: {
//        settingsPage.mapChoice = oriMapChoice;

        settingsPage.batteryWarningThreshold = oriBatteryWarningThreshold;
        batterySlider.initializeBatteryThreshold(batteryWarningThreshold);
        linearVelocitySlider.initializeVelocity(0.3);
        angularVelocitySlider.initializeVelocity(0.3)
        nameRobot = "";
        ipRobotWifi = "";
    }

    Flickable {
        id: flickPoint
        ScrollBar.vertical: ScrollBar {
            anchors.left: flickPoint.right
            anchors.leftMargin: 3
        }

        contentHeight: 1082

        anchors.fill: parent
        anchors.topMargin: 10

        Label {
            id: robotLabel
            anchors {
                left: parent.left
                top: parent.top
                topMargin: -10
            }

            color: Style.darkSkyBlue
            font.pointSize: 11
            text: langue === "English" ? qsTr("Robot Settings") : qsTr("机器人设置")
        }

        PopupMenuItem {
            id: robotList
            height: Style.menuItemHeight
            width: parent.width
            labelText: {
                if (nameRobot === "") {
                    langue === "English" ? "Select a robot" : "选择机器人"
                } else {
                    langue === "English" ? "Robot " + nameRobot + " selected" : "选择了 " + nameRobot + " 机器人"
                }
            }

            anchors.left: parent.left
            anchors.top: robotLabel.bottom
            anchors.topMargin: 10

            Image {
                asynchronous: true
                source: "qrc:/icons/arrow"
                fillMode: Image.Pad // For not stretching image
                anchors.verticalCenter: parent.verticalCenter
                anchors.right: parent.right
                anchors.rightMargin: 12
            }
            onHoveredChanged: if(visible){
                                robotListInPopup.open();
                                settingsPage.currentMenuIndex = 0
                              } /// desktop
//            onClicked: if(visible){ currentMenuIndex = 0 } /// android

            RobotListInPopup {
                id: robotListInPopup
                x: robotList.width
                visible: settingsPage.currentMenuIndex === 0
                robotModel: settingsPage.robotModel
                onRobotSelected: {
                    ipRobotWifi = ip;
                    nameRobot = name;

                    /// updating velocity
                    var lv = robotModel.getLinearVelocity(ip);
                    var av = robotModel.getAngularVelocity(ip);
                    linearVelocitySlider.value = getLinearVelocityReverse(lv);
                    angularVelocitySlider.value = getAngularVelocityReverse(av);

                    /// updating battery
                    batteryWarningThreshold = batterySlider.value;
                    var batteryLevel = robotModel.getBatteryWarning(ip);
                    batterySlider.value = getBattery(batteryLevel);

                    function getBattery(bat) {
                        if (bat === 50) {
                            bat = 1
                        } else if (bat >= 40 && bat < 50) {
                            bat = 0.7 + (bat - 40)*0.02;
                        } else if (bat >= 30 && bat < 40) {
                            bat = 0.5 + (bat - 30)*0.02;
                        } else if (bat >= 20 && bat < 30) {
                            bat = 0.3 + (bat - 20)*0.02;
                        } else if (bat >= 10 && bat < 20) {
                            bat = 0.1 + (bat - 10)*0.02;
                        } else if (bat === 0) {
                            bat = 0;
                        } else {
                            bat = bat * 0.007;
                        }

                        return (bat)
                    }

                    function getLinearVelocityReverse(lv) {
                        return lv === 0.2 ? lv.value = 0.0 : lv = lv - 0.1
                    }

                    function getAngularVelocityReverse(av) {
                        return av === 20 ? av = 0 : av = (av/100) - 0.1
                    }

                    settingsPage.currentMenuIndex = -1;

                }
            }
        }

        ToolSeparator {
            id: horizontalSeparation40
            orientation: Qt.Horizontal
            anchors {
                top: robotList.bottom
                left: parent.left
                right: parent.right
                topMargin: 10
            }
        }

        Label {
            id: wifiLabel
            anchors {
                left: parent.left
                top: horizontalSeparation40.bottom
                topMargin: 10
            }

            color: Style.darkSkyBlue
            font.pointSize: 11
            text: langue === "English" ? qsTr("WiFi Settings") : qsTr("无线网设置")
        }

        Label {
            id: nameWifi
            text: langue === "English" ? qsTr("Name WiFi : ") : qsTr("无线网名称")
            color: Style.greyText
            font.pointSize: 10
            anchors {
                left: parent.left
                top: wifiLabel.bottom
                right: parent.right
            }
            anchors.topMargin: 20
        }

        PopupMenuItem {
            id: userInputWifiName
            height: Style.menuItemHeight
            width: parent.width
            labelText: {
                if (inputNameWifi === "") {
                    langue === "English" ? "Select a WiFi" : "选择无线网"
                } else {
                    langue === "English" ?  inputNameWifi :  inputNameWifi
                }
            }

            anchors.left: parent.left
            anchors.top: nameWifi.bottom
            anchors.topMargin: 10

            Image {
                asynchronous: true
                source: "qrc:/icons/arrow"
                fillMode: Image.Pad // For not stretching image
                anchors.verticalCenter: parent.verticalCenter
                anchors.right: parent.right
                anchors.rightMargin: 12
            }
            onHoveredChanged: if(visible){
                                wifiMenu.open();
                                settingsPage.currentMenuIndex = 1;
                              } /// desktop

//            onClicked: if(visible){ currentMenuIndex = 1} /// android

            Menu {
                id: wifiMenu
                x: userInputWifiName.width
                visible: settingsPage.currentMenuIndex === 1
                padding: 0
                width: 300

                background: Rectangle {
                    color: Style.lightGreyBackground
                    border.color: Style.lightGreyBorder
                    radius: 5
                }

                Column {
                    anchors {
                        left: parent.left
                        right: parent.right
                    }

                    Repeater {
                        model: toto
                        delegate: PopupMenuItem {
                            width: parent.width
                            height: Style.menuItemHeight
                            labelText: modelData
                            enabled: true
                            leftPadding: Style.menuItemLeftPadding
                            onTriggered: {
                                inputNameWifi = modelData;
                                settingsPage.currentMenuIndex = -1;
                                wifiMenu.close();
                            }
                        }
                    }
                }
            }
        }

        Label {
            id: pwdWifi
            text: langue === "English" ? qsTr("Password : ") : qsTr("密码 : ")
            color: Style.greyText
            font.pointSize: 10
            anchors {
                left: nameWifi.left
                top: userInputWifiName.bottom
                right: parent.right
            }
            anchors.topMargin: 15
        }

        TextField {
            id: userInputWifiPwd
            objectName: "wifiPwd"
            selectByMouse: true
            placeholderText: langue === "English" ? qsTr("Enter Password") : qsTr("输入无线网密码")
            background: Rectangle {
                    implicitWidth: 174
                    implicitHeight: 15
                    border.color: Style.midGrey
            }
            font.pointSize: 10
            text: inputPwdWifi
            echoMode: TextInput.Password
            anchors {
                left: pwdWifi.left
                bottom: pwdWifi.bottom
            }
            anchors.leftMargin: 76

            onTextChanged: {
                inputPwdWifi = userInputWifiPwd.text
            }
        }

        // apply button to save the changes but keep the window open
        SaveButton {
               id: applyButtonWifi
               langue: settingsPage.langue
               txt: langue == "English" ? "Apply" : "应用"
               width: 70
               font.pointSize: 11

               anchors.top: userInputWifiPwd.bottom
               anchors.topMargin: 15
               anchors.left: parent.left
               anchors.right: parent.right

               onReleased: {
                   if ((inputNameWifi !== "") && (ipRobotWifi !== "")) {
                       saveWifiSignal(ipRobotWifi, inputNameWifi, inputPwdWifi);
                   } else {
                       wifiDialog.open();
                   }
               }
           }

        ToolSeparator {
            id: horizontalSeparation2
            orientation: Qt.Horizontal
            anchors {
                top: applyButtonWifi.bottom
                left: parent.left
                right: parent.right
                topMargin: 10
            }
        }

        Label {
            id: velocityLabel
            anchors {
                left: parent.left
                top: horizontalSeparation2.bottom
                topMargin: 10
            }

            color: Style.darkSkyBlue
            font.pointSize: 11
            text: langue === "English" ? qsTr("Velocity Settings") : qsTr("速度设置")
        }

        Item {

            id: linearVelocityLabel

            height: 15

            anchors {
                top: velocityLabel.bottom
                topMargin: 10
            }

            Label {
                id: linearVelocityHelp

                anchors {
                    left: parent.left
                    top: parent.top
                }

//                color: "#8F8E94"
                color: Style.blackMenuTextColor
                font.pointSize: 10
                text: langue === "English" ? qsTr("Linear velocity") : qsTr("线速度")
            }

            HelpButton {

                height: 20
                width: 20

                anchors {
                    left: linearVelocityHelp.right
                    leftMargin: 5
                    verticalCenter: linearVelocityHelp.verticalCenter
                }

                tooltipText: langue === "English" ? "Recommended Value: 0.4 m/s" : "推荐值: 0.4 m/s"
            }
        }

        VelocityLevelSlider {
            id: linearVelocitySlider
            anchors {
                top: linearVelocityLabel.bottom
                topMargin: 10
                left: parent.left
                right: parent.right
            }
        }

        SliderLineMeasurement {
            id: lineMeasurement11
            anchors {
                top: linearVelocitySlider.bottom
                left: parent.left
                leftMargin: linearVelocitySlider.width/10-linearVelocitySlider.cursor_width/2
            }
            txt: "0.2"
        }

        SliderLineMeasurement {
            id: lineMeasurement21
            anchors {
                top: linearVelocitySlider.bottom
                left: lineMeasurement11.right
                leftMargin: (linearVelocitySlider.width/2 - lineMeasurement11.x) / 2 - lineMeasurement11.width - 4
            }
            txt: "0.4"
        }

        SliderLineMeasurement {
            id: lineMeasurement31
            anchors {
                top: linearVelocitySlider.bottom
                left: lineMeasurement21.right
                horizontalCenter: linearVelocitySlider.horizontalCenter
            }
            txt: "0.6"
        }

        SliderLineMeasurement {
            id: lineMeasurement41
            anchors {
                top: linearVelocitySlider.bottom
                left: lineMeasurement31.right
                leftMargin: 4
            }
            txt: "0.8"
        }

        SliderLineMeasurement {
            id: lineMeasurement51
            anchors {
                top: linearVelocitySlider.bottom
                right: parent.right
                rightMargin: linearVelocitySlider.width/10-linearVelocitySlider.cursor_width/2
            }
            txt: "1"
        }

        Item {

            id: angularVelocityLabel

            height: 15

            anchors {
                top: lineMeasurement51.bottom
                topMargin: 10
            }

            Label {
                id: angularVelocityHelp

                anchors {
                    left: parent.left
                    top: parent.top
                }

//                color: "#8F8E94"
                color: Style.blackMenuTextColor
                font.pointSize: 10
                text: langue === "English" ? qsTr("Angular Velocity") : qsTr("角速度")
            }

            HelpButton {

                height: 20
                width: 20

                anchors {
                    left: angularVelocityHelp.right
                    leftMargin: 5
                    verticalCenter: angularVelocityHelp.verticalCenter
                }

                tooltipText: langue === "English" ? "Recommend Value: 40 deg/s" : "推荐值: 40 deg/s"
            }
        }

        VelocityLevelSlider {
            id: angularVelocitySlider
            anchors {
                top: angularVelocityLabel.bottom
                topMargin: 10
                left: parent.left
                right: parent.right
            }
        }

        SliderLineMeasurement {
            id: lineMeasurement12
            anchors {
                top: angularVelocitySlider.bottom
                left: parent.left
                leftMargin: angularVelocitySlider.width/10-angularVelocitySlider.cursor_width/2
            }
            txt: "20"
        }

        SliderLineMeasurement {
            id: lineMeasurement22
            anchors {
                top: angularVelocitySlider.bottom
                left: lineMeasurement12.right
                leftMargin: (angularVelocitySlider.width/2 - lineMeasurement12.x) / 2 - lineMeasurement12.width - 4
            }
            txt: "40"
        }

        SliderLineMeasurement {
            id: lineMeasurement32
            anchors {
                top: angularVelocitySlider.bottom
                left: lineMeasurement22.right
                horizontalCenter: angularVelocitySlider.horizontalCenter
            }
            txt: "60"
        }

        SliderLineMeasurement {
            id: lineMeasurement42
            anchors {
                top: angularVelocitySlider.bottom
                left: lineMeasurement32.right
                leftMargin: 4
            }
            txt: "80"
        }

        SliderLineMeasurement {
            id: lineMeasurement52
            anchors {
                top: angularVelocitySlider.bottom
                right: parent.right
                rightMargin: angularVelocitySlider.width/10-angularVelocitySlider.cursor_width/2
            }
            txt: "100"
        }

        SaveButton {
                id: applyButtonVelocity
                langue: settingsPage.langue
                txt: langue == "English" ? "Apply" : "应用"
                width: 70
                font.pointSize: 11

                anchors.top: lineMeasurement52.bottom
                anchors.topMargin: 15
                anchors.left: parent.left
                anchors.right: parent.right

                onReleased: {
                    if ((ipRobotWifi !== "")) {
                        saveVelocitySignal(ipRobotWifi, getLinearVelocity(linearVelocitySlider.value), getAngularVelocity(angularVelocitySlider.value));
                    } else {
                        robotSelectionDialog.open();
                    }
                }

                function getLinearVelocity(lv) {
                    if (lv === 0) {
                        lv = 0.2;
                    } else if (lv === 1) {
                        lv = 1;
                    } else {
                        lv = lv + 0.1;
                    }
                    return lv;
                }

                function getAngularVelocity(av) {
                    if (av === 0) {
                        av = 20;
                    } else if (av === 1) {
                        av = 100;
                    } else {
                        av = 100*av + 10
                    }
                    return av;
                }
            }

        ToolSeparator {
            id: horizontalSeparation3
            orientation: Qt.Horizontal
            anchors {
                top: applyButtonVelocity.bottom
                left: parent.left
                right: parent.right
                topMargin: 10
            }
        }

        Item {

            id: batteryLabel

            height: 15

            anchors {
                top: horizontalSeparation3.bottom
                topMargin: 10
            }

            Label {
                id: batteryHelp

                anchors {
                    left: parent.left
                    top: parent.top
                }

                color: Style.darkSkyBlue
                font.pointSize: 11
                text: langue === "English" ? qsTr("Low Battery Settings") : qsTr("低电量设置")
            }

            HelpButton {

                height: 20
                width: 20

                anchors {
                    left: batteryHelp.right
                    leftMargin: 5
                    verticalCenter: batteryHelp.verticalCenter
                }

                tooltipText: langue === "English" ? "When battery is lower than this value, Robot starts auto docking" : "当电池低于设置值,机器人开始自动充电"
            }
        }

        BatteryLevelSlider {
            id: batterySlider
            anchors {
                top: batteryLabel.bottom
                topMargin: 10
                left: parent.left
                right: parent.right
            }
        }

        SliderLineMeasurement {
            id: lineMeasurement1
            anchors {
                top: batterySlider.bottom
    //            topMargin: batterySlider.cursor_height-10
                left: parent.left
                leftMargin: batterySlider.width/10-batterySlider.cursor_width/2
            }
            txt: "10%"
        }

        SliderLineMeasurement {
            id: lineMeasurement2
            anchors {
                top: batterySlider.bottom
    //            topMargin: batterySlider.cursor_height-10
                left: lineMeasurement1.right
                leftMargin: (batterySlider.width/2 - lineMeasurement1.x) / 2 - lineMeasurement1.width - 4
            }
            txt: "20%"
        }

        SliderLineMeasurement {
            id: lineMeasurement3
            anchors {
                top: batterySlider.bottom
    //            topMargin: batterySlider.cursor_height-10
                left: lineMeasurement2.right
                horizontalCenter: batterySlider.horizontalCenter
            }
            txt: "30%"
        }

        SliderLineMeasurement {
            id: lineMeasurement4
            anchors {
                top: batterySlider.bottom
    //            topMargin: batterySlider.cursor_height-10
                left: lineMeasurement3.right
                leftMargin: 4
            }
            txt: "40%"
        }

        SliderLineMeasurement {
            id: lineMeasurement5
            anchors {
                top: batterySlider.bottom
    //            topMargin: batterySlider.cursor_height-10
                right: parent.right
                rightMargin: batterySlider.width/10-batterySlider.cursor_width/2
            }
            txt: "50%"
        }

        SaveButton {
                id: applyButtonBattery
                langue: settingsPage.langue
                txt: langue == "English" ? "Apply" : "应用"
                width: 70
                font.pointSize: 11

                anchors.top: lineMeasurement5.bottom
                anchors.topMargin: 15
                anchors.left: parent.left
                anchors.right: parent.right

                onReleased: {
                    batteryWarningThreshold = batterySlider.value;
                    if ((ipRobotWifi !== "")) {
                        saveBatterySignal(ipRobotWifi, getBattery(batterySlider.value))
                    } else {
                        robotSelectionDialog.open();
                    }
                }

                function getBattery(bat) {
                    bat = 100*bat - 50*bat + 5
                    if (bat === 5) {
                        bat = 0;
                    } else if (bat >= 50) {
                        bat = 50;
                    }
                    return bat
                }

        }

        ToolSeparator {
            id: horizontalSeparation7
            orientation: Qt.Horizontal
            anchors {
                top: applyButtonBattery.bottom
                left: parent.left
                right: parent.right
                topMargin: 10
            }
        }

        Label {
            id: choiceMapLabel
            anchors {
                left: parent.left
                top: horizontalSeparation7.bottom
                topMargin: 20
            }

            color: Style.darkSkyBlue
            font.pointSize: 11
            text: langue === "English" ? qsTr("Map Sync. Settings") : qsTr("地图同步设置")
        }

        HelpButton {
            height: 20
            width: 20
            anchors {
                left: choiceMapLabel.right
                leftMargin: 5
                verticalCenter: choiceMapLabel.verticalCenter
            }
            tooltipText: langue === "English" ? "Where to find the map in which your robot operates" : "设置选择机器人或本地地图的优先级"
        }

        // the radio buttons to choose which map is used for the robots
        Rectangle {

            id: mapChoices

            height: 90

            anchors {
                top: choiceMapLabel.bottom
                topMargin: 20
                left: parent.left
                leftMargin: 20
            }

            ButtonGroup {
                id: mapChoiceGroup
            }

            RoundCheckBox {
                id: mapChoice1
                ButtonGroup.group: mapChoiceGroup
                checked: mapChoice == 0
                text: langue === "English" ? qsTr("The robot's map") : qsTr("机器人地图")
                onClicked: mapChoice = 0
            }

            RoundCheckBox {
                id: mapChoice2
                ButtonGroup.group: mapChoiceGroup
                checked: mapChoice == 1

                anchors {
                    left: parent.left
                    top: mapChoice1.bottom
                    topMargin: 12
                }

                text: langue === "English" ? qsTr("The application's map") : qsTr("本地地图")
                onClicked: {
                    mapChoice = 1
                    batteryWarningThreshold = batterySlider.value;
                    saveSettingsSignal(mapChoice, batterySlider.value, languageChoice);
                }
            }

            RoundCheckBox {
                id: mapChoice3
                ButtonGroup.group: mapChoiceGroup
                checked: mapChoice == 2

                anchors {
                    left: parent.left
                    top: mapChoice2.bottom
                    topMargin: 12
                }

                text: langue === "English" ? qsTr("Always ask me") : qsTr("总是询问我")
                onClicked: {
                    mapChoice = 2
                    batteryWarningThreshold = batterySlider.value;
                    saveSettingsSignal(mapChoice, batterySlider.value, languageChoice);
                }
            }
        }

//        SaveButton {
//               id: applyButtonMapChoice
//               langue: settingsPage.langue
//               txt: langue == "English" ? "应用" : "Apply"
//               width: 70
//               font.pointSize: 11

//               anchors.top: mapChoices.bottom
//               anchors.topMargin: 15
//               anchors.left: parent.left
//               anchors.right: parent.right

//               onReleased: {
//                   batteryWarningThreshold = batterySlider.value;
//                   saveSettingsSignal(mapChoice, batterySlider.value, languageChoice);
//               }
//           }

        ToolSeparator {
            id: horizontalSeparation4
            orientation: Qt.Horizontal
            anchors {
                top: mapChoices.bottom
                left: parent.left
                right: parent.right
                topMargin: 10
            }
        }

        Label {
            id: choiceLanguageLabel
            anchors {
                left: parent.left
                top: horizontalSeparation4.bottom
                topMargin: 20
            }

            color: Style.darkSkyBlue
            font.pointSize: 11
            text: langue === "English" ? qsTr("Language Settings") : qsTr("语言设置")
        }

        // the radio buttons to choose which map is used for the robots
        Rectangle {

            id: languageChoices

            height: 50

            anchors {
                top: choiceLanguageLabel.bottom
                topMargin: 20
                left: parent.left
                leftMargin: 20
            }

            ButtonGroup {
                id: languageChoiceGroup
            }

            RoundCheckBox {
                id: languageChoice1
                ButtonGroup.group: languageChoiceGroup
                checked: languageChoice == 0
                text: qsTr("中文")
                anchors {
                    left: parent.left
                }

                onClicked: {
                    languageChoice = 0;
                    langue = "中文";
                    pathModel.langue = "中文";
//                    openCreatePointMenu();
                    pathModel.languageChoice(languageChoice);
                    saveSettingsSignal(settingsPage.mapChoice, batterySlider.value, languageChoice);
                }
            }

            RoundCheckBox {
                id: languageChoice2
                ButtonGroup.group: languageChoiceGroup
                checked: languageChoice == 1

                anchors {
                    left: parent.left
                    top: languageChoice1.bottom
                    topMargin: 12
                }
                text: qsTr("English")

                onClicked: {
                    languageChoice = 1;
                    langue = "English";
                    pathModel.langue = "English";
//                    openCreatePointMenu();
                    pathModel.languageChoice(languageChoice);
                    saveSettingsSignal(settingsPage.mapChoice, batterySlider.value, languageChoice);
                }
            }
        }

        ToolSeparator {
            id: horizontalSeparation10
            orientation: Qt.Horizontal
            anchors {
                top: languageChoices.bottom
                left: parent.left
                right: parent.right
                topMargin: 5
            }
        }

//        Button {
//                id: deconnexionBtn
//                height: 40
//                width: 70
//                checkable: true
////                text: langue === "English" ? "断开" : "Exit"


//                objectName: "deconnexionBtn"
//                signal deconnexion()
//                contentItem: Label {
//                    text: deconnexionBtn.text
//                    font: deconnexionBtn.font
//                    verticalAlignment: Text.AlignVCenter
//                    color: Style.midGrey2
//                }

//                background: Rectangle {
//                    color: deconnexionBtn.checked ? "Red" : "transparent"
//                }

//                anchors {
//                    left : parent.left
//                    top : horizontalSeparation10.bottom;
//                    topMargin: 5
//                }

//                onClicked: {
////                    deconnexion()
//                    Qt.quit()
//                }
//            }

//        ToolSeparator {
//            id: horizontalSeparation11
//            orientation: Qt.Horizontal
//            anchors {
//                top: deconnexionBtn.bottom
//                left: parent.left
//                right: parent.right
//                topMargin: 5
//            }
//        }

        Button {
            id: btnExit
            height: 23

            CustomLabel {
                text: langue == "English" ? "Exit Application" : "退出程序"
                font.pointSize: 11
                color: Style.darkSkyBlue
                verticalAlignment: Text.AlignVCenter
                anchors {
                    left: parent.left
                    leftMargin: 0
                }
            }

            background: Rectangle {
                color: btnExit.pressed ? Style.lightBlue : "transparent"
                width: parent.width
                height: 50
            }

            width: parent.width

            anchors.top: horizontalSeparation10.bottom
            anchors.left: parent.left
            anchors.topMargin: 10

            onClicked: {
                exitDialog.open();
            }
        }

        ToolSeparator {
            id: horizontalSeparation11
            orientation: Qt.Horizontal
            anchors {
                top: btnExit.bottom
                left: parent.left
                right: parent.right
                topMargin: 10
            }
        }

        Label {
            id: versionApp
            text: "Version 1.04 updated on 24/07/2018"
            color: Style.midGrey
            font.italic: true
            font.pointSize: 8
            anchors {
//                top: horizontalSeparation11.bottom
//                topMargin: 20
                bottom: parent.bottom
                left: parent.left
                right: parent.right
            }
        }

        CustomDialog {
            id: exitDialog
            parent: ApplicationWindow.overlay
            x: (parent.width - width) / 2
            y: (parent.height - height) / 2
            height: 130
            topMarginLabel: langue === "English" ? 10 : 5;
            bottomMarginLabel: langue === "English" ? 10 : 20;
            leftMarginLabel: langue === "English" ? 40 : 120;
            rightMarginLabel: langue === "English" ? 40 : 120
            title: langue == "English" ? "Warning"  : "警告"
            message: langue == "English" ? "\nDo you want to exit the application ？" : "\n你想退出应用程序吗?"
            acceptMessage: langue == "English" ? "Yes" : "确认"
            rejectMessage: langue == "English" ? "Cancel" : "取消"
            onAccepted: {
                Qt.quit();
            }
//            onRejected: console.log("Cancel");
        }



//        CancelButton {
//            id: cancelButton
//            langue: settingsPage.langue
//            width: 70

//            anchors.bottom: parent.bottom
//            anchors.left: parent.left
//            anchors.right: parent.right

//            onClicked: {
//                nameRobot = "";
//                settingsPage.close()
//            }
//        }

        CustomDialog {
            id: wifiDialog
            parent: ApplicationWindow.overlay
            x: (parent.width - width) / 2
            y: (parent.height - height) / 2
            height: 130
            topMarginLabel: langue === "English" ? 10 : 5;
            bottomMarginLabel: langue === "English" ? 10 : 0;
            leftMarginLabel: langue === "English" ? 20 : 80;
            rightMarginLabel: langue === "English" ? 20 : 80;
            title: langue == "English" ? "Warning"  : "警告"
            message: langue == "English" ? "\nPlease select a robot or fill the WiFi name" : "\n请选择机器人或输入WiFi名称"
            acceptMessage: langue == "English" ? "Ok" : "是"
        }

        CustomDialog {
            id: robotSelectionDialog
            parent: ApplicationWindow.overlay
            x: (parent.width - width) / 2
            y: (parent.height - height) / 2
            height: 130
            topMarginLabel: langue === "English" ? 10 : 5;
            bottomMarginLabel: langue === "English" ? 10 : 20;
            leftMarginLabel: langue === "English" ? 100 : 80;
            rightMarginLabel: langue === "English" ? 100 : 80
            title: langue == "English" ? "Warning"  : "警告";
            message: langue == "English" ? "\nPlease select a robot" : "\n请选择机器人或输入WiFi名称"
            acceptMessage: langue == "English" ? "Ok" : "是"

        }
    }

    function setSettings(mapChoice, languageChoice){
        oriMapChoice = mapChoice;
        oriLanguageChoice = languageChoice;
        settingsPage.mapChoice = mapChoice;
        settingsPage.languageChoice = languageChoice;
    }

    function getWifiList(wifiList, count) {
        ssidWifi.push(wifiList);
//        console.log("wifiList = " + wifiList);
    }

    property int sizeWifiList

    property variant toto

    function getSizeWifiList(sizeListWifi) {
        sizeWifiList = ssidWifi.length
        toto = ssidWifi
    }

}
