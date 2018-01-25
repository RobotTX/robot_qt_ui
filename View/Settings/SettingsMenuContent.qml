import QtQuick 2.7
import QtQuick.Controls 2.1
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
    signal saveSettingsSignal(int mapChoice, double _batteryThreshold)
    signal saveWifiSignal(string ip_wifi, string wifi, string pwd_wifi)
    signal changeLanguage(string language)
    signal openCreatePointMenu()
    signal saveVelocitySignal(string ipRobotWifi, double linearVelocity, double angularVelocity)
    signal saveBatterySignal(string ipRobotWifi, double battery)

    property string inputNameWifi: ""
    property string inputPwdWifi: ""
    property string ipRobotWifi: ""
    property int currentMenuIndex: -1

    property real batteryWarningThreshold
    property int mapChoice
    property string langue
    property string nameRobot

    property real oriBatteryWarningThreshold
    property int oriMapChoice

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
        settingsPage.mapChoice = oriMapChoice;
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
        contentHeight: contentItem.childrenRect.height
        anchors.fill: parent
        anchors.topMargin: 10

        Label {
            id: robotLabel
            anchors {
                left: parent.left
                top: parent.top
                topMargin: -10
            }

            color: "#8F8E94"
            text: langue === "English" ? qsTr("哪一个无线网将被使用 ?") : qsTr("Robot Selection")
        }

        PopupMenuItem {
            id: robotList
            height: Style.menuItemHeight
            width: parent.width
            labelText: {
                console.log("nameRobot = " + nameRobot);
                if (nameRobot === "") {
                    langue === "English" ? "选择机器人" : "Select a robot"
                } else {
                    langue === "English" ? "选择了 " + nameRobot + " 机器人" : "Robot " + nameRobot + " selected"
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
            onHoveredChanged: if(visible){ currentMenuIndex = 0 } /// desktop
//            onClicked: if(visible){ currentMenuIndex = 0 } /// android

            RobotListInPopup {
                x: robotList.width
                visible: settingsPage.currentMenuIndex === 0
                onVisibleChanged: if(!visible) currentMenuIndex = -1
                robotModel: settingsPage.robotModel
                onRobotSelected: {
                    console.log("ip = " + ip + " name = " + name);
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

            color: "#8F8E94"
            text: langue === "English" ? qsTr("哪一个无线网将被使用 ?") : qsTr("WiFi Settings")
        }

        Label {
            id: nameWifi
            text: langue === "English" ? qsTr("无线网名称") : qsTr("Name WiFi : ")
            color: Style.greyText
            font.pointSize: 10
            anchors {
                left: parent.left
                top: wifiLabel.bottom
                right: parent.right
            }
            anchors.topMargin: 20
        }

        TextField {
            id: userInputWifiName
            objectName: "wifiName"
            selectByMouse: true
            placeholderText: langue === "English" ? qsTr("请输入无线网名称") : qsTr("Enter the name of the WiFi")
            background: Rectangle {
                    implicitWidth: 100
                    implicitHeight: 15
                    border.color: Style.midGrey
            }
            font.pointSize: 10
            text: inputNameWifi
            anchors {
                left: nameWifi.right
                bottom: nameWifi.bottom
            }
            anchors.leftMargin: -170
            onTextChanged: {
                inputNameWifi = userInputWifiName.text
            }
        }

        Label {
            id: pwdWifi
            text: langue === "English" ? qsTr("密码 : ") : qsTr("Password : ")
            color: Style.greyText
            font.pointSize: 10
            anchors {
                left: nameWifi.left
                top: nameWifi.bottom
                right: parent.right
            }
            anchors.topMargin: 15
        }

        TextField {
            id: userInputWifiPwd
            objectName: "wifiPwd"
            selectByMouse: true
            placeholderText: langue === "English" ? qsTr("输入无线网密码") : qsTr("Enter password")
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
                console.log("input user pwd = " + userInputWifiPwd.text);
                inputPwdWifi = userInputWifiPwd.text
            }
        }

        // apply button to save the changes but keep the window open
           SaveButton {
               id: applyButtonWifi
               langue: settingsPage.langue
               txt: langue == "English" ? "应用" : "Apply"
               width: 70

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

            color: "#8F8E94"
            text: langue === "English" ? qsTr("Velocity chinese") : qsTr("Velocity Settings")
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

                color: "#8F8E94"
                text: langue === "English" ? qsTr("Linear velocity chinese") : qsTr("Linear Velocity")
            }

            HelpButton {

                height: 20
                width: 20

                anchors {
                    left: linearVelocityHelp.right
                    leftMargin: 5
                    verticalCenter: linearVelocityHelp.verticalCenter
                }

                tooltipText: langue === "English" ? "Linear velocity chinese" : "Recommend 0.4 m/s"
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

                color: "#8F8E94"
                text: langue === "English" ? qsTr("Angular velocity chinese") : qsTr("Angular Velocity")
            }

            HelpButton {

                height: 20
                width: 20

                anchors {
                    left: angularVelocityHelp.right
                    leftMargin: 5
                    verticalCenter: angularVelocityHelp.verticalCenter
                }

                tooltipText: langue === "English" ? "Angular velocity chinese" : "Recommend 40 deg/s"
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
                txt: langue == "English" ? "应用" : "Apply"
                width: 70

                anchors.top: lineMeasurement52.bottom
                anchors.topMargin: 15
                anchors.left: parent.left
                anchors.right: parent.right

                onReleased: {
                    if ((ipRobotWifi !== "")) {
                        console.log("linearSlider = ", linearVelocitySlider.value);
                        console.log("angularVelocitySlider = ", angularVelocitySlider.value)
                        saveVelocitySignal(ipRobotWifi, getLinearVelocity(linearVelocitySlider.value), getAngularVelocity(angularVelocitySlider.value));
                    } else {
                        wifiDialog.open();
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

                color: "#8F8E94"
                text: langue === "English" ? qsTr("低电量警告") : qsTr("Battery level Settings")
            }

            HelpButton {

                height: 20
                width: 20

                anchors {
                    left: batteryHelp.right
                    leftMargin: 5
                    verticalCenter: batteryHelp.verticalCenter
                }

                tooltipText: langue === "English" ? "电池低于多少百分比将发出警告" : "Level of battery under which you receive a warning"
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
                txt: langue == "English" ? "应用" : "Apply"
                width: 70

                anchors.top: lineMeasurement5.bottom
                anchors.topMargin: 15
                anchors.left: parent.left
                anchors.right: parent.right

                onReleased: {
                    batteryWarningThreshold = batterySlider.value;
                    console.log("***batterySlider.value*** = ",batterySlider.value);
                    if ((ipRobotWifi !== "")) {
                        saveBatterySignal(ipRobotWifi, getBattery(batterySlider.value))
                        console.log("value battery = ", getBattery((batterySlider.value)))
                    } else {
                        wifiDialog.open();
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

            color: "#8F8E94"
            text: langue === "English" ? qsTr("哪一个地图将被使用 ?") : qsTr("Which map do you want to use ?")
        }

        HelpButton {
            height: 20
            width: 20
            anchors {
                left: choiceMapLabel.right
                leftMargin: 5
                verticalCenter: choiceMapLabel.verticalCenter
            }
            tooltipText: langue === "English" ? "设置选择机器人或本地地图的优先级" : "Where to find the map in which your robot operates"
        }

        // the radio buttons to choose which map is used for the robots

        Rectangle {

            id: mapChoices

            height: 130

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
                text: langue === "English" ? qsTr("机器人地图") : qsTr("The robot's map")
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

                text: langue === "English" ? qsTr("本地地图") : qsTr("The application's map")
                onClicked: mapChoice = 1
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

                text: langue === "English" ? qsTr("总是询问我") : qsTr("Always ask me")
                onClicked: mapChoice = 2
            }

            RoundCheckBox {
                id: mapChoice4
                ButtonGroup.group: mapChoiceGroup
                checked: mapChoice == 3
                anchors {
                    left: parent.left
                    top: mapChoice3.bottom
                    topMargin: 12
                }
                text: langue === "English" ? qsTr("最新的地图") : qsTr("The newest map")
                onClicked: mapChoice = 3
            }

            RoundCheckBox {
                id: mapChoice5
                ButtonGroup.group: mapChoiceGroup
                checked: mapChoice == 4
                anchors {
                    left: parent.left
                    top: mapChoice4.bottom
                    topMargin: 12
                }

                text: langue === "English" ? qsTr("最旧的地图") : qsTr("The oldest map")
                onClicked: mapChoice = 4
            }
        }

        SaveButton {
               id: applyButtonMapChoice
               langue: settingsPage.langue
               txt: langue == "English" ? "应用" : "Apply"
               width: 70

               anchors.top: mapChoices.bottom
               anchors.topMargin: 15
               anchors.left: parent.left
               anchors.right: parent.right

               onReleased: {
                   console.log("batterySLider.value = ", batterySlider.value)
                   batteryWarningThreshold = batterySlider.value;
                   saveSettingsSignal(mapChoice, batterySlider.value);
               }
           }

        ToolSeparator {
            id: horizontalSeparation4
            orientation: Qt.Horizontal
            anchors {
                top: applyButtonMapChoice.bottom
                left: parent.left
                right: parent.right
                topMargin: 10
            }
        }

        Button {
                id: changeLanguageBtn
                height: 40
                width: 70
                checkable: true
                text: ""
                objectName: "language"
                contentItem: Label {
                    text: changeLanguageBtn.checked ? "English" : "中文"
                    font: changeLanguageBtn.font
                    verticalAlignment: Text.AlignVCenter
                    color: Style.midGrey2
                }

                background: Rectangle {
                    color: "transparent"
                }

                anchors {
                    left : parent.left
                    top : horizontalSeparation4.bottom;
                }
                onClicked: {
                    if (changeLanguageBtn.checked) {
                        changeLanguage("English");
                        langue = "English";
                        robotModel.langue = "English"
                        pathModel.langue = "English"
                    } else {
                        changeLanguage("Chinese");
                        langue = "Chinese";
                        robotModel.langue = "Chinese"
                        pathModel.langue = "English"
                    }
                    openCreatePointMenu()
                }
            }

        ToolSeparator {
            id: horizontalSeparation10
            orientation: Qt.Horizontal
            anchors {
                top: changeLanguageBtn.bottom
                left: parent.left
                right: parent.right
                topMargin: 10
            }
        }

        Button {
                id: deconnexionBtn
                height: 40
                width: 70
                checkable: true
                text: ""//langue === "English" ? "断开" : "Deconnection"

                objectName: "deconnexionBtn"
                signal deconnexion()
                contentItem: Label {
                    text: deconnexionBtn.text
                    font: deconnexionBtn.font
                    verticalAlignment: Text.AlignVCenter
                    color: Style.midGrey2
                }

                background: Rectangle {
                    color: deconnexionBtn.checked ? "Red" : "transparent"
                }

                anchors {
                    left : parent.left
                    top : changeLanguageBtn.bottom;
                }

                onClicked: {
                    deconnexion()
        //            Qt.quit()
                }
            }

        CancelButton {
            id: cancelButton
            langue: settingsPage.langue
            width: 70

            anchors.bottom: parent.bottom
            anchors.left: parent.left
            anchors.right: parent.right

            onClicked: {
                nameRobot = "";
                settingsPage.close()
            }
        }

        CustomDialog {
            id: wifiDialog
            x: settingsPage.width
            y: settingsPage.height / 2 - height
            height: 60
            title: langue == "English" ? "警告"  : "Warning"
            acceptMessage: langue == "English" ? "请选择机器人或输入WiFi名称" : "Please select a robot or fill the WiFi name"
        }
    }

    function setSettings(mapChoice){
        oriMapChoice = mapChoice;
        settingsPage.mapChoice = mapChoice;
    }
}
