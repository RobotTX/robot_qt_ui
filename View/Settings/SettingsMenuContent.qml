import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style
import "../../Model/Robot"
import "../../View/Custom/"
import "../../Model/Tutorial"
import "../Robot"

Frame {

    id: settingsPage
    objectName: "settings"

    signal close()
    signal saveSettingsSignal(int mapChoice, double _batteryThreshold)
    signal saveWifiSignal(string ip_wifi, string wifi, string pwd_wifi)
    signal changeLanguage(string language)

    property string inputNameWifi: ""
    property string inputPwdWifi: ""
    property string ipRobotWifi: ""
    property string nameRobot: ""
    property int currentMenuIndex: -1

    property real batteryWarningThreshold
    property int mapChoice
    property string langue

    property real oriBatteryWarningThreshold
    property int oriMapChoice

    property Robots robotModel
    property Tutorial tutorial

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
    }

    Label {
        id: wifiLabel
        anchors {
            left: parent.left
            top: parent.top
            topMargin: 5
        }

        color: "#8F8E94"
        text: langue === "English" ? qsTr("哪一个无线网将被使用 ?") : qsTr("Which WiFi do you want to use ?")
    }

    PopupMenuItem {
        id: robotList
        height: Style.menuItemHeight
        width: parent.width
//        labelText: nameRobot === "" ? "Select a robot" : "Robot " + nameRobot + " selected"
        labelText: {
            if (nameRobot === "") {
                langue === "English" ? "选择机器人" : "Select a robot"
            } else {
                langue === "English" ? "选择了 " + nameRobot + " 机器人" : "Robot " + nameRobot + " selected"
            }
        }

//        leftPadding: Style.menuItemLeftPadding
        anchors.left: parent.left
        anchors.top: wifiLabel.bottom
        anchors.topMargin: 10

        Image {
            asynchronous: true
            source: "qrc:/icons/arrow"
            fillMode: Image.Pad // For not stretching image
            anchors.verticalCenter: parent.verticalCenter
            anchors.right: parent.right
            anchors.rightMargin: 12
        }
        onHoveredChanged: if(visible){ currentMenuIndex = 0 }

        RobotListInPopup {
            x: robotList.width
            visible: settingsPage.currentMenuIndex === 0
            onVisibleChanged: if(!visible) currentMenuIndex = -1
            robotModel: settingsPage.robotModel
            onRobotSelected: {
                console.log("ip = " + ip + " name = " + name);
                ipRobotWifi = ip;
                nameRobot = name;
            }
        }
    }

    Label {
        id: nameWifi
        text: langue === "English" ? qsTr("无线网名称") : qsTr("Name WiFi : ")
        color: Style.greyText
        font.pointSize: 10
        anchors {
            left: parent.left
            top: robotList.bottom
            right: parent.right
        }
        anchors.topMargin: 15
    }

    TextField {
        id: userInputWifiName
        objectName: "wifiName"
        selectByMouse: true
        placeholderText: langue === "English" ? qsTr("请输入无线网名称") : qsTr("Enter the name of the WiFi")
        background: Rectangle {
                implicitWidth: 100
                implicitHeight: 15
//                color: control.enabled ? "transparent" : "#353637"
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
            console.log("input of wifi name = " + userInputWifiName.text);
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
//                color: control.enabled ? "transparent" : "#353637"
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

    ToolSeparator {
        id: horizontalSeparation7
        orientation: Qt.Horizontal
        anchors {
            top: userInputWifiPwd.bottom
            left: parent.left
            right: parent.right
            topMargin: 20
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

    ToolSeparator {
        id: horizontalSeparation2
        orientation: Qt.Horizontal
        anchors {
            top: mapChoices.bottom
            left: parent.left
            right: parent.right
            topMargin: 20
        }
    }

    Item {

        id: batteryLabel

        height: 15

        anchors {
            top: horizontalSeparation2.bottom
            topMargin: 20
        }

        Label {
            id: batteryHelp

            anchors {
                left: parent.left
                top: parent.top
            }

            color: "#8F8E94"
            text: langue === "English" ? qsTr("低电量警告") : qsTr("Battery level warning trigger")
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
            topMargin: 16
            left: parent.left
            right: parent.right
        }
    }

    SliderLineMeasurement {
        id: lineMeasurement1
        anchors {
            top: batterySlider.bottom
            topMargin: batterySlider.cursor_height-10
            left: parent.left
            leftMargin: batterySlider.width/10-batterySlider.cursor_width/2
        }
        txt: "10%"
    }

    SliderLineMeasurement {
        id: lineMeasurement2
        anchors {
            top: batterySlider.bottom
            topMargin: batterySlider.cursor_height-10
            left: lineMeasurement1.right
            leftMargin: (batterySlider.width/2 - lineMeasurement1.x) / 2 - lineMeasurement1.width - 4
        }
        txt: "20%"
    }

    SliderLineMeasurement {
        id: lineMeasurement3
        anchors {
            top: batterySlider.bottom
            topMargin: batterySlider.cursor_height-10
            left: lineMeasurement2.right
            horizontalCenter: batterySlider.horizontalCenter
        }
        txt: "30%"
    }

    SliderLineMeasurement {
        id: lineMeasurement4
        anchors {
            top: batterySlider.bottom
            topMargin: batterySlider.cursor_height-10
            left: lineMeasurement3.right
            leftMargin: 4
        }
        txt: "40%"
    }

    SliderLineMeasurement {
        id: lineMeasurement5
        anchors {
            top: batterySlider.bottom
            topMargin: batterySlider.cursor_height-10
            right: parent.right
            rightMargin: batterySlider.width/10-batterySlider.cursor_width/2
        }
        txt: "50%"
    }

    ToolSeparator {
        id: horizontalSeparation3
        orientation: Qt.Horizontal
        anchors {
            top: lineMeasurement1.bottom
            left: parent.left
            right: parent.right
            topMargin: 20
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
                top : horizontalSeparation3.bottom;
            }
            onClicked: {
                if (changeLanguageBtn.checked) {
                    changeLanguage("English");
                    langue = "English";
                    robotModel.langue = "English"
                } else {
                    changeLanguage("Chinese");
                    langue = "Chinese";
                    robotModel.langue = "Chinese"
                }

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
            text: langue === "English" ? "断开" : "Deconnection"
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
                top : horizontalSeparation10.bottom;
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

        onClicked: settingsPage.close()
    }

    // apply button to save the changes but keep the window open
    SaveButton {
        id: applyButton
        langue: settingsPage.langue
        txt: langue == "English" ? "应用" : "Apply"
        width: 70

        anchors.horizontalCenter: parent.horizontalCenter
        anchors.bottom: parent.bottom

        onReleased: {
            batteryWarningThreshold = batterySlider.value;
            saveSettingsSignal(mapChoice, batterySlider.value);
            if ((inputNameWifi !== "") && (ipRobotWifi !== "")) {
                console.log("ip = " + ipRobotWifi + " wifi name = " + inputNameWifi + " pwd = " + inputPwdWifi);
                saveWifiSignal(ipRobotWifi, inputNameWifi, inputPwdWifi);
            } else {
                wifiDialog.open();
            }
            nameRobot = "";


        }
    }

    CustomDialog {
        id: wifiDialog
        x: settingsPage.x / 2
        y: settingsPage.x / 2
        height: 60
        title: langue == "English" ? "警告"  : "Warning"
        acceptMessage: "Please select a robot or fill the WiFi name"
    }

    SaveButton {
        id: saveButton
        width: 70
        anchors.right: parent.right
        anchors.bottom: parent.bottom
        langue: settingsPage.langue
        onReleased: {
            if ((inputNameWifi !== "") && (ipRobotWifi !== "")) {
                console.log("ip = " + ipRobotWifi + " wifi name = " + inputNameWifi + " pwd = " + inputPwdWifi);
                saveWifiSignal(ipRobotWifi, inputNameWifi, inputPwdWifi);
            } else {
                wifiDialog.open();
            }
            batteryWarningThreshold = batterySlider.value;
            saveSettingsSignal(mapChoice, batterySlider.value);
            console.log("save settings signal called " + mapChoice + " " + batterySlider.value);
            nameRobot = "";
            settingsPage.close();
        }
    }

    function setSettings(mapChoice){
        oriMapChoice = mapChoice;
        settingsPage.mapChoice = mapChoice;
    }
}


