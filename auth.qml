import QtQuick 2.7
import QtQuick.Controls 1.4
import QtQuick.Controls 2.1
import QtQuick.Layouts 1.0
import QtQuick.Window 2.0
import "Helper/style.js" as Style
import "Helper/helper.js" as Helper

ApplicationWindow {
    id: auth
    objectName: "auth"
    property bool loginOk: false
    x: Screen.width / 2
    y: Screen.height / 2
    visible: true
    width: 350
    height: 120
    maximumWidth: 350
    maximumHeight:  120
    minimumWidth: 350
    minimumHeight: 120
    title: qsTr("Authentification Mobot Studio")
    Component.onCompleted: {
        setX(Screen.width / 2 - width / 2);
        setY(Screen.height / 2 - height / 2);
    }

    Item {

        Label {
            id: userId
            text: qsTr("User id")
            color: Style.midGrey2
            anchors {
                left: parent.left
                top: parent.top
                right: parent.right
            }
            anchors.topMargin: 20
            anchors.leftMargin: 30
        }

        TextField {
            id: userIdTextField
            objectName: "userIdObj"
            selectByMouse: true
            placeholderText: qsTr("Enter your user id")
            text: "admin"
            height: 30
            anchors {
                left: userId.right
                top: userId.top
            }
            anchors.leftMargin: 120
            anchors.topMargin: -5
        }

        Label {
            id: pwd
            text: qsTr("Password")
            color: Style.midGrey2
            anchors {
                left: userId.left
                top: userId.bottom
                right: parent.right
            }
            anchors.topMargin: 15
        }

        TextField {
            id: pwdTextField
            objectName: "pwdObj"
            selectByMouse: true
            placeholderText: qsTr("Enter your password")
            text: "admin"
            echoMode: TextInput.Password
            height: 30
            anchors {
                left: userIdTextField.left
                top: pwd.top
            }
            anchors.topMargin: -5
        }

        Button {
            id: loginBtn
            checkable: true
            text: "Login"

            objectName: "loginBtn"
            signal checkLogin()

            contentItem: Label {
                text: loginBtn.text
                font: loginBtn.font
                verticalAlignment: Text.AlignVCenter
                color: Style.loginBlue
            }

            width: 50
            height: 20
            background: Rectangle {
                color: loginBtn.checked ? "Red" : "transparent"
            }

            anchors {
                left: userIdTextField.left
                top: pwd.bottom
            }
            anchors.leftMargin: 30
            anchors.topMargin: 15

            onClicked: {
//                checkLogin()
                checkLog();
                if (loginOk) {
                    checkLogin()
                } else {

                }



            }
        }
    }

    function checkLog() {
        var userIdObj = Helper.formatName(userIdTextField.text);
        var pwdObj = Helper.formatName(pwdTextField.text);

        if (userIdObj !== "" && userIdObj === "admin" && pwdObj !== "" && pwdObj === "admin") {
//            console.log("authentification success")
            loginOk = true;
        } else {
            console.log("error authentification")
        }
    }

    function loadLanguage() {

    }
}
