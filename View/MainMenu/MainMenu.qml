import QtQuick 2.7
import QtQuick.Controls 2.0

Frame {
    id: mainMenuFrame
    objectName: "mainMenuFrame"

    width: 66
    padding: 0

    anchors {
        left: parent.left
        top: parent.top
        bottom: parent.bottom
    }

    background: Rectangle {
        color: "#cc26303a"
    }

    signal selectMenu(string txt, bool checked)

    MenuButton {
        id: robotButton
        txt: "Robot"
        imgSrc: "qrc:/icons/robot"
        anchors {
            left: parent.left
            top: parent.top
        }
        onClicked: mainMenuFrame.uncheckButtons(txt, checked)
    }

    MenuButton {
        id: pathButton
        txt: "Path"
        imgSrc: "qrc:/icons/path"
        anchors.top: robotButton.bottom
        onClicked: mainMenuFrame.uncheckButtons(txt, checked)
    }

    MenuButton {
        id: pointButton
        txt: "Point"
        imgSrc: "qrc:/icons/point"
        anchors.top: pathButton.bottom
        onClicked: mainMenuFrame.uncheckButtons(txt, checked)
    }

    MenuButton {
        id: mapButton
        txt: "Map"
        imgSrc: "qrc:/icons/map"
        anchors.top: pointButton.bottom
        onClicked: mainMenuFrame.uncheckButtons(txt, checked)
    }

    MenuButton {
        id: settingsButton
        txt: "Settings"
        imgSrc: "qrc:/icons/settings"
        anchors.bottom: parent.bottom
        onClicked: mainMenuFrame.uncheckButtons(txt, checked)
    }

    /// Uncheck the other buttons when a new one is checked
    function uncheckButtons(txt, checked){

        if(checked){
            switch(txt){
                case 'Robot':
                    pathButton.checked = false;
                    pointButton.checked = false;
                    mapButton.checked = false;
                    settingsButton.checked = false;
                break;
                case 'Path':
                    robotButton.checked = false;
                    pointButton.checked = false;
                    mapButton.checked = false;
                    settingsButton.checked = false;
                break;
                case 'Point':
                    robotButton.checked = false;
                    pathButton.checked = false;
                    mapButton.checked = false;
                    settingsButton.checked = false;
                break;
                case 'Map':
                    robotButton.checked = false;
                    pathButton.checked = false;
                    pointButton.checked = false;
                    settingsButton.checked = false;
                break;
                case 'Settings':
                    robotButton.checked = false;
                    pathButton.checked = false;
                    pointButton.checked = false;
                    mapButton.checked = false;
                break;
            }
        }

        mainMenuFrame.selectMenu(txt, checked)
    }
}
