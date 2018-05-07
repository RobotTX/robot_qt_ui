import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Model/Speech"
import "../../Model/Robot"
import "../Custom"
import "../Speech"

Page {
    id: page
    anchors.fill: parent

    property Speechs speechModel
    property Robots robotModel
    property string langue

    signal closeMenu()
    signal setMessageTop(int status, string msg)
    signal editSpeech(string name, string groupName)

    property int menuIndex: 0

    onVisibleChanged: {
        if(visible)
            menuIndex = 0;
    }

    Frame {
        id: speechMenuFrame
        visible: menuIndex == 0
        anchors.fill: parent
        padding: 0
        SpeechMenuHeader {
            id: speechMenuHeader
            langue: page.langue
            onOpenCreateSpeechMenu: menuIndex = 1;
            onOpenCreateGroupMenu: menuIndex = 2;
            onCloseMenu: page.closeMenu()
        }

        SpeechMenuContent {
            id: speechMenuContent
            objectName: "speechMenuContent"
            speechModel: page.speechModel
            robotModel: page.robotModel
            langue: page.langue
            anchors {
                left: parent.left
                top: speechMenuHeader.bottom
                right: parent.right
                bottom: parent.bottom
            }
            onRenameGroup: {
                createGroupMenuContent.oldName = name;
                menuIndex = 2;
            }
            onEditSpeech: {
                createSpeechMenuContent.oldName = name;
                createSpeechMenuContent.oldGroup = groupName;
                // will open the point menu
                menuIndex = 1;
            }
        }
    }

    Frame {
        id: createPointMenuFrame
        visible: menuIndex == 1
        anchors.fill: parent
        padding: 0

        CreateMenuHeader {
            id: createPointMenuHeader
            onBackToMenu: menuIndex = 0;
            langue: page.langue
            txt: langue == "English" ? "语音" : "Speech"
        }

        CreateSpeechMenuContent {
            id: createSpeechMenuContent
            speechModel: page.speechModel
            langue: page.langue
            anchors {
                left: parent.left
                top: createPointMenuHeader.bottom
                right: parent.right
                bottom: parent.bottom
            }
            onBackToMenu: menuIndex = 0;
            onSetMessageTop: page.setMessageTop(status, msg)
        }
    }

    Frame {
        id: createGroupMenuFrame
        visible: menuIndex == 2
        anchors.fill: parent
        padding: 0

        CreateGroupMenuHeader {
            id: createGroupMenuHeader
            langue: page.langue
            onBackToMenu: menuIndex = 0;
        }

        CreateGroupMenuContent {
            id: createGroupMenuContent
            objectName: "createSpeechGroupMenu"
            langue: page.langue
            anchors {
                left: parent.left
                top: createGroupMenuHeader.bottom
                right: parent.right
                bottom: parent.bottom
            }
            onBackToMenu: {
                oldName = "";
                menuIndex = 0;
            }
            onSetMessageTop: page.setMessageTop(status, msg)
        }
    }
}


