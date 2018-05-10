import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style
import "../../Helper/helper.js" as Helper
import "../../Model/Speech"
import "../../Model/Robot"
import "../Custom"

Frame {
    id: speechMenuFrame
    objectName: "speechMenuContent"
    padding: 0
    property string langue
    property Speechs speechModel
    property Robots robotModel

    signal deleteSpeech(string name, string groupName)
    signal deleteGroup(string name)
    signal renameGroup(string name)
    signal moveTo(string name, string oldGroup, string newGroup)
    signal editSpeech(string name, string groupName)

    background: Rectangle {
        color: Style.lightGreyBackground
        border.color: Style.lightGreyBorder
        border.width: 1
    }

    /// This frame is displayed when there is no speech
    EmptyMenu {
        /// Only the invisible "Default" left and it's empty
        visible: (speechModel.count === 1 && speechModel.get(0).speechs.count === 0) || speechModel.count === 0
        txt: langue == "English" ? "没有任何语音文本，点击 '+' 按钮，创建语音文本" : "You don't have any speechs yet, click the '+' button to create a speech."
        imgSrc: "qrc:/icons/big_speech"

    }

    Component {
        id: delegate
        SpeechListItem {
            column: columnId
            width: speechMenuFrame.width
            speechModel: speechMenuFrame.speechModel
            robotModel: speechMenuFrame.robotModel
            langue: speechMenuFrame.langue
            onRenameGroup: speechMenuFrame.renameGroup(name)
            onEditSpeech: speechMenuFrame.editSpeech(name, groupName)
        }
    }

    Flickable {
        id: flickSpeech
        ScrollBar.vertical: ScrollBar { }
        contentHeight: contentItem.childrenRect.height
        anchors.fill: parent
        anchors.topMargin: 10

        Column {
            id: columnId
            property string selectedGroup: langue == "English" ? Helper.noGroupChinese : Helper.noGroup
            property string selectedSpeech: (speechModel.count > 0) ? speechModel.get(0).speechs.count > 0 ? speechModel.get(0).speechs.get(0).name : "" : ""
            /// The list containing both the graphical and model of the speechs in the menu
            Repeater {
                model: {
                    speechModel
                }
                delegate: delegate
            }

        }
    }
}
