import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQuick.Controls.Styles 1.4
import "../../Helper/style.js" as Style
import "../../Helper/helper.js" as Helper
import "../../Model/Speech"
import "../Custom"
import "../Speech"

Frame {
    id: createSpeechMenuFrame
    objectName: "createSpeechMenuFrame"
    property Speechs speechModel
    property string langue
    property string oldName: ""
    property string oldGroup: ""
    property string tts: ""
    property bool nameError: true
    property bool wasDisplayed: false
    property string errorMsg

    signal backToMenu()
    signal createSpeech(string name, string groupName, string tts, string oldName, string oldGroup)
    signal setMessageTop(int status, string msg)
    signal checkSpeech(string name, string oldname)


    onVisibleChanged: {
        ttsTextField.text = ""; /// initialized value of textfield when creating new speech
        if(!visible){
            /// When you finish or cancel an edition, we show the speech you were editing
            if(oldName !== ""){
                for(var i = 0; i < speechModel.count; i++)
                    if(speechModel.get(i).groupName === oldGroup)
                        for(var j = 0; j < speechModel.get(i).speechs.count; j++)
                            if(speechModel.get(i).speechs.get(j).name === oldName)
                                speechModel.get(i).speechs.setProperty(j, "isVisible", true);
            }

            oldName = "";
            oldGroup = "";
            var displayTextChange = "";
            if (langue == "English") {
                displayTextChange = Helper.noGroup;
            } else {
                displayTextChange = Helper.noGroupChinese;
            }
            groupComboBox.currentIndex = 0;
            groupComboBox.displayText = displayTextChange;
        } else {
            // if editing
            if(oldName !== ""){
                for(var i = 0; i < speechModel.count; i++)
                    if(speechModel.get(i).groupName === oldGroup)
                        for(var j = 0; j < speechModel.get(i).speechs.count; j++)
                            if(speechModel.get(i).speechs.get(j).name === oldName){
                                    speechModel.get(i).speechs.setProperty(j, "isVisible", false);
                                ttsTextField.text = speechModel.get(i).speechs.get(j).tts;
//                                textfield.text = speechModel.get(i).speechs.get(j).tts;
                                groupComboBox.displayText = oldGroup;
                            }
            }
        }
        speechTextField.text = oldName;
//        ttsTextField.text = tts;
        errorMsg = "";
        setMessageTop(1, errorMsg);
    }

    padding: 20

    background: Rectangle {
        color: Style.lightGreyBackground
        border.color: Style.lightGreyBorder
        border.width: 1

    }

    Label {
        id: speechLabel
        text: langue == "English" ? qsTr("Speech Label") : qsTr("语音名称")
        font.pointSize: Style.ubuntuSubHeadingSize
        color: Style.midGrey2
        anchors {
            left: parent.left
            top: parent.top
        }
    }

    TextField {
        id: speechTextField
        selectByMouse: true
        placeholderText: langue == "English" ? qsTr("Enter Label") : qsTr("输入语音名称")
        font.pointSize: Style.ubuntuSubHeadingSize
        text: oldName
        verticalAlignment: TextInput.AlignVCenter
        anchors {
            left: parent.left
            top: speechLabel.bottom
            right: parent.right
            topMargin: 8
        }
        onVisibleChanged: {
            if (visible)
                speechTextField.forceActiveFocus()
        }

        background: Rectangle {
            radius: 2
            border.color: nameError ? Style.errorColor : speechTextField.activeFocus ? Style.lightBlue : Style.lightGreyBorder
            border.width: speechTextField.activeFocus || nameError ? 3 : 1
        }

        onTextChanged: checkSpeech(Helper.formatName(speechTextField.text), oldName)
    }

    Label {
        id: groupLabel
        text: langue == "English" ? qsTr("Choose Group") : qsTr("选择分组")
        font.pointSize: Style.ubuntuSubHeadingSize
        color: Style.midGrey2
        anchors {
            left: parent.left
            top: speechTextField.bottom
            right: parent.right
            topMargin: 20
        }
    }

    CustomComboBox {
        id: groupComboBox
        model: speechModel
        font.pointSize: Style.ubuntuSubHeadingSize
        displayText: {
            if (oldName) {
                oldGroup
            } else {
                langue == "English" ? Helper.noGroup : Helper.noGroupChinese
            }
        }
        langue: createSpeechMenuFrame.langue
        anchors {
            left: parent.left
            top: groupLabel.bottom
            right: parent.right
            topMargin: 8
        }
    }

    Label {
        id: ttsLabel
        text: langue == "English" ? qsTr("Text") : qsTr("正文")
        font.pointSize: Style.ubuntuSubHeadingSize
        color: Style.midGrey2
        anchors {
            left: parent.left
            top: groupComboBox.bottom
            topMargin: 20
        }
    }

//    Rectangle {
//        id: ttsTextField
//        width: parent.width
//        radius: 2
//        border.color: nameError ? Style.errorColor : ttsTextField.activeFocus ? Style.lightBlue : Style.lightGreyBorder
//        border.width: ttsTextField.activeFocus || nameError ? 3 : 1
//        height: 200

//        anchors {
//            left: parent.left
//            top: ttsLabel.bottom
//            right: parent.right
//            topMargin: 8
//        }

//        TextField {
//            id: textfield
//            anchors.fill: parent
//            placeholderText: "Enter text"
//            wrapMode: "WrapAtWordBoundaryOrAnywhere"
//            text: tts
//            selectByMouse: true

//        }

//        MouseArea {
//            anchors.fill: parent
////            propagateComposedEvents: true
//            onClicked: {
//                mouse.accepted = false
//                textfield.focus = true
//            }
//        }
//    }

    TextField {
        id: ttsTextField
//        anchors.fill: parent
        placeholderText: langue == "English" ? "Enter Text" : "输入正文"
        font.pointSize: Style.ubuntuSubHeadingSize
        wrapMode: "WrapAtWordBoundaryOrAnywhere"
        text: tts
        selectByMouse: true
        width: parent.width
        height: 200

        anchors {
            left: parent.left
            top: ttsLabel.bottom
            right: parent.right
            topMargin: 8
        }

        background: Rectangle {
            radius: 2
            border.width: ttsTextField.activeFocus || nameError ? 3 : 1
            border.color: nameError ? Style.errorColor : ttsTextField.activeFocusOnPress ? Style.lightBlue : Style.lightGreyBorder
        }
    }

    CancelButton {
        langue: createSpeechMenuFrame.langue
        anchors {
            left: parent.left
            right: parent.horizontalCenter
            bottom: parent.bottom
            rightMargin: 5
        }
        onClicked: backToMenu()
    }

    SaveButton {
        id: saveButton
        langue: createSpeechMenuFrame.langue
        anchors {
            left: parent.horizontalCenter
            right: parent.right
            bottom: parent.bottom
            leftMargin: 5
        }
        canSave: false
        tooltip: errorMsg
        onReleased: if(saveButton.canSave) {
            var newName = Helper.formatName(speechTextField.text);
            var groupName = groupComboBox.displayText;
            if (groupComboBox.displayText === Helper.noGroupChinese) {
                groupComboBox.displayText = Helper.noGroup;
            }
            var ttsText = Helper.formatName(ttsTextField.text);
//            var tts = Helper.formatName(textfield.text);

            /// where we create the speech
            createSpeech(newName, groupComboBox.displayText, ttsText, oldName, oldGroup); /// get speech case

            backToMenu();
            var mess1 = ''
            var mess2 = ''
            if (langue == "English") {
                 mess1 = "Created the speech \"" + newName + "\" in \"" + groupName + "\""
//                mess2 = "已修改 \"" + oldGroup + "\" 中的语音 \"" + oldName + "\" ，并保存为 \"" + groupName + "\" 的语音 \"" + newName + "\"";
            } else {
                mess1 = "已创建路径 \"" + newName + "\" 在 \"" + groupName + "\"";
//                mess2 = "Created the speech \"" + oldName + "\" from \"" + oldGroup + "\" to \"" + newName + "\" in \"" + groupName + "\""
            }

//            setMessageTop(3, oldName === "" ? mess1 : mess2)
            setMessageTop(3, mess1);
        }
    }

    function enableSave(_nameError){
        nameError = _nameError;
        saveButton.canSave = !nameError;

        errorMsg = "";
        var mess1 = ''
        var mess2 = ''
        if (langue == "English") {
              mess1 = "The speech label cannot be empty";
              mess2 = "The speech name \"" + Helper.formatName(speechTextField.text) + "\" is already taken";
        } else {
              mess1 = "语音名称不可以为空";
              mess2 = "语音名 \"" + Helper.formatName(speechTextField.text) + "\" 已经被占用";
       }

        if(!saveButton.canSave){
            if(Helper.formatName(speechTextField.text) === "")
                errorMsg = mess1;
            else if(nameError)
                errorMsg = mess2;
        }
        setMessageTop(1, errorMsg);
    }
}

