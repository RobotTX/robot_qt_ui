import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../View/Speech"
import "../../Helper/helper.js" as Helper

ListModel {

    signal hideShow(string groupName, string name)
    signal deleteSpeechSignal(string groupName, string name)
    signal deleteGroupSignal(string groupName)
    signal moveToSignal(string name, string oldGroup, string newGroup)
    signal setMessageTop(int status, string msg)
    signal createGroup(string name)

    property string langue
    property bool openGroup

    function addGroup(name){
        console.log("we are creating a group for speech");
        append({
           "groupName": name,
           "isOpen": langue == "English" ? name === Helper.noGroupChinese : name === Helper.noGroup,
           "speechs": []
        });
    }

    /// a speech has a label, belong to a group and has a text
    function addSpeech(name, groupName, tts){
        console.log('we are adding a new speech');
        console.log(name + " " + groupName + " " + tts );
        for(var i = 0; i < count; i++){
            if(get(i).groupName === groupName){
                get(i).speechs.append({
                     "name": name,
                     "tts": tts,
                     "descriptionIsOpen": false
                });
            }
        }
        console.log(name + " " + groupName + " " + tts );
    }

    function hideShowDescription(groupName, name) {
        for (var i = 0; i < count; i++) {
            if (get(i).groupName === groupName) {
                for(var j = 0; j < get(i).speechs.count; j++) {
                    if (get(i).speechs.get(j).name === name) {
                        get(i).speechs.setProperty(j,"descriptionIsOpen", !get(i).speechs.get(j).descriptionIsOpen);
                    }
                }
            }
        }
    }

    function editSpeech(oldName, oldGroup, name, groupName, tts){
        console.log('we are in editSpeech - Speechs.qml');
        deleteSpeech(oldGroup, oldName); /// while editing, not to have two times the same group which is edited
        for(var i = 0; i < count; i++){
            if(get(i).groupName === groupName){
                get(i).speechs.append({
                     "name": name,
                     "tts": tts
                });
            }
        }
        console.log("name and tts = " + name + " " + tts);
    }

    function deleteSpeech(groupName, name){
        var message = ''
        for(var i = 0; i < count; i++)
            if(get(i).groupName === groupName)
                for(var j = 0; j < get(i).speechs.count; j++)
                    if(get(i).speechs.get(j).name === name){
                       if (langue == 'English') {
                           message = "删除目标点 \"" + name + "\" 在 \"" + groupName + "\""
                       } else {
                           message = "Deleted the speech \"" + name + "\" in \"" + groupName + "\""
                       }
                        get(i).speechs.remove(j);
                        setMessageTop(2, message);
                    }
    }

    function deleteGroup(groupName){
        var message = ''
        for(var i = 0; i < count; i++)
            if(get(i).groupName === groupName){
                remove(i);
                if (langue == 'English') {
                    message = "删除群组 \"" + groupName + "\""
                } else {
                    message = "Deleted the group \"" + groupName + "\""
                }
                setMessageTop(2, message);
            }
        deleteGroupSignal(groupName);
    }

    function hideShowGroup(groupName){
        for(var i = 0; i < count; i++) {
            if(get(i).groupName === groupName) {
                setProperty(i, "isOpen", !get(i).isOpen);
                openGroup = get(i).isOpen;
            }
        }

    }

    function renameGroup(newName, oldName){
        for(var i = 0; i < count; i++){
            if(get(i).groupName === oldName)
                setProperty(i, "groupName", newName);
        }
    }

    // moves the speech <name> from <oldGroup> to <newGroup>
    function moveTo(name, oldGroup, newGroup){
        var speech = {};
        for(var i = 0; i < count; i++)
            if(get(i).groupName === oldGroup)
                for(var j = 0; j < get(i).speechs.count; j++)
                    if(get(i).speechs.get(j).name === name){
                        speech = {
                            "name": get(i).speechs.get(j).name,
                            "tts": get(i).speechs.get(j).tts,
                        }
                        get(i).speechs.remove(j);
                    }
        var message = ''
        for(i = 0; i < count; i++)
            if(get(i).groupName === newGroup)
                get(i).speechs.append(speech);
                if (langue == 'English') {
                    message = "移动目标点 \"" + name + "\" 从 \"" + oldGroup + "\" 到 \"" + newGroup + "\""
                } else {
                    message = "Moved the speech \"" + name + "\" from \"" + oldGroup + "\" to \"" + newGroup + "\""
                }
        setMessageTop(2, message);
        moveToSignal(name, oldGroup, newGroup)
    }

    function deleteAllGroups(){
        clear();
    }
}
