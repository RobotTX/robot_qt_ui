import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../View/Point"
import "../../Helper/helper.js" as Helper

ListModel {

    signal hideShow(string groupName, string name)
    signal deletePointSignal(string groupName, string name)
    signal deleteGroupSignal(string groupName)
    signal moveToSignal(string name, string oldGroup, string newGroup)
    signal setMessageTop(int status, string msg)
    signal saveCurrentHome(string homeName, string homeX, string homeY, string homeOri)
    signal editPointB(string name, string groupName)
    signal createGroup(string name)

//    property string namePoint: ""
    property string langue
    property bool openGroup

    function addGroup(name){
        append({
           "groupName": name,
           "isOpen": langue == "English" ? name === Helper.noGroupChinese : name === Helper.noGroup,
           "points": []
        });
    }

    function addPoint(name, isVisible, groupName, x, y, home, orientation){
        //console.log("Add point " + name + " to group " + groupName + " " + x + " " + y);
        for(var i = 0; i < count; i++){
            if(get(i).groupName === groupName){
                get(i).points.append({
                     "name": name,
                     "isVisible": isVisible,
                     "posX": x,
                     "posY": y,
                     "home": home,
                     "orientation": orientation
                });
            }
        }
    }

    function editPoint(oldName, oldGroup, name, isVisible, groupName, x, y, home, orientation){
        //console.log("Add point " + name + " to group " + groupName + " " + x + " " + y);
        deletePoint(oldGroup, oldName);
        for(var i = 0; i < count; i++){
            if(get(i).groupName === groupName){
                get(i).points.append({
                     "name": name,
                     "isVisible": isVisible,
                     "posX": x,
                     "posY": y,
                     "home": home,
                     "orientation": orientation
                });
            }
        }
    }

    function deletePoint(groupName, name){
        var message = ''
        for(var i = 0; i < count; i++)
            if(get(i).groupName === groupName)
                for(var j = 0; j < get(i).points.count; j++)
                    if(get(i).points.get(j).name === name){
                       if (langue == 'English') {
                           message = "删除目标点 \"" + name + "\" 在 \"" + groupName + "\""
                       } else {
                           message = "Deleted the point \"" + name + "\" in \"" + groupName + "\""
                       }
                        get(i).points.remove(j);
                        setMessageTop(3, message);
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
                setMessageTop(3, message);
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

    function hideShowPoint(groupName, name){
        for(var i = 0; i < count; i++)
            if(get(i).groupName === groupName)
                for(var j = 0; j < get(i).points.count; j++)
                    if(get(i).points.get(j).name === name)
                        get(i).points.setProperty(j, "isVisible", !get(i).points.get(j).isVisible);
        hideShow(groupName, name);
    }

    function renameGroup(newName, oldName){
        for(var i = 0; i < count; i++){
            if(get(i).groupName === oldName)
                setProperty(i, "groupName", newName);
        }
    }

    function getGroupPoints(groupName) {
        for (var i = 0; i < count; i++) {
            if (get(i).groupName === groupName) {
                for (var j = 0; j < get(i).points.count; j++) {
                    console.log("point point point = " + get(i).points.get(j).name);
                }
            }
        }
        console.log("---------------------------------------------------------------getting the points")
    }

    // moves the point <name> from <oldGroup> to <newGroup>
    function moveTo(name, oldGroup, newGroup){
        var point = {};
        for(var i = 0; i < count; i++)
            if(get(i).groupName === oldGroup)
                for(var j = 0; j < get(i).points.count; j++)
                    if(get(i).points.get(j).name === name){
                        point = {
                            "name": get(i).points.get(j).name,
                            "isVisible": get(i).points.get(j).isVisible,
                            "posX": get(i).points.get(j).posX,
                            "posY": get(i).points.get(j).posY,
                            "home": get(i).points.get(j).home,
                            "orientation": get(i).points.get(j).orientation
                        }
                        get(i).points.remove(j);
                    }
        var message = ''
        for(i = 0; i < count; i++)
            if(get(i).groupName === newGroup)
                get(i).points.append(point);
                if (langue == 'English') {
                    message = "移动目标点 \"" + name + "\" 从 \"" + oldGroup + "\" 到 \"" + newGroup + "\""
                } else {
                    message = "Moved the point \"" + name + "\" from \"" + oldGroup + "\" to \"" + newGroup + "\""
                }
        setMessageTop(3, message);
        moveToSignal(name, oldGroup, newGroup)
    }

    function deleteAllGroups(){
        clear();
    }

    function getNbHome(groupName){
        var nb = 0;
        for(var i = 0; i < count; i++)
            if(get(i).groupName === groupName)
                for(var j = 0; j < get(i).points.count; j++)
                    if(get(i).points.get(j).home)
                        nb++;
        return nb;
    }
}
