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

    property string namePoint: ""
    property string langue

    function addGroup(name){
        //console.log("Add group " + name);
        append({
           "groupName": name,
           "isOpen": (name === Helper.noGroup) ? true : false,
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
                namePoint = name
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
        for(var i = 0; i < count; i++)
            if(get(i).groupName === groupName)
                for(var j = 0; j < get(i).points.count; j++)
                    if(get(i).points.get(j).name === name){
                        get(i).points.remove(j);
                        setMessageTop(2, "Deleted the point \"" + name + "\" in \"" + groupName + "\"");
                    }
    }

    function deleteGroup(groupName){
        for(var i = 0; i < count; i++)
            if(get(i).groupName === groupName){
                remove(i);
                setMessageTop(2, "Deleted the group \"" + groupName + "\"");
            }
        deleteGroupSignal(groupName);
    }

    function hideShowGroup(groupName){
        for(var i = 0; i < count; i++)
            if(get(i).groupName === groupName)
                setProperty(i, "isOpen", !get(i).isOpen);
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

        for(i = 0; i < count; i++)
            if(get(i).groupName === newGroup)
                get(i).points.append(point);

        setMessageTop(2, "Moved the point \"" + name + "\" from \"" + oldGroup + "\" to \"" + newGroup + "\"");
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
