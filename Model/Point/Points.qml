import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../View/Point"
import "../../Helper/helper.js" as Helper

ListModel {
    signal hideShow(string groupName, string name)
    signal deletePointSignal(string groupName, string name)
    signal deleteGroupSignal(string groupName)
    signal moveToSignal(string name, string oldGroup, string newGroup)

    function addGroup(name){
        //console.log("Add group " + name);
        append({
           "groupName": name,
           "isOpen": (name === Helper.noGroup) ? true : false,
           "points": []
        });
    }

    function addPoint(name, isVisible, groupName, x, y){
        //console.log("Add point " + name + " to group " + groupName + " " + x + " " + y);
        for(var i = 0; i < count; i++){
            if(get(i).groupName === groupName){
                get(i).points.append({
                     "name": name,
                     "isVisible": isVisible,
                     "posX": x,
                     "posY": y
                });
            }
        }
    }

    function editPoint(oldName, oldGroup, name, isVisible, groupName, x, y){
        //console.log("Add point " + name + " to group " + groupName + " " + x + " " + y);
        deletePoint(oldGroup, oldName);
        for(var i = 0; i < count; i++){
            if(get(i).groupName === groupName){
                get(i).points.append({
                     "name": name,
                     "isVisible": isVisible,
                     "posX": x,
                     "posY": y
                });
            }
        }
    }

    function deletePoint(groupName, name){
        for(var i = 0; i < count; i++)
            if(get(i).groupName === groupName)
                for(var j = 0; j < get(i).points.count; j++)
                    if(get(i).points.get(j).name === name)
                        get(i).points.remove(j);

        deletePointSignal(groupName, name);
    }

    function deleteGroup(groupName){
        for(var i = 0; i < count; i++)
            if(get(i).groupName === groupName)
                remove(i);
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
                            "posY": get(i).points.get(j).posY
                        }
                        get(i).points.remove(j);
                    }

        for(i = 0; i < count; i++)
            if(get(i).groupName === newGroup)
                get(i).points.append(point);

        moveToSignal(name, oldGroup, newGroup)
    }

    function deleteAllGroups(){
        clear();
    }
}
