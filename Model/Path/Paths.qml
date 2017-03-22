import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/helper.js" as Helper

ListModel {
    signal deletePathSignal(string groupName, string name)
    signal deleteGroupSignal(string groupName)
    signal moveToSignal(string name, string oldGroup, string newGroup)

    function addGroup(name){
        //console.log("Add group " + name);
        append({
           "groupName": name,
           "groupIsOpen": (name === Helper.noGroup) ? true : false,
           "paths": []
        });
    }

    function addPath(name, groupName){
        //console.log("Add path " + name + " to group " + groupName);
        for(var i = 0; i < count; i++)
            if(get(i).groupName === groupName)
                get(i).paths.append({
                     "pathName": name,
                     "pathIsOpen": false,
                     "pathIsVisible": false,
                     "pathPoints": []
                });
    }

    function addPathPoint(name, pathName, groupName, x, y, waitTime){
        //console.log("Add pathpoint " + name + " to path " + pathName + " in group " + groupName);
        for(var i = 0; i < count; i++)
            if(get(i).groupName === groupName)
                for(var j = 0; j < get(i).paths.count; j++)
                    if(get(i).paths.get(j).pathName === pathName)
                        get(i).paths.get(j).pathPoints.append({
                             "name": name,
                             "posX": x,
                             "posY": y,
                             "waitTime": waitTime
                        });
    }

    function deleteGroup(groupName){
        for(var i = 0; i < count; i++)
            if(get(i).groupName === groupName)
                remove(i);
        deleteGroupSignal(groupName);
    }

    function deletePath(groupName, name){
        for(var i = 0; i < count; i++)
            if(get(i).groupName === groupName)
                for(var j = 0; j < get(i).paths.count; j++)
                    if(get(i).paths.get(j).pathName === name)
                        get(i).paths.remove(j);

        deletePathSignal(groupName, name);
    }

    function hideShowGroup(groupName){
        for(var i = 0; i < count; i++)
            if(get(i).groupName === groupName)
                setProperty(i, "groupIsOpen", !get(i).groupIsOpen);
    }

    function hideShowPath(groupName, name){
        for(var i = 0; i < count; i++)
            if(get(i).groupName === groupName)
                for(var j = 0; j < get(i).paths.count; j++)
                    if(get(i).paths.get(j).pathName === name)
                        get(i).paths.setProperty(j, "pathIsOpen", !get(i).paths.get(j).pathIsOpen);
    }

    function hideShowPathOnMap(groupName, name){
        for(var i = 0; i < count; i++)
            for(var j = 0; j < get(i).paths.count; j++){
                if(get(i).groupName === groupName && get(i).paths.get(j).pathName === name)
                    get(i).paths.setProperty(j, "pathIsVisible", !get(i).paths.get(j).pathIsVisible);
                else
                    get(i).paths.setProperty(j, "pathIsVisible", false);
            }
     }

    function renameGroup(newName, oldName){
        for(var i = 0; i < count; i++){
            if(get(i).groupName === oldName)
                setProperty(i, "groupName", newName);
        }
    }

    function moveTo(name, oldGroup, newGroup){
        var path = {};
        for(var i = 0; i < count; i++)
            if(get(i).groupName === oldGroup)
                for(var j = 0; j < get(i).paths.count; j++)
                    if(get(i).paths.get(j).name === name){
                        path = {
                            "name": get(i).paths.get(j).pathName,
                            "pathIsOpen": get(i).paths.get(j).pathIsOpen,
                            "pathIsVisible": get(i).paths.get(j).pathIsVisible,
                            "pathPoints": get(i).paths.get(j).pathPoints
                        }
                        get(i).paths.remove(j);
                    }

        for(i = 0; i < count; i++)
            if(get(i).groupName === newGroup)
                get(i).paths.append(path);

        moveToSignal(name, oldGroup, newGroup)
    }
}
