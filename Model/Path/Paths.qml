import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/helper.js" as Helper

ListModel {
    signal deletePathSignal(string groupName, string name)
    signal deleteGroupSignal(string groupName)
    signal moveToSignal(string name, string oldGroup, string newGroup)
    signal visiblePathChanged()

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
                visiblePathChanged();
            }
     }

    function renameGroup(newName, oldName){
        for(var i = 0; i < count; i++){
            if(get(i).groupName === oldName)
                setProperty(i, "groupName", newName);
        }
    }

    function moveTo(name, oldGroup, newGroup){
        console.log("moveTo " + name + " " + oldGroup + " " + newGroup);
        var path = {};
        var pathPoints = [];
        for(var i = 0; i < count; i++){
            if(get(i).groupName === oldGroup){
                console.log("Found the group");
                for(var j = 0; j < get(i).paths.count; j++){
                    if(get(i).paths.get(j).pathName === name){
                        console.log("Found the path");
                        for(var k = 0; k < get(i).paths.get(j).pathPoints.count; k++){
                            pathPoints.push({
                                "name": get(i).paths.get(j).pathPoints.get(k).name,
                                "posX": get(i).paths.get(j).pathPoints.get(k).posX,
                                "posY": get(i).paths.get(j).pathPoints.get(k).posY,
                                "waitTime": get(i).paths.get(j).pathPoints.get(k).waitTime
                           });
                        }

                        path = {
                            "pathName": get(i).paths.get(j).pathName,
                            "pathIsOpen": get(i).paths.get(j).pathIsOpen,
                            "pathIsVisible": get(i).paths.get(j).pathIsVisible,
                            "pathPoints": pathPoints
                        }
                        get(i).paths.remove(j);
                    }
                }
            }
        }

        for(i = 0; i < count; i++)
            if(get(i).groupName === newGroup)
                get(i).paths.append(path);

        moveToSignal(name, oldGroup, newGroup)
    }

    function clearTmpPath(){
        clear();
        addGroup("tmpGroup");
        addPath("tmpPath", "tmpGroup");
        hideShowPathOnMap("tmpGroup", "tmpPath");
        addPathPoint("pathPoint 1", "tmpPath", "tmpGroup", 1000, 1000, 0);
        addPathPoint("pathPoint 2", "tmpPath", "tmpGroup", 1300, 1000, 0);
        addPathPoint("pathPoint 3", "tmpPath", "tmpGroup", 1300, 1300, 0);
    }
}
