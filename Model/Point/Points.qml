import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../View/Point"
import "../../Helper/helper.js" as Helper

ListModel {
    id: listModel
    function addGroup(index, name){
        console.log("Add group " + name);
        insert(index, {"_name": name, "_isVisible": true, "_groupName": "",
                              "_x": 0, "_y": 0, "_groupIsOpen": (name === Helper.noGroup) ? true : false});
    }

    function addPoint(index, name, isVisible, groupName, x, y){
        console.log("Add point " + name + " to group " + groupName + " " + x + " " + y);
        insert(index, {"_name": name, "_isVisible": isVisible, "_groupName": groupName,
                              "_x": x, "_y": y, "_groupIsOpen": groupIsOpen(groupName)});
    }

    function removePoint(index){
        remove(index);
    }

    function removeGroup(begin, end){
        for(var i = begin; i <= end; i++)
            remove(begin);
    }

    function hideShow(index, show){
        //console.log("hideShow " + index + " " + get(index)._groupName + " " + get(index)._name + " " + get(index)._isVisible + " " + show);
        if(get(index)._groupName === ""){
            setProperty(index, "_groupIsOpen", show);
            for(var i = 0; i < count; i++)
                if(get(i)._groupName === get(index)._name)
                    get(i)._groupIsOpen = show;

        } else
            setProperty(index, "_isVisible", show);
    }

    function renameGroup(newName, oldName){
        for(var i = 0; i < count; i++){
            if(get(i)._groupName === "" && get(i)._name === oldName)
                get(i)._name = newName;
            else if(get(i)._groupName === oldName)
                get(i)._groupName = newName;
        }
    }

    function movePoint(oldIndex, newIndex, newGroupName){
        //console.log("move to " + oldIndex + " " + newIndex+ " " + count + " " + newGroupName );
        setProperty(oldIndex, "_groupName", newGroupName);
        setProperty(oldIndex, "_groupIsOpen", groupIsOpen(newGroupName))
        move(oldIndex, newIndex, 1);
    }

    function displayList(){
        console.log("\nList of points :");
        for(var i = 0; i < count; i++)
            console.log(i + " : " + get(i)._groupName + " " + get(i)._name);
    }

    /// Get the index of the given point
    function getIndex(name, groupName){
        for(var i = 0; i < count; i++)
            if(get(i)._groupName === groupName && get(i)._name === name)
                return i;

        return -1;
    }

    /// Get the index of the given group compared to other groups
    function getGroupIndex(name){
        var nb = 0;
        for(var i = 0; i < count; i++){
            if(get(i)._groupName === ""){
                if(get(i)._name === name)
                    return nb;
                else if(get(i)._name !== Helper.noGroup)
                    nb++;
            }
        }

        return -1;
    }

    /// To check whether or not the group we are in is displayed or not
    function groupIsOpen(_groupName) {

        /// We don't want to display the group "No group"
        if(_groupName === Helper.noGroup)
            return true;

        for(var i = 0; i < count; i++)
            if(get(i)._groupName === "" && get(i)._name === _groupName)
                return get(i)._groupIsOpen;

        console.log("\n**********************************************************");
        console.log("We should not be there " + _groupName + " " + count);

        for(i = 0; i < count; i++)
            console.log(get(i)._groupName + " : " + get(i)._name + " vs " + _groupName);

        console.log("**********************************************************\n");
        return false;
    }

    /// Get the list of groups except "No Group" as an array
    function getGroupList(){
        var groups = [];
        for(var i = 0; i < count; i++)
            if(get(i)._groupName === "" && get(i)._name !== Helper.noGroup)
                groups.push(get(i)._name);

        return groups;
    }

    /// Get the nb of groups
    function getNbGroup(){
        var nb = 0;
        for(var i = 0; i < count; i++)
            if(get(i)._groupName === "" && get(i)._name !== Helper.noGroup)
                nb++;

        return nb;
    }
}
