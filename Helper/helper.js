
var PointViewType = {
    PERM: 1,
    TEMP: 2,
    HOME_PERM: 3
};

var noGroup = "No Group";

/// To check whether or not the group we are in is displayed or not
function isVisible(pointModel, _name, _groupName) {

    /// We don't want to display the group "No group"
    if(_name === noGroup && _groupName === "")
        return false;

    /// We always display in the menu the points in "No Group"
    /// + we always display the groups
    if(_groupName === noGroup || _groupName === "")
        return true;

    for(var i = 0; i < pointModel.count; i++){
        console.log(pointModel.get(i)._groupName + " : " + pointModel.get(i)._name + " vs " + _groupName + " : " + _name);
        console.log((pointModel.get(i)._groupName === "") + " : " + (pointModel.get(i)._name === _name));
        if(pointModel.get(i)._groupName === "" && pointModel.get(i)._name === _groupName){
            return pointModel.get(i)._isVisible;
        }
    }

    console.log("\n**********************************************************");
    console.log("We should not be there " + _groupName + " : " + _name);

    for(i = 0; i < pointModel.count; i++){
        console.log(pointModel.get(i)._groupName + " : " + pointModel.get(i)._name + " vs " + _groupName + " : " + _name);
        console.log((pointModel.get(i)._groupName === "") + " : " + (pointModel.get(i)._name === _name));
    }
    console.log("**********************************************************\n");
    return false;
}

/// To check whether or not the group we are in is displayed or not
function groupIsOpen(pointModel, _groupName) {

    /// We don't want to display the group "No group"
    if(_groupName === noGroup)
        return true;

    for(var i = 0; i < pointModel.count; i++)
        if(pointModel.get(i)._groupName === "" && pointModel.get(i)._name === _groupName)
            return pointModel.get(i)._groupIsOpen;

    console.log("\n**********************************************************");
    console.log("We should not be there " + _groupName + " " + pointModel.count);

    for(i = 0; i < pointModel.count; i++)
        console.log(pointModel.get(i)._groupName + " : " + pointModel.get(i)._name + " vs " + _groupName);

    console.log("**********************************************************\n");
    return false;
}


/// Get the list of groups only as an array
function getGroupList(pointModel){
    var groups = [];
    for(var i = 0; i < pointModel.count; i++)
        if(pointModel.get(i)._groupName === "" && pointModel.get(i)._name !== noGroup)
            groups.push(pointModel.get(i)._name );

    return groups;
}

