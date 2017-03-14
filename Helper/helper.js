
var PointViewType = {
    PERM: 1,
    TEMP: 2,
    HOME_PERM: 3
};

/// To check whether or not the group we are in is displayed or not
function isVisible(pointModel, _name, _groupName) {

    /// We don't want to display the group "No group"
    if(_name === "No Group" && _groupName === "")
        return false;

    /// We always display in the menu the points in "No Group"
    /// + we always display the groups
    if(_groupName === "No Group" || _groupName === "")
        return true;

    for(var i = 0; i < pointModel.count; i++){
        //console.log(pointModel.get(i)._groupName + " : " + pointModel.get(i)._name + " vs " + _groupName + " : " + _name);
        //console.log((pointModel.get(i)._groupName === "") + " : " + (pointModel.get(i)._name === _name));
        if(pointModel.get(i)._groupName === "" && pointModel.get(i)._name === _groupName){
            return pointModel.get(i)._isVisible;
        }
    }
    //console.log("Do we even get there " + _groupName + " : " + _name);

}


/// Get the list of groups only as an array
function getGroupList(pointModel){
    var groups = ["No Group"];
    for(var i = 0; i < pointModel.count; i++)
        if(pointModel.get(i)._groupName === "" && pointModel.get(i)._name !== "No Group")
            groups.push(pointModel.get(i)._name );


    return groups;
}

