
var PointViewType = {
    PERM: 1,
    TEMP: 2,
    HOME_PERM: 3
};


/// To get the current index of the given item
function itemIndex(item) {
    /// if item is not parented, -1 is returned
    if (item.parent === null)
        return -1
    var siblings = item.parent.children
    for (var i = 0; i < siblings.length; i++)
        if (siblings[i] === item)
            return i
    return -1 //will never happen
}

/// To check whether or not the group we are in is displayed or not
function isVisible(item, _group) {

    /// We don't want to display the group "No group"
    if(item.name === "No Group" && item.groupName === "")
        return false;

    /// We always display in the menu the points in "No Group"
    /// + we always display the groups
    if(item.groupName === "No Group" || item.groupName === "")
        return true;

    if (item.parent === null)
        return false;


    return groupIsVisible(item, _group);
}

function groupIsVisible(item, _group){
    var index = itemIndex(item)
    if(index > 0){
        /// If the previous item is a group,
        /// it is OUR group so we check if we display the point/path
        if (item.parent.children[itemIndex(item) - 1].groupName === ""
                && item.parent.children[itemIndex(item) - 1].name === _group){
            if(item.parent.children[itemIndex(item) - 1].isVisible)
                return true;
            else
                return false;
        } else
            /// If the previous item is not a group,
            /// we look for its previous item until we find our group
            return groupIsVisible(item.parent.children[itemIndex(item) - 1], _group);
    }
    return false;
}
