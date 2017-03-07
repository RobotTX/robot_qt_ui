
/// To get the current index of this item
function itemIndex(item) {
    /// if item is not parented, -1 is returned
    if (item.parent === null)
        return -1
    var siblings = item.parent.children
    for (var i = 0; i < siblings.length; i++)
        if (siblings[i] === item)
            return i
    return -1 //will item.never happen
}

/// To check whether or not the group we are in is displayed or not
function previousGroupIsVisible(item) {
    /// We always display in the menu the points in "No Group"
    /// + we always display the groups
    if(item.groupName === "No Group" || item.groupName === "")
        return true;

    if (item.parent === null)
        return false;

    var index = itemIndex(item)
    if(index > 0){
        /// If the previous item is a group,
        /// it is OUR group so we check if we display the point/path
        if (item.parent.children[itemIndex(item) - 1].groupName === ""){
            if(item.parent.children[itemIndex(item) - 1].isVisible)
                return true;
            else
                return false;
        } else
            /// If the previous item is not a group,
            /// we look for its previous item until we find our group
            return previousGroupIsVisible(item.parent.children[itemIndex(item) - 1]);
    }

    return false;
}
