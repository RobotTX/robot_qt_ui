import QtQuick 2.0

ListModel {
    ListElement {
        _name: "Point 1"
        _isVisible: true
        _groupName: "No Group"
    }
    ListElement {
        _name: "Point 2"
        _isVisible: false
        _groupName: "No Group"
    }
    ListElement {
        _name: "Group 1"
        _isVisible: true
        _groupName: ""
    }
    ListElement {
        _name: "Point 3"
        _isVisible: true
        _groupName: "Group"
    }
    ListElement {
        _name: "Point 4"
        _isVisible: true
        _groupName: "Group"
    }
    ListElement {
        _name: "Group 2"
        _isVisible: false
        _groupName: ""
    }
    ListElement {
        _name: "Point 5"
        _isVisible: true
        _groupName: "Group 2"
    }
    ListElement {
        _name: "Point 6"
        _isVisible: true
        _groupName: "Group 2"
    }
    ListElement {
        _name: "Group 3"
        _isVisible: true
        _groupName: ""
    }
    ListElement {
        _name: "Point 7"
        _isVisible: true
        _groupName: "Group 3"
    }
    ListElement {
        _name: "Point 8"
        _isVisible: true
        _groupName: "Group 3"
    }
}

