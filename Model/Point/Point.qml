import QtQuick 2.0

ListModel {
    ListElement {
        name: "Point 1"
        isVisible: true
        group: "No Group"
    }
    ListElement {
        name: "Point 2"
        isVisible: false
        group: "No Group"
    }
    ListElement {
        name: "Group 1"
        isVisible: true
        group: ""
    }
    ListElement {
        name: "Point 3"
        isVisible: true
        group: "Group"
    }
    ListElement {
        name: "Point 4"
        isVisible: true
        group: "Group"
    }
    ListElement {
        name: "Group 2"
        isVisible: false
        group: ""
    }
    ListElement {
        name: "Point 5"
        isVisible: true
        group: "Group 2"
    }
    ListElement {
        name: "Point 6"
        isVisible: true
        group: "Group 2"
    }
    ListElement {
        name: "Group 3"
        isVisible: true
        group: ""
    }
    ListElement {
        name: "Point 7"
        isVisible: true
        group: "Group 3"
    }
    ListElement {
        name: "Point 8"
        isVisible: true
        group: "Group 3"
    }
}

