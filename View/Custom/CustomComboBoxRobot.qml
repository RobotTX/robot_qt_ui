import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQuick.Controls.Styles 1.4
import "../../Helper/style.js" as Style

ComboBox {
    id: groupComboBox
    height: 25
    currentIndex: 0

    /// Customisation of the indicator on the right of the combo box
    indicator: Rectangle {
        id: canvas
        x: groupComboBox.width - width
        y: groupComboBox.topPadding + (groupComboBox.availableHeight - height) / 2
        width: 18
        height: groupComboBox.height

        gradient: Gradient {
            GradientStop { position: 0.0; color: Style.stepperBegin }
            GradientStop { position: 1.0; color: Style.stepperEnd }
        }

        radius: 2

        Image {
            id: image
            asynchronous: true
            source: "qrc:/icons/white_stepper"
            fillMode: Image.Pad // For not stretching image
            anchors.horizontalCenter: parent.horizontalCenter // Centering text
            anchors.fill: parent
        }
    }

    /// What to display in the popup list once we clicked the box
    delegate: ItemDelegate {
        width: groupComboBox.width
        contentItem: CustomLabel {
            text: name
            verticalAlignment: Text.AlignVCenter
        }
        highlighted: groupComboBox.highlightedIndex == index
        onClicked: displayText = name
    }

    background: Rectangle {
        border.color: groupComboBox.pressed ? Style.lightBlue : Style.lightGreyBorder
        border.width: groupComboBox.visualFocus ? 2 : 1
        radius: 2
    }

    /// Customisation of the popup when the box has been clicked
    popup: Popup {
        y: groupComboBox.height - 1
        width: groupComboBox.width
        implicitHeight: listview.contentHeight
        padding: 1

        contentItem: ListView {
            id: listview
            clip: true
            model: groupComboBox.popup.visible ? groupComboBox.delegateModel : null
            currentIndex: groupComboBox.highlightedIndex

            ScrollIndicator.vertical: ScrollIndicator { }
        }

        background: Rectangle {
            border.color: Style.lightGreyBorder
            radius: 5
        }
    }
}
