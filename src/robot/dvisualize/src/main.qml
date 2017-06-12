import QtQuick 2.5
import QtQuick.Window 2.2
import QtQuick.Controls 1.4
import QtQuick.Dialogs 1.0

ApplicationWindow {
    id: root
    visible: true
    width:  pic.width + flick.width + 3 * padding
    height: 650
    //title: width + "x" + height
    title: "Camera Caliration"

    property int padding: 10
    property int pictureWidth: 640
    property int pictureHeight: 480

//  maximumHeight: height
//  minimumHeight: height

//  maximumWidth: width
//  minimumWidth: width

    FileDialog {
        id: filedialog
        visible: false
        nameFilters: ["Image files (*.jpg *.png)", "All files (*)"]
        onAccepted: {
            edit.text = folder
        }
    }


    menuBar: MenuBar {
        Menu {
            title: "File"
            MenuItem {
                text: "Open..."
                shortcut: "Ctrl+O"
                onTriggered: filedialog.visible = true
            }

            MenuItem {
                text: "Save"
                shortcut: "Ctrl+S"
                onTriggered: console.log("Saving")
            }
        }
    }


    Row {
        x: root.padding
        y: root.padding
        id :column
        spacing: root.padding

        Rectangle {
            id: pic
            width: fullview.width
            height: fullview.height + col.spacing + overview.height

            Column {
                id: col
                spacing: 10
                Rectangle {
                    id: fullview
                    width: root.pictureWidth
                    height: root.pictureHeight
                    color: "black"

                    MouseArea {
                        anchors.fill: parent
                        onClicked: {
                            console.log("clicked" + mouseX + " " + mouseY)
                        }
                    }
                }

                Rectangle {
                    id: overview
                    width: root.pictureWidth
                    height: root.pictureHeight / 4
                    color: "black"
                }
            }
        }


        // text edit
        Flickable {
            id: flick
            width: 200
            height: pic.height
            visible: true
            opacity: 1
            clip: false
            contentWidth: edit.paintedWidth
            contentHeight: edit.paintedHeight

            TextEdit {
                id: edit
                width: flick.width
                height: flick.height
                cursorVisible: true
                focus: true
                mouseSelectionMode: TextEdit.SelectCharacters
                selectByMouse: true


                MouseArea {
                    anchors.fill: parent
                    cursorShape: Qt.IBeamCursor
                }
            }
        }
    }
}
