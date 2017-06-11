import QtQuick 2.5
import QtQuick.Window 2.2
import QtQuick.Controls 1.4
import QtQuick.Dialogs 1.0
import 'common'

ApplicationWindow {
    id: root
    visible: true
    width:  pic.width + flick.width + 3 * padding
    height: pic.height + 2 * padding
    title: width + "x" + height

    property int padding: 10
    property int pictureWidth: 640
    property int pictureHeight: 480

    //maximumHeight: height
    // minimumHeight: 600

    //maximumWidth: width
    // minimumWidth: 800

    FileDialog {
        id: filedialog
        visible: false
        onAccepted: console.log('fuck')
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
                    color: "yellow"
                }

                Rectangle {
                    id: overview
                    width: root.pictureWidth
                    height: root.pictureHeight / 4
                    color: "black"
                }
            }
        }


        Flickable {
            id: flick
            width: 200
            height: pic.height
            contentWidth: edit.paintedWidth
            contentHeight: edit.paintedHeight

            TextEdit {
                id: edit
                width: flick.width
                height: flick.height
                focus: true
                wrapMode: TextEdit.Wrap
            }
        }
    }

}
