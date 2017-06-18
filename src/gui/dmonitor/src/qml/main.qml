import QtQuick 2.3
import QtQuick.Controls 1.2
import DMonitor 1.0
import "../js/componentCreation.js" as RobotSpawner

ApplicationWindow {
    id: root
    visible: true
    width: drawArea.width + 300
    height: drawArea.height + 20
    color: "white"
    title: qsTr("DMonitor")

    property string udpAddress: '127.0.0.1'


    Rectangle {
        id: drawArea
        x: 10
        y: 10
        width: field.width
        height: field.height
        //color: "black"

        property var updateRate: 30
        property var robots: []

        Timer {
            interval: 1000 / parent.updateRate
            running: true
            repeat: true
            onTriggered: drawArea.updateView()
        }

        // create field before robots
        Field {
            id: field
            width: 900
            height: 600
        }


        function updateView() {
            console.log(drawArea.width, drawArea.height)
            robots.forEach(function(rbt, index, array) {
               rbt.update()
            })
        }

        Component.onCompleted: {
            robots = RobotSpawner.createObjects();
            // field only needs to paint once
            field.update()
        }


    }
}
