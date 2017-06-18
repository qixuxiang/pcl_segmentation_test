import QtQuick 2.3
import QtQuick.Controls 1.2
import DMonitor 1.0
import "../js/componentCreation.js" as Spawner

ApplicationWindow {
    id: root
    visible: true
    width: drawArea.width + controlArea.width + controlArea.anchors.leftMargin + 10 * 2
    height: drawArea.height + 10 * 2
    color: "black"
    title: qsTr("DMonitor")

    property string udpAddress: '127.0.0.1'


    Rectangle {
        id: drawArea
        x: 10
        y: 10
        width: field.width
        height: field.height

        property var updateRate: 30
        property var robots: []
        property var balls: []

        Timer {
            interval: 1000 / parent.updateRate
            running: true
            repeat: true
            onTriggered: drawArea.updateView()
        }

        // create field before robots
        Field {
            id: field
            width: 900; height: 600
        }


        function updateView() {
            controlArea.modelTab.update()
            robots.forEach(function(rbt, index, array) {
                rbt.update();
                rbt.setIsMonitor(controlArea.modelTab.isMonitor)
            })

            balls.forEach(function(ball, index, array){
                ball.update();
                ball.setIsMonitor(controlArea.modelTab.isMonitor);
            })
        }

        Component.onCompleted: {
            robots = Spawner.createRobots();
            balls = Spawner.createBalls();
            field.update()
        }


    }

    Rectangle {
        id: controlArea
        y: 10
        width: 300
        height: drawArea.height
        anchors.left: drawArea.right
        anchors.topMargin: 10
        anchors.leftMargin: 6
        color: "black"
        property alias modelTab: modelTab

        TabView {
            id: modelTab
            x: 10
            y: 0
            width: parent.width - 20
            height: parent.height - 10
            property bool isMonitor: currentIndex === 0 ? true : false

            Tab {
                id: monitor
                title: "Monitor"
            }

            Tab {
                id: simulator
                title: "Simulator"
            }
        }
    }
}
