//-------------------------------------------------------------------------------------------------
// ROSBRIDGE CONNECTION
//-------------------------------------------------------------------------------------------------

const ros = window.roslib.connectToRos('ws://localhost:9090');

export function setupRosBridgeHandlers() {
    const robotStatusElement = document.getElementById('robot-status');
    const robotPositionElement = document.getElementById('robot-position');

    ros.onConnection(() => {
        robotStatusElement.textContent = 'Connected';
        robotStatusElement.style.color = 'green';
        console.log('Renderer: Connected to ROSBridge');
    });

    ros.onError((error) => {
        console.error('Renderer: Error connecting to ROSBridge:', error);
        robotStatusElement.textContent = 'Error';
        robotStatusElement.style.color = 'orange';
    });

    ros.onClose(() => {
        robotStatusElement.textContent = 'Disconnected';
        robotStatusElement.style.color = 'red';
        robotPositionElement.textContent = 'N/A';
        console.log('Renderer: Connection to ROSBridge closed');
    });

}

export { ros };
