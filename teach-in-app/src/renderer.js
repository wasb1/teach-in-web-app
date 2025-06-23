/**
 * This file is loaded via the <script> tag in the index.html file and will
 * be executed in the renderer process for that window. No Node.js APIs are
 * available in this process because `nodeIntegration` is turned off and
 * `contextIsolation` is turned on. Use the contextBridge API in `preload.js`
 * to expose Node.js functionality from the main process.
 */
import { setupRosBridgeHandlers, ros } from './components/rosbridge.js';
import { setupRobotInfoBox } from './components/robotInfoBox.js';
import { setupRealtimeControlMenu } from './components/realtimeControl.js';
import { loadExecuteScenarios, setupExecuteScenarioListeners } from './components/scenariosExecution.js';
import { toggleVisibility, showCenterNotification} from './components/uiUtils.js';
import { setupTeachInMenu } from './components/teachInMenu.js';


//-----------------------------------------------------------------------------------------
// EVENT LISTENERS
//-----------------------------------------------------------------------------------------
document.getElementById('franka-card').addEventListener('click', () => {
    document.getElementById('menu').classList.remove('hidden');
    document.getElementById('main-menu').classList.remove('hidden');
    document.getElementById('robot-info-box').classList.remove('hidden')
    document.getElementById('robot-selection').classList.add('hidden');
    document.getElementById('robot-selection').classList.remove('robot-selection-container');
    
});

document.getElementById('jaime-card').addEventListener('click', () => {
    showCenterNotification("NOT IMPLEMENTED YET!");
});

document.getElementById('yuki-zuki-card').addEventListener('click', () => {
    showCenterNotification("NOT IMPLEMENTED YET!");
});

document.getElementById('reachy-card').addEventListener('click', () => {
    showCenterNotification("NOT IMPLEMENTED YET!");
});

document.getElementById('back-to-robot-selection').addEventListener('click', () => {
    toggleVisibility(['robot-selection'], ['menu','main-menu','robot-info-box']);
    document.getElementById('robot-selection').classList.remove('hidden');
    document.getElementById('robot-selection').classList.add('robot-selection-container')

});

document.getElementById('realtime-control-btn').addEventListener('click', () => {
    toggleVisibility(['realtime-menu'], ['main-menu']);
});

document.getElementById('execute-btn').addEventListener('click', () => {
    loadExecuteScenarios();
    toggleVisibility(['execute-menu'], ['main-menu']);
});

document.getElementById('teach-in-btn').addEventListener('click', () => {
    toggleVisibility(['teach-in-menu'], ['main-menu']);
});

document.getElementById('realtime-back-btn').addEventListener('click', () => {
    toggleVisibility(['main-menu'], ['realtime-menu']);
});

document.getElementById('back-to-main').addEventListener('click', () => {
    toggleVisibility(['main-menu'], ['execute-menu']);
});

document.getElementById('back-to-main-teach').addEventListener('click', () => {
    toggleVisibility(['main-menu'], ['teach-in-menu']);
});

document.getElementById("back-to-execute-menu").addEventListener("click", () => {
    toggleVisibility(["execute-menu"], ["execute-submenu"]);
});

document.getElementById('add-new-scenario-btn').addEventListener('click', () => {
    toggleVisibility(['add-new-scenario-menu'], ['teach-in-menu']);
});

const textInputs = document.querySelectorAll('input[type="text"]');

    textInputs.forEach(input => {
        input.addEventListener('click', () => {
            input.focus();
        });
    });

//-------------------------------------------------------------------------------------------------
// ROSBRIDGE CONNECTION
//-------------------------------------------------------------------------------------------------
setupRosBridgeHandlers();

//-----------------------------------------------------------------------------------------
// ROBOT INFO BOX CONTROL
//-----------------------------------------------------------------------------------------
setupRobotInfoBox(ros);

//-----------------------------------------------------------------------------------------
// REALTIME CONTROL MENU
//-----------------------------------------------------------------------------------------
setupRealtimeControlMenu(ros);

//-------------------------------------------------------------------------------------------------
// SCENARIOS EXECUTION
//-------------------------------------------------------------------------------------------------
loadExecuteScenarios();
setupExecuteScenarioListeners();

//-------------------------------------------------------------------------------------------------
// TEACH-IN MENU
//-------------------------------------------------------------------------------------------------
setupTeachInMenu();