import { toggleVisibility, showCenterNotification } from './uiUtils.js';
import { ros } from './rosbridge.js';

let activeScenarioName = null;
let currentPoses = {};

// Function to load scenarios into the Execute Menu
export async function loadExecuteScenarios() {
    const scenarios = await window.electronAPI.getScenarios();
    const executeScenarioList = document.getElementById('execute-scenario-list');

    //  Clear any existing buttons
    executeScenarioList.innerHTML = '';

    //create buttons for each scenario
    scenarios.forEach((scenario) => {
        const button = document.createElement('button');
        //remove .json from display
        button.textContent = scenario.replace('.json', '');
        button.classList.add('scenario-btn');
        executeScenarioList.appendChild(button);
    });
}

// Eventlistener for clicking on a scenario button in the execute menu
export function setupExecuteScenarioListeners() {
    document.getElementById('execute-scenario-list').addEventListener('click', (event) => {
        if (event.target.tagName === 'BUTTON') {
            const scenarioName = event.target.textContent;
            activeScenarioName = scenarioName;
            loadScenarioDetailsForExecution(scenarioName);
        }
    });

    document.getElementById('execute-commands-btn').addEventListener('click', () => {
        executeScenario();
        showCenterNotification("Scenario execution started!")
    });
}


async function loadScenarioDetailsForExecution(scenarioName) {
    document.getElementById('submenu-scenario-title').textContent = `Scenario: ${scenarioName}`;

    try {
        const poses = await window.electronAPI.getPosesFromScenario(scenarioName);
        const commands = await window.electronAPI.getCommandsFromScenario(scenarioName);

        // fill  the pose list
        const poseList = document.getElementById('pose-list-execute');
        poseList.innerHTML = '';
        for (const pose in poses) {
            const listItem = document.createElement('li');
            listItem.textContent = `${pose}: [${poses[pose].joint_positions.join(', ')}]`;
            poseList.appendChild(listItem);
        }

        //fill the command list
        const commandList = document.getElementById('command-list-execute');
        commandList.innerHTML = '';
        commands.forEach((command) => {
            const listItem = document.createElement('li');
            listItem.textContent = command;
            commandList.appendChild(listItem);
        });

        //Show the submenu and hide the execute menu
        toggleVisibility(['execute-submenu'], ['execute-menu']);
    } catch (error) {
        console.error('Error loading scenario details:', error);
    }
}

//function to execute the active scenario
async function executeScenario() {
    if (!activeScenarioName) {
        showCenterNotification('No active scenario selected.');
        return;
    }

    //load poses for the active scenario
    const poses = await window.electronAPI.getPosesFromScenario(activeScenarioName);
    currentPoses = poses; // Ensure currentPoses is populated
    console.log('Loaded poses for scenario:', currentPoses);

    //load commands for the active scenario
    const commands = await window.electronAPI.getCommandsFromScenario(activeScenarioName);

    if (!commands || commands.length === 0) {
        showCenterNotification('No commands available in the active scenario.');
        return;
    }

    const commandList = document.getElementById('command-list-execute');
    const commandItems = commandList.querySelectorAll('li');

    for (let i = 0; i < commands.length; i++) {
        const command = commands[i];
        const commandItem = commandItems[i];

        try {
            //Progress Bar (enrico) the current command
            commandItem.style.backgroundColor = 'rgba(130, 255, 99, 0.98)';
            commandItem.style.transition = 'background-color 0.3s';

            //parse and execute the command
            const [action, ...args] = command.split(' ');

            switch (action) {
                case 'goto':
                    await executeGoto(args[0], parseFloat(args[1] || 5));
                    break;
                case 'sleep':
                    await executeSleep(parseInt(args[0], 10));
                    break;
                case 'open':
                    await executeOpenGripper();
                    break;
                case 'close':
                    await executeCloseGripper();
                    break;
                case 'homing':
                    await executeHoming();
                    break;
                case 'grasp':
                    await executeGrasp();
                    break;
                case 'recover':
                    await executeRecover();
                    break;
                case 'width':
                    await executeGripperWidth(parseFloat(args[0]));
                    break;
                default:
                    console.error(`Unknown command: ${action}`);
            }

            //reset the command's background color after execution
            commandItem.style.backgroundColor = '';
        } catch (error) {
            console.error(`Error executing command: ${command}`, error);
            showCenterNotification(`Failed to execute command: ${command}`);

            commandItem.style.backgroundColor = 'red';
        }
    }

    showCenterNotification('Scenario execution complete.');
}



async function executeGoto(poseName, duration) {
    if (!currentPoses[poseName]) {
        console.error(`Pose '${poseName}' not found in current scenario.`);
        showCenterNotification(`Pose '${poseName}' not found. Skipping command.`);
        return;
    }

    const jointPositions = currentPoses[poseName].joint_positions;

    console.log(`Executing 'goto' command for pose: ${poseName}`, jointPositions);
    await ros.sendGotoCommand('/position_joint_trajectory_controller/command', jointPositions, duration);
}

async function executeSleep(duration) {
    console.log(`Executing 'sleep' command for ${duration} seconds.`);
    await ros.sendSleepCommand(duration);
}

async function executeOpenGripper() {
    console.log("Executing 'open' gripper command.");
    await ros.sendOpenCommand();
}

async function executeCloseGripper() {
    console.log("Executing 'close' gripper command.");
    await ros.sendCloseCommand();
}

async function executeHoming() {
    console.log("Executing 'homing' command.");
    await ros.sendHomingCommand();
}

async function executeGrasp() {
    console.log("Executing 'grasp' command.");
    await ros.sendGraspCommand();
}

async function executeRecover() {
    console.log("Executing 'recover' command.");
    try {
        await ros.sendRecoverCommand();
        console.log('Recover command executed successfully.');
    } catch (error) {
        console.error('Error during recover command execution:', error);
    }
}

async function executeGripperWidth(width) {
    console.log("Executing gripper width command.");
    await ros.sendGripperWidth(width);
}