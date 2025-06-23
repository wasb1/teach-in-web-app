import { toggleVisibility, showNotification, showCenterNotification } from './uiUtils.js';
import { latestJointStates } from './robotInfoBox.js';


export function setupTeachInMenu() {
//-------------------------------------------------------------------------------------------------
// TEACH-IN MENU: EDIT OR ADD
//-------------------------------------------------------------------------------------------------

//to track the active scenario name globally (should be changed)
let activeScenarioName = null;

//load Scenarios
async function loadScenarios() {
    const scenarios = await window.electronAPI.getScenarios();
    const scenariosList = document.getElementById('scenarios-list');
    scenariosList.innerHTML = ''; 
    scenarios.forEach((scenario) => {
        const button = document.createElement('button');
        button.textContent = scenario;
        button.id = `scenario-${scenario.replace(/\s+/g, '-').replace(/[^a-zA-Z0-9-_]/g, '')}`;

        button.addEventListener('click', () => {
            activeScenarioName = scenario;
            currentPoses = {};
            renderPoseList(); 
            refreshDropdown();

            toggleVisibility(['teach-in-scenario-menu'], ['edit-existing-scenario-menu']);
            document.getElementById('scenario-title').textContent = `Scenario: ${scenario.slice(0, -5)}`;
            
            loadCommands(); 
            loadPoses(); 
        });

        scenariosList.appendChild(button);
    });
}


//Event listener for editing existing scenarios
document.getElementById('edit-existing-scenario-btn').addEventListener('click', () => {
    loadScenarios();
    toggleVisibility(['edit-existing-scenario-menu'], ['teach-in-menu']);
});

document.getElementById('create-scenario-btn').addEventListener('click', () => {
    const scenarioName = document.getElementById('new-scenario-name').value;

    if (scenarioName) {
        activeScenarioName = scenarioName;
        window.electronAPI.createScenarioFile(scenarioName);

        //clear current poses
        currentPoses = {};
        renderPoseList(); 
        refreshDropdown();

        console.log(`Active scenario set to: ${activeScenarioName}`);
        document.getElementById('scenario-title').innerText = `Scenario: ${scenarioName}`;
        toggleVisibility(['teach-in-scenario-menu'], ['add-new-scenario-menu']);
        loadCommands(); //tis should now only load commands for the new, empty scenario
        showNotification(`Scenario '${scenarioName}' created successfully!`);
    } else {
        showCenterNotification("Please enter a valid scenario name.");
    }
});


document.getElementById('back-to-teach-in-menu').addEventListener('click', () => {
    toggleVisibility(['teach-in-menu'], ['add-new-scenario-menu']);
});

document.getElementById('back-to-teach-in-menu-from-edit').addEventListener('click', () => {
    toggleVisibility(['teach-in-menu'], ['edit-existing-scenario-menu']);
});

document.getElementById('back-to-scenario-selection').addEventListener('click', () => {
    loadScenarios();
    toggleVisibility(['edit-existing-scenario-menu'], ['teach-in-scenario-menu']);
});

// ------------------------------------------------------------------------------------
// TEACH-IN SCENARIOS MENU: COMMANDS ADDING
// ------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------
// TEACH-IN SCENARIOS MENU: COMMAND INPUT FIELDS TOGGLING
//------------------------------------------------------------------------------------

const commandInputs = {
    goto: document.getElementById("gotoInputContainer"), //updated for dropdown container
    width: document.getElementById("widthInput"),
    sleep: document.getElementById("sleepInput"),
};

//hide all command input fields initially
Object.values(commandInputs).forEach((input) => {
    if (input) input.style.display = "none";
});

//show or hide relevant input fields based on selected command
document.querySelectorAll('input[name="command"]').forEach((radio) => {
    radio.addEventListener("change", () => {
        Object.keys(commandInputs).forEach((cmd) => {
            if (commandInputs[cmd]) {
                commandInputs[cmd].style.display = radio.value === cmd ? "inline-block" : "none";
            }
        });
    });
});

//store selected pose for the "goto" command
let selectedPose = null;

//Event listener for "goto" radio button
document.getElementById('goto').addEventListener('change', async () => {
    if (document.getElementById('goto').checked) {
        const dropdownContainer = document.getElementById('gotoInputContainer'); // Container for dropdown
        dropdownContainer.innerHTML = ''; //clear existing content

        try {
            //Fetch poses for the current scenario
            const poses = await window.electronAPI.getPosesFromScenario(activeScenarioName);

            if (Object.keys(poses).length > 0) {
                //Create and populate a dropdown
                const dropdown = document.createElement('select');
                dropdown.id = 'poseDropdown';
                dropdown.innerHTML = Object.keys(poses)
                    .map(poseName => `<option value="${poseName}">${poseName}</option>`)
                    .join('');

                //Listen for pose selection
                dropdown.addEventListener('change', (event) => {
                    selectedPose = event.target.value; // Store selected pose
                });

                dropdownContainer.appendChild(dropdown);
            } else {
                dropdownContainer.innerHTML = '<p> No poses available in this scenario. Please add new poses below.</p>';
                selectedPose = null; //reset selected pose
            }
        } catch (error) {
            console.error('Error fetching poses:', error);
            dropdownContainer.innerHTML = '<p>Error loading poses.</p>';
        }
    }
});
//Update the Add Command button handler
document.getElementById('addButton').addEventListener('click', () => {
    if (!activeScenarioName) {
        return;
    }

    const selectedCommand = document.querySelector('input[name="command"]:checked');
    if (!selectedCommand) {
        return;
    }

    const command = selectedCommand.value;

    if (command === 'goto') {
        if (!selectedPose) {
            return;
        }
        //construct the command with the selected pose
        const commandString = `goto ${selectedPose}`;
        window.electronAPI.addCommandToScenario(activeScenarioName, commandString);
        loadCommands(); // Refresh the command list
        showNotification('Command added to the list of commands!')
    } else {
        //handle other commands (existing logic)
        const argument = commandInputs[command]?.value || null;
        const commandString = argument ? `${command} ${argument}` : command;
        window.electronAPI.addCommandToScenario(activeScenarioName, commandString);
        loadCommands();
        showNotification('Command added to the list of commands!')
    }

});

//------------------------------------------------------------------------------------
// TEACH-IN SCENARIOS MENU: COMMANDS LIST MANAGEMENT
//------------------------------------------------------------------------------------
//store commands for the active scenario
let currentCommands = [];

function loadCommands() {
    if (window.electronAPI && activeScenarioName) {
        window.electronAPI.getCommandsFromScenario(activeScenarioName)
            .then((commands) => {
                currentCommands = commands;
                renderCommandList();
            })
            .catch((error) => console.error("Error loading commands:", error));
    }
}

function renderCommandList() {
    const commandListElement = document.getElementById('command-list');
    //clear previous items
    commandListElement.innerHTML = '<h3>List Of Commands (Drag&Drop)</h3>';

    currentCommands.forEach((command, index) => {
        const commandItem = document.createElement('div');
        commandItem.classList.add('command-item');
        commandItem.draggable = true;
        commandItem.innerHTML = `
            <span>${command}</span>
            <button class="delete-command" data-index="${index}">Delete</button>
        `;
        commandItem.addEventListener('dragstart', (event) => onDragStart(event, index));
        commandItem.addEventListener('dragover', (event) => event.preventDefault());
        commandItem.addEventListener('drop', (event) => onDrop(event, index));
        commandListElement.appendChild(commandItem);
    });
}

document.getElementById('command-list').addEventListener('click', (event) => {
    if (event.target.classList.contains('delete-command')) {
        const index = event.target.getAttribute('data-index');
        currentCommands.splice(index, 1);
        showNotification('Command deleted from list of commands!')
        renderCommandList();
        window.electronAPI.updateCommandsInScenario(activeScenarioName, currentCommands)
            .catch(error => console.error("Error updating scenario file:", error));
    }
});

//Drag-and-Drop Reordering
let draggedCommandIndex = null;

function onDragStart(event, index) {
    draggedCommandIndex = index;
}

function onDrop(event, index) {
    if (draggedCommandIndex !== null) {
        const draggedCommand = currentCommands[draggedCommandIndex];
        currentCommands.splice(draggedCommandIndex, 1);
        currentCommands.splice(index, 0, draggedCommand);
        draggedCommandIndex = null;
        renderCommandList();
        window.electronAPI.updateCommandsInScenario(activeScenarioName, currentCommands)
            .catch(error => console.error("Error updating scenario file:", error));
    }
}

// ---------------------------------------------------------------------------------------
// TEACH-IN SCENARIOS: POSES MANAGEMENT
// ---------------------------------------------------------------------------------------
let currentPoses = {};

function refreshDropdown() {
    const dropdownContainer = document.getElementById('gotoInputContainer');
    dropdownContainer.innerHTML = ''; // Clear existing dropdowncontent

    if (Object.keys(currentPoses).length > 0) {
        const dropdown = document.createElement('select');
        dropdown.id = 'poseDropdown';

        //Add a placeholder option
        const placeholder = document.createElement('option');
        placeholder.textContent = 'Select a pose';
        placeholder.value = '';
        placeholder.disabled = true;
        placeholder.selected = true; //!! to make sure that its selected by default
        dropdown.appendChild(placeholder);

        //fill with options
        Object.keys(currentPoses).forEach((poseName) => {
            const option = document.createElement('option');
            option.value = poseName;
            option.textContent = poseName;
            dropdown.appendChild(option);
        });

        //Add event listener for dropdown changes
        dropdown.addEventListener('change', (event) => {
            selectedPose = event.target.value; // Store the selected pose
            console.log(`Selected pose: ${selectedPose}`);
        });

        dropdownContainer.appendChild(dropdown);

        //Handle the single pose case
        if (Object.keys(currentPoses).length === 1) {
            const singlePose = Object.keys(currentPoses)[0];
            dropdown.value = singlePose; //Set the dropdown value to the single pose
            selectedPose = singlePose; 
            console.log(`Single pose selected by default: ${selectedPose}`);
        }
    } else {
        dropdownContainer.innerHTML = '<p>No poses available in this scenario.</p>';
        selectedPose = null; //Reset the selected pose
    }
}



function loadPoses() {
    if (window.electronAPI && activeScenarioName) {
        currentPoses = {}; //Clear existing poses
        window.electronAPI.getPosesFromScenario(activeScenarioName)
            .then((poses) => {
                //load poses specific to the current scenario
                currentPoses = poses || {}; 
                renderPoseList();
                refreshDropdown();
            })
            .catch((error) => {
                console.error("Error loading poses:", error);
                //Ensure poses are cleared on error
                currentPoses = {};
                renderPoseList();
                refreshDropdown();
            });
    }
}


function renderPoseList() {
    const poseListElement = document.getElementById('pose-list');
    poseListElement.innerHTML = ''; 

    for (const poseName in currentPoses) {
        const jointPositions = currentPoses[poseName].joint_positions;
        const poseItem = document.createElement('div');
        poseItem.classList.add('pose-item');
        poseItem.innerHTML = `
            <span>${poseName}: [${jointPositions.join(', ')}]</span>
            <button class="delete-pose" data-name="${poseName}">Delete</button>
        `;
        poseListElement.appendChild(poseItem);
    }
}

document.getElementById('add-pose-btn').addEventListener('click', () => {
    const poseNameInput = document.getElementById('pose-name');
    const poseName = poseNameInput.value.trim();
    const isSingleWord = /^[^\s]+$/.test(poseName);

    //Validate pose name
    if (!poseName || !isSingleWord) {
        showCenterNotification("Please enter a valid pose name without spaces. It should be a single word.");
        poseNameInput.focus();
        return;
    }

    //Check for duplicate pose name
    if (currentPoses[poseName]) {
        showCenterNotification(`Pose "${poseName}" already exists. Please use a unique name.`);
        poseNameInput.focus();
        return;
    }

    //Determine the selected pose type
    const addManualPoseSelected = document.getElementById('manualPose').checked;
    const addCurrentPoseSelected = document.getElementById('currentPose').checked;

    if (addCurrentPoseSelected) {
        //Current Pose Logic
        if (latestJointStates && latestJointStates.length === 7 && latestJointStates.every(val => !isNaN(val))) {
            console.log('Using latestJointStates:', latestJointStates);

            currentPoses[poseName] = { joint_positions: latestJointStates };

            window.electronAPI.updatePosesInScenario(activeScenarioName, currentPoses)
                .then(() => {
                    renderPoseList();
                    refreshDropdown();
                    showNotification('Current live pose added to the list of poses!');
                    poseNameInput.value = '';
                    poseNameInput.focus();
                })
                .catch(error => {
                    console.error("Error saving current pose:", error);
                    showCenterNotification("Failed to save the current pose. Please try again.");
                });
        } else {
            showCenterNotification("Failed to capture valid joint positions. Ensure the robot is in a valid state.");
            poseNameInput.focus();
        }
    } else if (addManualPoseSelected) {
        //Manual Pose Logic
        const jointPositions = [
            Number(document.getElementById('joint1').value),
            Number(document.getElementById('joint2').value),
            Number(document.getElementById('joint3').value),
            Number(document.getElementById('joint4').value),
            Number(document.getElementById('joint5').value),
            Number(document.getElementById('joint6').value),
            Number(document.getElementById('joint7').value),
        ];

        console.log('Manual Input Values:', jointPositions);

        //Validate manual joint values
        if (jointPositions.length === 7 && jointPositions.every(val => !isNaN(val))) {
            currentPoses[poseName] = { joint_positions: jointPositions };

            window.electronAPI.updatePosesInScenario(activeScenarioName, currentPoses)
                .then(() => {
                    renderPoseList();
                    refreshDropdown();
                    showNotification('Manual pose added to the list of poses!');
                    document.getElementById('joint1').value = 0;
                    document.getElementById('joint2').value = 0;
                    document.getElementById('joint3').value = 0;
                    document.getElementById('joint4').value = 0;
                    document.getElementById('joint5').value = 0;
                    document.getElementById('joint6').value = 0;
                    document.getElementById('joint7').value = 0;
                    poseNameInput.value = '';
                    poseNameInput.focus();
                })
                .catch(error => {
                    console.error("Error saving manual pose:", error);
                    showCenterNotification("Failed to save the manual pose. Please try again.");
                });
        } else {
            showCenterNotification("Please enter valid numbers for all joint values.");
            poseNameInput.focus();
        }
    } else {
        showCenterNotification("Please select a pose type (current or manual).");
        poseNameInput.focus();
    }
});



document.getElementById('pose-list').addEventListener('click', (event) => {
    if (event.target.classList.contains('delete-pose')) {
        const poseName = event.target.getAttribute('data-name'); //this used to get posename

        //remove the pose fromlist
        delete currentPoses[poseName];
        showNotification('Pose deleted from the list of poses!');

        //update the scenario JSON
        window.electronAPI.updatePosesInScenario(activeScenarioName, currentPoses)
            .then(() => {
                renderPoseList(); //Refresh to show that it was deleted
                refreshDropdown(); //update any dropdowns dependent on poses
            })
            .catch((error) => console.error("Error updating JSON file:", error));
    }
});

}