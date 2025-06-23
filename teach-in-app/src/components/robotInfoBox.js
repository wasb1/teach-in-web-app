import { showCenterNotification, showNotification } from './uiUtils.js';

//-------------------------------------------------------------------------------------------------
// ROBOT INFO BOX CONTROL
//-----------------------------------------------------------------------=--------------------------

let latestJointStates = null;
let robotInfoWidthSlider = null;


const robotInfoSliders = {
    joint1: null,
    joint2: null,
    joint3: null,
    joint4: null,
    joint5: null,
    joint6: null,
    joint7: null,
};

const robotInfoJointLimits = {
    joint1: [-2.8973, 2.8973],
    joint2: [-1.7628, 1.7628],
    joint3: [-2.8973, 2.8973],
    joint4: [-3.0718, -0.0698],
    joint5: [-2.8973, 2.8973],
    joint6: [-0.0175, 3.7525],
    joint7: [-2.8973, 2.8973],
};

const robotInfoWidthLimits = [0.0, 0.08];
const startPositionJointStates = [0, -0.7856, 0, -2.3559, 0, 1.5717, 0.7854]; 

//function to initialize Robot Info Box controls
export function setupRobotInfoBox(ros) {
    const robotInfoInputContainer = document.getElementById('robot-info-box-inputs');
    //Subscribe to joint states
    ros.subscribeToJointStates((positions) => {
        const robotPositionElement = document.getElementById('robot-position');
        if (robotPositionElement) {
            robotPositionElement.textContent = positions;
        }
        latestJointStates = positions.split(',').slice(0, 7).map(Number);
    });

    //Emergency Stop Button
    const emergencyStopButton = document.getElementById('emergency-stop-btn');
    if (emergencyStopButton) {
        emergencyStopButton.addEventListener('click', () => {
            ros.sendEmergencyStop();
            ros.disconnect()
            showCenterNotification('Emergency Stop Command Sent!');
            console.log('Robot Info Box: Emergency Stop Sent');
        });
    }

    //thisis to set the sliders for goto and width commands
    function initializeRobotInfoInputs(command) {
        robotInfoInputContainer.innerHTML = '';

        if (command === 'goto') {
            for (let i = 1; i <= 7; i++) {
                const jointKey = `joint${i}`;
                const [min, max] = robotInfoJointLimits[jointKey];

                //label for the joint
                const label = document.createElement('label');
                label.setAttribute('for', jointKey);
                label.textContent = `Joint ${i} (${min.toFixed(4)} to ${max.toFixed(4)}):`;

                //Create a range slider
                const slider = document.createElement('input');
                slider.type = 'range';
                slider.id = jointKey;
                slider.min = min;
                slider.max = max;
                slider.step = 0.0001;
                slider.value = (min + max) / 2; //start at the middle (to be changed later if time is enough to current subscribed values)

                robotInfoSliders[jointKey] = slider;

                //Create a text display for the slider value
                const valueDisplay = document.createElement('span');
                valueDisplay.id = `${jointKey}-value`;
                valueDisplay.textContent = slider.value;

                //Update display on slider change
                slider.addEventListener('input', () => {
                    valueDisplay.textContent = slider.value;
                });

                //Append to the container
                robotInfoInputContainer.appendChild(label);
                robotInfoInputContainer.appendChild(slider);
                robotInfoInputContainer.appendChild(valueDisplay);
                robotInfoInputContainer.appendChild(document.createElement('br'));
            }
        } else if (command === 'width') {
            const [min, max] = robotInfoWidthLimits;

            //Create a label for the width
            const label = document.createElement('label');
            label.setAttribute('for', 'robot-info-width-slider');
            label.textContent = `Width (${min} to ${max}):`;

            //Create the width slider
            robotInfoWidthSlider = document.createElement('input');
            robotInfoWidthSlider.type = 'range';
            robotInfoWidthSlider.id = 'robot-info-width-slider';
            robotInfoWidthSlider.min = min;
            robotInfoWidthSlider.max = max;
            robotInfoWidthSlider.step = 0.001;
            robotInfoWidthSlider.value = (min + max) / 2; //Start at the middle (to be changed later if time is enough to current subscribed values)

    
            const valueDisplay = document.createElement('span');
            valueDisplay.id = 'robot-info-width-slider-value';
            valueDisplay.textContent = robotInfoWidthSlider.value;

            // Update
            robotInfoWidthSlider.addEventListener('input', () => {
                valueDisplay.textContent = robotInfoWidthSlider.value;
            });

            //Append to the container
            robotInfoInputContainer.appendChild(label);
            robotInfoInputContainer.appendChild(robotInfoWidthSlider);
            robotInfoInputContainer.appendChild(valueDisplay);
            robotInfoInputContainer.appendChild(document.createElement('br'));
        }
    }

    //Command execution logic
    document.getElementById('robot-info-execute-btn').addEventListener('click', async () => {
        const selectedCommand = document.querySelector('input[name="robot-info-command"]:checked');
        if (!selectedCommand) {
           showCenterNotification('Please select a command.');
            return;
        }

        const command = selectedCommand.value;

        if (command === 'goto') {
            //collect joint values from sliders
            const jointStates = Object.keys(robotInfoSliders).map(
                (jointKey) => parseFloat(robotInfoSliders[jointKey].value)
            );

            if (jointStates.length !== 7 || jointStates.some(isNaN)) {
                showCenterNotification('Invalid joint states! Please adjust all sliders.');
                console.error('Invalid joint states input:', jointStates);
                return;
            }
            showNotification('Move command sent!');
            console.log('Robot Info: Parsed Joint States:', jointStates);

            try {
                await ros.sendGotoCommand(
                    '/position_joint_trajectory_controller/command',
                    jointStates,
                    5
                );
                console.log('Robot Info: Goto command sent successfully.');
            } catch (error) {
                console.error('Robot Info: Failed to send Goto command:', error);
            }
        } else if (command === 'width') {
            if (!robotInfoWidthSlider) {
                showCenterNotification('Width slider is not initialized. Please reselect the Width command.');
                return;
            }

            const width = parseFloat(robotInfoWidthSlider.value);
            if (isNaN(width) || width < robotInfoWidthLimits[0] || width > robotInfoWidthLimits[1]) {
                showCenterNotification('Invalid width. Must be between 0.0 and 0.08.');
                return;
            }
            ros.sendGripperWidth(width);
            console.log(`Robot Info: Width Command Executed with Value: ${width}`);
            showNotification(`Gripper Width command sent!`);
        } else if (command === 'start') {
            showNotification('Ready Position command sent!.');
            try {
                await ros.sendGotoCommand(
                    '/position_joint_trajectory_controller/command',
                    startPositionJointStates,
                    5
                );
                console.log('Robot Info: Ready Position command sent successfully.');
            } catch (error) {
                console.error('Robot Info: Failed to send Ready Position command:', error);
            }
        } else {
            console.error('Unsupported command selected.');
        }
    });

    document.querySelectorAll('input[name="robot-info-command"]').forEach((radio) => {
        radio.addEventListener('change', (event) => {
            initializeRobotInfoInputs(event.target.value);
        });
    });
}

export { latestJointStates };