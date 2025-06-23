import {showNotification, showCenterNotification} from './uiUtils.js';
//-------------------------------------------------------------------------------------------------
// REALTIME CONTROL MENU
//-------------------------------------------------------------------------------------------------


const jointSliders = {
    joint1: null,
    joint2: null,
    joint3: null,
    joint4: null,
    joint5: null,
    joint6: null,
    joint7: null,
};

const jointLimits = {
    joint1: [-2.8973, 2.8973],
    joint2: [-1.7628, 1.7628],
    joint3: [-2.8973, 2.8973],
    joint4: [-3.0718, -0.0698],
    joint5: [-2.8973, 2.8973],
    joint6: [-0.0175, 3.7525],
    joint7: [-2.8973, 2.8973],
};

let widthSlider = null;
const widthLimits = [0.0, 0.08];

const realtimeCommandInputs = {
    goto: document.getElementById("realtime-gotoInputContainer"),
    width: document.getElementById("widthInputContainer"),
    sleep: document.getElementById("realtime-sleep-input"),
    open: null,
    close: null,
    grasp: null,
    homing: null,
    recover: null,
};

// this Function to initialize Realtime Control Menu
export function setupRealtimeControlMenu(ros) {
    // Hide all input fields initially
    Object.values(realtimeCommandInputs).forEach((input) => {
        if (input) input.style.display = "none";
    });

    // Show the relevant input field when a command is selected
    document.querySelectorAll('input[name="realtime-command"]').forEach((radio) => {
        radio.addEventListener("change", () => {
            Object.keys(realtimeCommandInputs).forEach((cmd) => {
                if (realtimeCommandInputs[cmd]) {
                    if (cmd === "goto" && radio.value === "goto") {
                        const dropdownContainer = realtimeCommandInputs[cmd];
                        dropdownContainer.innerHTML = "";

                        for (let i = 1; i <= 7; i++) {
                            const jointKey = `joint${i}`;
                            const [min, max] = jointLimits[jointKey];

                            //label for the joint
                            const label = document.createElement("label");
                            label.setAttribute("for", jointKey);
                            label.textContent = `Joint ${i} (${min.toFixed(4)} to ${max.toFixed(4)}):`;

                            //Create a range slider
                            const slider = document.createElement("input");
                            slider.type = "range";
                            slider.id = jointKey;
                            slider.min = min;
                            slider.max = max;
                            slider.step = 0.0001;
                            slider.value = (min + max) / 2; //start at the middle (to be changed later if time is enough to current subscribed values)

                            //Store reference to the slider
                            jointSliders[jointKey] = slider;

                            //Create a text display for the slider value
                            const valueDisplay = document.createElement("span");
                            valueDisplay.id = `${jointKey}-value`;
                            valueDisplay.textContent = slider.value;

                            //Update
                            slider.addEventListener("input", () => {
                                valueDisplay.textContent = slider.value;
                            });

                            //Append to the container
                            dropdownContainer.appendChild(label);
                            dropdownContainer.appendChild(slider);
                            dropdownContainer.appendChild(valueDisplay);
                            dropdownContainer.appendChild(document.createElement("br"));
                        }

                        dropdownContainer.style.display = "block";
                    } else if (cmd === "width" && radio.value === "width") {
                        const widthContainer = realtimeCommandInputs[cmd];
                        widthContainer.innerHTML = "";

                        //Create a label for the width
                        const label = document.createElement("label");
                        label.setAttribute("for", "width-slider");
                        label.textContent = `Width (${widthLimits[0]} to ${widthLimits[1]}):`;

                        //Create the width slider
                        widthSlider = document.createElement("input");
                        widthSlider.type = "range";
                        widthSlider.id = "width-slider";
                        widthSlider.min = widthLimits[0];
                        widthSlider.max = widthLimits[1];
                        widthSlider.step = 0.001;
                        widthSlider.value = (widthLimits[0] + widthLimits[1]) / 2; //start at the middle (to be changed later if time is enough to current subscribed values)

                        //Create a text display for the slider value
                        const valueDisplay = document.createElement("span");
                        valueDisplay.id = "width-slider-value";
                        valueDisplay.textContent = widthSlider.value;

                        //Update
                        widthSlider.addEventListener("input", () => {
                            valueDisplay.textContent = widthSlider.value;
                        });

                        //Append to the container
                        widthContainer.appendChild(label);
                        widthContainer.appendChild(widthSlider);
                        widthContainer.appendChild(valueDisplay);
                        widthContainer.appendChild(document.createElement("br"));

                        widthContainer.style.display = "block";
                    } else {
                        realtimeCommandInputs[cmd].style.display =
                            radio.value === cmd ? "block" : "none";
                    }
                }
            });
        });
    });

    //Event listener for "Execute" button
    document.getElementById("realtime-execute-btn").addEventListener("click", async () => {
        const selectedCommand = document.querySelector(
            'input[name="realtime-command"]:checked'
        );
        if (!selectedCommand) {
            showCenterNotification("Please select a command.");
            return;
        }

        const command = selectedCommand.value;

        if (command === "goto") {
            const jointStates = Object.keys(jointSliders).map((jointKey) =>
                parseFloat(jointSliders[jointKey].value)
            );

            if (jointStates.length !== 7 || jointStates.some(isNaN)) {
                showCenterNotification("Invalid joint states! Please adjust all sliders.");
                console.error("Invalid joint states input:", jointStates);
                return;
            }
            showNotification("Move command sent!");
            try {
                await ros.sendGotoCommand(
                    "/position_joint_trajectory_controller/command",
                    jointStates,
                    5
                );
                console.log("Goto command sent successfully.");
            } catch (error) {
                console.error("Failed to send Goto command:", error);
            }
        } else if (command === "width") {
            if (!widthSlider) {
                showCenterNotification("Width slider is not initialized. Please reselect the Width command.");
                return;
            }

            const width = parseFloat(widthSlider.value);
            if (
                isNaN(width) ||
                width < widthLimits[0] ||
                width > widthLimits[1]
            ) {
                showCenterNotification("Invalid width. Must be between 0.0 and 0.08.");
                return;
            }
            showNotification("Gripper width command sent!");
            ros.sendGripperWidth(width);
            console.log(`Width command executed with value: ${width}`);
        } else if (command === "sleep") {
            const duration = parseInt(realtimeCommandInputs.sleep.value, 10);
            if (isNaN(duration) || duration <= 0) {
                showCenterNotification("Invalid sleep duration. Must be a positive integer.");
                return;
            }
            showNotification("Sleep command sent!");
            ros.sendSleepCommand(duration);
            console.log(`Sleep command executed with duration: ${duration}`);
        } else if (command === "open") {
            showNotification("Open Gripper command sent!");
            ros.sendOpenCommand();
            console.log("Open command executed.");
        } else if (command === "close") {
            showNotification("Close Gripper command sent!");
            ros.sendCloseCommand();
            console.log("Close command executed.");
        } else if (command === "grasp") {
            showNotification("Grasp command sent!");
            ros.sendGraspCommand();
            console.log("Grasp command executed.");
        } else if (command === "homing") {
            showNotification("Homing command sent!");
            ros.sendHomingCommand();
            console.log("Homing command executed.");
        } else if (command === "recover") {
            showNotification("Recover command sent!");
            ros.sendRecoverCommand();
            console.log("Recover command executed.");
        } else {
            console.error("Unsupported command selected.");
        }
    });
}
