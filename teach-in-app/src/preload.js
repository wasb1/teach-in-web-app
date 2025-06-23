/**
 * The preload script runs before `index.html` is loaded
 * in the renderer. It has access to web APIs as well as
 * Electron's renderer process modules and some polyfilled
 * Node.js functions.
 *
 * https://www.electronjs.org/docs/latest/tutorial/sandbox
 */

const { contextBridge, ipcRenderer} = require('electron');
const fs = require('fs');
const path = require('path');
const ROSLIB = require('roslib');

console.log("Preload: Preload.js is running.");

// ---------------------------------------------------------------------------------------
// ROSLIB / ROSBRIDE 
// ---------------------------------------------------------------------------------------

contextBridge.exposeInMainWorld('roslib', {
    connectToRos: (url) => {
        const ros = new ROSLIB.Ros();

        //Connect to ROSBridge
        console.log('Preload: Attempting to connect to ROSBridge at:', url);
        ros.connect(url);

        //Log connection events
        ros.on('connection', () => {
            console.log('Preload: Connected to ROSBridge');
        });

        ros.on('error', (error) => {
            console.error('Preload: Error connecting to ROSBridge:', error);
        });

        ros.on('close', () => {
            console.log('Preload: Connection to ROSBridge closed');
        });

        return {
            isConnected: () => ros.isConnected,
            onConnection: (callback) => ros.on('connection', callback),
            onError: (callback) => ros.on('error', callback),
            onClose: (callback) => ros.on('close', callback),

            //Subscribe to Joint States
            subscribeToJointStates: (callback) => {
                const jointStateTopic = new ROSLIB.Topic({
                    ros,
                    name: '/joint_states',
                    messageType: 'sensor_msgs/JointState',
                });

                jointStateTopic.subscribe((message) => {
                    const positions = message.position.map(p => p.toFixed(4)).join(', ');
                    callback(positions);
                });
            },
            // Disconnect from ROSBridge
            disconnect: () => {
            console.log('Preload: Disconnecting from ROSBridge');
            ros.close(); 
        },
    

            //Send Emergency Stop
            sendEmergencyStop: () => {
                const stopTopic = new ROSLIB.Topic({
                    ros,
                    name: '/emergency_stop',
                    messageType: 'std_msgs/Bool',
                });

                stopTopic.publish(new ROSLIB.Message({ data: true }));
                console.log('Preload: Emergency Stop Sent');
            },
            sendGotoCommand: (topicName, jointPositions, duration = 5) => {
                return new Promise((resolve, reject) => {
                    try {
                        const topic = new ROSLIB.Topic({
                            ros,
                            name: topicName,
                            messageType: 'trajectory_msgs/JointTrajectory',
                        });
        
                        const jointNames = [
                            'panda_joint1',
                            'panda_joint2',
                            'panda_joint3',
                            'panda_joint4',
                            'panda_joint5',
                            'panda_joint6',
                            'panda_joint7',
                        ];
        
                        const point = {
                            positions: jointPositions,
                            velocities: Array(jointPositions.length).fill(0.0),
                            time_from_start: { secs: 1, nsecs: 0 },
                        };
        
                        const message = new ROSLIB.Message({
                            header: { seq: 0, stamp: { secs: 0, nsecs: 0 }, frame_id: '' },
                            joint_names: jointNames,
                            points: [point],
                        });
        
                        topic.publish(message);
                        console.log(`Published Goto Command to ${topicName}:`, message);
        
                        setTimeout(() => {
                            resolve();
                            console.log(`Goto command completed after ${duration} seconds.`);
                        }, duration*1000);
                    } catch (error) {
                        reject(error);
                    }
                });
            },
        
            sendOpenCommand: () => {
                return new Promise((resolve, reject) => {
                    try {
                        const client = new ROSLIB.ActionClient({
                            ros,
                            serverName: '/franka_gripper/move',
                            actionName: 'franka_gripper/MoveAction',
                        });
        
                        const goal = new ROSLIB.Goal({
                            actionClient: client,
                            goalMessage: {
                                speed: 0.05,
                                width: 0.08,
                            }
                        });
        
                        goal.send();
                        goal.on('result', (result) => {
                            console.log('Open Gripper Status:', result);
                            resolve();
                        });
        
                        goal.on('error', (error) => {
                            console.error('Open Gripper Error:', error);
                            reject(error);
                        });
                    } catch (error) {
                        reject(error);
                    }
                });
            },
        
            sendCloseCommand: () => {
                return new Promise((resolve, reject) => {
                    try {
                        const client = new ROSLIB.ActionClient({
                            ros,
                            serverName: '/franka_gripper/move',
                            actionName: 'franka_gripper/MoveAction',
                        });
        
                        const goal = new ROSLIB.Goal({
                            actionClient: client,
                            goalMessage: {
                                speed: 0.05,
                                width: 0.0,
                            }
                        });
        
                        goal.send();
                        goal.on('result', (result) => {
                            console.log('Close Gripper Status:', result);
                            resolve();
                        });
        
                        goal.on('error', (error) => {
                            console.error('Close Gripper Error:', error);
                            reject(error);
                        });
                    } catch (error) {
                        reject(error);
                    }
                });
            },
        
            sendGraspCommand: () => {
                return new Promise((resolve, reject) => {
                    try {
                        const client = new ROSLIB.ActionClient({
                            ros,
                            serverName: '/franka_gripper/grasp',
                            actionName: 'franka_gripper/GraspAction',
                        });
        
                        const goal = new ROSLIB.Goal({
                            actionClient: client,
                            goalMessage: {
                                speed: 0.1,
                                width: 0.046,
                                force: 40,
                                epsilon: { inner: 0.005, outer: 0.005 },
                            }
                        });
        
                        goal.send();
                        goal.on('result', (result) => {
                            console.log('Grasp Status:', result);
                            resolve();
                        });
        
                        goal.on('error', (error) => {
                            console.error('Grasp Error:', error);
                            reject(error);
                        });
                    } catch (error) {
                        reject(error);
                    }
                });
            },
        
            sendHomingCommand: () => {
                return new Promise((resolve, reject) => {
                    try {
                        const client = new ROSLIB.ActionClient({
                            ros,
                            serverName: '/franka_gripper/homing',
                            actionName: 'franka_gripper/HomingAction',
                        });
        
                        const goal = new ROSLIB.Goal({
                            actionClient: client,
                            goalMessage: {}
                        });
        
                        goal.send();
                        goal.on('result', (result) => {
                            console.log('Homing Status:', result);
                            resolve();
                        });
        
                        goal.on('error', (error) => {
                            console.error('Homing Error:', error);
                            reject(error);
                        });
                    } catch (error) {
                        reject(error);
                    }
                });
            },
        
            sendSleepCommand: (duration) => {
                return new Promise((resolve) => {
                    console.log(`Sleeping for ${duration} seconds...`);
                    setTimeout(() => {
                        console.log(`Sleep completed after ${duration} seconds.`);
                        resolve();
                    }, duration * 1000);
                });
            },
        
            sendGripperWidth: (width) => {
                return new Promise((resolve, reject) => {
                    try {
                        if (width < 0.0 || width > 0.08) {
                            reject(new Error('Width out of range. Must be between 0.0 and 0.08.'));
                            return;
                        }
        
                        const client = new ROSLIB.ActionClient({
                            ros,
                            serverName: '/franka_gripper/move',
                            actionName: 'franka_gripper/MoveAction',
                        });
        
                        const goal = new ROSLIB.Goal({
                            actionClient: client,
                            goalMessage: {
                                speed: 0.05,
                                width: width,
                            }
                        });
        
                        goal.send();
                        goal.on('result', (result) => {
                            console.log('Gripper Width Status:', result);
                            resolve();
                        });
        
                        goal.on('error', (error) => {
                            console.error('Gripper Width Error:', error);
                            reject(error);
                        });
                    } catch (error) {
                        reject(error);
                    }
                });
            },
            sendRecoverCommand: () => {
                return new Promise((resolve, reject) => {
                    try {
                        const actionClient = new ROSLIB.ActionClient({
                            ros,
                            serverName: '/franka_control/error_recovery',
                            actionName: 'franka_msgs/ErrorRecoveryAction',
                        });
            
                        const goal = new ROSLIB.Goal({
                            actionClient: actionClient,
                            goalMessage: {}, // Empty goal message for error recovery
                        });
            
                        goal.send();
                        console.log("Sent recovery command to /franka_control/error_recovery");
            
                        goal.on('result', (result) => {
                            console.log("Recovery completed:", result);
                            resolve(result);
                        });
            
                        goal.on('error', (error) => {
                            console.error("Error during recovery:", error);
                            reject(error);
                        });
                    } catch (error) {
                        console.error("Failed to send recovery command:", error);
                        reject(error);
                    }
                });
            },
            

        };
    },
});

// ---------------------------------------------------------------------------------------
// FILE MANAGEMENT
// ---------------------------------------------------------------------------------------

contextBridge.exposeInMainWorld('electronAPI', {

    // ---------------------------------------------------------------------------------------
    // SCNEARIO MANAGEMENT
    // ---------------------------------------------------------------------------------------

    //creates a new scenario file as JSON with boilerplate structure as originally developped
    createScenarioFile: (scenarioName) => {
        try {
            const scenariosDir = path.join(__dirname, '..', 'scenarios');
            if (!fs.existsSync(scenariosDir)) {
                fs.mkdirSync(scenariosDir);
            }
            //only add .json if it is not already present
            const filePath = path.join(scenariosDir, scenarioName.endsWith('.json') ? scenarioName : `${scenarioName}.json`);
            if (!fs.existsSync(filePath)) {
                const initialScenario = {
                    name: scenarioName,
                    poses: {},
                    commands: [],
                    pose_type: "sensor_msgs.msg.JointState",
                    pose_topic: "/joint_states"
                };
                fs.writeFileSync(filePath, JSON.stringify(initialScenario, null, 2));
                console.log(`Scenario '${scenarioName}' created successfully at ${filePath}`);
            }
        } catch (error) {
            console.error("Error creating scenario file:", error);
            throw error;
        }
    },

    getScenarios: async () => {
        const scenariosPath = path.resolve(__dirname, '../scenarios');
        if (fs.existsSync(scenariosPath)) {
            return fs.readdirSync(scenariosPath);
        } else {
            //return empty array if directory does not exist to avioid errors
            return [];
        }
    },

    // ---------------------------------------------------------------------------------------
    // COMMAND MANAGEMENT
    // ---------------------------------------------------------------------------------------

    // Adds a command to an existing scenario file
    addCommandToScenario: (scenarioName, command) => {
        try {
            const scenariosDir = path.join(__dirname, '..', 'scenarios');
            //Only adds .json if it is not already present
            const filePath = path.join(scenariosDir, scenarioName.endsWith('.json') ? scenarioName : `${scenarioName}.json`);

            if (!fs.existsSync(filePath)) {
                throw new Error(`Scenario file ${scenarioName}.json does not exist. Create the scenario first.`);
            }

            const scenario = JSON.parse(fs.readFileSync(filePath, 'utf-8'));
            scenario.commands.push(command);

            fs.writeFileSync(filePath, JSON.stringify(scenario, null, 2));
            console.log(`Command '${command}' added to scenario '${scenarioName}'.`);
        } catch (error) {
            console.error("Error adding command to scenario:", error);
            throw error;
        }
    },

    getCommandsFromScenario: (scenarioName) => {
        return new Promise((resolve, reject) => {
            const scenariosDir = path.join(__dirname, '..', 'scenarios');
            const filePath = path.join(scenariosDir, scenarioName.endsWith('.json') ? scenarioName : `${scenarioName}.json`);

            try {
                if (fs.existsSync(filePath)) {
                    const scenarioData = JSON.parse(fs.readFileSync(filePath, 'utf8'));
                    resolve(scenarioData.commands || []); // Return commands array or empty if not found
                } else {
                    console.error(`Scenario file not found for ${scenarioName}`);
                    resolve([]); //No commands if file doesnt exist
                }
            } catch (error) {
                console.error("Error reading commands from scenario file:", error);
                reject(error);
            }
        });
    },
    //new function to update commands in the scenario file
    updateCommandsInScenario: (scenarioName, updatedCommands) => {
        return new Promise((resolve, reject) => {
            const scenariosDir = path.join(__dirname, '..', 'scenarios');
            const filePath = path.join(scenariosDir, scenarioName.endsWith('.json') ? scenarioName : `${scenarioName}.json`);

            try {
                if (fs.existsSync(filePath)) {
                    const scenarioData = JSON.parse(fs.readFileSync(filePath, 'utf8'));
                    scenarioData.commands = updatedCommands;
                    fs.writeFileSync(filePath, JSON.stringify(scenarioData, null, 2));
                    console.log(`Commands for scenario '${scenarioName}' updated successfully.`);
                    resolve();
                } else {
                    console.error(`Scenario file not found for ${scenarioName}`);
                    reject(new Error("File not found"));
                }
            } catch (error) {
                console.error("Error updating commands in scenario file:", error);
                reject(error);
            }
        });
    },
    // ---------------------------------------------------------------------------------------
    // POSES MANAGEMENT
    // ---------------------------------------------------------------------------------------
    //retrieve poses from the scenario file
    getPosesFromScenario: (scenarioName) => {
        return new Promise((resolve, reject) => {
            const scenariosDir = path.join(__dirname, '..', 'scenarios');
            const filePath = path.join(scenariosDir, scenarioName.endsWith('.json') ? scenarioName : `${scenarioName}.json`);
    
            try {
                if (fs.existsSync(filePath)) {
                    const scenarioData = JSON.parse(fs.readFileSync(filePath, 'utf8'));
                    console.log(`Loaded poses for scenario '${scenarioName}':`, scenarioData.poses);
                    resolve(scenarioData.poses || {}); //this makes sure that it always returns an object
                } else {
                    console.error(`Scenario file not found for ${scenarioName}`);
                    resolve({}); //return empty object if file doesn't exist
                }
            } catch (error) {
                console.error("Error reading poses from scenario file:", error);
                reject(error);
            }
        });
    },
    

    //update poses in the scenario file
    updatePosesInScenario: (scenarioName, updatedPoses) => {
        //ensure the file name ends with '.json'
        const fileName = scenarioName.endsWith('.json') ? scenarioName : `${scenarioName}.json`;
        const scenariosDir = path.resolve(__dirname, '../scenarios');
        const scenarioPath = path.join(scenariosDir, fileName);

        return new Promise((resolve, reject) => {
            //check if the file exists before attempting to read
            if (!fs.existsSync(scenarioPath)) {
                console.error(`Scenario file not found: ${scenarioPath}`);
                reject(new Error(`Scenario file not found: ${scenarioPath}`));
                return;
            }

            //read, modify, and write back to the file
            fs.readFile(scenarioPath, 'utf-8', (err, data) => {
                if (err) {
                    console.error('Failed to read scenario file:', err);
                    reject(err);
                    return;
                }

                try {
                    const scenarioData = JSON.parse(data);
                    scenarioData.poses = updatedPoses; //update the poses section
                    fs.writeFile(scenarioPath, JSON.stringify(scenarioData, null, 2), (err) => {
                        if (err) {
                            console.error('Failed to write updated scenario file:', err);
                            reject(err);
                        } else {
                            console.log('Scenario file updated successfully.');
                            resolve();
                        }
                    });
                } catch (error) {
                    console.error('Error parsing scenario file:', error);
                    reject(error);
                }
            });
        });
    },
});

window.addEventListener('DOMContentLoaded', () => {
    const replaceText = (selector, text) => {
        const element = document.getElementById(selector);
        if (element) element.innerText = text;
    };

    for (const type of ['chrome', 'node', 'electron']) {
        replaceText(`${type}-version`, process.versions[type]);
    }

});