import React, { Component } from 'react';
import { Container, Row, Col, Form, FormGroup, Button, Dropdown } from 'react-bootstrap';
import ros_config from '../../configs/ros_config';
import ROSConnect from '../ros-bridge/ROSConnect';
import RosTopicList from '../ros-bridge/RosTopicList';

class Settings extends Component {
    constructor(props) {
        super(props);

        this.state = {
            ros: null,
            rosbridgeServerIP: localStorage.getItem('rosbridgeServerIP') || ros_config.ROSBRIDGE_SERVER_IP,
            rosbridgeServerPort: localStorage.getItem('rosbridgeServerPort') || ros_config.ROSBRIDGE_SERVER_PORT,
            imageWidth: localStorage.getItem('imageWidth') || ros_config.ROSBRIDGE_IMAGE_WIDTH,
            imageHeight: localStorage.getItem('imageHeight') || ros_config.ROSBRIDGE_IMAGE_HEIGHT,
            frameWidth: localStorage.getItem('frameWidth') || ros_config.ROSBRIDGE_FRAME_WIDTH,
            frameHeight: localStorage.getItem('frameHeight') || ros_config.ROSBRIDGE_FRAME_HEIGHT,
            batteryStatus: localStorage.getItem('batteryStatus') !== null ? localStorage.getItem('batteryStatus') === "true" : ros_config.ROSBRIDGE_BATTERY_STATUS,
            manualTeleop: localStorage.getItem('manualTeleop') !== null ? localStorage.getItem('manualTeleop') === "true" : ros_config.ROSBRIDGE_MANUAL_TELEOP,
            isDarkMode: localStorage.getItem('darkMode') !== null ? localStorage.getItem('darkMode') === "true" : ros_config.DARK_MODE,
            invalidIP: false,
            invalidPort: false,
            invalidWidth: false,
            invalidHeight: false,
            invalidFrameWidth: false,
            invalidFrameHeight: false,
            showConfig: false
        }

        this.setRos = this.setRos.bind(this);
        this.updateManualTeleopState = this.updateManualTeleopState.bind(this);
        this.toggleConfig = this.toggleConfig.bind(this);
    }

    setRos(ros) {
        this.setState({ ros });
    }

    //Updates the manualTeleop state and stores the value in local storage (to switch to input boxes instead of joystick).
    updateManualTeleopState(newState) {
        localStorage.setItem('manualTeleop', newState);
        return newState;
    }

    //Updates the batteryStatus state and stores the value in local storage (to show the battery percentage or not).
    updateShowBattery(newState) {
        localStorage.setItem('batteryStatus', newState);
        return newState;
    }

    //Updates the darkMode status to change the theme of the site between dark mode and light mode
    updateDarkMode(newState) {
        localStorage.setItem('darkMode', newState);
        window.location.reload();
        return newState;
    }

    //Toggles the showConfig state based on the isOpen parameter (to show a list of the current setting configurations)
    toggleConfig(isOpen) {
        this.setState({ showConfig: isOpen });
    }

    //Handles the input change event for the form inputs. Updates the corresponding state and performs validation for certain fields.
    handleInputChange = (event) => {
        const { name, value } = event.target;
      
        // Only allow numbers and periods for rosbridgeServerIP
        if (name === "rosbridgeServerIP" && !/^[\d.]+$/.test(value)) {
            this.setState({ invalidIP: true });
            return;
        }

        // Only allow numbers for rosbridgeServerPort, imageWidth, and imageHeight
        if (name === "rosbridgeServerPort" && !/^\d+$/.test(value)) {
            this.setState({ invalidPort: true });
        }
        else if (name === "imageWidth" && !/^\d+$/.test(value)) {
            this.setState({ invalidWidth: true });
        }
        else if (name === "imageHeight" && !/^\d+$/.test(value)) {
            this.setState({ invalidHeight: true });
        }
        else if (name === "frameWidth" && !/^\d+$/.test(value)) {
            this.setState({ invalidFrameWidth: true });
        }
        else if (name === "frameHeight" && !/^\d+$/.test(value)) {
            this.setState({ invalidFrameHeight: true });
        }
        else {
            this.setState({ [name]: value, invalidIP: false, invalidPort: false, invalidWidth: false, invalidHeight: false, invalidQuality: false, invalidFrameWidth: false, invalidFrameHeight: false });
        }
      
    };

    //Handles the form submit event. Prevents the default form submission behavior.
    handleFormSubmit = (event) => {
        event.preventDefault();
    };

    //Handles the save button click event. Stores the input values in local storage and reloads the page if the server IP or port has changed. Displays an alert if any input field values are invalid.
    handleSaveClick = () => {
        const storedIP = localStorage.getItem('rosbridgeServerIP') || ros_config.rosbridgeServerIP;
        const storedPort = localStorage.getItem('rosbridgeServerPort') || ros_config.rosbridgeServerPort;
        localStorage.setItem('rosbridgeServerIP', this.state.rosbridgeServerIP);
        localStorage.setItem('rosbridgeServerPort', this.state.rosbridgeServerPort);
        localStorage.setItem('imageWidth', this.state.imageWidth);
        localStorage.setItem('imageHeight', this.state.imageHeight);
        localStorage.setItem('frameWidth', this.state.frameWidth);
        localStorage.setItem('frameHeight', this.state.frameHeight);
        if (this.state.rosbridgeServerIP !== storedIP || this.state.rosbridgeServerPort !== storedPort) {
            alert("Your settings have been applied successfully.")
            window.location.reload();
        }
        if (this.state.invalidIP === true || this.state.invalidPort === true || this.state.invalidWidth === true || this.state.invalidHeight === true || this.state.invalidFrameWidth === true || this.state.invalidFrameHeight === true) {
            alert("One or more input field values are invalid, please re-enter a valid input!")
        }
        else {
            alert("Your settings have been applied successfully.")
        }
    };

    //Handles the reset button click event. Clears the local storage and reloads the page if the server IP or port has changed.
    handleResetClick = () => {
        const storedIP = this.state.rosbridgeServerIP
        const storedPort = this.state.rosbridgeServerPort
        localStorage.clear();
        if (ros_config.ROSBRIDGE_SERVER_IP !== storedIP || ros_config.ROSBRIDGE_SERVER_PORT !== storedPort) {
            window.location.reload();
        }
    };

    //Handles the clear button click event. Clears the input values and resets the corresponding invalid flags in the state.
    handleClearClick = () => {
        document.getElementsByName("rosbridgeServerIP")[0].value = "";
        document.getElementsByName("rosbridgeServerPort")[0].value = "";
        document.getElementsByName("imageWidth")[0].value = "";
        document.getElementsByName("imageHeight")[0].value = "";
        document.getElementsByName("frameWidth")[0].value = "";
        document.getElementsByName("frameHeight")[0].value = "";
        this.setState({ invalidIP: false, invalidPort: false, invalidWidth: false, invalidHeight: false, invalidQuality: false, invalidFrameWidth: false, invalidFrameHeight: false });
    };
      
    render() {
        const { ros, batteryStatus, manualTeleop, showConfig, isDarkMode, invalidIP, invalidPort, invalidWidth, invalidHeight, invalidFrameWidth, invalidFrameHeight } = this.state;
        const currentConfigBG = isDarkMode ? 'BG-dark' : 'BG-light';

        return (
            <Container>
                <h1 id="-project-command-control-" style={{paddingTop: "1%"}}><strong>Settings</strong></h1>

                <div className="divider"/>

                <Row>
                    <Col>
                        <ROSConnect setRos={this.setRos}/>
                    </Col>
                </Row>

                <Form onSubmit={this.handleFormSubmit}>
                    <FormGroup>
                        <Row>
                            <Col>
                                <Form.Label>Rosbridge Server IP Address</Form.Label>
                                <Form.Control name="rosbridgeServerIP" onChange={this.handleInputChange} placeholder="Ex: 127.0.0.1" style={{width: "50%"}}/>
                                {invalidIP && (
                                    <span style={{ color: "red" }}>
                                    Invalid rosbridge server IP, please re-enter a number value!
                                    </span>
                                )}
                            </Col>
                            <Col>
                                <Form.Label>Port</Form.Label>
                                <Form.Control name="rosbridgeServerPort" onChange={this.handleInputChange} placeholder="Ex: 9090" style={{width: "20%"}}/>
                                {invalidPort && (
                                    <span style={{ color: "red" }}>
                                    Invalid rosbridge server port, please re-enter a number value!
                                    </span>
                                )}
                            </Col>
                        </Row>
                    </FormGroup>
    
                    <FormGroup style={{ marginTop: "2%" }}>
                        <Row>
                            <Col>
                                <Form.Label>Video Resolution Width</Form.Label>
                                <Form.Control name="imageWidth" onChange={this.handleInputChange} placeholder="Ex: 640" style={{width: "20%"}}/>
                                {invalidWidth && (
                                    <span style={{ color: "red" }}>
                                    Invalid image width input, please re-enter a number value!
                                    </span>
                                )}
                            </Col>
                            <Col>
                                <Form.Label>Video Resolution Height</Form.Label>
                                <Form.Control name="imageHeight" onChange={this.handleInputChange} placeholder="Ex: 360" style={{width: "20%"}}/>
                                {invalidHeight && (
                                    <span style={{ color: "red" }}>
                                    Invalid image height input, please re-enter a number value!
                                    </span>
                                )}
                            </Col>
                        </Row>
                    </FormGroup>

                    <FormGroup style={{ marginTop: "2%" }}>
                        <Row>
                            <Col>
                                <Form.Label>Video Frame Width</Form.Label>
                                <Form.Control name="frameWidth" onChange={this.handleInputChange} placeholder="Ex: 640" style={{width: "20%"}}/>
                                {invalidFrameWidth && (
                                    <span style={{ color: "red" }}>
                                    Invalid video frame width input, please re-enter a number value!
                                    </span>
                                )}
                            </Col>
                            <Col>
                                <Form.Label>Video Frame Height</Form.Label>
                                <Form.Control name="frameHeight" onChange={this.handleInputChange} placeholder="Ex: 360" style={{width: "20%"}}/>
                                {invalidFrameHeight && (
                                    <span style={{ color: "red" }}>
                                    Invalid video frame height input, please re-enter a number value!
                                    </span>
                                )}
                            </Col>
                        </Row>
                    </FormGroup>
    
                    <FormGroup style={{ marginTop: "4%" }}>
                        <Button onClick={this.handleSaveClick} variant="primary" style={{marginRight: "1%"}}>
                            Save
                        </Button>

                        <Button variant="warning" onClick={this.handleClearClick}>
                            Clear
                        </Button>
                    </FormGroup>

                    <div className="divider"/>
    
                    <div style={{ display: 'flex', justifyContent: 'row', marginTop: "2%"}}>
                        <RosTopicList ros={ros}/>
                        <Dropdown onToggle={this.toggleConfig} show={showConfig} style={{marginLeft: "1.5%"}}>
                            <Dropdown.Toggle variant="info">
                            {showConfig
                                ? 'Hide Current Configuration'
                                : 'Show Current Configuration'}
                            </Dropdown.Toggle>
                            <Dropdown.Menu className={currentConfigBG}>
                                <Dropdown.Item className={currentConfigBG}> [ Rosbridge Server IP ] : {localStorage.getItem('rosbridgeServerIP') || ros_config.ROSBRIDGE_SERVER_IP} </Dropdown.Item>
                                <Dropdown.Item className={currentConfigBG}> [ Rosbridge Server Port ]: {localStorage.getItem('rosbridgeServerPort') || ros_config.ROSBRIDGE_SERVER_PORT} </Dropdown.Item>
                                <Dropdown.Item className={currentConfigBG}> [ Video Resolution Width & Height ]: {localStorage.getItem('imageWidth') || ros_config.ROSBRIDGE_IMAGE_WIDTH} x {localStorage.getItem('imageHeight') || ros_config.ROSBRIDGE_IMAGE_HEIGHT}</Dropdown.Item>
                                <Dropdown.Item className={currentConfigBG}> [ Video Frame Width & Height]: {localStorage.getItem('frameWidth') || ros_config.ROSBRIDGE_FRAME_WIDTH} x {localStorage.getItem('frameHeight') || ros_config.ROSBRIDGE_FRAME_HEIGHT}</Dropdown.Item>
                                <Dropdown.Item className={currentConfigBG}> [ Show Battery Status ]: {localStorage.getItem('batteryStatus') !== null ? (localStorage.getItem('batteryStatus') === 'true' ? 'On' : 'Off') : (ros_config.ROSBRIDGE_BATTERY_STATUS ? 'On' : 'Off')} </Dropdown.Item>
                                <Dropdown.Item className={currentConfigBG}> [ Manual Input Teleoperation ]: {localStorage.getItem('manualTeleop') !== null ? (localStorage.getItem('manualTeleop') === 'true' ? 'On' : 'Off') : (ros_config.ROSBRIDGE_MANUAL_TELEOP ? 'On' : 'Off')} </Dropdown.Item>
                                <Dropdown.Item className={currentConfigBG}> [ Dark Mode ]: {localStorage.getItem('darkMode') !== null ? (localStorage.getItem('darkMode') === 'true' ? 'On' : 'Off') : (ros_config.DARK_MODE ? 'On' : 'Off')} </Dropdown.Item>
                            </Dropdown.Menu>
                        </Dropdown>
                    </div>

                    <div style={{display: "flex", justifyContent: "row", marginTop: "2%"}}>
                        <FormGroup style={{ marginRight: "5%" }}>
                            <Form.Check 
                                name="showBattery" 
                                type="checkbox"
                                checked={batteryStatus} 
                                onChange={(event) =>
                                    this.setState({
                                        batteryStatus: this.updateShowBattery(event.target.checked),
                                    })
                                }
                                label="Show Battery Status" />
                        </FormGroup>

                        <FormGroup style={{marginRight: "5%"}}>
                            <Form.Check
                                name="manualTeleop"
                                type="checkbox"
                                label="Manual Input Teleoperation"
                                checked={manualTeleop}
                                onChange={(event) =>
                                    this.setState({
                                        manualTeleop: this.updateManualTeleopState(event.target.checked),
                                    })
                                }
                            />
                        </FormGroup>

                        <FormGroup>
                            <Form.Check
                                name="darkMode"
                                type="checkbox"
                                label="Dark Mode"
                                checked={isDarkMode}
                                onChange={(event) =>
                                    this.setState({
                                        isDarkMode: this.updateDarkMode(event.target.checked),
                                    })
                                }
                            />
                        </FormGroup>
                    </div>
                    
                    <div className="divider"/>

                    <Button onClick={this.handleResetClick} variant="danger" style={{marginBottom: "2%"}}>
                        Reset to Default
                    </Button>
                </Form>
            </Container>
        );
    }
    
}

export default Settings;