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

    updateManualTeleopState(newState) {
        localStorage.setItem('manualTeleop', newState);
        return newState;
    }

    updateShowBattery(newState) {
        localStorage.setItem('batteryStatus', newState);
        return newState;
    }

    toggleConfig(isOpen) {
        this.setState({ showConfig: isOpen });
    }

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

    handleFormSubmit = (event) => {
        event.preventDefault();
    };

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
            window.location.reload();
        }
        if (this.state.invalidIP === true || this.state.invalidPort === true || this.state.invalidWidth === true || this.state.invalidHeight === true || this.state.invalidFrameWidth === true || this.state.invalidFrameHeight === true) {
            alert("One or more input field values are invalid, please re-enter a valid input!")
        }
    };

    handleResetClick = () => {
        const storedIP = this.state.rosbridgeServerIP
        const storedPort = this.state.rosbridgeServerPort
        localStorage.clear();
        if (ros_config.ROSBRIDGE_SERVER_IP !== storedIP || ros_config.ROSBRIDGE_SERVER_PORT !== storedPort) {
            window.location.reload();
        }
    };

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
        const { ros, batteryStatus, manualTeleop, showConfig, invalidIP, invalidPort, invalidWidth, invalidHeight, invalidFrameWidth, invalidFrameHeight } = this.state;
        return (
            <Container style={{ margin: "2%" }}>
                <h1 id="-project-command-control-"><strong>Settings</strong></h1>

                <div className="divider"></div>

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

                    <div className="divider"></div>
    
                    <div style={{ display: 'flex', justifyContent: 'row', marginTop: "2%"}}>
                        <RosTopicList ros={ros}/>
                        <Dropdown onToggle={this.toggleConfig} show={showConfig} style={{marginLeft: "1.5%"}}>
                            <Dropdown.Toggle variant="info">
                            {showConfig
                                ? 'Hide Current Configuration'
                                : 'Show Current Configuration'}
                            </Dropdown.Toggle>
                            <Dropdown.Menu>
                                <Dropdown.Item> [ Rosbridge Server IP ] : {localStorage.getItem('rosbridgeServerIP') || ros_config.ROSBRIDGE_SERVER_IP} </Dropdown.Item>
                                <Dropdown.Item> [ Rosbridge Server Port ]: {localStorage.getItem('rosbridgeServerPort') || ros_config.ROSBRIDGE_SERVER_PORT} </Dropdown.Item>
                                <Dropdown.Item> [ Video Resolution Width & Height ]: {localStorage.getItem('imageWidth') || ros_config.ROSBRIDGE_IMAGE_WIDTH} x {localStorage.getItem('imageHeight') || ros_config.ROSBRIDGE_IMAGE_HEIGHT}</Dropdown.Item>
                                <Dropdown.Item> [ Video Frame Width & Height]: {localStorage.getItem('frameWidth') || ros_config.ROSBRIDGE_FRAME_WIDTH} x {localStorage.getItem('frameHeight') || ros_config.ROSBRIDGE_FRAME_HEIGHT}</Dropdown.Item>
                                <Dropdown.Item> [ Show Battery Status ]: {localStorage.getItem('batteryStatus') !== null ? (localStorage.getItem('batteryStatus') === 'true' ? 'On' : 'Off') : (ros_config.ROSBRIDGE_BATTERY_STATUS ? 'On' : 'Off')} </Dropdown.Item>
                                <Dropdown.Item> [ Manual Input Teleoperation ]: {localStorage.getItem('manualTeleop') !== null ? (localStorage.getItem('manualTeleop') === 'true' ? 'On' : 'Off') : (ros_config.ROSBRIDGE_MANUAL_TELEOP ? 'On' : 'Off')} </Dropdown.Item>
                            </Dropdown.Menu>
                        </Dropdown>
                    </div>
        
                    <FormGroup style={{ marginTop: "2%" }}>
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

                    <FormGroup style={{marginTop: "2%"}}>
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
                    
                    <div className="divider"></div>

                    <Button onClick={this.handleResetClick} variant="danger">
                        Reset to Default
                    </Button>
                </Form>
            </Container>
        );
    }
    
}

export default Settings;