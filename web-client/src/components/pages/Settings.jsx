import React, { Component } from 'react';
import { Container, Row, Col, Form, FormGroup, Button, Dropdown, DropdownButton} from 'react-bootstrap';
import ros_config from '../../configs/ros_config';
import ROSConnect from '../ros-bridge/ROSConnect';
import ManualTeleop from '../ros-bridge/ManualTeleop';
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
            imageQuality: localStorage.getItem('imageQuality') || ros_config.ROSBRIDGE_IMAGE_QUALITY,
            showBattery: false,
            batteryTopic: localStorage.getItem('batteryTopic') || ros_config.ROSBRIDGE_BATTERY_TOPIC,
            invalidIP: false,
            invalidPort: false,
            invalidWidth: false,
            invalidHeight: false,
            invalidQuality: false
        }

        this.setRos = this.setRos.bind(this);
    }

    setRos(ros) {
        this.setState({ ros });
    }

    handleInputChange = (event) => {
        const { name, value } = event.target;
      
        // Only allow numbers and periods for rosbridgeServerIP
        if (name === "rosbridgeServerIP" && !/^[\d.]+$/.test(value)) {
            this.setState({ invalidIP: true });
            return;
        }

        // Only allow numbers for rosbridgeServerPort, imageWidth, imageHeight, and imageQuality
        if (name === "rosbridgeServerPort" && !/^\d+$/.test(value)) {
            this.setState({ invalidPort: true });
        }
        else if (name === "imageWidth" && !/^\d+$/.test(value)) {
            this.setState({ invalidWidth: true });
        }
        else if (name === "imageHeight" && !/^\d+$/.test(value)) {
            this.setState({ invalidHeight: true });
        }
        else if (name === "imageQuality" && !/^\d+$/.test(value)) {
            this.setState({ invalidQuality: true });
        }
        else {
            this.setState({ [name]: value, invalidIP: false, invalidPort: false, invalidWidth: false, invalidHeight: false, invalidQuality: false });
        }
      
    };
      

    handleCheckboxChange = (event) => {
        const { checked } = event.target;
        this.setState({ showBattery: checked });
    };

    handleFormSubmit = (event) => {
        event.preventDefault();
    };

    handleSaveClick = () => {
        const storedIP = localStorage.getItem('rosbridgeServerIP');
        const storedPort = localStorage.getItem('rosbridgeServerPort');
        localStorage.setItem('rosbridgeServerIP', this.state.rosbridgeServerIP);
        localStorage.setItem('rosbridgeServerPort', this.state.rosbridgeServerPort);
        localStorage.setItem('imageWidth', this.state.imageWidth);
        localStorage.setItem('imageHeight', this.state.imageHeight);
        localStorage.setItem('imageQuality', this.state.imageQuality);
        localStorage.setItem('batteryTopic', this.state.batteryTopic);
        if (this.state.rosbridgeServerIP !== storedIP || this.state.rosbridgeServerPort !== storedPort) {
            window.location.reload();
        }
    };
      
    
    render() {
        const { ros, rosbridgeServerIP, rosbridgeServerPort, imageWidth, imageHeight, imageQuality, showBattery, batteryTopic } = this.state;
        return (
            <Container style={{ margin: "2%" }}>
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
                                <Form.Control name="rosbridgeServerIP" onChange={this.handleInputChange} placeholder="xxx.x.x.x" style={{width: "50%"}}/>
                                {this.state.invalidIP && (
                                    <span style={{ color: "red" }}>
                                    Invalid rosbridge server IP, please re-enter a number value!
                                    </span>
                                )}
                            </Col>
                            <Col>
                                <Form.Label>Port</Form.Label>
                                <Form.Control name="rosbridgeServerPort" onChange={this.handleInputChange} placeholder="Port" style={{width: "20%"}}/>
                                {this.state.invalidPort && (
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
                                <Form.Label>Image Width</Form.Label>
                                <Form.Control name="imageWidth" onChange={this.handleInputChange} placeholder="Width" style={{width: "20%"}}/>
                                {this.state.invalidWidth && (
                                    <span style={{ color: "red" }}>
                                    Invalid image width input, please re-enter a number value!
                                    </span>
                                )}
                            </Col>
                            <Col>
                                <Form.Label>Image Height</Form.Label>
                                <Form.Control name="imageHeight" onChange={this.handleInputChange} placeholder="Height" style={{width: "20%"}}/>
                                {this.state.invalidHeight && (
                                    <span style={{ color: "red" }}>
                                    Invalid image height input, please re-enter a number value!
                                    </span>
                                )}
                            </Col>
                        </Row>
                    </FormGroup>
    
                    <FormGroup style={{ marginTop: "2%" }}>
                        <Form.Label>Image Quality</Form.Label>
                        <Form.Control name="imageQuality" onChange={this.handleInputChange} placeholder="0-100" style={{width: "20%"}}/>
                        {this.state.invalidQuality && (
                            <span style={{ color: "red" }}>
                            Invalid image quality input, please re-enter a number value!
                            </span>
                        )}
                    </FormGroup>

                    <div style={{marginTop: "2%"}}>
                        <RosTopicList ros={ros}/>
                    </div>
    
                    <FormGroup style={{ marginTop: "1%" }}>
                        <Form.Check name="showBattery" checked={showBattery} onChange={this.handleCheckboxChange} label="Battery" />
                        {showBattery && (
                            <Form.Control name="batteryTopic" onChange={this.handleInputChange} placeholder="/battery" style={{width: "20%"}}/>
                        )}
                    </FormGroup>
    
                    <div style={{ display: 'flex', justifyContent: 'row', marginTop: "2%"}}>
                        <Button onClick={this.handleSaveClick} variant="primary">
                            Save
                        </Button>
                        <div style={{marginLeft: "2%"}}>
                            <Dropdown>
                                <Dropdown.Toggle variant="secondary">
                                    Show Current Configurations
                                </Dropdown.Toggle>
                                <Dropdown.Menu>
                                    <Dropdown.Item>[ ROSBRIDGE_SERVER_IP ] : {localStorage.getItem('rosbridgeServerIP').toString() || ros_config.ROSBRIDGE_SERVER_IP}</Dropdown.Item>
                                    <Dropdown.Item>[ ROSBRIDGE_SERVER_PORT ]: {localStorage.getItem('rosbridgeServerPort').toString() || ros_config.ROSBRIDGE_SERVER_Port}</Dropdown.Item>
                                    <Dropdown.Item>[ ROSBRIDGE_IMAGE_WIDTH ]: {localStorage.getItem('imageWidth').toString() || ros_config.ROSBRIDGE_IMAGE_WIDTH}</Dropdown.Item>
                                    <Dropdown.Item>[ ROSBRIDGE_IMAGE_HEIGHT ] : {localStorage.getItem('imageHeight').toString() || ros_config.ROSBRIDGE_IMAGE_HEIGHT}</Dropdown.Item>
                                    <Dropdown.Item>[ ROSBRIDGE_IMAGE_QUALITY ]: {localStorage.getItem('imageQuality').toString() || ros_config.ROSBRIDGE_IMAGE_QUALITY}</Dropdown.Item>
                                    <Dropdown.Item>[ ROSBRIDGE_SHOW_BATTERY ]: {showBattery.toString()}</Dropdown.Item>
                                    <Dropdown.Item>[ ROSBRIDGE_BATTERY_TOPIC ]: {localStorage.getItem('batteryTopic').toString() || ros_config.ROSBRIDGE_BATTERY_TOPIC}</Dropdown.Item>
                                </Dropdown.Menu>
                            </Dropdown>
                        </div>
                    </div>
                </Form>
            </Container>
        );
    }
    
}

export default Settings;