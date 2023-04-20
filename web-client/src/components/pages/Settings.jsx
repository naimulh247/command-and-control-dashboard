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
        }

        this.setRos = this.setRos.bind(this);
    }

    setRos(ros) {
        this.setState({ ros });
    }

    handleInputChange = (event) => {
        const { name, value } = event.target;
        this.setState({ [name]: value });
    };

    handleCheckboxChange = (event) => {
        const { checked } = event.target;
        this.setState({ showBattery: checked });
    };

    handleFormSubmit = (event) => {
        event.preventDefault();
    };

    handleSaveClick = () => {
        // Update local storage with the new values entered by the user
        localStorage.setItem('rosbridgeServerIP', this.state.rosbridgeServerIP);
        localStorage.setItem('rosbridgeServerPort', this.state.rosbridgeServerPort);
        localStorage.setItem('imageWidth', this.state.imageWidth);
        localStorage.setItem('imageHeight', this.state.imageHeight);
        localStorage.setItem('imageQuality', this.state.imageQuality);
        localStorage.setItem('batteryTopic', this.state.batteryTopic);
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
                            </Col>
                            <Col>
                                <Form.Label>Port</Form.Label>
                                <Form.Control name="rosbridgeServerPort" onChange={this.handleInputChange} placeholder="Port" style={{width: "20%"}}/>
                            </Col>
                        </Row>
                    </FormGroup>
    
                    <FormGroup style={{ marginTop: "2%" }}>
                        <Row>
                            <Col>
                                <Form.Label>Image Width</Form.Label>
                                <Form.Control name="imageWidth" onChange={this.handleInputChange} placeholder="Width" style={{width: "20%"}}/>
                            </Col>
                            <Col>
                                <Form.Label>Image Height</Form.Label>
                                <Form.Control name="imageHeight" onChange={this.handleInputChange} placeholder="Height" style={{width: "20%"}}/>
                            </Col>
                        </Row>
                    </FormGroup>
    
                    <FormGroup style={{ marginTop: "2%" }}>
                        <Form.Label>Image Quality</Form.Label>
                        <Form.Control name="imageQuality" onChange={this.handleInputChange} placeholder="0-100" style={{width: "20%"}}/>
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
                                    <Dropdown.Item>[ ROSBRIDGE_SERVER_IP ] : {rosbridgeServerIP.toString()}</Dropdown.Item>
                                    <Dropdown.Item>[ ROSBRIDGE_SERVER_PORT ]: {rosbridgeServerPort.toString()}</Dropdown.Item>
                                    <Dropdown.Item>[ ROSBRIDGE_IMAGE_WIDTH ]: {imageWidth.toString()}</Dropdown.Item>
                                    <Dropdown.Item>[ ROSBRIDGE_IMAGE_HEIGHT ] : {imageHeight.toString()}</Dropdown.Item>
                                    <Dropdown.Item>[ ROSBRIDGE_IMAGE_QUALITY ]: {imageQuality.toString()}</Dropdown.Item>
                                    <Dropdown.Item>[ ROSBRIDGE_SHOW_BATTERY ]: {showBattery.toString()}</Dropdown.Item>
                                    <Dropdown.Item>[ ROSBRIDGE_BATTERY_TOPIC ]: {batteryTopic.toString()}</Dropdown.Item>
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