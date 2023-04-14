import React, { Component } from 'react';
import { Container, Row, Col, Form, FormGroup, Button} from 'react-bootstrap';
import ros_config from '../../configs/ros_config';

class Settings extends Component {
  state = {
    ipAddress: ros_config.ROSBRIDGE_SERVER_IP || '',
    port: ros_config.ROSBRIDGE_SERVER_PORT || '',
    imageWidth: ros_config.ROSBRIDGE_IMAGE_WIDTH || '',
    imageHeight: ros_config.ROSBRIDGE_IMAGE_HEIGHT || '',
    imageQuality: ros_config.ROSBRIDGE_IMAGE_QUALITY || '',
    showBattery: false,
    batteryTopic: ros_config.ROSBRIDGE_BATTERY_TOPIC || '',
  };

  handleInputChange = (event) => {
    const { name, value } = event.target;
    this.setState({ [name]: value });
  };

  handleCheckboxChange = (event) => {
    const { checked } = event.target;
    this.setState({ showBattery: checked });
  };

  handleSaveClick = () => {
    // Update ros_config with the new values entered by the user, this doesn't currently work though since it only
    //updates the value in its run-time environment
    ros_config.ROSBRIDGE_SERVER_IP = this.state.ipAddress;
    ros_config.ROSBRIDGE_SERVER_PORT = this.state.port;
    ros_config.ROSBRIDGE_IMAGE_WIDTH = this.state.imageWidth;
    ros_config.ROSBRIDGE_IMAGE_HEIGHT = this.state.imageHeight;
    ros_config.ROSBRIDGE_IMAGE_QUALITY = this.state.imageQuality;
    ros_config.ROSBRIDGE_BATTERY_TOPIC = this.state.batteryTopic;
    console.log("SAVED: " + this.state.port);
    console.log("ROSCONFIG_SERVER_PORT: " + ros_config.ROSBRIDGE_SERVER_PORT);
  };

  render() {
    const { ipAddress, port, imageWidth, imageHeight, imageQuality, showBattery, batteryTopic } = this.state;

    return (
      <Container style={{ margin: "5%" }}>
        <Form>
          <FormGroup>
            <Row>
              <Col>
                <Form.Label>Rosbridge Server IP Address</Form.Label>
                <Form.Control name="ipAddress" value={ipAddress || ros_config.ROSBRIDGE_SERVER_IP || ""} onChange={this.handleInputChange} placeholder="xxx.x.x.x" style={{width: "50%"}}/>
              </Col>
              <Col>
                <Form.Label>Port</Form.Label>
                <Form.Control name="port" value={port || ros_config.ROSBRIDGE_SERVER_PORT || ""} onChange={this.handleInputChange} placeholder="Port" style={{width: "20%"}}/>
              </Col>
            </Row>
          </FormGroup>

          <FormGroup style={{ marginTop: "2%" }}>
            <Row>
              <Col>
                <Form.Label>Image Width</Form.Label>
                <Form.Control name="imageWidth" value={imageWidth || ros_config.IMAGE_WIDTH || ""} onChange={this.handleInputChange} placeholder="Width" style={{width: "20%"}}/>
              </Col>
              <Col>
                <Form.Label>Image Height</Form.Label>
                <Form.Control name="imageHeight" value={imageHeight || ros_config.IMAGE_HEIGHT || ""} onChange={this.handleInputChange} placeholder="Height" style={{width: "20%"}}/>
              </Col>
            </Row>
          </FormGroup>

          <FormGroup style={{ marginTop: "2%" }}>
            <Form.Label>Image Quality</Form.Label>
            <Form.Control name="imageQuality" value={imageQuality || ros_config.IMAGE_QUALITY || ""} onChange={this.handleInputChange} placeholder="0-100" style={{width: "20%"}}/>
          </FormGroup>

          <FormGroup style={{ marginTop: "2%" }}>
            <Form.Check name="showBattery" checked={showBattery} onChange={this.handleCheckboxChange} label="Battery" />
            {showBattery && (
              <Form.Control name="batteryTopic" value={batteryTopic || ros_config.BATTERY_TOPIC || ""} onChange={this.handleInputChange} placeholder="/battery" style={{width: "20%"}}/>
            )}
          </FormGroup>

          <div style={{ marginTop: '2%' }}>
            <Button onClick={this.handleSaveClick} variant="primary">
                Save
            </Button>
           </div>
        </Form>
    </Container>
    );
  }
}

export default Settings;