import React, { Component } from 'react';
import ros_config from '../../configs/ros_config'

const Container = (props) => <div className="Container" {...props} />;
const Section = (props) => <div className="Section" {...props} />;
const Label = (props) => <div className="Label" {...props} />;
const InputContainer = (props) => <div className="InputContainer" {...props} />;
const IPInput = (props) => <input className="IPInput" {...props} />;
const PortInput = (props) => <input className="PortInput" {...props} />;
const Checkbox = (props) => <input type="checkbox" className="Checkbox" {...props} />;
const LabelCheckbox = (props) => <div className="LabelCheckbox" {...props} />;

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
    console.log("SAVED: " + this.state.port)
    console.log("ROSCONFIG_SERVER_PORT: " + ros_config.ROSBRIDGE_SERVER_PORT);
  };

  render() {
    const { ipAddress, port, imageWidth, imageHeight, imageQuality, showBattery, batteryTopic } = this.state;

    return (
        <Container style={{margin: "5%"}}>
        <Section>
          <Label>Rosbridge Server IP Address</Label>
          <InputContainer>
            <IPInput name="ipAddress" value={ipAddress || ros_config.ROSBRIDGE_SERVER_IP || ""} onChange={this.handleInputChange} placeholder="xxx.x.x.x" style={{marginRight: "0.5%"}}/>
            <span>:</span>
            <PortInput name="port" value={port || ros_config.ROSBRIDGE_SERVER_PORT || ""} onChange={this.handleInputChange} placeholder="Port" style={{marginLeft: "0.5%"}}/>
          </InputContainer>
        </Section>
      
        <Section style={{marginTop: "2%"}}>
          <Label>Image Width</Label>
          <InputContainer>
            <input name="imageWidth" value={imageWidth || ros_config.IMAGE_WIDTH || ""} onChange={this.handleInputChange} placeholder="Width" style={{marginRight: "0.5%"}}/>
            <span>:</span>
            <input name="imageHeight" value={imageHeight || ros_config.IMAGE_HEIGHT || ""} onChange={this.handleInputChange} placeholder="Height" style={{marginLeft: "0.5%"}}/>
          </InputContainer>
        </Section>
      
        <Section style={{marginTop: "2%"}}>
          <Label>Image Quality</Label>
          <InputContainer>
            <input name="imageQuality" value={imageQuality || ros_config.IMAGE_QUALITY || ""} onChange={this.handleInputChange} placeholder="0-100"/>
          </InputContainer>
        </Section>
      
        <Section style={{marginTop: "2%"}}>
          <LabelCheckbox>
            <Checkbox checked={showBattery} onChange={this.handleCheckboxChange} />
            Battery
          </LabelCheckbox>
          {showBattery && (
            <InputContainer>
              <input name="batteryTopic" value={batteryTopic || ros_config.BATTERY_TOPIC || ""} onChange={this.handleInputChange} placeholder="/battery" />
            </InputContainer>
          )}
        </Section>
      
        <Section style={{marginTop: "2%"}}>
          <button onClick={this.handleSaveClick}>Save</button>
        </Section>
      </Container>      
    );
  }
}

export default Settings;
