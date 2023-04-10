import React, { Component } from 'react';
import { Col, Container, Row } from 'react-bootstrap';
import ros_config from '../../configs/ros_config';

class ManualTeleop extends Component {
    constructor(props) {
        super(props)
    }

    publishTwist() {
        const { ros } = this.props
        // if ros is not intialized return
        if (!ros) {
            console.warn("ROS/ RosBridge not intialized")
            return
        }
        const linear = document.getElementsByName("linear")
        const angular = document.getElementsByName("angular")

        //validate input to check for non number values
        for (let i=0; i < linear.length; i++) {
            const linearValue = linear[i].value.trim();
            const angularValue = angular[i].value.trim();
        
            if (!/^[+-]?\d+(\.\d+)?$/.test(linearValue) || !/^[+-]?\d+(\.\d+)?$/.test(angularValue)) {
                alert("Please enter numbers for twist values!");
                return;
            }
        }
        
        const cmd_vel = new window.ROSLIB.Topic({
            ros: ros,
            name: `${ros_config.ROSBRIDGE_CMD_VEL}`,
            messageType: 'geometry_msgs/Twist',
        });

        //create twist message based off input results
        const twist = new window.ROSLIB.Message({
            linear: {
                x: Number(linear[0].value),
                y: Number(linear[1].value),
                z: Number(linear[2].value),
            },
            angular: {
                x: Number(angular[0].value),
                y: Number(angular[1].value),
                z: Number(angular[2].value),
            },
        });

        cmd_vel.publish(twist)
    }

    //stop function for stop button to publish twist message with 0 values for both linear and angular to stop robot
    stop() {
        const { ros } = this.props
        // is ros is not intialized return
        if (!ros) {
            console.warn("ROS/ RosBridge not intialized")
            return
        }

        const cmd_vel = new window.ROSLIB.Topic({
            ros: ros,
            name: `${ros_config.ROSBRIDGE_CMD_VEL}`,
            messageType: 'geometry_msgs/Twist',
        });

        const twist = new window.ROSLIB.Message({
            linear: {
                x: 0,
                y: 0,
                z: 0,
            },
            angular: {
                x: 0,
                y: 0,
                z: 0,
            },
        });

        cmd_vel.publish(twist)
    }

    //clear function for clear button to clear input text
    clear() {
       document.getElementById("linear_x").value = "";
       document.getElementById("linear_y").value = "";
       document.getElementById("linear_z").value = "";

       document.getElementById("angular_x").value = "";
       document.getElementById("angular_y").value = "";
       document.getElementById("angular_z").value = "";
    }

    render() {
        return (
            <Container className="bgStyle" style={{textAlign: "center"}}>
                <Row>
                    <h4 className='mt-4 title'>Manual Twist Messages</h4>
                    <Col>
                        <h4 className='mt-4'>Linear Velocity</h4>

                        <label>x: </label>
                        <input type="number" className="input" name="linear" id="linear_x"/>
                        <br/>

                        <label>y: </label>
                        <input type="number" className="input" name="linear" id="linear_y"/>
                        <br/>

                        <label>z: </label>
                        <input type="number" className="input" name="linear" id="linear_z"/>
                    </Col>
                    
                    <Col>
                        <h4 className='mt-4'>Angular Velocity</h4>

                        <label>x: </label>
                        <input type="number" className="input" name="angular" id="angular_x"/>
                        <br/>

                        <label>y: </label>
                        <input type="number" className="input" name="angular" id="angular_y"/>
                        <br/>

                        <label>z: </label>
                        <input type="number" className="input" name="angular" id="angular_z"/>
                    </Col>
                </Row>
                <Row>
                    <div className="buttonDiv">
                        <button className="button-3" onClick={this.publishTwist.bind(this)}>Publish</button>
                        <button className="button-3" style={{backgroundColor: "red", marginLeft: "5%"}} onClick={this.stop.bind(this)}>Stop</button>
                        <button className="button-3" style={{backgroundColor: "gray", marginLeft: "5%"}} onClick={this.clear.bind(this)}>Clear</button>
                    </div>
                </Row>
            </Container>
        );
    }
}

export default ManualTeleop;