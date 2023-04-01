import React from 'react';
import ROSConnect from './ROSConnect.jsx';
import RobotVitals from './RobotVitals.jsx';
import TeleopController from './TeleopController.jsx';
import { Row, Col } from 'reactstrap'; 
import ROSLIB from 'roslib';

class ROSManager extends React.Component {

  state = {
    ros: null,
    connected: false,
    laptopCharge: 0,
    robotCharge: 0,
    angularVelocity: 0,
    linearVelocity: 0,
    ip: null,
    waypoints: null,
  }
  
  configureRos = (ip, port) => {
    
    var ros = new ROSLIB.Ros({
      url : 'ws://' + ip + ':' + port
    });

    ros.on('connection', () => {
      console.log('Connected to websocket server.');
      this.setState({
        connected: true,
        ros: ros,
        ip: ip,
      });
      setInterval(this.publishVelocityData, 100);
      this.subscribeLaptopBatteryData();
      this.fetchWaypoints();
    });

    ros.on('error', (error) => {
      console.log('Error connecting to websocket server: ', error);
      this.setState({
        connected: false,
        ros: null,
        ip: null,
      });
    });

    ros.on('close', () => {
      console.log('Connection to websocket server closed.');
      this.setState({
        connected: false,
        ros: null,
        ip: null,
      });
    });
  }

  // ===========================
  // TELEOP VELOCITY MANAGERMENT
  // ===========================

  handleLinearVelocityChange = (change) => {
    var linearVelocity = this.state.linearVelocity;
    linearVelocity += change;
    linearVelocity = Math.round(linearVelocity * 10) / 10
    this.setState({linearVelocity: linearVelocity});
  }

  handleAngularVelocityChange = (change) => {
    var angularVelocity = this.state.angularVelocity;
    angularVelocity += change;
    angularVelocity = Math.round(angularVelocity * 10) / 10
    this.setState({angularVelocity: angularVelocity});
  }

  resetVelocities = () => {
    this.setState({
      angularVelocity: 0,
      linearVelocity: 0
    })
  }

  publishVelocityData = () => {
    var ros = this.state.ros
    if (!ros || (this.state.linearVelocity === 0 && this.state.angularVelocity === 0)) { return }

    var cmdVel = new ROSLIB.Topic({
      ros : ros,
      name : 'cmd_vel_mux/input/teleop',
      messageType : 'geometry_msgs/Twist'
    });

    var twist = new ROSLIB.Message({
      linear : {
        x : this.state.linearVelocity,
        y : 0,
        z : 0
      },
      angular : {
        x : 0,
        y : 0,
        z : this.state.angularVelocity
      }
    });
    cmdVel.publish(twist);
  }

  subscribeLaptopBatteryData = () => {
    var ros = this.state.ros
    if (!ros) { return }

    var listener = new ROSLIB.Topic({
      ros : ros,
      name : '/laptop_charge',
      messageType : 'smart_battery_msgs/SmartBatteryStatus'
    });

    listener.subscribe((message) => {
      this.setState({laptopCharge: message.percentage});
    });
  }

  fetchWaypoints = () => {
    var waypointsClient = new ROSLIB.Service({
      ros : this.state.ros,
      name : '/waypoints',
      serviceType : 'campus_rover/Waypoints'
    });
  
    var request = new ROSLIB.ServiceRequest({});
  
    waypointsClient.callService(request, (result) => {
      const waypoints = result.waypoints.waypoints
      this.setState({
        waypoints
      })
    });  
  }

  deployToWaypoint = (navigationGoalPose) => {

    var moveBaseGoal = new ROSLIB.Topic({
      ros: this.state.ros,
      name: '/move_base_simple/goal',
      messageType: 'geometry_msgs/PoseStamped '
    });

    var poseStamped = {
      header: {
        stamp: Date.now(),
        frame_id: "map"
      },
      pose: navigationGoalPose
    };
    console.log(poseStamped)
    moveBaseGoal.publish(poseStamped)
  }

  disconnect = () => {
    if (this.state.ros) {
      this.state.ros.close();
    }
  }

  render() {
    return(
      <div>
        <Row className='justify-content-center'>
          <Col
            xs='12'
          >
            <TeleopController 
              onLinearVelocityChange={this.handleLinearVelocityChange}
              onAngularVelocityChange={this.handleAngularVelocityChange}
              onStopButtonPress={this.resetVelocities}
              linearVelocity={this.state.linearVelocity}
              angularVelocity={this.state.angularVelocity}
              ip={this.state.ip}
              waypoints={this.state.waypoints}
              onDeploy={this.deployToWaypoint}
            />
          </Col>
          <Col
            xs='12'
          >
            <ROSConnect
              ros={this.state.ros}
              onConnect={this.configureRos}
              onDisconnect={this.disconnect}
              connected={this.state.connected}
            />
          </Col>
          <Col
            xs='6'
          >
            <RobotVitals
              laptopCharge={this.state.laptopCharge}
            />
          </Col>
        </Row>
      </div>
    )
  }
}

export default ROSManager;
