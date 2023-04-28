import React, { Component } from 'react';

class Help extends Component {
  render() {
    return (
      <div style={{ margin: "3%" }}>
        <h1>Help</h1>

        <div className="divider"></div>

        <h2>Installation & Running</h2>
        <p>To install and run the web client, follow these steps:</p>
        <ol>
          <li>
            Clone the GitHub repository containing the web client code.
          </li>
          <li>
            Install the necessary dependencies by running the following command in the terminal:
            <code style={{marginLeft: "1%"}}>npm install</code>
          </li>
          <li>
            Start the web client by running the following command in the terminal:
            <code style={{marginLeft: "1%"}}>npm start</code>
          </li>
          <li>
            Open a web browser and enter the URL: <code style={{marginLeft: "1%"}}>http://localhost:3000</code>.
          </li>
          <li>
            Configure the web client settings by navigating to the settings page. The available settings and their explanations are listed below.
          </li>
        </ol>

        <h2>Web Client Settings</h2>
        <p>The web client provides various settings to customize its behavior. Here are the available settings and their explanations:</p>
        <ul>
            <li><strong>Rosbridge Server IP Address:</strong> Enter the IP address of the Rosbridge server. This is the server that connects the web application to the ROS environment.</li>
            <li><strong>Port:</strong> Specify the port number for the Rosbridge server.</li>
            <li><strong>Video Resolution Width:</strong> Set the width of the video resolution for the ROS camera feed.</li>
            <li><strong>Video Resolution Height:</strong> Set the height of the video resolution for the ROS camera feed.</li>
            <li><strong>Video Frame Width:</strong> Specify the width of the video frame for the ROS camera feed.</li>
            <li><strong>Video Frame Height:</strong> Specify the height of the video frame for the ROS camera feed.</li>
            <li><strong>Show Battery Status:</strong> Toggle to display the battery status in the application.</li>
            <li><strong>Manual Input Teleoperation:</strong> Enable or disable manual teleoperation control input (default is using the joystick).</li>
        </ul>

        <h2>Running with a ROS Robot</h2>
        <p>If you want to run the web client with a ROS robot, make sure to follow these additional steps:</p>
        <p>(P.S. this is assuming you already have ROS installed on your system, if not you can follow this <a href="http://wiki.ros.org/ROS/Installation">link</a> for instructions. Please note that for our testing purposes, we used the noetic version of ROS):</p>
        <ol>
          <li>
            Launch the necessary ROS launch file for real robot testing:
            <code style={{marginLeft: "1%"}}>roslaunch ros-robot real.launch</code>
          </li>
          <li>
            Launch the necessary ROS launch file for simulation robot testing:
            <code style={{marginLeft: "1%"}}>roslaunch ros-robot sim.launch</code>
          </li>
          <li>
            Ensure that the ROSbridge server is running on the specified IP address and port within the launch file parameters.
          </li>
          <li>
            SSH into your robot (for our intents and purposes, we utilized a TurtleBot3 Burger model)
          </li>
          <li>
            If you want the camera to show up, make sure that the correct camera topic is being published properly and that the VideoFeed component has the correct topic listed within the subscriber as sometimes it can be /raspicam or /camera, there should be config values for both already
          </li>
          <li>
            Enjoy using the web client to control and monitor your ROS robot!
          </li>
        </ol>
      </div>
    );
  }
}

export default Help;
