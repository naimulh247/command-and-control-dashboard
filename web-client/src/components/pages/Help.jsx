import React, { Component } from 'react'

class About extends Component {
    state = {  } 
    render() { 
        return (
            <div className="title">
                <h1 id="-project-command-control-"><strong>Installation & Running</strong></h1>
                <p>Steps </p>
                <ul>
                    <li>Enter the following command into the VNC --{'>'} roslaunch ros-robot ros-robot.launch</li>
                    <li>SSH to TurtleBot in VNC or run a robot in Gazebo simulation (if you want the camera to show up, make sure you run a camera node), an example you can use is roslaunch prrexamples linemission.launch model:=waffle</li>
                    <li>Enjoy the website</li>
                </ul>
            </div>
        );
    }
}
 
export default About;