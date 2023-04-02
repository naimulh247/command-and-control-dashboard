import React, { Component } from 'react';
import { Alert } from 'react-bootstrap';

class ROSConnect extends Component {

    constructor() {
        super()
        this.state = { connected: false, ros: null } 
        
    }

    componentDidMount() {
        this.init_connection()
    }
    init_connection() {
        this.state.ros = new window.ROSLIB.Ros()
        this.state.ros.on("connection", () => {
            console.log("robot connected")
            this.setState({connected: true})
        })

        this.state.ros.on("close", () => {
            console.log("robot disconnected");
            this.setState({connected: false})
            // try to reconnect to rosbridge every 3 seconds
            setTimeout(() => {
                try{
                    // change the ip address to local storage
                    this.state.ros.connect('ws://127.0.0.1:9090')
                }catch (error) {
                    console.log("connection error:", error);
                }
            }, 3000); 
        })

        try{
            // change the ip address to local storage
            this.state.ros.connect('ws://127.0.0.1:9090')
        }catch (error) {
            console.log("connection error:", error);
        }

        console.log(this.state.ros);

    }

    render() { 
        return (
            <div>
                <Alert className='text-center m-3' variant={this.state.connected ? "success" : "danger"}>
                    {this.state.connected ? "Robot Connected" : "Robot Disconnected"}
                </Alert>
            </div>
        );
    }
}
 
export default ROSConnect;