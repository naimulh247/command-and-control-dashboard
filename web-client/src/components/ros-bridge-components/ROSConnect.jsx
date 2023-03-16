import React, { Component } from 'react'
import { Alert } from 'react-bootstrap';

class ROSConnect extends Component {

    // state = { connected: false, ros: null}
    // state = {}
    // initialize a constructor so it can call ros on page load
    constructor(){
        super();
        this.state = { connected: false, ros: null}
        
        // this.init_connection();
    }

    componentDidMount(){
        this.init_connection();
    }
    
    init_connection(){
        
        this.state.ros = new window.ROSLIB.Ros();
        console.log(this.state.ros);

        this.state.ros.on("connection", () => {
            console.log("connected to robot");
            this.setState({connected: true});
        });

        this.state.ros.on("close", () => {
            console.log("connection closed");
            this.setState({connected: false});
            // try to reconnect every 3 second
            setTimeout(() => {
                try{

                    this.state.ros.connect('ws://0.0.0.0:9090')
                }
                catch (error) {
                    console.log("connection error:", error)
                }
            }, 3000);
        });
        
        try{

            this.state.ros.connect('ws://0.0.0.0:9090')
        }
        catch (error) {
            console.log("connection error:", error)
        }
        
    }

    render() { 
        return (

            <div>
                <Alert className='text-center m-3' variant={this.state.connected ? "success" : "danger"}>
                    {this.state.connected ? "Connected" : "Disconnected"}
                </Alert>
            </div>


        );
    }
}
 
export default ROSConnect;