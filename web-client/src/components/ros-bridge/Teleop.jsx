import React, { Component } from 'react';
import { Container } from 'react-bootstrap';
import { Joystick } from 'react-joystick-component'
class Teleop extends Component {
    state = {}

    handleMove() {}

    handleStop() {}
    render() {
        return (
            <Container>
                <Joystick 
                size={100} 
                baseColor="#EEEEEE" 
                stickColor="#BBBBBB" 
                move={this.handleMove} 
                stop={this.handleStop}
                ></Joystick>
            </Container>
        );
    }
}

export default Teleop;