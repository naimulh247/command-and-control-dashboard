import React, { Component } from 'react';
import ROSConnect from '../ros-bridge/ROSConnect';

class Home extends Component {
    state = {  } 
    render() { 
        return (
            <main>
                <ROSConnect/>
            </main>
        );
    }
}
 
export default Home;