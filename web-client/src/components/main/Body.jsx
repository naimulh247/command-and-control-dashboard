import React, { Component } from 'react'
import { Container } from 'react-bootstrap';
import {Route, BrowserRouter as Router, Routes} from 'react-router-dom'
import Home from '../pages/Home';
import About from '../pages/About';
import Help from '../pages/Help';
import Settings from '../pages/Settings';

class Body extends Component {
    state = {  } 
    render() { 
        return (
            <Container>
            <Router>
                <Routes>
                    <Route path='/' Component={Home} />
                    <Route path='/about' Component={About} />
                    <Route path='/help' Component={Help} />
                    <Route path='/settings' Component={Settings} />
                </Routes>
            </Router>
            </Container>
        );
    }
}
 
export default Body;