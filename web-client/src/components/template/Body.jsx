import React, { Component } from 'react'
import {Route, BrowserRouter as Router, Routes} from 'react-router-dom'
import Home from '../pages/Home'
import About from '../pages/About'

class Body extends Component {
    state = {  } 
    render() { 
        return (
            <Router>
                <Routes>
                    <Route path='/' Component={Home}/>
                    <Route path='/about' Component={About}/>
                </Routes>
            </Router>
        );
    }
}
 
export default Body;