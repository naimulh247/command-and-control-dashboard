import React, { Component } from 'react'
import {Navbar, Nav, Container} from 'react-bootstrap'
import ros_config from '../../configs/ros_config';

class Header extends Component {
    state = {
        isDarkMode: localStorage.getItem('darkMode') !== null ? localStorage.getItem('darkMode') === "true" : ros_config.DARK_MODE
    }

    render() { 
        const { isDarkMode } = this.state;
        const navbarClassName = isDarkMode ? 'navbar-dark-style' : 'navbar-style';

        return (
            <Container>
                <Navbar expand="lg" className={navbarClassName}>
                    <Navbar.Brand href="/" className="fw-bold navbar-text-style">Campus Rover Dashboard</Navbar.Brand>
                    <Navbar.Toggle aria-controls="basic-navbar-nav" />
                    <Navbar.Collapse id="basic-navbar-nav">
                        <Nav className="me-auto">
                        <Nav.Link href="/" className="navbar-text-style">Home</Nav.Link>
                        <Nav.Link href="/about" className="navbar-text-style">About</Nav.Link>
                        <Nav.Link href="/help" className="navbar-text-style">Help</Nav.Link>
                        <Nav.Link href="/settings" className="navbar-text-style">Settings</Nav.Link>
                        <Nav.Link href="https://github.com/campusrover/command-control" className="navbar-text-style">GitHub</Nav.Link>
                        </Nav>
                    </Navbar.Collapse>
                </Navbar>
            </Container>
        );
    }
}
 
export default Header;