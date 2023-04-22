import React, { Component } from 'react'
import {Navbar, Nav, Container} from 'react-bootstrap'

class Header extends Component {
    render() { 
        return (
            <Container>
                <Navbar expand="lg" className="navbar-style" >
                    <Navbar.Brand href="/" className="fw-bold navbar-text-style">Campus Rover Dashboard</Navbar.Brand>
                    <Navbar.Toggle aria-controls="basic-navbar-nav" />
                    <Navbar.Collapse id="basic-navbar-nav">
                        <Nav className="me-auto">
                        <Nav.Link href="/" className="navbar-text-style">Home</Nav.Link>
                        <Nav.Link href="/about" className="navbar-text-style">About</Nav.Link>
                        <Nav.Link href="/help" className="navbar-text-style">Help</Nav.Link>
                        <Nav.Link href="/settings" className="navbar-text-style">Settings</Nav.Link>
                        <Nav.Link href="https://github.com/campusrover/command-control" style={{ color: "#fff" }}>GitHub</Nav.Link>
                        </Nav>
                    </Navbar.Collapse>
                </Navbar>
            </Container>
        );
    }
}
 
export default Header;