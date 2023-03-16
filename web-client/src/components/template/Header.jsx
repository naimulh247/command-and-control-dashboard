import React, { Component } from 'react'
import { Navbar, Nav } from 'react-bootstrap';
class Header extends Component {
    state = {  } 
    render() { 
        return (

        <Navbar bg="light" expand="lg">
            <Navbar.Brand href="#home">Campus Rover Dashboard</Navbar.Brand>
            <Navbar.Toggle aria-controls="basic-navbar-nav" />
            <Navbar.Collapse id="basic-navbar-nav">
                <Nav className="me-auto">
                <Nav.Link href="/">Home</Nav.Link>
                <Nav.Link href="/about">About</Nav.Link>
                </Nav>
            </Navbar.Collapse>
        </Navbar>
        );
    }
}
 
export default Header;