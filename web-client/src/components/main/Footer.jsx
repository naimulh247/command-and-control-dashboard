import React, { Component } from 'react'
import { Container } from 'react-bootstrap';

class Footer extends Component {
    render() { 
        return (
            <Container className='text-center footer-style'>
                <p className='footer-p-style'>Designed by Naimul, James, Brandon &copy; 2023</p>
            </Container>
        );
    }
}
 
export default Footer;