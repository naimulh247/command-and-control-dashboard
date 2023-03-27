import React from 'react';
import ROSManager from './ROSManager.jsx';
import { Container } from 'reactstrap';

class App extends React.Component {
  render() {
    return(
      <Container>
        <ROSManager />
      </Container>
    )
  }
}

export default App;
