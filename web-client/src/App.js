import './App.css';
import { Container } from 'react-bootstrap';
import Body from './components/template/Body';
import Footer from './components/template/Footer';
import Header from './components/template/Header';
import ROSManager from './components/ROSManager';

function App() {
  return (
    <Container>
      <Header/>
      <ROSManager/>
      <Footer/>
    </Container>
  );
}

export default App;
