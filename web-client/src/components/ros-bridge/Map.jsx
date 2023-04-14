import React, { Component } from 'react';
import MapGL from 'react-map-gl';

class Map extends Component {
  constructor(props) {
    super(props);

    this.state = {
      viewport: {
        longitude: -100,
        latitude: 40,
        zoom: 10,
        width: 600,
        height: 400
      }
    };
  }

  render() {
    const { viewport } = this.state;

    return (
      <MapGL
        {...viewport}
        mapStyle="mapbox://styles/mapbox/streets-v9"
        mapboxApiAccessToken='pk.eyJ1IjoibmFpbXVsaCIsImEiOiJjbGdnMTR4MnUwNzBoM2RydjFveHQ0emVpIn0.Jeb-L3dsxMkt6v6HTI8QyQ'
        
      />
    );
  }
}

export default Map;
