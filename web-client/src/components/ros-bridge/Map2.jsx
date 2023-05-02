import React, { useState, useEffect } from 'react';
import Map, { Marker } from 'react-map-gl';


function Map2({ros}) {
    const [viewport, setviewport] = useState({
        // initialize in the Robotics Lab
        latitude: 42.3660,
        longitude: -71.2587,
        // set the initial map zoom level
        zoom: 17
    });

    useEffect(() => {
        if (!ros) return;
        
        // subscribe to the gps data being published from the app through ros
        const gpsSubscriber = new window.ROSLIB.Topic({
          ros: ros,
          name: '/gps',
          messageType: 'std_msgs/String'
        });
    
        gpsSubscriber.subscribe(message => {
            // console.log(message)
            const data = JSON.parse(message.data);
            // console.log(data.latitude, data.longitude, viewport.latitude)
            // update the gps data
            setviewport({
                longitude: data.longitude,
                latitude: data.latitude
            })
        });
    
      }, [ros]);

    return (

        <Map
            {...viewport}
            mapStyle="mapbox://styles/mapbox/streets-v11"
            mapboxAccessToken='pk.eyJ1IjoibmFpbXVsaCIsImEiOiJjbGdnMTR4MnUwNzBoM2RydjFveHQ0emVpIn0.Jeb-L3dsxMkt6v6HTI8QyQ'
        >
            {/* place a marker on the map */}
            <Marker longitude={viewport.longitude} latitude={viewport.latitude}>
                {/* <div style={{color: 'red', fontSize: 24}}>📍</div> */}
                <div style={{ color: 'red', fontSize: 24 }}>🤖</div>
            </Marker>
        </Map>
    )
}

export default Map2
