import React, { useState, useEffect } from 'react';
import Map, { Marker } from 'react-map-gl';
import ros_config from '../../configs/ros_config';

function Map2({ros}) {
    const [viewport, setviewport] = useState({
        // initialize in the Robotics Lab
        latitude: 42.3660,
        longitude: -71.2587,
        // set the initial map zoom level
        zoom: 17
    });

    const [isDarkMode, setIsDarkMode] = useState(
      localStorage.getItem('darkMode') !== null ? localStorage.getItem('darkMode') === 'true' : ros_config.DARK_MODE
    );

    const [latitude, setLatitude] = useState(0);
    const [longitude, setLongitude] = useState(0);

    useEffect(() => {
        if (!ros) return;
        
        // subscribe to the gps data being published from the app through ros
        const gpsSubscriber = new window.ROSLIB.Topic({
          ros: ros,
          name: '/gps',
          messageType: 'std_msgs/String',
          queue_size: 1,
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

    const darkMapStatus = isDarkMode ? 'mapbox://styles/mapbox/navigation-night-v1' : 'mapbox://styles/mapbox/streets-v11';

    return (

        <Map
            {...viewport}
            // width="100%"
            // height="100%"
            mapStyle={darkMapStatus}
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
