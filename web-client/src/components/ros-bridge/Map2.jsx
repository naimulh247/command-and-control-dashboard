import React, { useState, useEffect } from 'react';
import Map, { Marker } from 'react-map-gl';


function Map2({ros}) {
    const [viewport, setviewport] = useState({
        latitude: 42.366251,
        longitude: -71.258724,
        zoom: 13
    });

    const [latitude, setLatitude] = useState(0);
    const [longitude, setLongitude] = useState(0);

    useEffect(() => {
        if (!ros) return;
    
        const gpsSubscriber = new window.ROSLIB.Topic({
          ros: ros,
          name: '/gps',
          messageType: 'std_msgs/String'
        });
    
        gpsSubscriber.subscribe(message => {
          const data = JSON.parse(message.data);
          setLatitude(data.latitude);
          setLongitude(data.longitude);
        });
    
        return () => {
          gpsSubscriber.unsubscribe();
        };
      }, [ros]);

    return (
        //   <Map
        //     {...viewport}
        //     style={{width: 600, height: 400}}
        //     mapStyle="mapbox://styles/mapbox/streets-v9"
        //     mapboxAccessToken='pk.eyJ1IjoibmFpbXVsaCIsImEiOiJjbGdnMTR4MnUwNzBoM2RydjFveHQ0emVpIn0.Jeb-L3dsxMkt6v6HTI8QyQ'
        //   />;
        <Map
            // initialViewState={{
            // longitude: -100,
            // latitude: 40,
            // zoom: 3.5
            // }}

            // initialViewState={viewport}
            // style={{ width: 600, height: 400 }}
            // mapStyle="mapbox://styles/mapbox/streets-v9"
            // mapboxAccessToken='pk.eyJ1IjoibmFpbXVsaCIsImEiOiJjbGdnMTR4MnUwNzBoM2RydjFveHQ0emVpIn0.Jeb-L3dsxMkt6v6HTI8QyQ'
        
            {...viewport}
            // width="100%"
            // height="100%"
            mapStyle="mapbox://styles/mapbox/streets-v11"
            mapboxAccessToken='pk.eyJ1IjoibmFpbXVsaCIsImEiOiJjbGdnMTR4MnUwNzBoM2RydjFveHQ0emVpIn0.Jeb-L3dsxMkt6v6HTI8QyQ'
            onViewportChange={viewport => setviewport(viewport)}

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
