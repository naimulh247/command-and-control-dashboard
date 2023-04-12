import React, { Component } from 'react';
class VideoFeed extends Component {

    constructor(props) {
        super(props);
        this.canvasRef = React.createRef();
        this.getVideoFeed = this.getVideoFeed.bind(this)
    }
    componentDidMount() {
        this.getVideoFeed()
    }

    getVideoFeed () {
        const { ros } = this.props;
        const canvas = this.canvasRef.current;
        if (!ros) {
            console.warn("ROS/ RosBridge not intialized: VideoFeed");
            return;
        }
        // console.log(ros);
        
        // create a new video_subscriber with the topics
        const video_subscriber = new window.ROSLIB.Topic({
            ros: ros,
            name: '/camera/rgb/image_raw/compressed',
            messageType: 'sensor_msgs/CompressedImage',
        })

        // if there is a message update the canvas with the video feed
        video_subscriber.subscribe((message) => {
            const data = message.data;
            const frame = new Image();
            frame.onload = () => {
                // draw the image on the canvas
                const ctx = canvas.getContext('2d');
                ctx.drawImage(frame, 0, 0, canvas.width, canvas.height);
            }
            frame.src = 'data:image/jpeg;base64,' + data;
        });

    }
    render() { 
        return (
            // this needs to be dynamic
            <canvas ref={this.canvasRef} width="640" height="480"></canvas>
        );
    }
}
 
export default VideoFeed;