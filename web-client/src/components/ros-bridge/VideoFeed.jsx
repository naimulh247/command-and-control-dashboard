import React, { Component } from 'react';
import { Container } from 'react-bootstrap';
import ros_config from '../../configs/ros_config';

class VideoFeed extends Component {

    constructor(props) {
        super(props);
        this.canvasRef = React.createRef();
        this.getVideoFeed = this.getVideoFeed.bind(this)

        this.state = {
            frameWidth: localStorage.getItem('frameWidth') || ros_config.ROSBRIDGE_FRAME_WIDTH,
            frameHeight: localStorage.getItem('frameHeight') || ros_config.ROSBRIDGE_FRAME_HEIGHT,
        }
    }

    componentDidMount() {
        this.getVideoFeed()
        this.imageConfigInterval = setInterval(() => {
            this.pubImageConfigs();
        }, ros_config.CHECK_IMAGE_CONFIG);
    }

    componentDidUpdate( ) {
        this.getVideoFeed()
    }

    componentWillUnmount() {
        clearInterval(this.imageConfigInterval); // clear the interval when component unmounts
    }

    pubImageConfigs() {
        const { ros } = this.props;

        // publish imageWidth and imageHeight as a rostopic
        const imageConfig_publisher = new window.ROSLIB.Topic({
            ros: ros,
            name: `${ros_config.ROSBRIDGE_IMAGE_CONFIGS}`,
            messageType: 'std_msgs/String'
        });
 
        const imageWidth = localStorage.getItem('imageWidth') || ros_config.ROSBRIDGE_IMAGE_WIDTH;
        const imageHeight = localStorage.getItem('imageHeight') || ros_config.ROSBRIDGE_IMAGE_HEIGHT;
        const message = new window.ROSLIB.Message({data: `${imageWidth},${imageHeight}`});

        imageConfig_publisher.publish(message);
    }

    getVideoFeed() {
        const { ros } = this.props;
        const canvas = this.canvasRef.current;

        if (!ros) {
            console.warn("ROS/ RosBridge not intialized: VideoFeed");
            return;
        }
        console.log(ros);
        
        // create a new video_subscriber with the topics
        const video_subscriber = new window.ROSLIB.Topic({
            ros: ros,
            name: `${ros_config.ROSBRIDGE_RASPICAM_TOPIC}`,
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
            <Container className='d-flex justify-content-center align-items-center' style={{paddingBottom: "1.5%"}}>
                <canvas className='center' ref={this.canvasRef} width={this.state.frameWidth} height={this.state.frameHeight}></canvas>
            </Container>
        );
    }
}
 
export default VideoFeed;
