const ros_config = {
    ROSBRIDGE_SERVER_IP: "127.0.0.1",
    ROSBRIDGE_SERVER_PORT: "9090",
    VIDEO_WEB_SERVER_PORT: "8090",
    RECONNECTION_TIMEOUT: 3000,
    ROSBRIDGE_CMD_VEL: "/cmd_vel",
    ROSBRIDGE_ODOM: "/odom",
    ROSBRIDGE_ROSTOPICS_LIST: "/rostopic_list",
}

export default ros_config