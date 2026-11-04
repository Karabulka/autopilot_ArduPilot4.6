# MAVLink OPTICAL_FLOW_RAD convention: sensor axes, integrated radians.
std_msgs/Header header
uint8 sensor_id
uint32 integration_time_us
float32 integrated_x
float32 integrated_y
float32[3] integrated_gyro
int16 temperature
uint8 quality
uint32 time_delta_distance_us
float32 distance
# Legacy OPTICAL_FLOW pixel displacement, supplied by the sensor.
int16 flow_px_x
int16 flow_px_y
