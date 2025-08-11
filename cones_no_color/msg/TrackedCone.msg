# custom_interface/msg/TrackedCone.msg
int32 track_id          # Unique identifier for the tracked cone
geometry_msgs/Point position # 3D position in os_sensor frame (x, y, z - float64)
string color            # Estimated color name (e.g., "Blue cone", "Unknown") 