from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='francor_yolo_hazmat',
            namespace='francor_yolo_hazmat',
            executable='francor_yolo_hazmat',
            name='francor_yolo_hazmat',
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    #"image_subs_topic"  : "/camera/tele/image_raw",
                    "image_subs_topic"  : "/image_raw",
                    "image_pub_topic"   : "/find_object_2d/img_hazmats",
                    "qos_sensor_data_on": 1,
                    "yolo_weights_file" : "2023-04-26-01-yolo5s-64e-objects.pt",
                    "object_mode"       : 1,
                    'yolo_confidence'   : 0.6,
                    'obj_max_size'      : 0.5,
                    'obj_min_ratio'     : 0.2,
                    'obj_max_ratio'     : 1.8,
                }
            ],
            arguments=['--ros-args', '--log-level', 'info']
        )
    ])
