from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    measurement_noise = DeclareLaunchArgument(
        'measurement_noise',
        default_value='50.0',
        description="Measurement scalar noise value")

    ma_confidence_threshold = DeclareLaunchArgument(
        'ma_confidence_threshold',
        default_value='0.40',
        description="Moving average confidence threshold to consider the valid keypoints")

    hallucination_threshold = DeclareLaunchArgument(
        'hallucination_threshold',
        default_value='0.25',
        description="Confidence under which the keypoint is not considered")

    single_keypoint_ma_confidence_threshold = DeclareLaunchArgument(
        'single_keypoint_ma_confidence_threshold',
        default_value='0.35',
        description="Single keypoint moving average confidence threshold to consider the keypoint valid")

    ma_window_size = DeclareLaunchArgument(
        'MA_window_size',
        default_value='40',
        description="Windows size for moving average")

    filter = DeclareLaunchArgument(
        'filter',
        default_value='false',
        description="Use or not the kalman filter for keypoints")

    predict = DeclareLaunchArgument(
        'predict',
        default_value='true',
        description="Use kalman filter predictions when measure is uncertain")

    debug = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description="Print debug info")

    skeleton_tracking = Node(
        package='image_processing',
        executable='skeleton_track_yolo',
        name='skeleton_track_yolo',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'measurement_noise': LaunchConfiguration('measurement_noise'),
            'ma_confidence_threshold': LaunchConfiguration('ma_confidence_threshold'),
            'hallucination_threshold': LaunchConfiguration('hallucination_threshold'),
            'single_keypoint_ma_confidence_threshold': LaunchConfiguration('single_keypoint_ma_confidence_threshold'),
            'MA_window_size': LaunchConfiguration('MA_window_size'),
            'filter': LaunchConfiguration('filter'),
            'predict': LaunchConfiguration('predict'),
            'debug': LaunchConfiguration('debug'),
        }],
    )

    return LaunchDescription([
        measurement_noise,
        use_sim_time,
        ma_window_size,
        ma_confidence_threshold,
        hallucination_threshold,
        single_keypoint_ma_confidence_threshold,
        filter,
        predict,
        debug,
        skeleton_tracking,
    ])
