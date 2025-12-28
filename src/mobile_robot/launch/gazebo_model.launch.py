import os 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
import xacro

def generate_launch_description():
    
    #name of the robot defined in the Xacro file 
    robotXacroName='castor_robot'
    
    #name of the package ,at the same time name of the folder that will be used to define paths
    namePackage='mobile_robot'
    
    #relative path of the xacro file defing the model 
    modelFileRelativePath='model/robot.xacro'
    
    #absolute path of the model
    pathModelFile=os.path.join(get_package_share_directory(namePackage),modelFileRelativePath)
    
    #you can add your own world files path of you have one 
    
    #getting the robot description xacro file
    robotDescription= xacro.process_file(pathModelFile).toxml()
    
    #launch file froom gazebo_ros package
    gazebo_rosPackageLaunch=PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('ros_gz_sim'),
                    'launch',
                    'gz_sim.launch.py')
    )

    #craeting an empty world
    gazeboLaunch=IncludeLaunchDescription(
        gazebo_rosPackageLaunch,
        launch_arguments={'gz_args':['-r -v -v4 empty.sdf'],'on_exit_shutdown':'true'}.items()
    )
    
    #Gazebo node
    spawnModelNodeGazebo= Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', robotXacroName,
        ],
        output='screen',
    )
    
    #Robot State Publisher Node
    nodeRobotStatePublisher= Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robotDescription,
        'use_sim_time': True}]
    )
    
    #parameters to control robot
    bridge_params = os.path.join(
        get_package_share_directory(namePackage),
        'parameters',
        'bridge_params.yaml'
    )
    
    #package to bridging the gap 
    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros_args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen',
    )
    
    #creating an empty launch description object
    LaunchDescriptionObject=LaunchDescription()
    
    #adding GazeboLaunch
    LaunchDescriptionObject.add_action(gazeboLaunch)
    
    #adding the nodes
    LaunchDescriptionObject.add_action(spawnModelNodeGazebo)
    LaunchDescriptionObject.add_action(nodeRobotStatePublisher)
    LaunchDescriptionObject.add_action(start_gazebo_ros_bridge_cmd)
    
    return LaunchDescriptionObject