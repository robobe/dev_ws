import launch

def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument('msg', default_value='hello world'),
        launch.actions.DeclareLaunchArgument('other'),
        launch.actions.LogInfo(msg=launch.substitutions.LaunchConfiguration('msg')),
        launch.actions.LogInfo(msg=launch.substitutions.LaunchConfiguration('other')),
    ])