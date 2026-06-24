from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "motorspindel_action",
                default_value="/peripherals/motorspindel/execute",
                description=(
                    "Action name expected from a future real "
                    "Motorspindel driver."
                ),
            ),
            DeclareLaunchArgument(
                "festo_action",
                default_value=(
                    "/peripherals/festo_pneumatiksteuerung/execute"
                ),
                description=(
                    "Action name expected from a future real "
                    "Festo Pneumatiksteuerung driver."
                ),
            ),
            LogInfo(
                msg=(
                    "peripherals_real.launch.py is a bringup placeholder. "
                    "Install/start real driver packages that publish the "
                    "same /peripherals/<device_id>/state topics and provide "
                    "the same /peripherals/<device_id>/execute actions."
                )
            ),
        ]
    )
