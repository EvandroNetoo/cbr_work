"""Launch the CBR omnidirectional motor controller."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _criar_no(context, config_file, python_executable):
    """Cria o nó usando o YAML e somente os overrides informados na CLI."""
    tipos_overrides = {
        'dry_run': bool,
        'command_mode': str,
        'max_motor_speed': float,
        'expansion_port': int,
    }
    overrides = {}
    for nome, tipo in tipos_overrides.items():
        valor = LaunchConfiguration(nome).perform(context)
        if valor != '':
            overrides[nome] = ParameterValue(
                LaunchConfiguration(nome), value_type=tipo)

    parametros = [config_file]
    if overrides:
        # Este dicionário vem depois do YAML intencionalmente: apenas valores
        # fornecidos na CLI devem sobrescrever o arquivo de configuração.
        parametros.append(overrides)

    return [Node(
        package='cbr_motor_control',
        executable='motor_node',
        name='cbr_motor_control',
        output='screen',
        emulate_tty=True,
        prefix=[python_executable],
        parameters=parametros,
    )]


def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare('cbr_motor_control'), 'config', 'motors.yaml'])
    python_executable = LaunchConfiguration('python_executable')

    return LaunchDescription([
        DeclareLaunchArgument(
            'dry_run', default_value='',
            description='Override opcional; vazio usa motors.yaml'),
        DeclareLaunchArgument(
            'command_mode', default_value='',
            description='Override opcional; vazio usa motors.yaml'),
        DeclareLaunchArgument(
            'max_motor_speed', default_value='',
            description='Override opcional; vazio usa motors.yaml'),
        DeclareLaunchArgument(
            'expansion_port', default_value='',
            description='Override opcional; vazio usa motors.yaml'),
        DeclareLaunchArgument(
            'python_executable',
            default_value=PathJoinSubstitution([
                EnvironmentVariable('VIRTUAL_ENV'), 'bin', 'python']),
            description='Python da venv usado explicitamente para executar o nó',
        ),
        OpaqueFunction(
            function=_criar_no,
            kwargs={
                'config_file': config_file,
                'python_executable': python_executable,
            },
        ),
    ])
