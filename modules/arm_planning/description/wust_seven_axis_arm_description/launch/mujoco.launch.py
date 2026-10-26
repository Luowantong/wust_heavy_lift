"""启动 MuJoCo 交互式查看器，加载本包的 MJCF 模型。

用法::

    ros2 launch wust_seven_axis_arm_description mujoco.launch.py
    ros2 launch wust_seven_axis_arm_description mujoco.launch.py model:=/abs/path/custom.xml

模型说明：

``wust_rm.xml``（默认）含 7 个位置执行器 ``pos_J1``~``pos_J7``（``kp=300 kv=3``），
启动后在右侧 Control 面板拖滑块，``ctrl`` 即目标关节角(rad)，关节会定位过去。
因增益有限，稳态有约 2° 的重力下垂。左键拖拽只旋转视角，要拖动机身请按
Ctrl + 左键。

注意：``wust_rm.xml`` 用 ``meshdir="meshes"`` 引用网格（相对 XML 自身目录解析）。
它与 ``meshes/`` 都由本包安装到 ``share/wust_seven_axis_arm_description/``，
故默认路径可用。
"""

import importlib.util
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

PKG_NAME = 'wust_seven_axis_arm_description'


def _resolve_model(raw):
    """包内相对路径按 share/<pkg>/ 解析；绝对路径原样返回。"""
    if os.path.isabs(raw):
        return raw
    return os.path.join(get_package_share_directory(PKG_NAME), raw)


def _launch_viewer(context, *args, **kwargs):
    model = _resolve_model(LaunchConfiguration('model').perform(context))

    if not os.path.isfile(model):
        raise RuntimeError(
            'MJCF 模型不存在: {}\n'
            '  · 若用的是包内模型，先执行 colcon build（wust_rm.xml 现已随包安装）\n'
            '  · 或传绝对路径: model:=/path/to/model.xml'.format(model)
        )

    # mujoco 装在用户 site-packages，缺失时提前给出可读的提示而不是让子进程报 traceback
    if importlib.util.find_spec('mujoco') is None:
        raise RuntimeError(
            '未找到 mujoco，请先安装: pip3 install mujoco'
        )

    return [
        ExecuteProcess(
            cmd=['python3', '-m', 'mujoco.viewer', '--mjcf', model],
            output='screen',
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value='wust_rm.xml',
            description='MJCF 模型路径：包内相对路径（相对 share/%s/）或绝对路径' % PKG_NAME,
        ),
        OpaqueFunction(function=_launch_viewer),
    ])
