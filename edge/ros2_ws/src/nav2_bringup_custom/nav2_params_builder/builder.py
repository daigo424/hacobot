"""nav2_params.yamlの本家base+hacobot固有オーバーライドのマージ処理。

spawn_robot(複数ロボット起動)とnav2_bringup_custom(単体起動用エントリポイント)の
両launchファイルから使われる共有モジュール。defaults/配下のYAML群(このパッケージが
share/params/defaultsとしてインストールしている)を実際に読み込む処理のため、
その所有者であるこのパッケージ側にモジュールとして置いている。
"""
import copy
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory


def build_nav2_params_yaml():
    """本家Nav2既定値(defaults/nav2_params.yaml)とslam_toolbox本家既定値
    (defaults/mapper_params_online_sync.yaml)をベースに、hacobot固有の差分だけを
    明示的に上書きしたnav2_params.yamlを作る。

    以前は本家からの差分・非差分が453行のYAMLに埋もれて区別できず、実際に
    collision_monitor.scan.source_timeout(waffle.yaml由来の0.2秒)が
    シミュレーター環境の処理遅延と噛み合わず誤停止の原因になった経緯があるため、
    差分をここに明示する。

    上書きする内容:
    - 車体依存値(waffle.yamlから実際に読み込む): costmapのrobot_radius/inflation_layer、
      obstacle_layer/voxel_layer(defaultsはcostmapごとにどちらか一方しか設定していないため、
      waffle.yamlの構成をそのまま読み込んで両方揃える)、collision_monitorの追加安全ゾーン
      (PolygonStop/PolygonSlow。defaultsはFootprintApproachのみ)
    - FootprintApproach.footprint_topicの相対パス化(マルチロボット名前空間対応のため
      絶対パスは不可。これはwaffle.yaml由来ではなくこのプロジェクト固有の変更)
    - docking_server(このNav2バージョンのnavigation_launch.pyがlifecycle_nodesに
      ハードコードしており、configureに失敗するとbringup全体が止まるため、
      実際には使わないダミードックで最低限configureを通す)
    - slam_toolbox.base_frame(このプロジェクトはbase_footprintで統一している。
      本家既定はbase_link)
    - controller_server.progress_checker.movement_time_allowance(safety_state_machineの
      DEGRADED状態がcmd_velを半速(degraded_speed_scale=0.50)に落とすため、本家既定の
      10.0秒のままだと「進んでいない」とNav2自身のprogress_checkerに誤検知され、
      DEGRADED中にfollow_pathが中断されてしまう。半速でも余裕を持って間に合うよう倍にする)

    それ以外(controller_server=MPPI、bt_navigator、planner_server等)は
    defaults/nav2_params.yamlをそのまま採用する。
    """
    defaults_dir = os.path.join(
        get_package_share_directory('nav2_bringup_custom'), 'params', 'defaults'
    )
    with open(os.path.join(defaults_dir, 'nav2_params.yaml'), 'r') as f:
        params = yaml.safe_load(f)
    with open(os.path.join(defaults_dir, 'mapper_params_online_sync.yaml'), 'r') as f:
        mapper_params = yaml.safe_load(f)
    with open(os.path.join(defaults_dir, 'waffle.yaml'), 'r') as f:
        waffle_params = yaml.safe_load(f)

    params['slam_toolbox'] = mapper_params['slam_toolbox']
    params['slam_toolbox']['ros__parameters']['base_frame'] = 'base_footprint'

    # safety_state_machineのDEGRADED状態がcmd_velを半速に落とすため、本家既定の
    # movement_time_allowance(10.0秒)のままだと「進んでいない」と誤検知されうる。
    params['controller_server']['ros__parameters']['progress_checker'][
        'movement_time_allowance'] = 20.0

    for costmap_name in ('local_costmap', 'global_costmap'):
        waffle_costmap_params = waffle_params[costmap_name][costmap_name]['ros__parameters']
        costmap_params = params[costmap_name][costmap_name]['ros__parameters']
        costmap_params['robot_radius'] = waffle_costmap_params['robot_radius']
        costmap_params['inflation_layer']['inflation_radius'] = \
            waffle_costmap_params['inflation_layer']['inflation_radius']
        costmap_params['inflation_layer']['cost_scaling_factor'] = \
            waffle_costmap_params['inflation_layer']['cost_scaling_factor']

    waffle_local_costmap_params = waffle_params['local_costmap']['local_costmap']['ros__parameters']
    local_costmap_params = params['local_costmap']['local_costmap']['ros__parameters']
    local_costmap_params['plugins'] = copy.deepcopy(waffle_local_costmap_params['plugins'])
    local_costmap_params['obstacle_layer'] = copy.deepcopy(
        waffle_local_costmap_params['obstacle_layer'])

    waffle_global_costmap_params = waffle_params['global_costmap']['global_costmap']['ros__parameters']
    global_costmap_params = params['global_costmap']['global_costmap']['ros__parameters']
    global_costmap_params['plugins'] = copy.deepcopy(waffle_global_costmap_params['plugins'])
    global_costmap_params['voxel_layer'] = copy.deepcopy(
        waffle_global_costmap_params['voxel_layer'])

    waffle_collision_monitor_params = waffle_params['collision_monitor']['ros__parameters']
    collision_monitor_params = params['collision_monitor']['ros__parameters']
    collision_monitor_params['polygons'] = copy.deepcopy(
        waffle_collision_monitor_params['polygons'])
    collision_monitor_params['PolygonStop'] = copy.deepcopy(
        waffle_collision_monitor_params['PolygonStop'])
    collision_monitor_params['PolygonSlow'] = copy.deepcopy(
        waffle_collision_monitor_params['PolygonSlow'])
    # 絶対パスのままだとロボットの名前空間(/tb3_builder等)が付かず、実際に
    # 配信されているトピックと一致しない(behavior_serverのlocal_footprint_topicと同様)
    collision_monitor_params['FootprintApproach']['footprint_topic'] = \
        'local_costmap/published_footprint'

    params['docking_server'] = {
        'ros__parameters': {
            'enable_stamped_cmd_vel': True,
            'dock_plugins': ['dummy_dock'],
            'dummy_dock': {
                'plugin': 'opennav_docking::SimpleNonChargingDock',
            },
        },
    }

    # cmd_vel_nav2_rawを共有するノード間でTwist/TwistStampedの型が食い違うと
    # publisher作成に失敗しbringup全体がクラッシュする(実測で確認)。
    # 本家既定値の実際のコンパイル時デフォルトに頼らず、明示的に全て揃える。
    for section_name in ('controller_server', 'behavior_server', 'collision_monitor', 'velocity_smoother'):
        params[section_name]['ros__parameters']['enable_stamped_cmd_vel'] = True

    params_tmp = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
    yaml.safe_dump(params, params_tmp)
    params_tmp.close()

    return params_tmp.name
