# vendor/

Jazzy + Gazebo Harmonic向けのapt(ros-jazzy-*)バイナリがまだ存在しない、
または存在してもビルドが壊れている依存パッケージを、ソースのままvendorしたディレクトリ。
`colcon build`はこのディレクトリも通常のworkspaceパッケージとして扱うため、追加設定は不要。

| パッケージ | 由来 | ライセンス |
|---|---|---|
| `turtlebot3_gazebo` | [ROBOTIS-GIT/turtlebot3_simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations) (jazzy branch) | Apache 2.0 |
| `explore_lite` | [robo-friends/m-explore-ros2](https://github.com/robo-friends/m-explore-ros2) | BSD |
| `explore_lite_msgs` | 同上 | BSD |
| `rviz_2d_overlay_msgs` | [teamspatzenhirn/rviz_2d_overlay_plugins](https://github.com/teamspatzenhirn/rviz_2d_overlay_plugins) (main branch) | BSD |
| `rviz_2d_overlay_plugins` | 同上 | BSD |

`rviz_2d_overlay_plugins`はapt版(`ros-jazzy-rviz-2d-overlay-plugins` 1.4.1-1noble.20260615)の
`librviz_2d_overlay_plugins.so`が`OverlayTextDisplay`のvtableシンボルを欠いたまま
ビルドされておりRViz起動時にdlopenが失敗するため、apt版の代わりにソースからvendorしている
(パッケージング側のビルド不良であり当プロジェクトのコード起因ではない)。

更新する場合は上記リポジトリから該当パッケージフォルダを取り直し、`.git`を含めずに
このディレクトリ配下へ上書きすること。
