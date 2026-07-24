# vendor/

Jazzy + Gazebo Harmonic向けのapt(ros-jazzy-*)バイナリがまだ存在しない依存パッケージを、
ソースのままvendorしたディレクトリ。`colcon build`はこのディレクトリも通常のworkspace
パッケージとして扱うため、追加設定は不要。

| パッケージ | 由来 | ライセンス |
|---|---|---|
| `turtlebot3_gazebo` | [ROBOTIS-GIT/turtlebot3_simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations) (jazzy branch) | Apache 2.0 |
| `explore_lite` | [robo-friends/m-explore-ros2](https://github.com/robo-friends/m-explore-ros2) | BSD |
| `explore_lite_msgs` | 同上 | BSD |

更新する場合は上記リポジトリから該当パッケージフォルダを取り直し、`.git`を含めずに
このディレクトリ配下へ上書きすること。
