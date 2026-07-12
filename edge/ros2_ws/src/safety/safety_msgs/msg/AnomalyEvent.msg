# heartbeat_monitor / watchdog / safety_state_machine間で共有する異常イベント通知。
# /safety/anomaly_event トピックにpublishされる。

builtin_interfaces/Time stamp

# どの監視対象が異常を起こしたか (例: "nav2", "lidar", "comm_bridge")
string source

# 重大度。WARNINGはsafety_state_machineをDEGRADED経由で遷移させ、
# CRITICALはDEGRADEDを飛ばして即座にSAFE_STOPへ遷移させる。
string WARNING=WARNING
string CRITICAL=CRITICAL
string severity

# 人間が読める異常理由 (例: "heartbeat timeout: 342ms since last heartbeat (threshold 300ms)")
string reason
