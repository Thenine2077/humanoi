# Third-party code in this repo

แพ็กเกจเหล่านี้ clone มาจากต้นทาง แล้วเก็บไว้ใน repo นี้ตรงๆ (vendored) เพื่อให้ build ได้ทันทีและเก็บ patch ของเราไว้
ประวัติ git เดิมของต้นทางสำรองไว้ที่ `C:\Users\User\Documents\humanoi_upstream_git_backup\` บน PC

| โฟลเดอร์ | ต้นทาง | commit | license | แก้อะไร |
|---|---|---|---|---|
| `ros2_ws/src/ldlidar_ros2` | https://github.com/ldrobotSensorTeam/ldlidar_ros2 | `0f6101c93db0b4448cbe7a654826adcdee50622f` | MIT | — |
| `ros2_ws/src/ldlidar_ros2/sdk` (submodule เดิม) | ldlidar SDK v3.3.1 | `6148cd3c33ae39adbd7e12faf82567fa8f8e7b0d` | MIT | `src/log_module.cpp`: เพิ่ม `#include <pthread.h>` (build บน Jazzy/GCC ใหม่ไม่ผ่านถ้าไม่มี) |
| `ros2_ws/src/rf2o_laser_odometry` | https://github.com/MAPIRlab/rf2o_laser_odometry (branch ros2) | `b38c68e46387b98845ecbfeb6660292f967a00d3` | GPL v3 | — |
