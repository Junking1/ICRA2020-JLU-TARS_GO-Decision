project(roborts_msgs)
cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR})
find_package(catkin REQUIRED COMPONENTS
    std_msgs
    nav_msgs
    geometry_msgs
    actionlib
    message_generation
    actionlib_msgs
    )

add_action_files(
    DIRECTORY action
    FILES
    LocalPlanner.action
    GlobalPlanner.action
    ArmorDetection.action
    GimbalSwing.action
)

add_message_files(
    DIRECTORY msg/referee_system
    FILES
    BonusStatus.msg
    GameResult.msg
    GameStatus.msg
    GameSurvivor.msg
    ProjectileSupply.msg
    RobotBonus.msg
    RobotDamage.msg
    RobotHeat.msg
    RobotShoot.msg
    RobotStatus.msg
    SupplierStatus.msg
)

add_message_files(
    DIRECTORY msg
    FILES
    PunishInfo.msg
    RobotInfo.msg
    TreeStatus.msg
    TwistAccel.msg
    GimbalAngle.msg
    GimbalRate.msg
    ObstacleMsg.msg
    ShootInfo.msg
    ShootState.msg
    SentryInfo.msg
    GimbalControl.msg
    PyArmorInfo.msg
    GimbalActionlib.msg
)

add_service_files(
    DIRECTORY srv
    FILES
    SwingDefend.srv
    GimbalMode.srv
    FricWhl.srv
    ShootCmd.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    nav_msgs
    actionlib_msgs
)

catkin_package()
