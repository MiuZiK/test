#!/bin/bash

# 功能：使用terminator的布局配置，在一个窗口中分割3个面板运行命令
# 布局：左侧1个面板，右侧上下2个面板（通过配置文件定义）

# 检查terminator是否安装
if ! command -v terminator &> /dev/null; then
    echo "错误：未安装terminator，请先执行：sudo apt install terminator"
    exit 1
fi

# 检查配置文件是否存在，不存在则创建
CONFIG_DIR=~/.config/terminator
CONFIG_FILE=$CONFIG_DIR/config
if [ ! -f "$CONFIG_FILE" ]; then
    mkdir -p $CONFIG_DIR
    echo "创建terminator配置文件..."
    cat > $CONFIG_FILE <<EOL
[global_config]
  enabled_plugins = TerminalShot, APTURLHandler, LaunchpadCodeURLHandler, GithubCodeURLHandler, Copycat, Trash
  geometry_hinting = False
  handle_size = 1
  title_font = Ubuntu Mono 11
  title_use_system_font = False
[keybindings]
[layouts]
  [[three_panels]]
    [[[child1]]]
      fullscreen = False
      maximised = False
      parent = window0
      profile = default
      type = Terminal
    [[[child2]]]
      fullscreen = False
      maximised = False
      parent = window0
      profile = default
      type = Terminal
    [[[child3]]]
      fullscreen = False
      maximised = False
      parent = window0
      profile = default
      type = Terminal
    [[[window0]]]
      order = 0
      parent = ""
      type = Window
      uuid = 12345678-1234-5678-1234-567812345678
      width = 1200
      height = 800
      position = 200:200
[plugins]
[profiles]
  [[default]]
    background_color = "#2e3440"
    foreground_color = "#d8dee9"
    cursor_color = "#d8dee9"
    font = Ubuntu Mono 12
    scrollback_lines = 10000
    use_system_font = False
EOL
fi

# 启动terminator并应用三窗口布局，分别在3个面板中执行命令
echo "启动三窗口布局..."
terminator -u -b --layout=three_panels \
    -x bash -c "
        # 面板1：运行odom.sh
        cd ~/lan_planner/start_scripts;
        echo '=== 运行odom.sh ===';
        ./odom.sh;
        exec bash;
    " \
    -x bash -c "
        # 面板2：启动launch文件（延迟2秒，等待odom启动）
        sleep 2;
        cd ~/catkin_ws;
        echo '=== 启动turn_on_wheeltec_robot.launch ===';
        source devel/setup.bash;
        roslaunch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch;
        exec bash;
    " \
    -x bash -c "
        # 面板3：启动控制器节点（延迟5秒，等待launch加载）
        sleep 5;
        cd ~/catkin_ws;
        echo '=== 启动straight_controller_cpp节点 ===';
        source devel/setup.bash;
        rosrun turn_on_wheeltec_robot straight_controller_cpp;
        exec bash;
    "

echo "所有面板已启动："
echo "1. 左侧面板：odom.sh"
echo "2. 右侧上面板：turn_on_wheeltec_robot.launch"
echo "3. 右侧下面板：straight_controller_cpp"
echo "在对应面板按Ctrl+C可终止进程，关闭窗口需手动操作"

