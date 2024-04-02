import argparse
import rospy
import torch
import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'
from Planner import OkayPlan
from Env import GazeboEnv, str2bool
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


'''Configurations:'''
parser = argparse.ArgumentParser()
parser.add_argument('--dvc', type=str, default='cuda', help='Running device of SEPSO_Down, cuda or cpu')
parser.add_argument('--DPI', type=str2bool, default=True, help='True for DPI(from OkayPlan), False for PI(from SEPSO)')
parser.add_argument('--KP', type=str2bool, default=True, help='whether to use Kinematics_Penalty')
parser.add_argument('--render', type=str2bool, default=True, help='whether to render with plt')
parser.add_argument('--plot_traj', type=str2bool, default=True, help='whether to plot trajectory')

# Planner related:
parser.add_argument('--Max_iterations', type=int, default=50, help='maximum number of particle iterations for each planning')
parser.add_argument('--N', type=int, default=200, help='number of particles in each group')
parser.add_argument('--D', type=int, default=16, help='particle dimension: number of waypoints = D/2')
parser.add_argument('--Quality', type=float, default=0.1, help='planning quality: the smaller, the better quality, and the longer time')

# Env related:
parser.add_argument('--targetpoint', type=tuple, default=(2.7, 2.7), help='target point')
parser.add_argument('--startpoint', type=tuple, default=(-2.7,-2.7), help='start point')
parser.add_argument('--map_size', type=int, default=6, help='map size: m')
parser.add_argument('--main_frame', type=str, default='gazebo_center', help='the main frame of navigation, eg: gazebo_center/map')
parser.add_argument('--D_FML_AGV', type=float, default=0.2, help='diameter of the real FML agv: m')
parser.add_argument('--CD', type=str2bool, default=False, help='Collision Detection: whether to detect AGV in bounding box')
parser.add_argument('--Step_V', type=float, default=0.05, help='step velocity of the controlled robot')


opt = parser.parse_args()
opt.dvc = torch.device(opt.dvc)
opt.NP = int(opt.D / 2)  # 每个粒子所代表的路径的端点数量
opt.Search_range = (-opt.map_size/2, opt.map_size/2)

if opt.render:
    import matplotlib.pyplot as plt
    plt.figure(figsize=(5, 5))  # 设置长宽比例
    plt.ion()  # 开启plt动态绘图



if __name__ == '__main__':
    # 初始化 ROS 节点
    rospy.init_node("OkayPlan_main")

    # 加载OkayPlan的超参数
    params = torch.load('Relax0.4_S0_ 2023-09-23 21_38.pt', map_location=opt.dvc)[-1]  # [-1] means the final parameters
    if not opt.KP: params[50] = 0

    env = GazeboEnv(opt) # env用于抽取gazebo的环境信息和移动小车
    planner = OkayPlan(opt, params) # 用于规划全局路径

    # 创建路径发布器
    path_puber = rospy.Publisher("OkayPlan", Path, queue_size=1)

    while not rospy.is_shutdown():
        env_info, done = env.reset(opt.startpoint), False
        planner.Priori_Path_Init(env_info['start_point'], env_info['target_point']) # 为第一次规划初始化先验路径
        while not done:
            '''根据环境信息规划路径'''
            path_torch, collision_free = planner.plan(env_info) # collision_free仅代表路径是否无碰撞，并不代表start_point的碰撞情况
            path_np = path_torch.cpu().numpy() # torch to numpy, [x0,x1,...,y0,y1,..], shape=(D,)
            if opt.render: env.render(path_np)  # 渲染

            '''组织&发布路径数据：'''
            Path_msg = Path() # 创建数据
            # 填写表头：
            current_time = rospy.Time.now()  # Current timestamp
            Path_msg.header.stamp = current_time
            Path_msg.header.frame_id = opt.main_frame
            # 填写路径坐标：
            for i in range(opt.NP):
                pose = PoseStamped()
                pose.header.frame_id = opt.main_frame  # Coordinate frame reference
                pose.header.stamp = current_time
                pose.pose.position.x = path_np[i]
                pose.pose.position.y = path_np[i+opt.NP]
                Path_msg.poses.append(pose)
            path_puber.publish(Path_msg) # 发布路径

            '''根据path移动car0并刷新环境信息，为下次规划做准备'''
            env_info = env.step(path_np, env_info['d2target'])
            done = env_info['Arrive'] + env_info['Collide']




















