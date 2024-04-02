import numpy as np
import torch
import rospy
import tf2_ros
from tf import transformations
from nav_msgs.msg import Odometry
import threading
import copy
import matplotlib.pyplot as plt
from gazebo_msgs.srv import SetModelState, SetModelStateRequest

# Odom线程
class OdomThread(threading.Thread):
    def __init__(self, topic):
        super(OdomThread, self).__init__()
        self.topic = topic
    def _callback(self, msg):
        self.Vl = msg.twist.twist.linear.x # v_linear, m/s
        self.Va = msg.twist.twist.angular.z # v_angular, rad/s
        Rt = msg.pose.pose.orientation  # 四元数： x,y,z,w
        self.theta = transformations.euler_from_quaternion([Rt.x, Rt.y, Rt.z, Rt.w])[2] # 绕Z轴的旋转角度

    def run(self):
        rospy.Subscriber(self.topic, Odometry, self._callback, queue_size=1)
        print(f"----------Subscribing: {self.topic}----------")
        rospy.spin()

class GazeboEnv():
    def __init__(self, opt):
        self.D = opt.D
        self.NP = opt.NP
        self.dvc = opt.dvc  # 运算平台
        self.map_size = opt.map_size
        self.main_frame = opt.main_frame
        self.Search_range = opt.Search_range

        '''Path Planer相关'''
        self.x_target, self.y_target = opt.targetpoint
        self.Vl_scale = 2
        self.Va_scale = 0.5

        '''Motion Planner相关'''
        self.delta_t = 0.01 # 每次移动位置的间隔时间
        assert self.delta_t >= 0.01 # 让gazebo有时间反应
        self.Step_V = opt.Step_V  # Start Point每次被设置时的步进速度

        # 创建car0位置设置服务:
        self.set_state_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        ''' Gazebo相关： '''
        # 障碍物:
        self.obs_list = ['car1', 'car2', 'car3', 'car4', 'car5', 'box_08_05', 'box_10_03']
        self.oip = [[0.5,0.0],[0.0,0.0],[-1.0,1.0],[-1.5,1.5],[0.5,-1.5],[-1.5,-1.0],[1.2,1.2]]  # (x,y) : obs_init_pose
        self.O = len(self.obs_list) # number of Obstacles
        self.DO = 0  # number of Dynamic Obstacle
        for name in self.obs_list:
            if name[0:3] == 'car': self.DO += 1
        self.CD = opt.CD # whether to detect AGV in bounding box
        self.Collide = False


        # 障碍物基座标系名称初始化:
        self.obs_TF_frame_list = copy.deepcopy(self.obs_list)
        for _ in range(self.DO): self.obs_TF_frame_list[_] += '/base_footprint'  # carX -> carX/base_footprint

        # 障碍物Bounding box初始化:
        self.D_FML_AGV = opt.D_FML_AGV # 真实飞马旅小车的直径 (不是car.xacro的)
        self.dilation = self.D_FML_AGV / 2 + 0.2 # bounding box的膨胀, 0.2为噪声余量
        self.obs_outline = np.array([[self.D_FML_AGV, self.D_FML_AGV]]*self.DO + [[0.8, 0.5]] + [[1.0, 0.3]]) # [width, height]
        self.bounding_box = self.obs_outline +self.dilation # 对障碍物轮廓进行膨胀，得到Bounding Box

        # 创建 话题:Odom 订阅对象:
        self.OdomList = []
        for i in range(self.DO):
            self.OdomList.append(OdomThread(self.obs_list[i]+'/odom'))
            self.OdomList[-1].start()
        # 用于存储障碍物的速度信息
        self.Obs_V = np.zeros((self.O, 1, 1, 2)) # O个障碍物的Vx,Vy

        # 创建 TF 订阅对象
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.tf_maxtry = 10 # 订阅的最大尝试次数

        # 用于存储障碍物的bounding box数据
        self.Grouped_Obs_Segments_np = np.zeros((self.O,4,2,2)) # O个障碍物，4条边，2个端点，2维坐标

        '''Render相关'''
        self.dynamic_obs_color = np.array([50, 50, 50]) / 255
        self.static_obs_color = np.array([225, 100, 0]) / 255
        self.predict_seg_color = np.array([255, 0, 255]) / 255
        self.path_color = np.array([0, 0, 255]) / 255
        self.start_traj_color = np.array([255, 174, 185]) / 255
        self.target_traj_color = np.array([202, 255, 112]) / 255
        self.obs_traj_color = np.array([200, 200, 200]) / 255
        self.plot_traj = opt.plot_traj

    def _Generate_BoundingBox(self, x: float, y: float, w: float, h: float, theta: float) -> np.ndarray:
        '''
        Input: coordinate of the obstacle's centre point (x,y), width(w), height(h), orientation(theta in radius)
        Output: coordinate of the Bounding Box in shape (4,2,2): 4条线段，2个端点，2个坐标
        '''

        theta_br, theta_tr, theta_tl = theta, theta + np.arctan(h / w), theta + np.pi / 2  # 这里应是h/w,老版本的w/h是错的
        diagonal = np.sqrt(w ** 2 + h ** 2)

        # calculate the non-shifted apexes:
        bl = np.array([x, y])  # bottom left: [x,y]
        br = np.array([x + w * np.cos(theta_br), (y + w * np.sin(theta_br))])  # bottom right:
        tr = np.array([x + diagonal * np.cos(theta_tr), (y + diagonal * np.sin(theta_tr))])  # top right
        tl = np.array([x + h * np.cos(theta_tl), (y + h * np.sin(theta_tl))])  # top left

        apexes = np.array([[bl, br], [br, tr], [tr, tl], [tl, bl]])  # (4,2,2): 4条线段，2个端点，2个坐标
        return apexes - (tr - bl) / 2  # shift the coordinates in centre manner


    def _Subscribe_TF(self) -> bool:
        '''订阅被控对象 和 障碍物 的TF变换信息。 订阅成功返回True，不成功返回False'''
        try:
            # 被控对象:
            tfs = self.buffer.lookup_transform(self.main_frame, 'car0/base_footprint', rospy.Time(0))
            Ts = tfs.transform.translation  # x,y,z
            self.x_start, self.y_start = Ts.x, Ts.y
            self.d2target = ((self.x_start - self.x_target) ** 2 +
                             (self.y_start - self.y_target) ** 2) ** 0.5  # distance from start to target
            # 障碍物:
            for i, obs_name in enumerate(self.obs_TF_frame_list):
                tfs = self.buffer.lookup_transform(self.main_frame,obs_name,rospy.Time(0))
                Ts = tfs.transform.translation # x,y,z
                Rt = tfs.transform.rotation # 四元数： x,y,z,w
                euler_angles = transformations.euler_from_quaternion([Rt.x, Rt.y, Rt.z, Rt.w])
                self.Grouped_Obs_Segments_np[i] = self._Generate_BoundingBox(Ts.x, Ts.y, self.bounding_box[i,0], self.bounding_box[i,1], euler_angles[2]) #膨胀后的bounding box(大)
            return True
        except:
            return False


    def _Subscribe_odom(self):
        ''' 订阅odom,获得障碍物的速度信息,生成预测线段'''
        for i in range(self.DO):
            theta = self.OdomList[i].theta + self.Va_scale * self.OdomList[i].Va
            self.Obs_V[i, 0, 0, 0] = self.OdomList[i].Vl * np.cos(theta)
            self.Obs_V[i, 0, 0, 1] = self.OdomList[i].Vl * np.sin(theta)

        # 生成障碍物预测线段:
        pdct_Grouped_Obs_endpoints = self.Grouped_Obs_Segments_np + self.Vl_scale * self.Obs_V
        self.Grouped_pdct_segments_np = np.stack((self.Grouped_Obs_Segments_np[0:self.DO, :, 0, :],
                                                  pdct_Grouped_Obs_endpoints[0:self.DO, :, 0, :]),
                                                 axis=2)  # (dynamic_obs,4,2,2)

    def _isInsideObs(self, node: torch.tensor, Grouped_Obs_Segments: torch.tensor) -> bool:
        '''
        判断 node 是否在凸四边形的Grouped_Obs_Segments内部
        node = torch.tensor, shape=(2,)
        Grouped_Obs_Segments = torch.tensor, shape=(O,4,2,2)
        '''
        Obs_node_vector = node - Grouped_Obs_Segments[:,:,0,:]  # (N,4,2), 点和四边形顶点组成的向量
        Obs_edge_vector = Grouped_Obs_Segments[:,:,1,:] - Grouped_Obs_Segments[:,:,0,:]  # (N,4,2), 四边形的四条边向量
        cross_prodct = Obs_node_vector[:,:,0] * Obs_edge_vector[:,:,1] - Obs_node_vector[:,:,1] * Obs_edge_vector[:,:,0] #(O,4), 利用交叉积判断点在四边形四边的左侧还是右侧
        inside = (cross_prodct>0).all(dim=-1) + (cross_prodct<0).all(dim=-1) #(O,), 判断node在哪些Obs内 (node在哪些四边形四条边的同侧)
        return inside.any() #node 在任意一个Obs内？

    def _get_envInfo(self) -> dict:
        '''抽取路径规划所需要的环境信息'''

        # 订阅 box/car 的TF信息
        for i in range(self.tf_maxtry+1):
            if self._Subscribe_TF(): break
            if i == self.tf_maxtry: print(f'Env.py Warning: Can not subscribe TF from Gazebo after {self.tf_maxtry} tries!!!')
        Grouped_Obs_Segments = torch.from_numpy(self.Grouped_Obs_Segments_np).to(self.dvc) # 用于碰撞检测
        if self.CD:
            self.Collide = self._isInsideObs(torch.tensor([self.x_start, self.y_start], device=self.dvc), Grouped_Obs_Segments)

        # 生成预测线
        self._Subscribe_odom()
        Flat_pdct_segments = torch.from_numpy(self.Grouped_pdct_segments_np).reshape(self.DO * 4, 2, 2).to(self.dvc)

        return dict(start_point = (self.x_start, self.y_start),
                    target_point = (self.x_target, self.y_target),
                    d2target = self.d2target,
                    Obs_Segments = Grouped_Obs_Segments.reshape((self.O*4,2,2)), # 障碍物bounding box线段，(4*O,2,2)
                    Flat_pdct_segments = Flat_pdct_segments, # 障碍物运动信息线段， (dynamic_obs*4,2,2)
                    Arrive = (self.d2target < 2*self.D_FML_AGV), # 是否到达终点，
                    Collide = self.Collide) # 是否发生碰撞

    def step(self, path: np.ndarray, d2target: float) -> dict:
        # 确认起始点，引导点
        start_point = np.array([path[0], path[self.NP]])
        if d2target < 0.08*self.map_size: lead_point = np.array([path[self.NP-1], path[-1]]) # target point
        else: lead_point = np.array([path[1], path[1+self.NP]]) # 最近的waypoint

        # 计算位置:
        V_vector = lead_point - start_point
        V_vector = self.Step_V * V_vector / (np.linalg.norm(V_vector) + 1e-6) # normalization and scale
        start_point += V_vector
        # 计算朝向:
        theta_z = np.arctan(V_vector[1] / V_vector[0])  # 围绕z轴的旋转角度
        if V_vector[0] < 0: theta_z += np.pi
        # 设置位置,朝向
        self._Set_object_pose(obj_name='car0', x=start_point[0], y=start_point[1], theta_z=theta_z)
        plt.pause(self.delta_t) # 等待位置生效, 并同时渲染图像

        return self._get_envInfo()

    def _Set_object_pose(self, obj_name: str, x: float, y: float, theta_z: float = None) -> None:
        '''设置gazebo中某个object的x,y,z,theta_z'''
        usv_state = SetModelStateRequest()
        usv_state.model_state.model_name = obj_name

        # 设置位置：
        usv_state.model_state.pose.position.x = x
        usv_state.model_state.pose.position.y = y

        if theta_z is not None:
            qtn = transformations.quaternion_from_euler(0, 0, theta_z)
            usv_state.model_state.pose.orientation.x = qtn[0]
            usv_state.model_state.pose.orientation.y = qtn[1]
            usv_state.model_state.pose.orientation.z = qtn[2]
            usv_state.model_state.pose.orientation.w = qtn[3]

        self.set_state_service(usv_state)
        # 记得延时，等待gazebo生效

    def reset(self, startpoint: tuple) -> dict:
        self._Set_object_pose(obj_name='car0', x=startpoint[0], y=startpoint[1])
        for i, name in enumerate(self.obs_list):
            self._Set_object_pose(obj_name=name, x=self.oip[i][0], y=self.oip[i][1], theta_z=3.14*np.random.rand())
        plt.pause(self.delta_t) # 等待位置生效, 并同时渲染图像

        if self.plot_traj:
            self.start_traj = []
            self.target_traj = []
            self.dynamic_obs_traj = []

        return self._get_envInfo()

    def render(self, path=None):
        plt.clf()  # 清除之前画的图

        # 绘制Bounding Box
        for i in range(self.O):
            if i < self.DO:
                plt.fill(self.Grouped_Obs_Segments_np[i, :, 0, 0],self.Grouped_Obs_Segments_np[i, :, 0, 1], color=self.dynamic_obs_color, zorder=1)
            else:
                plt.fill(self.Grouped_Obs_Segments_np[i, :, 0, 0],self.Grouped_Obs_Segments_np[i, :, 0, 1], color=self.static_obs_color, zorder=0)

        # 绘制预测线:
        for i in range(self.DO):
            for j in range(4):
                plt.plot(self.Grouped_pdct_segments_np[i, j, :, 0], self.Grouped_pdct_segments_np[i, j, :, 1], color=self.predict_seg_color, zorder=2)

        # 绘制起点，终点
        if self.Collide: plt.scatter(self.x_start, self.y_start, color=[0,0,0], zorder=2)
        else: plt.scatter(self.x_start, self.y_start, color=[1,0,0], zorder=2)
        plt.scatter(self.x_target, self.y_target, color=[0,1,0], zorder=2)

        # 绘制路径
        if path is not None:
            plt.plot(path[0:self.NP], path[self.NP:], color=self.path_color, zorder=0)

        # 绘制轨迹
        if self.plot_traj:
            # 起点轨迹
            self.start_traj.append((self.x_start, self.y_start))
            x,y = zip(*self.start_traj)
            plt.plot(x,y, zorder=0, color=self.start_traj_color)

            # 终点轨迹
            self.target_traj.append((self.x_target, self.y_target))
            x,y = zip(*self.target_traj)
            plt.plot(x,y, zorder=0, color=self.target_traj_color)

            # 动态障碍物轨迹：
            dynamic_obs_center = self.Grouped_Obs_Segments_np[0:self.DO,:,0,:].mean(axis=1) # (DO,2) # DO个动态障碍物的2维坐标
            self.dynamic_obs_traj.append(dynamic_obs_center)
            dynamic_obs_traj = np.array(self.dynamic_obs_traj)
            plt.plot(dynamic_obs_traj[:,:,0],dynamic_obs_traj[:,:,1], zorder=0, color=self.obs_traj_color)

        plt.xlim(self.Search_range)
        plt.ylim(self.Search_range)
        # plt.grid()
        plt.xticks([])  # 不显示x轴刻度
        plt.yticks([])  # 不显示y轴刻度
        # plt.pause(1e-6)  # 暂停一段时间（在step和reset里实现），不然画的太快会卡住显示不出来.

    def _get_LeadPoint(self, path: np.ndarray) -> tuple:
        # 根据self.trsd寻找path中合适的lead point
        # 输入: path=[x0,x1,...,y0,y1,...]
        # 输出: lead point 的 X,Y坐标
        '''本意是为了防止甩尾, 但是会让agent闯入bounding box, 暂未使用'''
        # self.trsd = opt.D_FML_AGV / 2 + 0.2  # 选取lead point 的阈值
        d = ((path[:self.NP]-path[0])**2 + (path[self.NP:]-path[self.NP])**2)**0.5 # waypoints距离startpoint的距离
        i = (np.where(d > self.trsd)[0]).min() # leadpoint 的 index
        return (path[i], path[i+self.NP])

def str2bool(v):
    '''Fix the bool BUG for argparse: transfer string to bool'''
    if isinstance(v, bool): return v
    if v.lower() in ('yes', 'True','true','TRUE', 't', 'y', '1', 'T'): return True
    elif v.lower() in ('no', 'False','false','FALSE', 'f', 'n', '0', 'F'): return False
    else: print('Wrong Input Type!')