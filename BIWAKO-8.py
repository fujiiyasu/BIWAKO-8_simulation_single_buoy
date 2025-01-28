import csv
import math
from geopy.distance import geodesic
from controller import Supervisor

from const import parameter
parameter = parameter()
# Supervisorのインスタンスを作成
supervisor = Supervisor()

class BIWAKO_8:
    def __init__(self):
        self.STRAIGHT_MAX_THRUST = parameter.STRAGHT_MAX_THRUST
        
        # シミュレーションの基本設定
        self.time_step = int(supervisor.getBasicTimeStep())
        
        # 必要なデバイスを取得（例: モーター、センサーなど）
        # リンク関節
        self.link_joint1 = supervisor.getDevice('servo11')
        self.link_joint2 = supervisor.getDevice('servo21')
        self.link_joint3 = supervisor.getDevice('servo31')
        self.link_joint4 = supervisor.getDevice('servo41')
        # スラスタ関節
        self.thruster_joint1 = supervisor.getDevice('servo12')
        self.thruster_joint2 = supervisor.getDevice('servo22')
        self.thruster_joint3 = supervisor.getDevice('servo32')
        self.thruster_joint4 = supervisor.getDevice('servo42')

        # リンク関節ポテンショメータ
        self.link_joint1_ps = supervisor.getDevice('ps11')
        self.link_joint2_ps = supervisor.getDevice('ps21')
        self.link_joint3_ps = supervisor.getDevice('ps31')
        self.link_joint4_ps = supervisor.getDevice('ps41')
        # サーボ関節ポテンショメータ
        self.thruster_joint1_ps = supervisor.getDevice('ps12')
        self.thruster_joint2_ps = supervisor.getDevice('ps22')
        self.thruster_joint3_ps = supervisor.getDevice('ps32')
        self.thruster_joint4_ps = supervisor.getDevice('ps42')
        # スラスタ
        self.thruster1 = supervisor.getDevice('thruster1')
        self.thruster2 = supervisor.getDevice('thruster2')
        self.thruster3 = supervisor.getDevice('thruster3')
        self.thruster4 = supervisor.getDevice('thruster4')
        # GPS(ノイズあり)
        self.gps_noise = supervisor.getDevice('gps')
        self.gps_noise_value = [0, 0, 0]
        # GPS(ノイズなし)
        self.gps = supervisor.getDevice('a_gps')
        self.gps_value = [0, 0, 0]
        self.speed = 0.0
        # コンパス
        self.compass = supervisor.getDevice('compass')
        self.compass_value = [0, 0, 0]
        
        # 電流，電圧
        self.A = 0.0
        self.V = 12.0
        self.W = 0.0

        # 位置姿勢に関する変数
        self.waypoint_list = [] # [緯度, 経度]で構成される目標位置
        self.waypoint_num = 0 # 目標位置の番号
        self.waypoint = [0.0, 0.0] # 目標位置，初期値は[0.0, 0.0]

        # モード
        self.mode = 0 # 0: 直進モード, 1: キープモード
    
    def init_device(self):
        # リンク関節の設定
        link_joint_list = [self.link_joint1, self.link_joint2, self.link_joint3, self.link_joint4]
        for link_joint in link_joint_list:
            link_joint.setPosition(0.0)
            link_joint.setVelocity(math.pi/2)
        # スラスタ関節の設定
        thruster_joint_list = [self.thruster_joint1, self.thruster_joint2, self.thruster_joint3, self.thruster_joint4]
        for thruster_joint in thruster_joint_list:
            thruster_joint.setPosition(0.0)
            thruster_joint.setVelocity(math.pi/4)
        # リンク関節ポテンショメータの設定
        link_joint_ps_list = [self.link_joint1_ps, self.link_joint2_ps, self.link_joint3_ps, self.link_joint4_ps]
        for link_joint_ps in link_joint_ps_list:
            link_joint_ps.enable(10)
        # スラスタ関節ポテンショメータの設定
        thruster_joint_ps_list = [self.thruster_joint1_ps, self.thruster_joint2_ps, self.thruster_joint3_ps, self.thruster_joint4_ps]
        for thruster_joint_ps in thruster_joint_ps_list:
            thruster_joint_ps.enable(10)
        # スラスタの設定
        thruster_list = [self.thruster1, self.thruster2, self.thruster3, self.thruster4]
        for thruster in thruster_list:
            thruster.setPosition(float('inf'))
            thruster.setVelocity(0.0)

        # センサ初期値取得
        self.gps_noise.enable(10)
        self.gps.enable(10)
        self.compass.enable(10)
        self.gps_noise_value = self.gps_noise.getValues()
        self.gps_value = self.gps.getValues()
        self.compass_value = self.compass.getValues()

    def load_waypoints_from_csv(self, file_path):
        """
        CSVファイルから経度と緯度を読み込み、waypointリストに格納する。

        :param file_path: CSVファイルのパス
        :[[経度1, 緯度1], [経度2, 緯度2], ...] 形式のリストとして入力
        """
        try:
            with open(file_path, mode='r', encoding='utf-8') as file:
                csv_reader = csv.reader(file)
                for row in csv_reader:
                    # 空行をスキップ
                    if not row:
                        continue

                    # 緯度と経度をfloatに変換してリストに追加
                    try:
                        longitude = float(row[0])
                        latitude = float(row[1])
                        self.waypoint_list.append([longitude, latitude])
                    except ValueError as e:
                        print(f"Skipping row due to error: {row}, Error: {e}")
                # 最初の目標位置をセットする
                self.waypoint = self.waypoint_list[self.waypoint_num]

        except FileNotFoundError:
            print(f"File not found: {file_path}")
        except Exception as e:
            print(f"An error occurred: {e}")

    # ロボットの変形関数
    def transform_robot(self, mode):
        # 0: 直進モード, 1: カタマランモード, 2: キープモード
        if mode == 0:
            self.set_straight_mode()
        elif mode == 1:
            self.set_catamaran_mode()
        elif mode == 2:
            self.set_keeping_mode()
        else:
            print("Invalid mode")
    
    # ストレートモードへ変形
    def set_straight_mode(self):
        link_angle = math.pi/2
        thruster_angle = math.pi/4
        # リンク関節の制御
        self.link_joint1.setPosition(link_angle)
        self.link_joint2.setPosition(-link_angle)
        self.link_joint3.setPosition(link_angle)
        self.link_joint4.setPosition(-link_angle)

        # スラスタ関節の制御
        self.thruster_joint1.setPosition(thruster_angle)
        self.thruster_joint2.setPosition(-thruster_angle)
        self.thruster_joint3.setPosition(thruster_angle)
        self.thruster_joint4.setPosition(-thruster_angle)

    # カタマランモードへの変形
    def set_catamaran_mode(self):
        link_angle = math.pi/4
        thruster_angle = math.pi/2
        # リンク関節の制御
        self.link_joint1.setPosition(link_angle)
        self.link_joint2.setPosition(-link_angle)
        self.link_joint3.setPosition(link_angle)
        self.link_joint4.setPosition(-link_angle)

        # スラスタ関節の制御
        self.thruster_joint1.setPosition(thruster_angle)
        self.thruster_joint2.setPosition(-thruster_angle)
        self.thruster_joint3.setPosition(thruster_angle)
        self.thruster_joint4.setPosition(-thruster_angle)

    # キープモードへ変形
    def set_keeping_mode(self):
        link_angle = 0.0
        thruster_angle = 0.0
        # リンク関節の制御
        self.link_joint1.setPosition(link_angle)
        self.link_joint2.setPosition(link_angle)
        self.link_joint3.setPosition(link_angle)
        self.link_joint4.setPosition(link_angle)

        # スラスタ関節の制御
        self.thruster_joint1.setPosition(thruster_angle)
        self.thruster_joint2.setPosition(thruster_angle)
        self.thruster_joint3.setPosition(thruster_angle)
        self.thruster_joint4.setPosition(thruster_angle)

    
    # スラスタの速度を設定
    def set_thruster_velocity(self, thruster_direction, thrust):
        thruster_list = [self.thruster1, self.thruster2, self.thruster3, self.thruster4]
        i = 0
        for thruster in thruster_list:
            thruster.setVelocity(thruster_direction[i]*thrust)
            i += 1
    
    # ロボットの状態を更新
    def update_robot_state(self, thrust, thruster_direction):
        self.update_gps_noise()
        self.update_robot_position()
        self.update_compass_value()
        self.update_ampere(thrust, thruster_direction)
        self.update_power()
        self.update_speed()

    # ノイズありのGPS値を更新
    def update_gps_noise(self):
        gps_noise_value = self.gps_noise.getValues()
        self.gps_noise_value = [gps_noise_value[1], gps_noise_value[0]]

    # ノイズなしのGPS値を更新
    def update_robot_position(self):
        gps_value = self.gps.getValues()
        self.gps_value = [gps_value[1], gps_value[0]]
    
    # コンパスの値を更新
    def update_compass_value(self):
        self.compass_value = self.compass.getValues()
    
    def calc_distance(self, gps_value):
        try:
            current_position = [gps_value[0], gps_value[1]]
            distance = geodesic(current_position, self.waypoint).m
            print("Distance to target:", distance)
            return distance
        except Exception as e:
            print(f"An error occurred: {e}")


    def calc_angle(self, compass_value, gps_value, target_position):
        """
        現在位置、目標位置、コンパスの値から方位差を計算。
        :param compass_value: コンパスの値 [north, east, down]
        :param gps_value: 現在位置 [latitude, longitude] (度単位)
        :param target_position: 目標位置 [latitude, longitude] (度単位)
        :return: 方位差 (度単位, 範囲: -180 ~ 180)
        """
        # 1. コンパスから現在の向きを計算（ロボットのワールド座標系の方位）
        # Webotsの座標系に基づき atan2(東, 北) を使用
        current_heading = -math.degrees(math.atan2(compass_value[2], compass_value[0]))  # x=East, z=North
        current_heading = (current_heading - 90 + 360) % 360 # 0~360°に正規化

        # 2. 現在位置と目標位置の方位を計算
        lat1, lon1 = map(math.radians, gps_value)
        lat2, lon2 = map(math.radians, target_position)
        dlon = lon2 - lon1
        target_bearing = math.degrees(math.atan2(
            math.sin(dlon) * math.cos(lat2),
            math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
        ))
        target_bearing = (target_bearing + 360) % 360  # 0~360°に正規化
        # 3. 方位差を計算（-180 ~ 180°に正規化）
        diff_angle = (target_bearing - current_heading + 180) % 360 - 180

        print("diff_angle:", diff_angle)

        return diff_angle

    # ひとまず直進モードの制御関数のみを定義
    def straight_control(self, prev_distance, curr_distance, prev_angle, curr_angle):
        # PD制御による距離制御
        def PD_distance_control(prev_distance, curr_distance):
            distance_diff = curr_distance - prev_distance
            control_output = 0.0
            K_p = parameter.distance_Kp
            K_d = parameter.distance_Kd

            # 距離差によるPD制御
            control_output = K_p * curr_distance - K_d * distance_diff
            thrust = max(min(control_output, self.STRAIGHT_MAX_THRUST), 0)
            print("thrust:", thrust)
            return thrust
        
        def control_heading(angle, preve_angle, curr_angle):
            angle_diff = curr_angle - prev_angle
            control_output = 0.0
            K_p = parameter.bearing_Kp
            K_d = parameter.bearing_Kd

            if 0 <= angle < 90:
                angle = abs(angle - 180)
            elif -180 <= angle < -90:
                angle = abs(angle + 180)
            elif -90 <= angle < 0:
                angle = abs(angle)

            # 方位差によるPD制御
            control_output = K_p * angle - K_d * angle_diff
            balance = max(min(1 - (2 * (control_output / 90)), 1), -1)
            return balance

        thruster_directions = [1, 1, 1, 1]

        t = control_heading(curr_angle, prev_angle, curr_angle)
        if t <= 0.5:
            t = 0.5
        """
        # 頭側だけで操舵
        if 0 <= curr_angle < 90:
            print("Forward Counter-Clockwise")
            t = control_heading(curr_angle, angle_diff)
            thruster_directions = [1, -t, -1, 1]
        elif 90 <= curr_angle < 180:
            print("Backward Clockwise")
            t = control_heading(curr_angle, angle_diff)
            thruster_directions = [1, -1, -t, 1]
        elif -180 <= curr_angle < -90:
            print("Backward Counter-Clockwise")
            t = control_heading(curr_angle, angle_diff)
            thruster_directions = [1, -1, -1, t]
        elif -90 <= curr_angle < 0:
            print("Forward Clockwise")
            t = control_heading(curr_angle, angle_diff)
            thruster_directions = [t, -1, -1, 1]
        """
        # 頭側と尾側の両方で操舵
        if 0 <= curr_angle < 90:
            print("Forward Counter-Clockwise")
            thruster_directions = [1, -t, -t, 1]
        elif 90 <= curr_angle < 180:
            print("Backward Clockwise")
            thruster_directions = [1, -t, -t, 1]
        elif -180 <= curr_angle < -90:
            print("Backward Counter-Clockwise")
            thruster_directions = [t, -1, -1, t]
        elif -90 <= curr_angle < 0:
            print("Forward Clockwise")
            thruster_directions = [t, -1, -1, t]

        # ひとまず距離によるスラスタの強度決定はせず，常に最大スラスタを出力
        # -> スラスタの強度が強いと小回りが利かないため，目標値に近づくにつれてスラスタの強度を小さくする制御変更する必要ありかもしれない
        thrust = PD_distance_control(prev_distance, curr_distance)
        # thrust = self.STRAIGHT_MAX_THRUST

        return thruster_directions, thrust


    # 電流計算関数
    def calc_ampere(self, thrust, thruster_direction):
        sum_t_dir = 0.0
        for i in thruster_direction:
            sum_t_dir += abs(i)**2
        self.A = 1.5*(thrust/self.STRAIGHT_MAX_THRUST)*sum_t_dir

    def update_ampere(self, thrust, thruster_direction):
        self.calc_ampere(thrust, thruster_direction)
    
    # 電力計算関数
    def calc_power(self):
        self.W = self.A * self.V

    def update_power(self):
        self.calc_power()

    def get_gps_noise(self):
        return self.gps_noise_value
    
    def get_gps_value(self):
        return self.gps_value
    
    def update_speed(self):
        self.speed = self.gps.getSpeed()

    def get_speed(self):
        return self.speed

    def get_compass_value(self):
        return self.compass_value

    def get_ampere(self):
        return self.A

    def get_power(self):
        return self.W

    def get_waypoint(self):
        return self.waypoint

    def get_waypoint_num(self):
        return self.waypoint_num

    def update_waypoint(self):
        self.waypoint = self.waypoint_list[self.waypoint_num]

    def update_waypoint_num(self):
        if self.waypoint_num < len(self.waypoint_list) - 1:
            self.waypoint_num += 1
        else:
            self.waypoint_num = -1
    
    def get_BIWAKO_8_state(self):
        state = [self.get_gps_noise, self.get_gps_value(), self.get_compass_value(), self.get_ampere(), self.get_power(), self.get_waypoint(), self.get_waypoint_num()]
        return state

# ==========以下，シミュレータメイン関数==========

# ロボットのインスタンスを作成
biwako_8 = BIWAKO_8()

# ウェイポイントの読み込み
biwako_8.load_waypoints_from_csv(parameter.way_point_file)
# BIWAKO-8の初期化
biwako_8.init_device()

# 直進モードへ変形
mode = parameter.mode
biwako_8.transform_robot(mode) # 0: 直進モード, 1: カタマランモード, 2: キープモード

# 環境の初期化
TIME_STEP = 100
fluid_node = supervisor.getFromDef("STILL_WATER") # type Node
stream_vel = fluid_node.getField("streamVelocity") # type Field
def set_disturbance(x, z):
    st_vel = [x, 0.0, z]
    stream_vel.setSFVec3f(st_vel)
set_disturbance(0.0, 0.0)

log_data = []
timestamp = 0.0

thrust = 0.0
thruster_direction = [0, 0, 0, 0]

prev_distance = 0.0
curr_distance = 0.0
prev_angle = 0.0
curr_angle = 0.0

if __name__ == "__main__":
    while supervisor.step(TIME_STEP) != -1:
        # 0. BIWAKO-8の状態を更新
        biwako_8.update_robot_state(thrust, thruster_direction)

        print("speed:", biwako_8.get_speed())
        # - BIWAKO-8の状態のログをとる
        biwako_8_state = biwako_8.get_BIWAKO_8_state()
        # - ログデータに追加
        log_data.append(biwako_8_state)

        # 1. センサー値の取得
        # - GPS値を取得（現在位置計算用）
        gps_noise_value = biwako_8.get_gps_noise()
        # - 補助GPS値を取得
        gps_value = biwako_8.get_gps_value()
        # - コンパス値を取得（現在の方位を計算用）
        compass_value = biwako_8.get_compass_value()

        # 2. 距離と角度の計算
        # - 現在位置と目標位置間の距離を計算
        prev_distance = curr_distance
        curr_distance = biwako_8.calc_distance(gps_noise_value)
        # print("distance:", distance)
        # - 現在の向きと目標位置の方位差を計算
        prev_angle = curr_angle
        curr_angle = biwako_8.calc_angle(compass_value, gps_value, biwako_8.get_waypoint())

        # 3. 行動の判断
        # - 目標位置に到達した場合
        if curr_distance < parameter.main_distance_tolerance:
            # - 次の目標位置に進む
            biwako_8.update_waypoint_num()
            biwako_8.update_waypoint()

            # - すべての目標に到達したら終了処理を実行
            if biwako_8.get_waypoint_num() == -1:
                biwako_8.set_keeping_mode()
                biwako_8.set_thruster_velocity([0, 0, 0, 0], 0.0)
                break
        else:
        #   - 目標に向かって移動する制御を実行
            thruster_direction, thrust = biwako_8.straight_control(prev_distance,curr_distance, prev_angle, curr_angle)
            # print("thruster_direction:", thruster_direction)
            # print("thrust:", thrust)
            biwako_8.set_thruster_velocity(thruster_direction, thrust)

        # 8. タイムスタンプの更新
        # - 現在のシミュレーション時間を更新
        timestamp += TIME_STEP/1000