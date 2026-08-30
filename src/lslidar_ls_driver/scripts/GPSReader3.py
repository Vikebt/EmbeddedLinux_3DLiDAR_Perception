# gps_reader.py
import time
import serial
import re
from contextlib import contextmanager
from typing import Optional, Dict
import logging

# 配置日志记录器
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')
gps_t = 0

class GPSData:
    """GPS数据容器类"""
    def __init__(self):
        self.utctime: str = ''

        self.latitude: float = 0.0
        self.longitude: float = 0.0
        self.altitude: float = 0.0

        self.satellites: int = 0

        self.speed_knots: float = 0.0
        self.speed_kph: float = 0.0

        self.course_true: float = 0.0
        self.course_mag : float = 0.0

        self.true_north: float = 0.0
        self.magnetic_north: float = 0.0
        
        self.unitlat: str = ''
        self.unitlon: str = ''

    def __str__(self):
        return (
            f"UTC Time: {self.utctime}\n"
            f"Latitude: {self.latitude:.6f}\n"
            f"Longitude: {self.longitude:.6f}\n"
            f"Satellites: {self.satellites}\n"
            f"Altitude: {self.altitude:.1f}m\n"
            f"Speed: {self.speed_kph:.1f} km/h\n"
            f"Course (True): {self.course_true:.1f}°\n"
            f"Course (Mag): {self.course_mag:.1f}°"
        )
def Convert_to_degrees(in_data1, in_data2):
    len_data1 = len(in_data1)
    str_data2 = "%05d" % int(in_data2)
    temp_data = int(in_data1)
    symbol = 1
    if temp_data < 0:
        symbol = -1
    degree = int(temp_data / 100.0)
    str_decimal = str(in_data1[len_data1-2]) + str(in_data1[len_data1-1]) + str(str_data2)
    f_degree = int(str_decimal)/60.0/100000.0
    # print("f_degree:", f_degree)
    if symbol > 0:
        result = degree + f_degree
    else:
        result = degree - f_degree
    return result


class GPSReader:
    """GPS数据读取解析类"""
    
    GGA_PATTERN = re.compile(
        r"\$GNGGA,"
        r"(\d{6}\.\d{2}),"      # UTC时间
        r"(\d+\.\d+),([NS]),"   # 纬度
        r"(\d+\.\d+),([EW]),"   # 经度
        r"\d,\d+,(\d+),"        # 卫星数量
        r"(-?\d+\.?\d*),"       # 海拔高度
        r"M.*"
    )
    
    VTG_PATTERN = re.compile(
        r"\$GNVTG,"
        r"(\d+\.\d+),T,"        # 真北航向
        r"(\d+\.\d+),M,"        # 磁北航向
        r"(\d+\.\d+),N,"        # 速度（节）
        r"(\d+\.\d+),K"         # 速度（公里/小时）
    )

    # def __init__(self, port: str = "/dev/wheeltec_gps", baudrate: int = 9600):
    #     self.port = port
    #     self.baudrate = baudrate
    #     self.ser: Optional[serial.Serial] = None
    #     self.buffer = bytearray()
    #     self.current_data = GPSData()
    def __init__(self, port="/dev/wheeltec_gps", baud_rate= 9600):
        self.ser = serial.Serial(port, baud_rate, timeout=0.002)
        self.port = port
        self.baudrate = baud_rate
        # self.ser: Optional[serial.Serial] = None
        self.buffer = bytearray()
        self.current_data =  {
            "utctime": '',
            "latitude": 0.0 ,
            "longitude": 0.0,
            "altitude": 0.0,

            "satellites": 0,

            "speed_knots": 0.0,
            "speed_kph": 0.0,

            "course_true": 0.0,
            "course_mag" : 0.0,

            "true_north": 0.0,
            "magnetic_north": 0.0,
            
            "unitlat":  '',
            "unitlon":  '',
        }
        # self.lock = threading.Lock()

    def GPS_read(self):
            global gps_t
            if self.ser.inWaiting():
                if self.ser.read(1) == b'G':
                    time.sleep(.05) 
                    if self.ser.inWaiting():
                        if self.ser.read(1) == b'N':
                            if self.ser.inWaiting():
                                choice = self.ser.read(1)
                                if choice == b'G':
                                    if self.ser.inWaiting():
                                        if self.ser.read(1) == b'G':
                                            if self.ser.inWaiting():
                                                if self.ser.read(1) == b'A':
                                                    #utctime = self.ser.read(7)
                                                    GGA = self.ser.read(70)
                                                    GGA_g = re.findall(r"\w+(?=,)|(?<=,)\w+", str(GGA))
                                                    # print(GGA_g)
                                                    if len(GGA_g) < 13:
                                                        # print("GPS no found")
                                                        gps_t = 0
                                                        return False
                                                    else:
                                                        # utctime = GGA_g[0]
                                                        # # lat = GGA_g[2][0]+GGA_g[2][1]+'°'+GGA_g[2][2]+GGA_g[2][3]+'.'+GGA_g[3]+'\''
                                                        # lat = "%.8f" % Convert_to_degrees(str(GGA_g[2]), str(GGA_g[3]))
                                                        # ulat = GGA_g[4]
                                                        # # lon = GGA_g[5][0]+GGA_g[5][1]+GGA_g[5][2]+'°'+GGA_g[5][3]+GGA_g[5][4]+'.'+GGA_g[6]+'\''
                                                        # lon = "%.8f" % Convert_to_degrees(str(GGA_g[5]), str(GGA_g[6]))
                                                        # ulon = GGA_g[7]
                                                        # numSv = GGA_g[9]
                                                        # msl = GGA_g[12]+'.'+GGA_g[13]+GGA_g[14]

                                                        self.current_data['utctime'] = GGA_g[0]
                                                        self.current_data['latitude'] = "%.8f" % Convert_to_degrees(str(GGA_g[2]), str(GGA_g[3]))
                                                        self.current_data['longitude'] = "%.8f" % Convert_to_degrees(str(GGA_g[5]), str(GGA_g[6]))
                                                        self.current_data['satellites'] = GGA_g[9]
                                                        self.current_data['altitude'] = GGA_g[12]+'.'+GGA_g[13]+GGA_g[14]
                                                        
                                                        self.current_data['unitlat'] = GGA_g[4]
                                                        self.current_data['unitlon'] = GGA_g[7]

                                                        gps_t = 1
                                                        return True
                                elif choice == b'V':
                                    if self.ser.inWaiting():
                                        if self.ser.read(1) == b'T':
                                            if self.ser.inWaiting():
                                                if self.ser.read(1) == b'G':
                                                    if gps_t == 1:
                                                        VTG = self.ser.read(40)
                                                        VTG_g = re.findall(r"\w+(?=,)|(?<=,)\w+", str(VTG))
                                                        self.current_data['true_north'] = VTG_g[0]+'.'+VTG_g[1]+'T'
                                                        if VTG_g[3] == 'M':
                                                            self.current_data['magnetic_north'] = '0.00'
                                                            self.current_data['speed_knots']= VTG_g[4]+'.'+VTG_g[5]
                                                            self.current_data['speed_kph'] = VTG_g[7]+'.'+VTG_g[8]
                                                        elif VTG_g[3] != 'M':
                                                            self.current_data['magnetic_north'] = VTG_g[3]+'.'+VTG_g[4]
                                                            self.current_data['speed_knots']= VTG_g[6]+'.'+VTG_g[7]
                                                            self.current_data['speed_kph'] = VTG_g[9]+'.'+VTG_g[10]
                                                    return True    
            return False
    @staticmethod
    def _dms_to_dd(degrees: str, minutes: str, direction: str) -> float:
        """度分秒转换为十进制度数"""
        dd = float(degrees[:2]) + float(degrees[2:])/60 + float(minutes)/3600
        return dd if direction in ['N', 'E'] else -dd

    def connect(self):
        """连接串口"""
        if not self.ser or not self.ser.is_open:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            self.ser.reset_input_buffer()

    def disconnect(self):
        """断开串口连接"""
        if self.ser and self.ser.is_open:
            self.ser.close()

    @contextmanager
    def connection(self):
        """上下文管理器"""
        try:
            self.connect()
            yield self
        finally:
            self.disconnect()

    def read_data(self) -> bool:
        """读取并解析GPS数据"""
        if not self.ser or not self.ser.is_open:
            return False

        # 读取并缓存数据
        self.buffer += self.ser.read_all()

        # 查找完整消息
        while b'\r\n' in self.buffer:
            frame, self.buffer = self.buffer.split(b'\r\n', 1)
            frame_str = frame.decode('ascii', errors='ignore').strip()
            
            if frame_str.startswith('$GNGGA'):
                self._parse_gga(frame_str)
                return True
            elif frame_str.startswith('$GNVTG'):
                self._parse_vtg(frame_str)
                return True
        return False
    
    def read_data_log(self) -> bool:
        """读取并解析GPS数据"""
        logging.debug("开始读取GPS数据")
        if not self.ser or not self.ser.is_open:
            logging.warning("串口未打开，无法读取数据")
            return False

        # 读取并缓存数据
        data = self.ser.read_all()
        logging.debug(f"读取到的数据: {data}")
        self.buffer += data

        # 查找完整消息
        while b'\r\n' in self.buffer:
            frame, self.buffer = self.buffer.split(b'\r\n', 1)
            frame_str = frame.decode('ascii', errors='ignore').strip()
            logging.debug(f"解析帧数据: {frame_str}")
            
            if frame_str.startswith('$GPGGA'):
                logging.info("找到GGA帧，开始解析")
                self._parse_gga(frame_str)
                return True
            elif frame_str.startswith('$GPVTG'):
                logging.info("找到VTG帧，开始解析")
                self._parse_vtg(frame_str)
                return True
        logging.debug("未找到完整消息")
        return False
    
    def update(self):
        return self.GPS_read()
    
    # def _parse_gga(self, frame: str):
    #     """解析GGA语句"""
    #     match = self.GGA_PATTERN.match(frame)
    #     if match:
    #         groups = match.groups()
    #         try:
    #             self.current_data.utctime = f"{groups[0][:2]}:{groups[0][2:4]}:{groups[0][4:6]}"
    #             self.current_data.latitude = self._dms_to_dd(groups[1][:2], groups[1][2:], groups[2])
    #             self.current_data.longitude = self._dms_to_dd(groups[3][:3], groups[3][3:], groups[4])
    #             self.current_data.satellites = int(groups[5])
    #             self.current_data.altitude = float(groups[6])
    #         except (ValueError, IndexError) as e:
    #             print(f"GGA解析错误: {e}")

    # def _parse_vtg(self, frame: str):
    #     """解析VTG语句"""
    #     match = self.VTG_PATTERN.match(frame)
    #     if match:
    #         groups = match.groups()
    #         try:
    #             self.current_data.course_true = float(groups[0])
    #             self.current_data.course_mag = float(groups[1])
    #             self.current_data.speed_knots = float(groups[2])
    #             self.current_data.speed_kph = float(groups[3])
    #         except (ValueError, IndexError) as e:
    #             print(f"VTG解析错误: {e}")

    @property
    def data(self) -> GPSData:
        """获取当前GPS数据"""
        return self.current_data
    def get_data(self) -> GPSData:
        """获取当前GPS数据"""
        # with self.lock:
        return self.current_data
        
# 使用示例
if __name__ == "__main__":
    # with GPSReader(port="/dev/wheeltec_gps", baud_rate= 9600).connection() as gps:
    #     while True:
    #         if gps.read_data():
    #             print("\n最新GPS数据:")
    #             print(gps.data)
    #             print("-" * 40)
    try:
        gps = GPSReader(port="/dev/wheeltec_gps", baud_rate= 9600)
        print("GPS Serial Opened! Baudrate=9600")
        while True:
            if gps.update():
                data = gps.get_data()
                print("\n最新GPS数据:")
                print("-" * 40)
                print(f"纬度: {data.latitude}")
                print("-" * 40)

            time.sleep(0.005)
    except Exception as e:
        print(f"Error: {e}")