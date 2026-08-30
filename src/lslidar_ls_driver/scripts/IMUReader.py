# -*- encoding: utf-8 -*-
import serial
import time

class IMUReader:
    # Protocol constants
    PROTOCOL_MIN_LEN = 7
    PROTOCOL_TID_LEN = 2
    PROTOCOL_PAYLOAD_LEN = 1
    PROTOCOL_CHECKSUM_LEN = 2
    PROTOCOL_TID_POS = 2
    PROTOCOL_PAYLOAD_LEN_POS = 4
    CRC_CALC_START_POS = 2
    PAYLOAD_POS = 5
    TLV_HEADER_LEN = 2

    # Data IDs
    SENSOR_TEMP_ID = 0x01
    ACC_ID = 0x10
    GYRO_ID = 0x20
    NORM_MAG_ID = 0x30
    RAW_MAG_ID = 0x31
    EULER_ID = 0x40
    QUATERNION_ID = 0x41
    UTC_ID = 0x50
    SAMP_TIMESTAMP_ID = 0x51
    DATAREADY_TIMESTAMP_ID = 0x52
    LOCATION_ID = 0x68
    SPEED_ID = 0x70
    STATUS_ID = 0x80

    # Data lengths
    SENSOR_TEMP_LEN = 0x02
    ACC_LEN = 0x0C
    GYRO_LEN = 0x0C
    NORM_MAG_LEN = 0x0C
    RAW_MAG_LEN = 0x0C
    EULER_LEN = 0x0C
    QUATERNION_LEN = 0x10
    UTC_LEN = 0x0B
    SAMP_TIMESTAMP_LEN = 0x04
    DATAREADY_TIMESTAMP_LEN = 0x04
    LOCATION_LEN = 0x14
    SPEED_LEN = 0x0C
    STATUS_LEN = 0x01

    # Data conversion factors
    DATA_FACTOR_NOT_RAW_MAG = 0.000001
    DATA_FACTOR_RAW_MAG = 0.001
    DATA_FACTOR_SENSOR_TEMP = 0.01
    DATA_FACTOR_HIGH_RES_LOC = 0.0000000001
    DATA_FACTOR_ALT = 0.001
    DATA_FACTOR_SPEED = 0.001

    def __init__(self, port, baud_rate):
        self.ser = serial.Serial(port, baud_rate, timeout=0.002)
        self.buf = [0] * 512
        self.buf_len = 0
        self.data = {
            'tid': 1, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
            'q0': 1.0, 'q1': 0.0, 'q2': 0.0, 'q3': 0.0,
            'sensor_temp': 25.0, 'acc_x': 0.0, 'acc_y': 0.0, 'acc_z': 1.0,
            'gyro_x': 0.0, 'gyro_y': 0.0, 'gyro_z': 0.0,
            'norm_mag_x': 0.0, 'norm_mag_y': 0.0, 'norm_mag_z': 0.0,
            'raw_mag_x': 0.0, 'raw_mag_y': 0.0, 'raw_mag_z': 0.0,
            'lat': 0.0, 'longt': 0.0, 'alt': 0.0,
            'vel_e': 0.0, 'vel_n': 0.0, 'vel_u': 0.0,
            'ms': 0, 'year': 2022, 'month': 8, 'day': 31,
            'hour': 12, 'minute': 0, 'second': 0,
            'samp_timestamp': 0, 'dataready_timestamp': 0, 'status': 0
        }
        # self.lock = threading.Lock()

    def update(self):
        """Read data from serial port and update IMU data"""
        data = self.ser.read_all()
        num = len(data)
        if num > 0:
            data_list = list(data)
            self._decode_data(data_list, num)
        #     return  True
        # else:
        #     return False

    def get_data(self):
        """Return a copy of the current IMU data"""
        # with self.lock:
        return self.data

    def _decode_data(self, data, num, debug_flg=False):
        pos = 0
        cnt = 0
        data_len = 0

        # Update buffer
        if self.buf_len + num > len(self.buf):
            self._clear_data(self.buf_len)
            return False
        self.buf[self.buf_len:self.buf_len + num] = data[:num]
        self.buf_len += num

        if self.buf_len < self.PROTOCOL_MIN_LEN:
            return False

        cnt = self.buf_len

        # Find header
        while cnt > 0:
            if self.buf[pos] == 0x59 and self.buf[pos + 1] == 0x53:
                break
            cnt -= 1
            pos += 1

        if cnt < self.PROTOCOL_MIN_LEN:
            self._clear_data(pos)
            return False

        data_len = self.buf[pos + self.PROTOCOL_PAYLOAD_LEN_POS]
        total_len_needed = self.PROTOCOL_MIN_LEN + data_len

        if total_len_needed > cnt:
            self._clear_data(pos)
            return False

        # Calculate checksum
        crc_calc_len = self._crc_calc_len(data_len)
        data_for_crc = self.buf[pos + self.CRC_CALC_START_POS : pos + self.CRC_CALC_START_POS + crc_calc_len]
        computed_checksum = self._calc_checksum(data_for_crc, crc_calc_len)
        crc_pos = pos + self._protocol_crc_data_pos(data_len)
        received_checksum = (self.buf[crc_pos + 1] << 8) | self.buf[crc_pos]

        if computed_checksum != received_checksum:
            self._clear_data(pos + total_len_needed)
            return False

        # Parse TID
        self.data['tid'] = (self.buf[pos + self.PROTOCOL_TID_POS + 1] << 8) | self.buf[pos + self.PROTOCOL_TID_POS]

        # Parse payload
        pos_payload = pos + self.PAYLOAD_POS
        remaining = data_len

        while remaining > 0 and pos_payload < self.buf_len:
            if pos_payload + self.TLV_HEADER_LEN > self.buf_len:
                break

            tlv_id = self.buf[pos_payload]
            tlv_len = self.buf[pos_payload + 1]
            payload_start = pos_payload + self.TLV_HEADER_LEN
            payload_end = payload_start + tlv_len

            if payload_end > self.buf_len:
                break

            tlv = {'id': tlv_id, 'len': tlv_len}
            payload = self.buf[payload_start:payload_end]
            ret = self._parse_data_by_id(tlv, payload, debug_flg)

            if ret:
                pos_payload += self.TLV_HEADER_LEN + tlv_len
                remaining -= self.TLV_HEADER_LEN + tlv_len
            else:
                pos_payload += 1
                remaining -= 1

        # Clear processed data
        self._clear_data(pos + total_len_needed)
        return True

    def _parse_data_by_id(self, tlv, payload, debug_flg):
        try:
            if tlv['id'] == self.SENSOR_TEMP_ID and tlv['len'] == self.SENSOR_TEMP_LEN:
                self.data['sensor_temp'] = self._get_int16_lit(payload) * self.DATA_FACTOR_SENSOR_TEMP

            elif tlv['id'] == self.ACC_ID and tlv['len'] == self.ACC_LEN:
                self.data['acc_x'] = self._get_int32_lit(payload[0:4]) * self.DATA_FACTOR_NOT_RAW_MAG
                self.data['acc_y'] = self._get_int32_lit(payload[4:8]) * self.DATA_FACTOR_NOT_RAW_MAG
                self.data['acc_z'] = self._get_int32_lit(payload[8:12]) * self.DATA_FACTOR_NOT_RAW_MAG

            elif tlv['id'] == self.GYRO_ID and tlv['len'] == self.GYRO_LEN:
                self.data['gyro_x'] = self._get_int32_lit(payload[0:4]) * self.DATA_FACTOR_NOT_RAW_MAG
                self.data['gyro_y'] = self._get_int32_lit(payload[4:8]) * self.DATA_FACTOR_NOT_RAW_MAG
                self.data['gyro_z'] = self._get_int32_lit(payload[8:12]) * self.DATA_FACTOR_NOT_RAW_MAG

            elif tlv['id'] == self.EULER_ID and tlv['len'] == self.EULER_LEN:
                self.data['pitch'] = self._get_int32_lit(payload[0:4]) * self.DATA_FACTOR_NOT_RAW_MAG
                self.data['roll'] = self._get_int32_lit(payload[4:8]) * self.DATA_FACTOR_NOT_RAW_MAG
                self.data['yaw'] = self._get_int32_lit(payload[8:12]) * self.DATA_FACTOR_NOT_RAW_MAG

            elif tlv['id'] == self.QUATERNION_ID and tlv['len'] == self.QUATERNION_LEN:
                self.data['q0'] = self._get_int32_lit(payload[0:4]) * self.DATA_FACTOR_NOT_RAW_MAG
                self.data['q1'] = self._get_int32_lit(payload[4:8]) * self.DATA_FACTOR_NOT_RAW_MAG
                self.data['q2'] = self._get_int32_lit(payload[8:12]) * self.DATA_FACTOR_NOT_RAW_MAG
                self.data['q3'] = self._get_int32_lit(payload[12:16]) * self.DATA_FACTOR_NOT_RAW_MAG

            elif tlv['id'] == self.NORM_MAG_ID and tlv['len'] == self.NORM_MAG_LEN:
                self.data['norm_mag_x'] = self._get_int32_lit(payload[0:4]) * self.DATA_FACTOR_NOT_RAW_MAG
                self.data['norm_mag_y'] = self._get_int32_lit(payload[4:8]) * self.DATA_FACTOR_NOT_RAW_MAG
                self.data['norm_mag_z'] = self._get_int32_lit(payload[8:12]) * self.DATA_FACTOR_NOT_RAW_MAG

            elif tlv['id'] == self.RAW_MAG_ID and tlv['len'] == self.RAW_MAG_LEN:
                self.data['raw_mag_x'] = self._get_int32_lit(payload[0:4]) * self.DATA_FACTOR_RAW_MAG
                self.data['raw_mag_y'] = self._get_int32_lit(payload[4:8]) * self.DATA_FACTOR_RAW_MAG
                self.data['raw_mag_z'] = self._get_int32_lit(payload[8:12]) * self.DATA_FACTOR_RAW_MAG

            else:
                return False
            return True
        except Exception as e:
            if debug_flg:
                print(f"Parse error: {e}")
            return False

    def _clear_data(self, clr_len):
        if clr_len <= 0:
            return
        remaining = self.buf_len - clr_len
        if remaining > 0:
            self.buf[:remaining] = self.buf[clr_len:clr_len + remaining]
            self.buf[remaining:] = [0] * (len(self.buf) - remaining)
        else:
            self.buf = [0] * 512
            remaining = 0
        self.buf_len = remaining

    @staticmethod
    def _crc_calc_len(payload_len):
        return payload_len + 3  # TID(2) + Len(1) + payload

    def _protocol_crc_data_pos(self, payload_len):
        return self.CRC_CALC_START_POS + self._crc_calc_len(payload_len)

    @staticmethod
    def _calc_checksum(data, length):
        check_a = 0
        check_b = 0
        for byte in data[:length]:
            check_a = (check_a + byte) % 256
            check_b = (check_b + check_a) % 256
        return (check_b << 8) | check_a

    @staticmethod
    def _get_int16_lit(data):
        return int.from_bytes(bytes(data[:2]), byteorder='little', signed=True)

    @staticmethod
    def _get_int32_lit(data):
        return int.from_bytes(bytes(data[:4]), byteorder='little', signed=True)

    @staticmethod
    def _get_int64_lit(data):
        return int.from_bytes(bytes(data[:8]), byteorder='little', signed=True)
    
    def connection(self):
        """上下文管理器"""
        try:
            self.connect()
            yield self
        finally:
            self.disconnect()
            
    def disconnect(self):
        """断开串口连接"""
        if self.ser and self.ser.is_open:
            self.ser.close()

if __name__ == '__main__':
    try:
        imu = IMUReader('/dev/wheeltec_IMU', 460800)
        while True:
            imu.update()
            data = imu.get_data()
            print(f"Roll: {data['roll']:.4f}, Pitch: {data['pitch']:.4f}, Yaw: {data['yaw']:.4f}")
            time.sleep(0.005)
    except Exception as e:
        print(f"Error: {e}")