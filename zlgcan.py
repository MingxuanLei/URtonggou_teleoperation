# 更新于 250925
from ctypes import *
import platform

ZCAN_DEVICE_TYPE = c_uint
ZCAN_RET_STATUS = c_uint
ZCAN_UDS_DATA_DEF = c_uint

INVALID_DEVICE_HANDLE = 0               # 无效设备句柄
INVALID_CHANNEL_HANDLE = 0              # 无效通道句柄
INVALID_LIN_SCHE_HANDLE = 2 ** 32 - 1   # 无效LIN调度表句柄

# 设备类型号
ZCAN_PCI5121 = ZCAN_DEVICE_TYPE(1)
ZCAN_PCI9810 = ZCAN_DEVICE_TYPE(2)
ZCAN_USBCAN1 = ZCAN_DEVICE_TYPE(3)
ZCAN_USBCAN2 = ZCAN_DEVICE_TYPE(4)
ZCAN_PCI9820 = ZCAN_DEVICE_TYPE(5)
ZCAN_CAN232 = ZCAN_DEVICE_TYPE(6)
ZCAN_PCI5110 = ZCAN_DEVICE_TYPE(7)
ZCAN_CANLITE = ZCAN_DEVICE_TYPE(8)
ZCAN_ISA9620 = ZCAN_DEVICE_TYPE(9)
ZCAN_ISA5420 = ZCAN_DEVICE_TYPE(10)
ZCAN_PC104CAN = ZCAN_DEVICE_TYPE(11)
ZCAN_CANETUDP = ZCAN_DEVICE_TYPE(12)
ZCAN_CANETE = ZCAN_DEVICE_TYPE(12)
ZCAN_DNP9810 = ZCAN_DEVICE_TYPE(13)
ZCAN_PCI9840 = ZCAN_DEVICE_TYPE(14)
ZCAN_PC104CAN2 = ZCAN_DEVICE_TYPE(15)
ZCAN_PCI9820I = ZCAN_DEVICE_TYPE(16)
ZCAN_CANETTCP = ZCAN_DEVICE_TYPE(17)
ZCAN_PCIE_9220 = ZCAN_DEVICE_TYPE(18)
ZCAN_PCI5010U = ZCAN_DEVICE_TYPE(19)
ZCAN_USBCAN_E_U = ZCAN_DEVICE_TYPE(20)
ZCAN_USBCAN_2E_U = ZCAN_DEVICE_TYPE(21)
ZCAN_PCI5020U = ZCAN_DEVICE_TYPE(22)
ZCAN_EG20T_CAN = ZCAN_DEVICE_TYPE(23)
ZCAN_PCIE9221 = ZCAN_DEVICE_TYPE(24)
ZCAN_WIFICAN_TCP = ZCAN_DEVICE_TYPE(25)
ZCAN_WIFICAN_UDP = ZCAN_DEVICE_TYPE(26)
ZCAN_PCIe9120 = ZCAN_DEVICE_TYPE(27)
ZCAN_PCIe9110 = ZCAN_DEVICE_TYPE(28)
ZCAN_PCIe9140 = ZCAN_DEVICE_TYPE(29)
ZCAN_USBCAN_4E_U = ZCAN_DEVICE_TYPE(31)
ZCAN_CANDTU_200UR = ZCAN_DEVICE_TYPE(32)
ZCAN_CANDTU_MINI = ZCAN_DEVICE_TYPE(33)
ZCAN_USBCAN_8E_U = ZCAN_DEVICE_TYPE(34)
ZCAN_CANREPLAY = ZCAN_DEVICE_TYPE(35)
ZCAN_CANDTU_NET = ZCAN_DEVICE_TYPE(36)
ZCAN_CANDTU_100UR = ZCAN_DEVICE_TYPE(37)
ZCAN_PCIE_CANFD_100U = ZCAN_DEVICE_TYPE(38)
ZCAN_PCIE_CANFD_200U = ZCAN_DEVICE_TYPE(39)
ZCAN_PCIE_CANFD_400U = ZCAN_DEVICE_TYPE(40)
ZCAN_USBCANFD_200U = ZCAN_DEVICE_TYPE(41)
ZCAN_USBCANFD_100U = ZCAN_DEVICE_TYPE(42)
ZCAN_USBCANFD_MINI = ZCAN_DEVICE_TYPE(43)
ZCAN_CANFDCOM_100IE = ZCAN_DEVICE_TYPE(44)
ZCAN_CANSCOPE = ZCAN_DEVICE_TYPE(45)
ZCAN_CLOUD = ZCAN_DEVICE_TYPE(46)
ZCAN_CANDTU_NET_400 = ZCAN_DEVICE_TYPE(47)
ZCAN_CANFDNET_200U_TCP = ZCAN_DEVICE_TYPE(48)
ZCAN_CANFDNET_200U_UDP = ZCAN_DEVICE_TYPE(49)
ZCAN_CANFDWIFI_100U_TCP = ZCAN_DEVICE_TYPE(50)
ZCAN_CANFDWIFI_100U_UDP = ZCAN_DEVICE_TYPE(51)
ZCAN_CANFDNET_400U_TCP = ZCAN_DEVICE_TYPE(52)
ZCAN_CANFDNET_400U_UDP = ZCAN_DEVICE_TYPE(53)
ZCAN_CANFDBLUE_200U = ZCAN_DEVICE_TYPE(54)
ZCAN_CANFDNET_100U_TCP = ZCAN_DEVICE_TYPE(55)
ZCAN_CANFDNET_100U_UDP = ZCAN_DEVICE_TYPE(56)
ZCAN_CANFDNET_800U_TCP = ZCAN_DEVICE_TYPE(57)
ZCAN_CANFDNET_800U_UDP = ZCAN_DEVICE_TYPE(58)
ZCAN_USBCANFD_800U = ZCAN_DEVICE_TYPE(59)
ZCAN_PCIE_CANFD_100U_EX = ZCAN_DEVICE_TYPE(60)
ZCAN_PCIE_CANFD_400U_EX = ZCAN_DEVICE_TYPE(61)
ZCAN_PCIE_CANFD_200U_MINI = ZCAN_DEVICE_TYPE(62)
ZCAN_PCIE_CANFD_200U_M2 = ZCAN_DEVICE_TYPE(63)
ZCAN_PCIE_CANFD_200U_EX = ZCAN_DEVICE_TYPE(62)
ZCAN_CANFDDTU_400_TCP = ZCAN_DEVICE_TYPE(64)
ZCAN_CANFDDTU_400_UDP = ZCAN_DEVICE_TYPE(65)
ZCAN_CANFDWIFI_200U_TCP = ZCAN_DEVICE_TYPE(66)
ZCAN_CANFDWIFI_200U_UDP = ZCAN_DEVICE_TYPE(67)
ZCAN_CANFDDTU_800ER_TCP = ZCAN_DEVICE_TYPE(68)
ZCAN_CANFDDTU_800ER_UDP = ZCAN_DEVICE_TYPE(69)
ZCAN_CANFDDTU_800EWGR_TCP = ZCAN_DEVICE_TYPE(70)
ZCAN_CANFDDTU_800EWGR_UDP = ZCAN_DEVICE_TYPE(71)
ZCAN_CANFDDTU_600EWGR_TCP = ZCAN_DEVICE_TYPE(72)
ZCAN_CANFDDTU_600EWGR_UDP = ZCAN_DEVICE_TYPE(73)
ZCAN_CANFDDTU_CASCADE_TCP = ZCAN_DEVICE_TYPE(74)
ZCAN_CANFDDTU_CASCADE_UDP = ZCAN_DEVICE_TYPE(75)
ZCAN_USBCANFD_400U = ZCAN_DEVICE_TYPE(76)
ZCAN_CANFDDTU_200U = ZCAN_DEVICE_TYPE(77)
ZCAN_CANFDBRIDGE_PLUS = ZCAN_DEVICE_TYPE(80)
ZCAN_CANFDDTU_300U = ZCAN_DEVICE_TYPE(81)
ZCAN_VIRTUAL_DEVICE = ZCAN_DEVICE_TYPE(99)

# ZCAN_RET_STATUS 返回值
ZCAN_STATUS_ERR = 0
ZCAN_STATUS_OK = 1
ZCAN_STATUS_ONLINE = 2
ZCAN_STATUS_OFFLINE = 3
ZCAN_STATUS_UNSUPPORTED = 4

# 帧类型 GetReceivenum参数
ZCAN_TYPE_CAN = c_uint(0)
ZCAN_TYPE_CANFD = c_uint(1)
ZCAN_TYPE_MERGE = c_uint(2)

# 数据类型 eZCANDataDEF    合并接收结构体类型
ZCAN_DT_ZCAN_CAN_CANFD_DATA = 1     # CAN/CANFD数据
ZCAN_DT_ZCAN_ERROR_DATA     = 2     #错误数据
ZCAN_DT_ZCAN_GPS_DATA       = 3     #GPS数据
ZCAN_DT_ZCAN_LIN_DATA       = 4     #LIN数据
ZCAN_DT_ZCAN_BUSUSAGE_DATA  = 5     #BusUsage数据
ZCAN_DT_ZCAN_LIN_ERROR_DATA = 6     #LIN错误数据
ZCAN_DT_ZCAN_LIN_EX_DATA    = 7     #LIN扩展数据
ZCAN_DT_ZCAN_LIN_EVENT_DATA = 8     #LIN事件数据

# 发送延时单位
ZCAN_TX_DELAY_NO_DELAY   = 0        #无发送延时
ZCAN_TX_DELAY_UNIT_MS    = 1        #发送延时单位毫秒
ZCAN_TX_DELAY_UNIT_100US = 2        #发送延时单位100微秒(0.1毫秒)

# LIN 校验和类型
DEFAULT = 0         # 默认，启动时配置
CLASSIC_CHKSUM = 1  # 经典校验
ENHANCE_CHKSUM = 2  # 增强校验
AUTOMATIC = 3       # 自动，设备自动识别校验方式（仅ZCAN_SetLINSubscribe时有效）

# LIN事件类型
ZCAN_LIN_WAKE_UP = 1
ZCAN_LIN_ENTERED_SLEEP_MODE = 2
ZCAN_LIN_EXITED_SLEEP_MODE = 3
ZCAN_LIN_SWITCH_SCHED = 4

# LIN调度表状态
ZCAN_LIN_SCHED_STATUS_IDLE = 0  # 空闲
ZCAN_LIN_SCHED_STATUS_RUN  = 1  # 正在运行

# LIN帧类型
ZCAN_LIN_FRAME_UNCONDITIONAL = 0    # 无条件帧
ZCAN_LIN_FRAME_EVENT = 1            # 事件触发帧
ZCAN_LIN_FRAME_SPORADIC = 2         # 偶发帧
ZCAN_LIN_FRAME_MST_REQ = 3          # 诊断主机请求帧
ZCAN_LIN_FRAME_SLV_RESP = 4         # 诊断从机应答帧
ZCAN_LIN_FRAME_RESERVED = 5         # 保留帧

#UDS传输协议版本
ZCAN_UDS_TRANS_VER_0 = 0  # ISO15765-2(2004版本)
ZCAN_UDS_TRANS_VER_1 = 1  # ISO15765-2(2016版本)

# 帧类型
ZCAN_UDS_FRAME_CAN       = 0  # CAN帧
ZCAN_UDS_FRAME_CANFD     = 1  # CANFD帧
ZCAN_UDS_FRAME_CANFD_BRS = 2  # CANFD加速帧

# 数据长度填充模式
ZCAN_UDS_FILL_MODE_SHORT = 0  # 小于8字节填充至8字节，大于8字节时按DLC就近填充
ZCAN_UDS_FILL_MODE_NONE  = 1  # 不填充
ZCAN_UDS_FILL_MODE_MAX   = 2  # 填充至最大数据长度(不建议)

# UDS错误码
ZCAN_UDS_ERROR_OK = 0x00                # 没错误
ZCAN_UDS_ERROR_TIMEOUT = 0x01           # 响应超时
ZCAN_UDS_ERROR_TRANSPORT = 0x02         # 发送数据失败
ZCAN_UDS_ERROR_CANCEL = 0x03            # 取消请求
ZCAN_UDS_ERROR_SUPPRESS_RESPONSE = 0x04 # 抑制响应
ZCAN_UDS_ERROR_BUSY = 0x05              # 忙碌中
ZCAN_UDS_ERROR_REQ_PARAM = 0x06         # 请求参数错误
ZCAN_UDS_ERROR_OTHTER = 0x64            # 其它未知错误

# UDS响应类型
ZCAN_UDS_RT_NEGATIVE = 0  # 消极响应
ZCAN_UDS_RT_POSITIVE = 1  # 积极响应
ZCAN_UDS_RT_NONE     = 2  # 无响应

# UDS控制类型
ZCAN_UDS_CTRL_STOP_REQ   = 0  # 停止UDS请求

# UDS控制结果
ZCAN_UDS_CTRL_RESULT_OK  = 0  # 成功
ZCAN_UDS_CTRL_RESULT_ERR = 1  # 失败

# 设备信息
class ZCAN_DEVICE_INFO(Structure):
    _fields_ = [("hw_Version", c_ushort),
                ("fw_Version", c_ushort),
                ("dr_Version", c_ushort),
                ("in_Version", c_ushort),
                ("irq_Num", c_ushort),
                ("can_Num", c_ubyte),
                ("str_Serial_Num", c_ubyte * 20),
                ("str_hw_Type", c_ubyte * 40),
                ("reserved", c_ushort * 4)]

    def __str__(self):
        return "Hardware Version:%s\nFirmware Version:%s\nDriver Interface:%s\nInterface Interface:%s\nInterrupt Number:%d\nCAN Number:%d\nSerial:%s\nHardware Type:%s\n" % (
            self.hw_version, self.fw_version, self.dr_version, self.in_version, self.irq_num, self.can_num, self.serial,
            self.hw_type)

    def _version(self, version):
        return ("V%02x.%02x" if version // 0xFF >= 9 else "V%d.%02x") % (version // 0xFF, version & 0xFF)

    @property
    def hw_version(self):
        return self._version(self.hw_Version)

    @property
    def fw_version(self):
        return self._version(self.fw_Version)

    @property
    def dr_version(self):
        return self._version(self.dr_Version)

    @property
    def in_version(self):
        return self._version(self.in_Version)

    @property
    def irq_num(self):
        return self.irq_Num

    @property
    def can_num(self):
        return self.can_Num

    @property
    def serial(self):
        serial = ''
        for c in self.str_Serial_Num:
            if c > 0:
                serial += chr(c)
            else:
                break
        return serial

    @property
    def hw_type(self):
        hw_type = ''
        for c in self.str_hw_Type:
            if c > 0:
                hw_type += chr(c)
            else:
                break
        return hw_type

# CAN初始化信息
class _ZCAN_CHANNEL_CAN_INIT_CONFIG(Structure):
    _fields_ = [("acc_code", c_uint),
                ("acc_mask", c_uint),
                ("reserved", c_uint),
                ("filter", c_ubyte),
                ("timing0", c_ubyte),
                ("timing1", c_ubyte),
                ("mode", c_ubyte)]

# CANFD初始化信息
class _ZCAN_CHANNEL_CANFD_INIT_CONFIG(Structure):
    _fields_ = [("acc_code", c_uint),
                ("acc_mask", c_uint),
                ("abit_timing", c_uint),
                ("dbit_timing", c_uint),
                ("brp", c_uint),
                ("filter", c_ubyte),
                ("mode", c_ubyte),
                ("pad", c_ushort),
                ("reserved", c_uint)]

# 通道初始化结构体中联和体
class _ZCAN_CHANNEL_INIT_CONFIG(Union):
    _fields_ = [("can", _ZCAN_CHANNEL_CAN_INIT_CONFIG),
                ("canfd", _ZCAN_CHANNEL_CANFD_INIT_CONFIG)]

# 通道初始化结构体
class ZCAN_CHANNEL_INIT_CONFIG(Structure):
    _fields_ = [("can_type", c_uint),
                ("config", _ZCAN_CHANNEL_INIT_CONFIG)]

# 通道错误信息
class ZCAN_CHANNEL_ERR_INFO(Structure):
    _fields_ = [("error_code", c_uint),
                ("passive_ErrData", c_ubyte * 3),
                ("arLost_ErrData", c_ubyte)]

# 通道状态
class ZCAN_CHANNEL_STATUS(Structure):
    _fields_ = [("errInterrupt", c_ubyte),
                ("regMode", c_ubyte),
                ("regStatus", c_ubyte),
                ("regALCapture", c_ubyte),
                ("regECCapture", c_ubyte),
                ("regEWLimit", c_ubyte),
                ("regRECounter", c_ubyte),
                ("regTECounter", c_ubyte),
                ("Reserved", c_ubyte)]

# CAN帧
class ZCAN_CAN_FRAME(Structure):
    _fields_ = [("can_id", c_uint, 32),
                # ("err", c_uint, 1),
                # ("rtr", c_uint, 1),
                # ("eff", c_uint, 1),
                ("can_dlc", c_ubyte),
                ("_pad", c_ubyte),      # 0-开启 1-关闭; bit5 设置发送回显 bit6 设置队列发送时间精度(是否设置0.1ms精度，默认精度是1ms)
                                        # bit7 设置队列发送
                ("_res0", c_ubyte),
                ("_res1", c_ubyte),
                ("data", c_ubyte * 8)]

# CANFD帧
class ZCAN_CANFD_FRAME(Structure):
    _pack_ = 1
    _fields_ = [("can_id", c_uint),
                # ("err", c_uint, 1),
                # ("rtr", c_uint, 1),
                # ("eff", c_uint, 1),
                ("len", c_ubyte),
                # ("brs", c_ubyte, 1),
                # ("esi", c_ubyte, 1),
                ("flags", c_ubyte),     # 0-开启 1-关闭; bit5 设置发送回显 bit6 设置队列发送时间精度(是否设置0.1ms精度，默认精度是1ms)
                                        # bit7 设置队列发送 bit0 设置加速报文
                ("_res0", c_ubyte),
                ("_res1", c_ubyte),
                ("data", c_ubyte * 64)]

# ZCANFDData Flag标志位
class ZCANdataFlag(Structure):
    _pack_ = 1
    _fields_ = [("frameType", c_uint, 2),
                ("txDelay", c_uint, 2),
                ("transmitType", c_uint, 4),
                ("txEchoRequest", c_uint, 1),
                ("txEchoed", c_uint, 1),
                ("reserved", c_uint, 22),
                ]

# CAN/CANFD数据(DataObj)
class ZCANFDData(Structure):  ##表示 CAN/CANFD 帧结构,目前仅作为 ZCANDataObj 结构的成员使用
    _pack_ = 1
    _fields_ = [("timestamp", c_uint64),
                ("flag", ZCANdataFlag),
                ("extraData", c_ubyte * 4),
                ("frame", ZCAN_CANFD_FRAME), ]

# CAN发送报文结构体
class ZCAN_Transmit_Data(Structure):
    _fields_ = [("frame", ZCAN_CAN_FRAME), ("transmit_type", c_uint)]

# CAN接收报文结构体
class ZCAN_Receive_Data(Structure):
    _fields_ = [("frame", ZCAN_CAN_FRAME), ("timestamp", c_ulonglong)]

# CANFD发送报文结构体
class ZCAN_TransmitFD_Data(Structure):
    _fields_ = [("frame", ZCAN_CANFD_FRAME), ("transmit_type", c_uint)]

# CANFD接收报文结构体
class ZCAN_ReceiveFD_Data(Structure):
    _fields_ = [("frame", ZCAN_CANFD_FRAME), ("timestamp", c_ulonglong)]

# LIN 初始化结构体
class ZCAN_LIN_INIT_CONFIG(Structure):
    _pack_ = 1
    _fields_ = [("linMode", c_ubyte),       # 0-从机，1主机
                ("chkSumMode", c_ubyte),       # 1-经典校验  2-增强  3-自动
                ("maxLength", c_ubyte),     # 最大数据长度，8~64
                ("reserved", c_ubyte),
                ("linBaud", c_uint)]        # 波特率，取值1000~20000

# LIN 响应结构体
class ZCAN_LIN_PUBLISH_CFG(Structure):
    _pack_ = 1
    _fields_ = [("ID", c_ubyte),            # ID
                ("dataLen", c_ubyte),       # dataLen范围为1-8
                ("data", c_ubyte * 8),
                ("chkSumMode", c_ubyte),    # 校验方式，0-默认，启动时配置 1-经典校验 2-增强校验
                ("reserved", c_ubyte * 5)]

# LIN 取消响应结构体
class ZCAN_LIN_SUBSCIBE_CFG(Structure):
    _pack_ = 1
    _fields_ = [("ID", c_ubyte),            # ID
                ("dataLen", c_ubyte),       # dataLen范围为1-8 当为255（0xff）则表示设备自动识别报文长度
                ("chkSumMode", c_ubyte),    # 校验方式，0-默认，启动时配置 1-经典校验 2-增强校验 3-自动
                ("reserved", c_ubyte * 5)]

# 偶发帧
class ZCAN_LIN_SCHED_ITEM_SPOR_ID(Structure):
    _pack_ = 1
    _fields_ = [("spor_related_id", c_ubyte * 16),  # 偶发帧关联的id列表，索引0为最高优先级，依次递减
                ("spor_count", c_ubyte)]            # 偶发帧的关联的id个数

# 事件触发帧
class ZCAN_LIN_SCHED_ITEM_EVENT_ID(Structure):
    _pack_ = 1
    _fields_ = [("event_id", c_ubyte),              # 事件触发帧的id
                ("event_related_id", c_ubyte * 16), # 事件触发帧关联的无条件帧id列表
                ("event_count", c_ubyte)]           # 事件触发帧关联的无条件帧的id个数

# 调度表帧联和体
class ZCAN_LIN_SCHED_ITEM_ID(Union):
    _pack_ = 1
    _fields_ = [("id", c_ubyte),                                # 无条件帧、诊断主机请求帧、诊断从机应答帧、保留帧的id
                ("sporadic_id", ZCAN_LIN_SCHED_ITEM_SPOR_ID),   # 偶发帧
                ("event_id", ZCAN_LIN_SCHED_ITEM_EVENT_ID)]     # 事件触发帧

# 调度表结构体
class ZCAN_LIN_SCHED_ITEM(Structure):
    _pack_ = 1
    _fields_ = [("type", c_ubyte),                  # 帧类型
                ("reserved1", c_ubyte * 3),
                ("slot", c_uint32),                 # 帧时隙(单位ms)
                ("resolve_handle", c_uint32),       # 冲突解决调度表句柄（该参数仅当前帧类型为事件触发帧有效）是否要将该参数放入事件触发帧的结构体内
                ("ids", ZCAN_LIN_SCHED_ITEM_ID),    # 帧信息
                ("reserved2", c_ubyte * 2)]

# LIN数据(data)
class ZCANLINRxData(Structure):
    _pack_ = 1
    _fields_ = [("timeStamp", c_uint64),    # 时间戳，单位微秒(us)
                ("dataLen", c_ubyte),       # 数据长度
                ("dir", c_ubyte),           # 传输方向，0-接收 1-发送
                ("chkSum", c_ubyte),        # 数据校验，部分设备不支持校验数据的获取
                ("reserved", c_ubyte * 13),
                ("data", c_ubyte * 8)]      # 数据

# LIN数据
class ZCANLINData(Structure):
    _pack_ = 1
    _fields_ = [("PID", c_ubyte),
                ("RxData", ZCANLINRxData),
                ("reserved", c_ubyte * 7)]

# LIN错误数据
class ZCANLINErrData(Structure):
    _pack_ = 1
    _fields_ = [("timeStamp", c_uint64),    # 时间戳
                ("PID", c_ubyte),         # 帧ID
                ("dataLen", c_ubyte),       # 数据长度
                ("Data", c_ubyte * 8),      # 数据
                ("errStage", c_ushort, 4),  # 错误阶段
                ("errReason", c_ushort, 4), # 错误原因
                ("errReserved", c_ushort, 8),
                ("dir", c_ubyte),           # 传输方向
                ("chkSum", c_ubyte),        # 数据校验，部分设备不支持校验数据的获取
                ("reserved", c_ubyte * 10)]

# LIN事件数据
class ZCANLINEventData(Structure):
    _pack_ = 1
    _fields_ = [("timeStamp", c_uint64),    # 时间戳，单位微秒(us)
                ("type", c_ubyte),          # ZCANLINEventType
                ("res", c_ubyte * 7)]

# LIN消息结构体(联合体)
class _ZLINData(Union):
    _pack_ = 1
    _fields_ = [
        ("zcanLINData", ZCANLINData),           # LIN数据
        ("zcanLINErrData", ZCANLINErrData),     # LIN错误数据
        ("zcanLINEventData", ZCANLINEventData), # LIN事件数据
        ("raw", c_uint8 * 46)]

# LIN消息结构体
class ZCAN_LIN_MSG(Structure):
    _pack_ = 1
    _fields_ = [("chnl", c_ubyte),      # 通道
                ("dataType", c_ubyte),  # 数据类型，0-LIN数据 1-LIN错误数据 2-LIN事件数据
                ("data", _ZLINData)]    # 实际数据，联合体，有效成员根据 dataType 字段而定

# 合并接收结构体(data)
class _ZCANData(Union):
    _pack_ = 1
    _fields_ = [("zcanfddata", ZCANFDData),             # CAN/CANFD
                ("zcanLINData", ZCANLINData),           # LIN数据
                ("zcanLINErr", ZCANLINErrData),         # LIN错误数据
                ("zcanLINEventData", ZCANLINEventData), # LIN事件数据
                ("raw", c_uint8 * 92)]

# 合并接收结构体
class ZCANDataObj(Structure):
    _pack_ = 1
    _fields_ = [("dataType", c_uint8),      # 数据类型，参考 eZCANDataDEF 中数据类型部分定义
                ("chnl", c_uint8),          # 通道
                ("flag", c_ushort),
                ("exterData", c_uint8 * 4),
                ("data", _ZCANData)]        # 实际数据，联合体，有效成员根据 dataType 字段而定

# CAN定时发送
class ZCAN_AUTO_TRANSMIT_OBJ(Structure):
    _fields_ = [("enable", c_ushort),
                ("index", c_ushort),
                ("interval", c_uint),
                ("obj", ZCAN_Transmit_Data)]

# CANFD定时发送
class ZCANFD_AUTO_TRANSMIT_OBJ(Structure):
    _fields_ = [("enable", c_ushort),
                ("index", c_ushort),
                ("interval", c_uint),
                ("obj", ZCAN_TransmitFD_Data)]

# CANFD定时发送参数
class ZCANFD_AUTO_TRANSMIT_OBJ_PARAM(Structure):  # auto_send delay
    _fields_ = [("indix", c_ushort),
                ("type", c_ushort),
                ("value", c_uint)]

# CAN UDS会话层参数
class UdsSessionParam(Structure):
    _pack_ = 1
    _fields_ = [
        ("timeout", c_uint),            # 响应超时时间(ms)。因PC定时器误差，建议设置不小于200ms
        ("enhanced_timeout", c_uint),   # 收到消极响应错误码为0x78后的超时时间(ms)。因PC定时器误差，建议设置不小于200ms
        ("check_any_negative_response", c_ubyte, 1),    # 接收到非本次请求服务的消极响应时是否需要判定为响应错误
        ("wait_if_suppress_response", c_ubyte, 1),      # 抑制响应时是否需要等待消极响应，等待时长为响应超时时间
        ("flag", c_ubyte, 6),
        ("reserved0", c_ubyte * 7),
    ]

# CAN UDS传输层参数
class UdsTransParam(Structure):
    _pack_ = 1
    _fields_ = [
        ("version", c_ubyte),           # 传输协议版本，VERSION_0，VERSION_1
        ("max_data_len", c_ubyte),      # 单帧最大数据长度，can:8，canfd:64
        ("local_st_min", c_ubyte),      # 本程序发送流控时用，连续帧之间的最小间隔，0x00-0x7F(0ms~127ms)，0xF1-0xF9(100us~900us)
        ("block_size", c_ubyte),        # 流控帧的块大小
        ("fill_byte", c_ubyte),         # 无效字节的填充数据
        ("ext_frame", c_ubyte),         # 0-标准帧 1-扩展帧
        ("is_modify_ecu_st_min", c_ubyte),  # 是否忽略ECU返回流控的STmin，强制使用本程序设置的 remote_st_min
        ("remote_st_min", c_ubyte),         # 发送多帧时用，is_ignore_ecu_st_min = 1 时有效，0x00-0x7F(0ms~127ms)，0xF1-0xF9(100us~900us)
        ("fc_timeout", c_uint),         # 接收流控超时时间(ms)，如发送首帧后需要等待回应流控帧
        ("fill_mode", c_ubyte),         # 数据长度填充模式
        ("reserved0", c_ubyte * 3),
    ]

# CAN UDS请求数据
class ZCAN_UDS_REQUEST(Structure):
    _pack_ = 1
    _fields_ = [
        ("req_id", c_uint),                 # 请求事务ID，范围0~65535，本次请求的唯一标识
        ("channel", c_ubyte),               # 设备通道索引 0~255
        ("frame_type", c_ubyte),            # 帧类型
        ("reserved0", c_ubyte * 2),
        ("src_addr", c_uint),               # 请求地址(物理地址)
        ("dst_addr", c_uint),               # 响应地址
        ("suppress_response", c_ubyte),     # 1-抑制响应
        ("sid", c_ubyte),                   # 请求服务id
        ("reserved1", c_ubyte * 6),
        ("session_param", UdsSessionParam), # 会话层参数
        ("trans_param", UdsTransParam),     # 传输层参数
        ("data", POINTER(c_ubyte)),         # 数据数组(不包含SID)
        ("data_len", c_uint),               # 数据数组的长度
        ("reserved2", c_uint),
    ]

# CAN UDS数据
class ZCANCANFDUdsData(Structure):
    _pack_ = 1
    _fields_ = [
        ("req", POINTER(ZCAN_UDS_REQUEST)),  # 指向 ZCAN_UDS_REQUEST 的指针
        ("reserved", c_ubyte * 24),
    ]

# LIN UDS会话层参数
class LinUdsSessionParam(Structure):
    _pack_ = 1
    _fields_ = [
        ("p2_timeout", c_uint),         # 响应超时时间(ms)。因PC定时器误差，建议设置不小于200ms
        ("enhanced_timeout", c_uint),   # 收到消极响应错误码为0x78后的超时时间(ms)。因PC定时器误差，建议设置不小于200ms
        ("check_any_negative_response", c_ubyte, 1),    # 接收到非本次请求服务的消极响应时是否需要判定为响应错误
        ("wait_if_suppress_response", c_ubyte, 1),      # 抑制响应时是否需要等待消极响应，等待时长为响应超时时间
        ("flag", c_ubyte, 6),
        ("reserved0", c_ubyte * 7),]

# LIN UDS传输层参数
class LinUdsTransParam(Structure):
    _pack_ = 1
    _fields_ = [
        ("fill_byte", c_ubyte),     # 无效字节的填充数据
        ("st_min", c_ubyte),        # 从节点准备接收诊断请求的下一帧或传输诊断响应的下一帧所需的最小时间
        ("reserved0", c_ubyte * 6),
    ]

# LIN UDS请求数据
class ZLIN_UDS_REQUEST(Structure):
    _pack_ = 1
    _fields_ = [
        ("req_id", c_uint),             # 请求事务ID，范围0~65535，本次请求的唯一标识
        ("channel", c_ubyte),           # 设备通道索引 0~255
        ("suppress_response", c_ubyte), # 1-抑制响应 0-不抑制
        ("sid", c_ubyte),               # 请求服务id
        ("Nad", c_ubyte),               # 节点地址
        ("reserved1", c_ubyte * 8),
        ("session_param", LinUdsSessionParam),  # 会话层参数
        ("trans_param", LinUdsTransParam),      # 传输层参数
        ("data", POINTER(c_ubyte)),     # 数据数组(不包含SID)
        ("data_len", c_uint),           # 数据数组的长度
        ("reserved2", c_uint),
    ]

# LIN UDS数据
class ZCANLINUdsData(Structure):
    _pack_ = 1
    _fields_ = [
        ("req", POINTER(ZLIN_UDS_REQUEST)),  # 指向 ZCAN_UDS_REQUEST 的指针
        ("reserved", c_ubyte * 24),
    ]

# ZCANUdsRequestDataObj(Union)
class ZCANUdsRequestDataUnion(Union):
    # _pack_ = 1
    _fields_ = [("zcanCANFDUdsData", ZCANCANFDUdsData), # CAN/CANFD
                ("zcanLINUdsData", ZCANLINUdsData),     # LIN
                ("raw", c_byte * 63)]

# 硬件UDS数据结构，支持CAN/LIN等UDS不同类型数据
class ZCANUdsRequestDataObj(Structure):
    _pack_ = 1
    _fields_ = [
        ("dataType", ZCAN_UDS_DATA_DEF),    # 类型
        ("data", ZCANUdsRequestDataUnion),  # 数据
        ("reserved", c_ubyte * 32),
    ]

# UdsResponseUnion 积极响应结构体
class UdsPositiveResponse(Structure):
    _pack_ = 1
    _fields_ = [
        ("sid", c_ubyte),           # 响应服务id
        ("data_len", c_uint),       # 数据长度(不包含SID), 数据存放在接口传入的dataBuf中
    ]

# UdsResponseUnion 消极响应结构体
class UdsNegativeResponse(Structure):
    _fields_ = [
        ("neg_code", c_ubyte),      # 固定为0x7F
        ("sid", c_ubyte),           # 请求服务id
        ("error_code", c_ubyte),    # 错误码
    ]

# ZCAN_UDS_RESPONSE(Union)
class UdsResponseUnion(Union):
    _fields_ = [
        ("positive", UdsPositiveResponse),  # 积极响应结构体
        ("negative", UdsNegativeResponse),  # 消极响应结构体
        ("raw", c_ubyte * 8),
    ]

# 硬件UDS响应数据
class ZCAN_UDS_RESPONSE(Structure):
    _fields_ = [
        ("status", c_ubyte),        # 响应状态
        ("reserved", c_ubyte * 6),
        ("type", c_ubyte),          # 响应类型
        ("u", UdsResponseUnion),    # 联合体
    ]

# 获取设备操控句柄结构体(老接口结构体)
class IProperty(Structure):
    _fields_ = [("SetValue", c_void_p),
                ("GetValue", c_void_p),
                ("GetPropertys", c_void_p)]

class BusUsage(Structure):
    # _pack_   = 1          # 与 C 端保持一致（1 字节对齐）
    _fields_ = [
        ("nTimeStampBegin", c_uint64),
        ("nTimeStampEnd",   c_uint64),
        ("nChnl",           c_ubyte),
        ("nReserved",       c_ubyte),
        ("nBusUsage",       c_ushort),
        ("nFrameCount",     c_uint32),
    ]

###### 动态配置 ######
# CAN的通道配置信息(CAN%d需进行格式化声明通道 范围是0-7)

# 动态配置结构体
class ZCAN_DYNAMIC_CONFIG_DATA(Structure):
    _pack_ = 1
    _fields_ = [("key",c_char*64),
                ("value",c_char*64)
    ]

# 设备名，最长为32字节（包括’\0’）
def ZCAN_DYNAMIC_CONFIG_DEVNAME():
    return "DYNAMIC_CONFIG_DEVNAME"

# 通道使能；1：使能，0：失能；CANFDNET系列产品通道默认使能。
def ZCAN_DYNAMIC_CONFIG_CAN_ENABLE(can_id):
    return f"DYNAMIC_CONFIG_CAN{can_id}_ENABLE"

# 工作模式，默认正常模式；0：正常模式；1：只听模式
def ZCAN_DYNAMIC_CONFIG_CAN_MODE(can_id):
    return f"DYNAMIC_CONFIG_CAN{can_id}_MODE"

# 发送失败是否重传：0：发送失败不重传1：发送失败重传，直到总线关闭（CANFDNET- 100 / 200无此项配置）
def ZCAN_DYNAMIC_CONFIG_CAN_TXATTEMPTS(can_id):
    return f"DYNAMIC_CONFIG_CAN{can_id}_TXATTEMPTS"

# CAN波特率或CANFD仲裁域波特率
def ZCAN_DYNAMIC_CONFIG_CAN_NOMINALBAUD(can_id):
    return f"DYNAMIC_CONFIG_CAN{can_id}_NOMINALBAUD"

# CANFD数据域波特率
def ZCAN_DYNAMIC_CONFIG_CAN_DATABAUD(can_id):
    return f"DYNAMIC_CONFIG_CAN{can_id}_DATABAUD"

# 终端电阻开关；0：关闭；1：打开
def ZCAN_DYNAMIC_CONFIG_CAN_USERES(can_id):
    return f"DYNAMIC_CONFIG_CAN{can_id}_USERES"

# 终端电阻开关；0：关闭；1：打开
def ZCAN_DYNAMIC_CONFIG_CAN_USERES(can_id):
    return f"DYNAMIC_CONFIG_CAN{can_id}_USERES"

# 报文发送间隔，0~255ms
def ZCAN_DYNAMIC_CONFIG_CAN_SNDCFG_INTERVAL(can_id):
    return f"DYNAMIC_CONFIG_CAN{can_id}_SNDCFG_INTERVAL"

# 总线利用率使能，使能后，将周期发送总线利用率到设定的TCP/UDP连接。1:使能，0：失能
def ZCAN_DYNAMIC_CONFIG_CAN_BUSRATIO_ENABLE(can_id):
    return f"DYNAMIC_CONFIG_CAN{can_id}_SNDCFG_INTERVAL"

# 总线利用率采集周期，取值200~2000ms
def ZCAN_DYNAMIC_CONFIG_CAN_BUSRATIO_ENABLE(can_id):
    return f"DYNAMIC_CONFIG_CAN{can_id}_SNDCFG_INTERVAL"

class ZCAN(object):
    def __init__(self):
        if platform.system() == "Windows":
            self.__dll = windll.LoadLibrary("./zlgcan.dll")
        else:
            print("No support now!")
        if self.__dll == None:
            print("DLL couldn't be loaded!")

    # 打开设备
    def OpenDevice(self, device_type, device_index, reserved):
        try:
            return self.__dll.ZCAN_OpenDevice(device_type, device_index, reserved)
        except:
            print("Exception on OpenDevice!")
            raise

    # 关闭设备
    def CloseDevice(self, device_handle):
        try:
            return self.__dll.ZCAN_CloseDevice(device_handle)
        except:
            print("Exception on CloseDevice!")
            raise

    # 获取设备信息
    def GetDeviceInf(self, device_handle):
        try:
            info = ZCAN_DEVICE_INFO()
            ret = self.__dll.ZCAN_GetDeviceInf(device_handle, byref(info))
            return info if ret == ZCAN_STATUS_OK else None
        except:
            print("Exception on ZCAN_GetDeviceInf")
            raise

    # 判断设备在线，原理同GetDeviceInf
    def DeviceOnLine(self, device_handle):
        try:
            return self.__dll.ZCAN_IsDeviceOnLine(device_handle)
        except:
            print("Exception on ZCAN_ZCAN_IsDeviceOnLine!")
            raise

    # 初始化通道
    def InitCAN(self, device_handle, can_index, init_config):
        try:
            return self.__dll.ZCAN_InitCAN(device_handle, can_index, byref(init_config))
        except:
            print("Exception on ZCAN_InitCAN!")
            raise

    # 启动通道
    def StartCAN(self, chn_handle):
        try:
            return self.__dll.ZCAN_StartCAN(chn_handle)
        except:
            print("Exception on ZCAN_StartCAN!")
            raise

    # 复位通道
    def ResetCAN(self, chn_handle):
        try:
            return self.__dll.ZCAN_ResetCAN(chn_handle)
        except:
            print("Exception on ZCAN_ResetCAN!")
            raise

    # 清空接收缓冲区
    def ClearBuffer(self, chn_handle):
        try:
            return self.__dll.ZCAN_ClearBuffer(chn_handle)
        except:
            print("Exception on ZCAN_ClearBuffer!")
            raise

    # 读取通道错误信息
    def ReadChannelErrInfo(self, chn_handle):
        try:
            ErrInfo = ZCAN_CHANNEL_ERR_INFO()
            ret = self.__dll.ZCAN_ReadChannelErrInfo(chn_handle, byref(ErrInfo))
            return ErrInfo if ret == ZCAN_STATUS_OK else None
        except:
            print("Exception on ZCAN_ReadChannelErrInfo!")
            raise

    # 获取通道状态
    def ReadChannelStatus(self, chn_handle):
        try:
            status = ZCAN_CHANNEL_STATUS()
            ret = self.__dll.ZCAN_ReadChannelStatus(chn_handle, byref(status))
            return status if ret == ZCAN_STATUS_OK else None
        except:
            print("Exception on ZCAN_ReadChannelStatus!")
            raise

    # 获取接收缓冲的数据量
    def GetReceiveNum(self, chn_handle, can_type=ZCAN_TYPE_CAN):
        try:
            return self.__dll.ZCAN_GetReceiveNum(chn_handle, can_type)
        except:
            print("Exception on ZCAN_GetReceiveNum!")
            raise

    # 发送CAN报文
    def Transmit(self, chn_handle, std_msg, len):
        try:
            return self.__dll.ZCAN_Transmit(chn_handle, byref(std_msg), len)
        except:
            print("Exception on ZCAN_Transmit!")
            raise

    # 接收CAN报文
    def Receive(self, chn_handle, rcv_num, wait_time=c_int(-1)):
        try:
            rcv_can_msgs = (ZCAN_Receive_Data * rcv_num)()
            ret = self.__dll.ZCAN_Receive(chn_handle, byref(rcv_can_msgs), rcv_num, wait_time)
            return rcv_can_msgs, ret
        except:
            print("Exception on ZCAN_Receive!")
            raise

    # 发送CANFD报文
    def TransmitFD(self, chn_handle, fd_msg, len):
        try:
            return self.__dll.ZCAN_TransmitFD(chn_handle, byref(fd_msg), len)
        except:
            print("Exception on ZCAN_TransmitFD!")
            raise

    # 接收CANFD报文
    def ReceiveFD(self, chn_handle, rcv_num, wait_time=c_int(-1)):
        try:
            rcv_canfd_msgs = (ZCAN_ReceiveFD_Data * rcv_num)()
            ret = self.__dll.ZCAN_ReceiveFD(chn_handle, byref(rcv_canfd_msgs), rcv_num, wait_time)
            return rcv_canfd_msgs, ret
        except:
            print("Exception on ZCAN_ReceiveF D!")
            raise

    # 合并发送
    def TransmitData(self, device_handle, msg, len):
        try:
            return self.__dll.ZCAN_TransmitData(device_handle, byref(msg), len)
        except:
            print("Exception on ZCAN_TransmitData!")
            raise

    # 合并接收
    def ReceiveData(self, device_handle, rcv_num, wait_time=c_int(-1)):
        try:
            rcv_can_data_msgs = (ZCANDataObj * rcv_num)()
            ret = self.__dll.ZCAN_ReceiveData(device_handle, byref(rcv_can_data_msgs), rcv_num, wait_time)
            return rcv_can_data_msgs, ret
        except:
            print("Exception on ZCAN_ReceiveData!")
            raise

    # 获取设备操控句柄(老接口不建议使用)
    def GetIProperty(self, device_handle):
        try:
            self.__dll.GetIProperty.restype = POINTER(IProperty)
            return self.__dll.GetIProperty(device_handle)
        except:
            print("Exception on ZCAN_GetIProperty!")
            raise

    # 设置设备参数(老接口不建议使用)
    def SetValue(self, iproperty, path, value):
        try:
            func = CFUNCTYPE(c_uint, c_char_p, c_char_p)(iproperty.contents.SetValue)
            return func(c_char_p(path.encode("utf-8")), c_char_p(value.encode("utf-8")))
        except:
            print("Exception on IProperty SetValue")
            raise

    # 设置设备参数(老接口不建议使用)
    def SetValue1(self, iproperty, path, value):  #############################
        try:
            func = CFUNCTYPE(c_uint, c_char_p, c_char_p)(iproperty.contents.SetValue)
            return func(c_char_p(path.encode("utf-8")), c_void_p(value))
        except:
            print("Exception on IProperty SetValue")
            raise

    # 获取设备参数(老接口不建议使用)
    def GetValue(self, iproperty, path):
        try:
            func = CFUNCTYPE(c_char_p, c_char_p)(iproperty.contents.GetValue)
            return func(c_char_p(path.encode("utf-8")))
            # self.__dll.ZCAN_GetValue.restype = c_void_p
            # return self.__dll.ZCAN_GetValue(iproperty, path.encode("utf-8"))
        except:
            print("Exception on IProperty GetValue")
            raise

    # 释放设备操控句柄(老接口不建议使用)
    def ReleaseIProperty(self, iproperty):
        try:
            return self.__dll.ReleaseIProperty(iproperty)
        except:
            print("Exception on ZCAN_ReleaseIProperty!")
            raise

    # 设置设备参数(新接口)
    def ZCAN_SetValue(self, device_handle, path, value):
        try:
            self.__dll.ZCAN_SetValue.argtypes = [c_void_p, c_char_p, c_void_p]
            return self.__dll.ZCAN_SetValue(device_handle, path.encode("utf-8"), value)
        except:
            print("Exception on ZCAN_SetValue")
            raise

    # 获取设备参数(新接口)
    def ZCAN_GetValue(self, device_handle, path):
        try:
            self.__dll.ZCAN_GetValue.argtypes = [c_void_p, c_char_p]
            self.__dll.ZCAN_GetValue.restype = c_void_p
            return self.__dll.ZCAN_GetValue(device_handle, path.encode("utf-8"))
        except:
            print("Exception on ZCAN_GetValue")
            raise

    # 初始化LIN通道
    def InitLIN(self, device_handle, lin_index, config):
        try:
            return self.__dll.ZCAN_InitLIN(device_handle, lin_index, byref(config))
        except:
            print("Exception on InitLIN")
            raise

    # 启动LIN通道
    def StartLIN(self, chn_handle):
        try:
            return self.__dll.ZCAN_StartLIN(chn_handle)
        except:
            print("Exception on StartLIN")
            raise

    # 复位LIN通道
    def ResetLIN(self, chn_handle):
        try:
            return self.__dll.ZCAN_ResetLIN(chn_handle)
        except:
            print("Exception on ResetLIN")
            raise

    # 发送LIN报文
    def TransmitLIN(self, chn_handle, msgs, num):
        try:
            return self.__dll.ZCAN_TransmitLIN(chn_handle, byref(msgs), num)
        except:
            print("Exception on TransmitLIN")
            raise

    # 获取LIN缓冲区数据
    def GetLINReceiveNum(self, chn_handle):
        try:
            return self.__dll.ZCAN_GetLINReceiveNum(chn_handle)
        except:
            print("Exception on GetLINReceiveNum")
            raise

    # 接收LIN报文
    def ReceiveLIN(self, chn_handle, num, wait_time=-1):
        try:
            rcv_msgs = (ZCAN_LIN_MSG * num)()
            rcv_num = self.__dll.ZCAN_ReceiveLIN(chn_handle, rcv_msgs, num, wait_time)
            if rcv_num == 0:
                del rcv_msgs
                return None, 0
            return rcv_msgs, rcv_num
        except:
            print("Exception on ReceiveLIN")
            raise

    # 取消响应
    def SetLINSubscribe(self, chn_handle, data, num):
        try:
            return self.__dll.ZCAN_SetLINSubscribe(chn_handle, byref(data), num)
        except:
            print("Exception on SetLINSubscribe")
            raise

    # 设置响应
    def SetLINPublish(self, chn_handle, data, count):
        try:
            return self.__dll.ZCAN_SetLINPublish(chn_handle, byref(data), count)
        except:
            print("Exception on ZCAN_SetLINPublish!")
            raise

    # 设置响应
    def SetLINResponseEx(self, chn_handle, res_msgs, num):
        try:
            return self.__dll.ZCAN_SetLINPublishEx(chn_handle, byref(res_msgs), num)
        except:
            print("Exception on SetLINResponse")
            raise

    # LIN调度表创建
    def CreateLINSchedule(self, device_handle, items, count):
        try:
            return self.__dll.ZCAN_CreateLINSchedule(device_handle, byref(items), count)
        except:
            print("Exception on CreateLINSchedule")
            raise

    # 删除LIN调度表
    def DestroyLINSchedule(self, device_handle, handle):
        try:
            return self.__dll.ZCAN_DestroyLINSchedule(device_handle, handle)
        except:
            print("Exception on DestroyLINSchedule")
            raise

    # 添加LIN调度表到通道
    def LINChnAddSchedule(self, chn_handle, sche_handle, run_count):
        try:
            return self.__dll.ZCAN_AddLINSchedule(chn_handle, sche_handle, run_count)
        except:
            print("Exception on LINChnAddSchedule")
            raise

    # 清空通道LIN调度表
    def LINChnClrSchedule(self, chn_handle):
        try:
            return self.__dll.ZCAN_ClrLINSchedule(chn_handle)
        except:
            print("Exception on LINChnClrSchedule")
            raise

    # 设置通道LIN调度表使能状态
    def SetLINScheduleEnable(self, chn_handle, sche_handle, enable):
        try:
            return self.__dll.ZCAN_SetLINScheduleEnabled(chn_handle, sche_handle, enable)
        except:
            print("Exception on SetLINScheduleEnable")
            raise

    # 设置通道LIN调度表表项使能状态
    def SetLINScheduleItemEnable(self, chn_handle, sche_handle, idx, enable):
        try:
            return self.__dll.ZCAN_SetLINScheduleItemEnabled(chn_handle, sche_handle, idx, enable)
        except:
            print("Exception on SetLINScheduleItemEnable")
            raise

    # 获取LIN调度表状态信息
    def GetLINScheduleStatus(self, chn_handle, sche_handle, status):
        try:
            return self.__dll.ZCAN_GetLINScheduleStatus(chn_handle, sche_handle, byref(status))
        except:
            print("Exception on GetLINScheduleStatus")
            raise

    # 启动LIN通道调度表
    def StartLINSchedule(self, chn_handle):
        try:
            return self.__dll.ZCAN_StartLINSchedule(chn_handle)
        except:
            print("Exception on StartLINSchedule")
            raise

    # 停止LIN通道调度表
    def StopLINSchedule(self, chn_handle):
        try:
            return self.__dll.ZCAN_StopLINSchedule(chn_handle)
        except:
            print("Exception on StopLINSchedule")
            raise