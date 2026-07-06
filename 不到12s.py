from machine import ADC, Pin, PWM
import time
import math

# ADC config
ADC_ATTEN = ADC.ATTN_11DB
ADC_WIDTH = ADC.WIDTH_12BIT
ADC_MAX_VAL = 4095
ADC_VOLT_REF = 3.6

# Sensor positions, left to right, in mm.
SENSOR_POS = {
    "CH1": -37.5,
    "CH2": -15.0,
    "CH3": 0.0,
    "CH4": 15.0,
    "CH5": 37.5,
}
SENSOR_ORDER = ["CH1", "CH2", "CH3", "CH4", "CH5"]
MAX_OFFSET_MM = 37.5

# 原来的电压阈值先保留，但现在实际循迹不再使用它们
BLACK_V_THRESH = 0.11
WHITE_V_THRESH = 0.06

# ================= 新场地标定结果 =================
# 当前标定结果：
# CH1: white=109, black=2027, BLACK_HIGH
# CH2: white=84,  black=1023, BLACK_HIGH
# CH3: white=83,  black=667,  BLACK_HIGH
# CH4: white=81,  black=1002, BLACK_HIGH
# CH5: white=94,  black=2005, BLACK_HIGH
CALIB = {
    "CH1": {"white": 109, "black": 2027},
    "CH2": {"white": 84,  "black": 1023},
    "CH3": {"white": 83,  "black": 667},
    "CH4": {"white": 81,  "black": 1002},
    "CH5": {"white": 94,  "black": 2005},
}
# ===============================================================

# ================= 直角弯特化参数 =================
PWM_FREQ = 20000
BASE_SPEED = 78
MAX_DIFF = 100
LEFT_TRIM = 1.00
RIGHT_TRIM = 1.00
CENTER_DEADZONE = 0.06
ERROR_CURVE = 1.0

# PID 参数保持不变
PID_KP = 85.0
PID_KI = 0.0
PID_KD = 10.0

# 丢线救车打转速度
SPIN_SEARCH_SPEED = 45
# ===============================================================

LOOP_DELAY = 10
PRINT_INTERVAL = 200
FILTER_LEN = 1

adc_pins_map = {
    "CH1": 27,
    "CH2": 33,
    "CH3": 32,
    "CH4": 35,
    "CH5": 34,
}

adc_dev = {}
adc_buf = {name: [] for name in SENSOR_ORDER}

for name, pin_num in adc_pins_map.items():
    adc = ADC(Pin(pin_num))
    adc.atten(ADC_ATTEN)
    adc.width(ADC_WIDTH)
    adc_dev[name] = adc

m1_in1 = PWM(Pin(13, Pin.OUT), freq=PWM_FREQ, duty=0)
m1_in2 = PWM(Pin(15, Pin.OUT), freq=PWM_FREQ, duty=0)

m2_in2 = PWM(Pin(14, Pin.OUT), freq=PWM_FREQ, duty=0)
m2_in1 = PWM(Pin(25, Pin.OUT), freq=PWM_FREQ, duty=0)

last_error = 0.0
integral = 0.0
last_line_pos = 0.0


def adc2voltage(adc_val):
    return (adc_val / ADC_MAX_VAL) * ADC_VOLT_REF


def voltage2signal(volt):
    # 这个函数现在保留但不再用于循迹
    if volt > BLACK_V_THRESH:
        return 1.0
    if volt < WHITE_V_THRESH:
        return 0.0
    mid = (BLACK_V_THRESH + WHITE_V_THRESH) / 2
    k = 30
    return 1 / (1 + math.exp(-k * (volt - mid)))


def raw2signal(ch_name, raw_val):
    """
    根据每一路 white / black 标定值，把 ADC 原始值转换成 0~1 的黑线强度。
    0.0 = 白底
    1.0 = 黑线
    """
    white = CALIB[ch_name]["white"]
    black = CALIB[ch_name]["black"]

    if black == white:
        return 0.0

    sig = (raw_val - white) / (black - white)

    if sig < 0.0:
        sig = 0.0
    elif sig > 1.0:
        sig = 1.0

    return sig


def read_all_adc():
    ret = {}

    for ch, adc in adc_dev.items():
        val = adc.read()
        buf = adc_buf[ch]

        buf.append(val)

        if len(buf) > FILTER_LEN:
            buf.pop(0)

        ret[ch] = sum(buf) / len(buf)

    return ret


def calc_line_pos(adc_data):
    global last_line_pos

    numerator = 0.0
    denominator = 0.0

    # 记录各通道黑线强度，用于直角 / 十字判断
    signals = {}

    for ch_name in SENSOR_ORDER:
        raw = adc_data[ch_name]
        sig = raw2signal(ch_name, raw)

        signals[ch_name] = sig

        numerator += SENSOR_POS[ch_name] * sig
        denominator += sig

    if denominator < 0.1:
        return last_line_pos, True

    # 正常加权平均计算
    pos_mm = numerator / denominator
    pos_norm = max(-1.0, min(1.0, pos_mm / MAX_OFFSET_MM))

    # ================= 直角 / 十字判断 =================
    # 十字路口特征：最左 CH1 和最右 CH5 同时压线
    is_crossroad = (signals["CH1"] > 0.7 and signals["CH5"] > 0.7)

    # 只有在非十字路口时，才允许触发直角转弯
    if not is_crossroad:
        if (
            signals["CH1"] > 0.7
            and signals["CH2"] > 0.7
            and signals["CH3"] > 0.7
        ):
            pos_norm = -1.2  # 左直角

        elif (
            signals["CH3"] > 0.7
            and signals["CH4"] > 0.7
            and signals["CH5"] > 0.7
        ):
            pos_norm = 1.2  # 右直角
    # ==================================================

    last_line_pos = pos_norm

    return pos_norm, False


def set_motor_speed(m_pwm1, m_pwm2, speed):
    """
    speed: -100 ~ 100
    采用最小启动PWM补偿，避免低速一走一停。
    """
    speed = max(-100, min(100, speed))

    duty_max = 1023

    # 你这辆车低于 590 左右容易不动，所以非零输出直接映射到 590~1023
    MOTOR_MIN_DUTY = 610

    if speed > 0:
        duty = MOTOR_MIN_DUTY + (speed / 100.0) * (duty_max - MOTOR_MIN_DUTY)
        duty = int(min(duty_max, duty))

        m_pwm1.duty(duty)
        m_pwm2.duty(0)

    elif speed < 0:
        duty = MOTOR_MIN_DUTY + (-speed / 100.0) * (duty_max - MOTOR_MIN_DUTY)
        duty = int(min(duty_max, duty))

        m_pwm1.duty(0)
        m_pwm2.duty(duty)

    else:
        m_pwm1.duty(0)
        m_pwm2.duty(0)


def motor_control_pid(pos, is_lost):
    global last_error, integral

    # 1. 丢线救车逻辑
    if is_lost:
        integral = 0

        spd_left = SPIN_SEARCH_SPEED if pos > 0 else -SPIN_SEARCH_SPEED
        spd_right = -SPIN_SEARCH_SPEED if pos > 0 else SPIN_SEARCH_SPEED

        set_motor_speed(m1_in1, m1_in2, spd_left * LEFT_TRIM)
        set_motor_speed(m2_in1, m2_in2, spd_right * RIGHT_TRIM)

        return spd_left, spd_right

    # 2. 计算 PID 误差
    error = pos
    #if abs(error) < CENTER_DEADZONE:
        #error = 0.0
    dt = LOOP_DELAY / 1000
    
    integral += error * dt
    integral = max(-5, min(5, integral))

    derivative = (error - last_error) / dt if dt > 0 else 0

    diff = PID_KP * error + PID_KI * integral + PID_KD * derivative
    diff = max(-MAX_DIFF, min(MAX_DIFF, diff))

    last_error = error

    # 3. 动态急弯降速逻辑
    if abs(pos) > 0.6:
        # 大弯 / 直角：切断前冲，全力旋转
        current_base_speed = 5

    elif abs(pos) > 0.35:
        # 中弯：轻微减速
        current_base_speed = BASE_SPEED * 0.67

    else:
        # 直道：全速前进
        current_base_speed = BASE_SPEED

    # 4. 速度合成
    spd_left = current_base_speed + diff
    spd_right = current_base_speed - diff

    set_motor_speed(m1_in1, m1_in2, spd_left * LEFT_TRIM)
    set_motor_speed(m2_in1, m2_in2, spd_right * RIGHT_TRIM)

    return spd_left, spd_right


def print_debug(adc_data, line_pos, is_lost, l_spd, r_spd):
    volts = [adc2voltage(adc_data[ch]) for ch in SENSOR_ORDER]
    sigs = [raw2signal(ch, adc_data[ch]) for ch in SENSOR_ORDER]

    state_str = "LOST!" if is_lost else "TRACK"

    print(
        "[{}] pos:{:+.3f} | L:{:4.0f} R:{:4.0f} | "
        "V:{:.2f},{:.2f},{:.2f},{:.2f},{:.2f} | "
        "S:{:.2f},{:.2f},{:.2f},{:.2f},{:.2f}".format(
            state_str,
            line_pos,
            l_spd,
            r_spd,
            volts[0],
            volts[1],
            volts[2],
            volts[3],
            volts[4],
            sigs[0],
            sigs[1],
            sigs[2],
            sigs[3],
            sigs[4],
        )
    )


def main():
    print("=== ESP32 5-sensor calibrated tracking started ===")
    print("Using CALIB white/black normalization.")
    time.sleep(2)

    last_print_time = time.ticks_ms()

    try:
        while True:
            adc_data = read_all_adc()

            line_pos, is_lost = calc_line_pos(adc_data)

            l_spd, r_spd = motor_control_pid(line_pos, is_lost)

            now = time.ticks_ms()

            if time.ticks_diff(now, last_print_time) >= PRINT_INTERVAL:
                print_debug(adc_data, line_pos, is_lost, l_spd, r_spd)
                last_print_time = now

            time.sleep_ms(LOOP_DELAY)

    except KeyboardInterrupt:
        print("\nStopped.")

        stop_motors()

        m1_in1.deinit()
        m1_in2.deinit()
        m2_in1.deinit()
        m2_in2.deinit()


if __name__ == "__main__":
    main()
