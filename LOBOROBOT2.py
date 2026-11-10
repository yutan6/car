'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
@  文件名：LOBOROBOT2.py 
@  驅動智慧小車的基本運動庫函數
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

import time
import math
import smbus
from gpiozero import LED

FORWARD = 'forward'
BACKWARD = 'backward'

class PCA9685:
    __MODE1 = 0x00
    __PRESCALE = 0xFE
    __LED0_ON_L = 0x06
    __ALLLED_ON_L = 0xFA
    __ALLLED_OFF_L = 0xFC

    def __init__(self, address, debug=False):
        self.bus = smbus.SMBus(1)
        self.address = address
        self.debug = debug
        self.write(self.__MODE1, 0x00)

    def write(self, reg, value):
        self.bus.write_byte_data(self.address, reg, value)
        if self.debug:
            print(f"I2C: Write 0x{value:02X} to register 0x{reg:02X}")

    def read(self, reg):
        result = self.bus.read_byte_data(self.address, reg)
        if self.debug:
            print(f"I2C: Read 0x{result:02X} from register 0x{reg:02X}")
        return result

    def setPWMFreq(self, freq):
        prescaleval = 25000000.0 / 4096.0 / float(freq) - 1.0
        prescale = math.floor(prescaleval + 0.5)
        oldmode = self.read(self.__MODE1)
        newmode = (oldmode & 0x7F) | 0x10
        self.write(self.__MODE1, newmode)
        self.write(self.__PRESCALE, int(prescale))
        self.write(self.__MODE1, oldmode)
        time.sleep(0.005)
        self.write(self.__MODE1, oldmode | 0x80)

    def setPWM(self, channel, on, off):
        self.write(self.__LED0_ON_L + 4 * channel, on & 0xFF)
        self.write(self.__LED0_ON_L + 4 * channel + 1, on >> 8)
        self.write(self.__LED0_ON_L + 4 * channel + 2, off & 0xFF)
        self.write(self.__LED0_ON_L + 4 * channel + 3, off >> 8)

    def setDutycycle(self, channel, percent):
        pulse = int(percent * 4096 / 100)
        self.setPWM(channel, 0, pulse)

    def setLevel(self, channel, value):
        self.setPWM(channel, 0, 4095 if value else 0)


class LOBOROBOT:
    def __init__(self):
        self.PWMA, self.AIN1, self.AIN2 = 0, 2, 1  # 左前輪
        self.PWMB, self.BIN1, self.BIN2 = 5, 3, 4  # 右前輪
        self.PWMC, self.CIN1, self.CIN2 = 6, 8, 7  # 左後輪
        self.PWMD, self.DIN1, self.DIN2 = 11, 25, 24  # 右後輪

        self.pwm = PCA9685(0x40, debug=False)
        self.pwm.setPWMFreq(50)

        self.motorD1 = LED(self.DIN1)
        self.motorD2 = LED(self.DIN2)

    def _validate_speed(self, speed):
        if not 0 <= speed <= 100:
            raise ValueError("Speed must be between 0 and 100")

    def _set_motor_direction(self, in1, in2, forward):
        self.pwm.setLevel(in1, int(forward))
        self.pwm.setLevel(in2, int(not forward))

    def MotorRun(self, motor, direction, speed):
        self._validate_speed(speed)
        forward = (direction == FORWARD)

        if motor == 0:
            self.pwm.setDutycycle(self.PWMA, speed)
            self._set_motor_direction(self.AIN1, self.AIN2, not forward)
        elif motor == 1:
            self.pwm.setDutycycle(self.PWMB, speed)
            self._set_motor_direction(self.BIN1, self.BIN2, forward)
        elif motor == 2:
            self.pwm.setDutycycle(self.PWMC, speed)
            self._set_motor_direction(self.CIN1, self.CIN2, forward)
        elif motor == 3:
            self.pwm.setDutycycle(self.PWMD, speed)
            if forward:
                self.motorD1.off()
                self.motorD2.on()
            else:
                self.motorD1.on()
                self.motorD2.off()

    def MotorStop(self, motor):
        for pwm in [self.PWMA, self.PWMB, self.PWMC, self.PWMD]:
            self.pwm.setDutycycle(pwm, 0)

    def move(self, direction, speed, t_time):
        for m in range(4):
            self.MotorRun(m, direction, speed)
        if t_time > 0:
            time.sleep(t_time)

    def move_with_offset(self, speed, l_ofs, r_ofs, t_time):
        self.MotorRun(0, FORWARD, speed + l_ofs)
        self.MotorRun(1, FORWARD, speed + r_ofs)
        self.MotorRun(2, FORWARD, speed + l_ofs)
        self.MotorRun(3, FORWARD, speed + r_ofs)
        if t_time > 0:
            time.sleep(t_time)

    def moveforward(self,speed,t_time):          # 前進
        self.MotorRun(0,'forward',speed)
        self.MotorRun(1,'forward',speed)
        self.MotorRun(2,'forward',speed)
        self.MotorRun(3,'forward',speed)
        if t_time > 0: time.sleep(t_time)  # 等幾秒，可以有小數點

    def movebackward(self,speed,t_time):        # 後退
        self.MotorRun(0,'backward',speed)
        self.MotorRun(1,'backward',speed)
        self.MotorRun(2,'backward',speed)
        self.MotorRun(3,'backward',speed)
        if t_time > 0: time.sleep(t_time)  # 等幾秒，可以有小數點

    def moveLeft(self,speed,t_time):        # 左移
        self.MotorRun(0,'backward',speed)
        self.MotorRun(1,'forward',speed)
        self.MotorRun(2,'forward',speed)
        self.MotorRun(3,'backward',speed)
        if t_time > 0: time.sleep(t_time)  # 等幾秒，可以有小數點

    def moveRight(self,speed,t_time):       # 右移
        self.MotorRun(0,'forward',speed)
        self.MotorRun(1,'backward',speed)
        self.MotorRun(2,'backward',speed)
        self.MotorRun(3,'forward',speed)
        if t_time > 0: time.sleep(t_time)  # 等幾秒，可以有小數點

    def turnLeft(self,speed,t_time):        # 左轉
        self.MotorRun(0,'backward',speed)
        self.MotorRun(1,'forward',speed)
        self.MotorRun(2,'backward',speed)
        self.MotorRun(3,'forward',speed)
        if t_time > 0: time.sleep(t_time)  # 等幾秒，可以有小數點

    def turnRight(self,speed,t_time):       # 右轉
        self.MotorRun(0,'forward',speed)
        self.MotorRun(1,'backward',speed)
        self.MotorRun(2,'forward',speed)
        self.MotorRun(3,'backward',speed)
        if t_time > 0: time.sleep(t_time)  # 等幾秒，可以有小數點

    def backward_Left(self,speed,t_time):   # 後左退
        self.MotorRun(0,'backward',speed)
        self.MotorStop(1)
        self.MotorStop(2)
        self.MotorRun(3,'backward',speed)
        if t_time > 0: time.sleep(t_time)  # 等幾秒，可以有小數點

    def backward_Right(self,speed,t_time):  # 後右退
        self.MotorStop(0)
        self.MotorRun(1,'backward',speed)
        self.MotorRun(2,'backward',speed)
        self.MotorStop(3)
        if t_time > 0: time.sleep(t_time)  # 等幾秒，可以有小數點

    def t_stop(self, t_time):
        for m in range(4):
            self.MotorStop(m)
        self.pwm.setPWM(9, 0, 0)            # 停止頂部舵機控制
        self.pwm.setPWM(10, 0, 0)           # 停止底座舵機控制
        if t_time > 0:
            time.sleep(t_time)

    def set_servo_pulse(self, channel, pulse_us):
        pulse_length = 1000000 // 60 // 4096
        pulse = pulse_us * 1000 // pulse_length
        self.pwm.setPWM(channel, 0, pulse)

    def set_servo_angle(self, channel, angle, t_time):
        pulse_width_us = (angle * 11) + 500
        duty_cycle = int(4096 * pulse_width_us / 20000)
        self.pwm.setPWM(channel, 0, duty_cycle)
        if t_time > 0:
            time.sleep(t_time)
            
    def stop_servo_angle(self, channel):
        self.pwm.setPWM(channel, 0, 0)

if __name__ == "__main__":
    robot = LOBOROBOT()

    robot.set_servo_angle(10, 70, 1)
    robot.set_servo_angle(10, 110, 1)
    robot.set_servo_angle(9, 25, 1)
    robot.set_servo_angle(9, 50, 1)

    robot.move(FORWARD, 50, 2)
    robot.t_stop(1)
    robot.move(BACKWARD, 50, 2)
    robot.t_stop(1)
