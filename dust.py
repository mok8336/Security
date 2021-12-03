import blynklib
import blynktimer
import time
import led
import datetime
import serial
import RPi.GPIO as GPIO
import I2C_LCD_driver
from PMS7003 import PMS7003
from datetime import datetime
BLYNK_AUTH = '8vL0Y4i4RAUCCa51tB4UxGCh5adkXC-j'
#=======================================
#led
GPIO.setmode(GPIO.BOARD)
GPIO.setup(38, GPIO.OUT)
GPIO.setup(36, GPIO.IN, GPIO.PUD_DOWN)

WRITE_EVENT_PRINT_MSG = "[WRITE_VIRTUAL_PIN_EVENT] Pin: V{} Value: '{}'"
READ_PRINT_MSG = "[READ_VIRTUAL_PIN_EVENT] Pin: V{}"

# blynk앱에서 버튼 누를경우 동작 - write (Virtual Pins 1)
@blynk.handle_event('write V1')
def write_virtual_pin_handler(pin, value):
  print(WRITE_EVENT_PRINT_MSG.format(pin,value))

  if(value == ['1']):
    GPIO.output(38, 1)
  else:
    GPIO.output(38, 0)
# blynk앱에서 주기적으로 호출하도록 설정 - Read (Virtual Pins 3)
@blynk.handle_event('read V3')
def read_virtual_pin_handler(pin):
  
  SW = GPIO.input(36)

  try:  
    if(SW != read_virtual_pin_handler.lastSW):
      print(READ_PRINT_MSG.format(pin))
      print("SW GPIO value : %d "% (SW))

      if(SW):
        blynk.virtual_write(3, " ON")
        read_virtual_pin_handler.lastSW = SW
        
      else:
        blynk.virtual_write(3, "OFF")
        read_virtual_pin_handler.lastSW = SW
  except AttributeError:
    read_virtual_pin_handler.lastSW = False
def my_user_task():

  try:
    if(my_user_task.LED_flag):
      blynk.virtual_write(4, 255)   # Vpin =  V4, value = 255
      my_user_task.LED_flag = False
      #print("V4 LED ON")
    else:
      blynk.virtual_write(4, 0)     # Vpin =  V4, value = 0
      my_user_task.LED_flag = True
      #print("V4 LED OFF")

  # 최초 1회 변수 선언
  except AttributeError:
    my_user_task.LED_flag = True
    blynk.virtual_write(4, 255)
    #print("V4 LED ON")


# Start Blynk, S
===============================================================================
#servo
servoPin = 12
SERVO_MAX_DUTY = 12
SERVO_MIN_DUTY = 3

GPIO.setmode(GPIO.BOARD)
GPIO.setup(servoPin, GPIO.OUT)
servo = GPIO.PWM(servoPin, 50)
servo.start(0)

def setServoPos(degree):
  # 각도는 180도를 넘을 수 없다.
  if degree > 180:
    degree = 180
  # 각도(degree)를 duty로 변경한다.
  duty = SERVO_MIN_DUTY+(degree*(SERVO_MAX_DUTY-SERVO_MIN_DUTY)/180.0)
  # duty 값 출력
  print("Degree: {} to {}(Duty)".format(degree, duty))

  # 변경된 duty값을 서보 pwm에 적용
  servo.ChangeDutyCycle(duty)
#========================================
# Baud Rate
Speed = 9600

# UART / USB Serial
USB0 = '/dev/ttyUSB0'
UART = '/dev/ttyAMA0'
# USE PORT
SERIAL_PORT = USB0  #기본값 USB0, 연결방식에 맞춰 변경

#serial setting
ser = serial.Serial(SERIAL_PORT, Speed, timeout = 1)
#========================================

# Initialize Blynk
blynk = blynklib.Blynk(BLYNK_AUTH)

# Create BlynkTimer Instance
timer = blynktimer.Timer()

# Create Dust sensor
dust = PMS7003()
# Add Timers
# timer : 설정해 둔 시간마다 실행됨
@timer.register(interval=2, run_once=False) # 2초마다 반복실행
def my_user_task():
  # do any non-blocking operations

  ser.flushInput()
  buffer = ser.read(1024)

  if(dust.protocol_chk(buffer)):
    data = dust.unpack_data(buffer)

    print("send - PM1.0: %d | PM2.5: %d | PM10: %d" %(data[dust.DUST_PM1_0_ATM],data[dust.DUST_PM2_5_ATM],data[dust.DUST_PM10_0_ATM]) )
    # Labeled Value (Display)
    blynk.virtual_write(7, data[dust.DUST_PM1_0_ATM])
    blynk.virtual_write(8, data[dust.DUST_PM2_5_ATM])
    blynk.virtual_write(9, data[dust.DUST_PM10_0_ATM])
    #LCD Display
    mylcd = I2C_LCD_driver.lcd()
    mylcd.lcd_display_string("PM1.0: %s" % data[dust.DUST_PM1_0_ATM],1)
    mylcd.lcd_display_string("PM10 : %s" % data[dust.DUST_PM10_0_ATM],2)
    # Value Display
    #blynk.virtual_write(7, ("PM1.0 : " + str(data[dust.DUST_PM1_0_ATM])))
    #blynk.virtual_write(8, ("PM2.5 : " + str(data[dust.DUST_PM2_5_ATM])))
    #blynk.virtual_write(9, ("PM10  : " + str(data[dust.DUST_PM10_0_ATM])))
    #motor

    blynk.virtual_write(6,'0')
    if data[dust.DUST_PM1_0_ATM] > 30:
        setServoPos(130)
        time.sleep(1)
    if data[dust.DUST_PM1_0_ATM] < 30:
        setServoPos(30)
        time.sleep(1)
  else:
    # protocol_chk fail

    print("data Err")
    blynk.virtual_write(6,'255')

#time stamp
# Start Blynk, Start timer
while True:
  now = datetime.now()

  current_time = now.strftime("%H:%M:%S")
  print("현재 시간=", current_time)
  time.sleep(0.5)
  blynk.run()
  timer.run()
  inputIO = GPIO.input(36)
  if inputIO == False:
      GPIO.output(38, GPIO.LOW)
  else:
      GPIO.output(38, GPIO.HIGH)


