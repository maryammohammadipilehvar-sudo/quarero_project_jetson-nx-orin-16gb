import Jetson.GPIO as GPIO

PIN = 38

GPIO.setmode(GPIO.BOARD)
GPIO.setup(PIN, GPIO.OUT, initial=GPIO.LOW)  # Pin 7 als Output

GPIO.output(PIN, GPIO.HIGH)
GPIO.output(PIN, GPIO.LOW)

GPIO.cleanup()