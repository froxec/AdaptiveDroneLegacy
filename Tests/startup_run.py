from gpiozero import Buzzer
from time import sleep

buzzer = Buzzer(23)
for i in range(3):
    buzzer.on()
    sleep(0.2)
    buzzer.off()
    sleep(0.2)  
buzzer.on()
sleep(0.5)
buzzer.off()
sleep(0.1)
buzzer.on()
sleep(0.2)
buzzer.off()