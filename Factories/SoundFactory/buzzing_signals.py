from time import sleep

def startup_signal(buzzer):
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

def vehicle_connected_signal(buzzer):
    for i in range(5):
        buzzer.on()
        sleep(0.4)
        buzzer.off()
        sleep(0.2)  
    buzzer.on()
    sleep(0.1)
    buzzer.off()