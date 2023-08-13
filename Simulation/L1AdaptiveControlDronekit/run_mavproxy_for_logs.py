import subprocess
import datetime
DRONE_ADDR = "localhost:8000"
OUT = "udp:localhost:8005"
mission_name = datetime.datetime.now()
subprocess.run(["mavproxy.py", '--master={}'.format(DRONE_ADDR), '--aircraft=quad' '--out={}'.format(OUT), '--streamrate=-1',
                '--state-basedir=./logs/'])