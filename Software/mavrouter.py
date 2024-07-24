import subprocess

cmd = 'mavproxy.py --master=/dev/ttyAMA0 --out=udp:192.168.0.109:14550 --out=udp:192.168.0.106:14550'
result = subprocess.run([cmd],shell=True)

print(result.stdout)