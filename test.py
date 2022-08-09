import subprocess
import os



password = 'a'
command = 'sudo poweroff'
str = os.system('echo %s | sudo -S %s' % (password,command))
print(str)