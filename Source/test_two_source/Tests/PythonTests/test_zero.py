

# Python
import sys
import os

# Project
path = os.path.dirname(__file__)
path = os.path.abspath(path + '/../../../../Install/lib/')
print path
sys.path.append(path)
os.environ['PATH'] = path + ';' + os.environ['PATH']
import dgv

print dgv.hello()
