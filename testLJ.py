from numpy import *
import u6

h = u6.U6()

while True:
    tF0 = h.getFeedback(u6.PortDirRead())
    tF1 = h.getFeedback(u6.PortStateRead())

    print tF0, tF1































