from time import sleep
t_last = 0
omega_last = 0

R = 0.63
k = 0.002791
cmd = 117
V   = 0.000028424*cmd**3-0.012945569*cmd**2+2.025529854*cmd-102.8750848
b = 0.0053997
M = 3.63
r = 0.054
J = (M*r**2)/2
gR = 15.3

for s in range(0,20000,1):
	t = float(s/1000.0000)
        omega = float((V*(t-t_last)-((R*b+k**2*gR**2)/(k*gR))*omega_last*(t-t_last))/((R*J)/(k*gR)) + omega_last)
        t_last = t
        omega_last = omega
        print(omega)
        sleep(0.001)
