import numpy as np

class Empty:
    def __init__(self):
        pass
    
class PositionControl:
    ''' A PID position control ler with friction compensation GAAAAY
            u = kp*(q_ref-q) + kd*(dq_ref-dq) + ki*integral(q_ref-q) + u_friction
        where u_friction can be computed either with the sign function:
            u_friction = sign(dq)*tau_c
        or with the tanh function:
            u_friction = tanh(tanh_gain*dq)*tau_c
        where tanh_gain is a user-specified parameter that defines how closely the 
        tanh function should approximate the sign function.
    '''
    def __init__(self, kp, kd, ki, dt, motor_params, tanh_fric_comp, tanh_gain=1.0):
        '''
            tanh_fric_comp: If True then friction is compensated using the tanh function,
                            if False then friction is compensated using the sign function
            tanh_gain: The gain used inside the tanh function
        '''
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        tau_coulomb = motor_params.tau_coulomb_gear/motor_params.N + motor_params.tau_coulomb
        self.i_coulomb = tau_coulomb / motor_params.K_b
        self.tanh_fric_comp = tanh_fric_comp
        self.tanh_gain = tanh_gain
        self.error_integral = 0.0

    def compute(self, q, dq, q_ref, dq_ref):
        u = 0.0
        if(self.tanh_fric_comp):
            u += 0.0
            u = self.kp*(q_ref-q) + self.kd*(dq_ref-dq) + self.error_integral*self.dt + self.i_coulomb*np.tanh(self.tanh_gain*dq)
        else:
            u += 0.0
            u = self.kp*(q_ref-q) + self.kd*(dq_ref-dq) + self.error_integral*self.dt + self.i_coulomb*np.sign(dq)
        return u

def run_simulation(N, dt, kp, kd, ki, motor_params, tanh_fric_comp, tanh_gain):
    # create motor and position controller
    motor = MotorWGear(dt, motor_params)
    controller = PositionControl(kp, kd, ki, dt, motor_params, tanh_fric_comp, tanh_gain)

    # simulate motor with linear+sinusoidal reference motor angle
    res = Empty()
    res.q = np.zeros(N)
    res.dq = np.zeros(N)
    res.dq_est = np.zeros(N)
    res.current = np.zeros(N)
    res.tau = np.zeros(N)
    res.tau_c = np.zeros(N)
    res.V = np.zeros(N)
    res.q_ref  = np.zeros(N)
    res.dq_ref = np.zeros(N)
    omega = 2*pi*q_w
    for i in range(N):
        t = i*dt
        # compute reference trajectory
        res.q_ref[i]  = q_a*t + q_b + q_A*sin(omega*t)
        res.dq_ref[i] = q_a + q_A*omega*cos(omega*t)
        res.q[i] = motor.q()
        res.dq[i] = motor.dq()
        res.dq_est[i] = res.dq[i] # ################################################+ 0.25*(random()-0.5) # add some random noise to the velocity estimation
        
        # compute motor current with controller
        res.current[i] = controller.compute(res.q[i], res.dq_est[i], res.q_ref[i], res.dq_ref[i])
        # send current to the motor
        motor.simulate(res.current[i])

        res.V[i] = motor.V()
        res.tau[i] = motor.tau()
        res.tau_c[i] = motor.tau_coulomb()
        
    print("Mean tracking error:", 1e3*norm(res.q-res.q_ref)/N)
    return res


def plot_stuff(res, title):
    # plot motor angle, velocity and current
    f, ax = plt.subplots(3,1,sharex=True)
    time = np.arange(0.0, T, dt)
    time = time[:N]
    ax[0].set_title(title)
    ax[0].plot(time, res.q, label ='q')
    ax[0].plot(time, res.q_ref, '--', label ='reference q')
    ax[1].plot(time, res.dq, label ='dq')
    ax[1].plot(time, res.dq_est, '.', label ='dq est')
    ax[1].plot(time, res.dq_ref, '--', label ='reference dq')
    ax[-1].plot(time, res.tau, label=r'$\tau$')
    ax[-1].plot(time, res.tau_c, label=r'$\tau_c$')
    for i in range(len(ax)): ax[i].legend()
    plt.xlabel('Time [s]')
    
def manage_cose(res, some, i ):
    some[i] = 1e3*norm(res.q-res.q_ref)/N
    return some
    
if __name__=='__main__':
    import arc.utils.plot_utils as plut
    from numpy import sin, cos, pi
    import matplotlib.pyplot as plt
    from dc_motor_w_gear import MotorWGear, get_motor_parameters
    from random import random
    from numpy.linalg import norm
    
    np.set_printoptions(precision=1, linewidth=200, suppress=True)
    
    dt = 1e-3               # controller time step
    T = 2                 # simulation time
    N = int(T/dt)           # number of time steps
    motor_name = 'Maxon148877'    
    kp = 1.0    # proportional gain
    kd = 1.0     # derivative gain            
    ki = 0.0        # integral gain
    q_b = 0.0       # initial motor angle
    q_a = 0.        # linear increase in motor angle per second
    ################################################
    q_A = 1.0       # amplitude of sinusoidal reference motor angle
    motor_params = get_motor_parameters(motor_name)
    time = np.zeros(N)
    
    # uncomment the following 2 lines if you wanna set Coulomb friction to zero
#    params.tau_coulomb_gear = 0.0
#    params.tau_coulomb = 0.0
    NN=int(100)
    step=0.01
    
    to_plot_sign = np.zeros(NN)
    to_plot_tanh_1 = np.zeros(NN)
    to_plot_tanh_2 = np.zeros(NN)
    f = open( 'for_plot.txt', 'w' )
    for q in range(NN):
        q_w = step*q
        print("\n Actual q_w = ", q_w , "\n")
        
        print("Friction compensation with sign")
        res_sign = run_simulation(N, dt, kp, kd, ki, motor_params, tanh_fric_comp=False, tanh_gain=0.0)
        print("Friction compensation with tanh(0.01*dq)")
        res_tanh_1  = run_simulation(N, dt, kp, kd, ki, motor_params, tanh_fric_comp=True, tanh_gain=0.01)
        print("Friction compensation with tanh(2*dq)")
        res_tanh_2 = run_simulation(N, dt, kp, kd, ki, motor_params, tanh_fric_comp=True, tanh_gain=2.0)
        
        manage_cose(res_sign,to_plot_sign,q)
        manage_cose(res_tanh_1,to_plot_tanh_1,q)
        manage_cose(res_tanh_2,to_plot_tanh_2,q)
        
       # to_plot_sign[q] = res_sign.q
       # to_plot_tanh_1[q] = res_tanh_1
       # to_plot_tanh_2[q] = res_tanh_2
        
        f.write( repr(to_plot_sign[q])+"\n" )
    f.close()
    frequency= np.linspace(0.0,step*float(NN),NN)
    S = zip(frequency,to_plot_sign,to_plot_tanh_1,to_plot_tanh_2)
    import csv
    with open('plot.csv','w') as f:
        writer = csv.writer(f,delimiter='\t')
        writer.writerows(zip(frequency,to_plot_sign,to_plot_tanh_1,to_plot_tanh_2))
    #frequency=frequency[:NN]
    #to_plot_sign = to_plot_sign[:NN]
    #plt(to_plot_sign)
    #plt.show()
      #plot_stuff(res_sign,  "sign(dq)")
      #plt.axis([0, T, -20 ,20])
      #plot_stuff(res_tanh_1,"tanh(0.01*dq)")
      #plt.axis([0, T, -20 ,20])
      #plot_stuff(res_tanh_2,"tanh(2*dq)")
      #plt.axis([0, T, -20 ,20])
      #plt.show()
      

