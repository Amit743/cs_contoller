# -*- coding: utf-8 -*-
#!/usr/bin/env python

import roslib
import rospy

import numpy as np
import individual as id
import function as fn
from config import Config as cf

from hammerhead_control.msg import Errors
from pid_controller.msg import PID


class cs_node(object):
    def __init__(self, PopulationSize, MaxDomain, MinDomain, Lambda, Pa, Step_Size, Dimension, Trial, Iteration):
        self.PoppulationSize = PopulationSize
        self.MaxDomain = MaxDomain
        self.MinDomain = MinDomain
        self.Lambda = Lambda
        self.Pa = Pa
        self.Step_Size = Step_Size
        self.Dimension = Dimension
        self.Trial = Trial
        self.Iteration = Iteration

        self.err = 0
        self.int_err = 0
        self.prev_err = 0
        self.status = 0

        # subscriber
        self.sum_err = 0
        rospy.Subscriber('genetic',Errors,self.run_cso)
        self.pid_vals_=[0,0,0]

    def run_cso(self, msg):

        err_ = list(msg.error)
        self.sum_err += (err_[5])
        err = 0
        if(self.status != 0):
            return
        self.status = 2

        self.err = err
        if(abs(self.int_err)>20):
            self.int_err = 0
        else:
            self.int_err += err
        """Assuming initial best values"""
        FinalBestPosition = [-1 -1 -1]
        FinalBestFitness = 0.00
        for trial in range(self.Trial):
        	np.random.seed(trial)

        	results_list = [] # fitness list
        	cs_list = []
        	"""Generate Initial Population"""
        	for p in range(self.PopulationSize):
        	    cs_list.append(id.Individual())

        	"""Sort List"""
        	cs_list = sorted(cs_list, key=lambda ID: ID.get_fitness())

        	"""Find Initial Best"""
        	TrialBestPosition = cs_list[0].get_position()
        	TrialBestFitness = fn.calculation(cs_list[0].get_position(),0)

        	"""↓↓↓Main Loop↓↓↓"""
        	for iteration in range(self.Iteration):

        	    """Generate New Solutions"""
        	    for i in range(len(cs_list)):
        	        cs_list[i].get_cuckoo()
        	        cs_list[i].set_fitness(fn.calculation(cs_list[i].get_position(),iteration))
	
        	        """random choice (say j)"""
        	        j = np.random.randint(low=0, high=self.PopulationSize)
        	        while j == i: #random id[say j] ≠ i
        	            j = np.random.randint(0, self.PopulationSize)
	

        	        if(cs_list[i].get_fitness() > cs_list[j].get_fitness()):
        	            cs_list[j].set_position(cs_list[i].get_position())
        	            cs_list[j].set_fitness(cs_list[i].get_fitness())
	
        	    """Sort (to Keep Best)"""
        	    cs_list = sorted(cs_list, key=lambda ID: ID.get_fitness())
	
        	    """Abandon Solutions (exclude the best)"""
        	    for a in range(1,len(cs_list)):
        	        r = np.random.rand()
        	        if(r < cf.get_Pa()):
        	            cs_list[a].abandon()
        	            cs_list[a].set_fitness(fn.calculation(cs_list[a].get_position(),iteration))

        	    """Sort to Find the Best"""
        	    cs_list = sorted(cs_list, key=lambda ID: ID.get_fitness())
	
        	    if cs_list[0].get_fitness() > TrialBestFitness:
        	        TrialBestFitness = cs_list[0].get_fitness()
        	        TrialBestPosition = cs_list[0].get_position()
                    
                    """Updating Final Best values"""
                    if TrialBestFitness > FinalBestFitness:
                        FinalBestFitness = TrialBestFitness
                        FinalBestPosition = TrialBestPosition
	
        	    """sys.stdout.write("\r Trial:%3d , Iteration:%7d, BestFitness:%.4f" % (trial , iteration, BestFitness))"""

        	    """results_list.append(str(TrialBestPosition))
	
        results_writer.writerow(results_list)"""
        p_ = int(FinalBestPosition[0])
        i_ = int(FinalBestPosition[1])
        d_ = int(FinalBestPosition[2])
        self.pid_vals_[0] = p_
	self.pid_vals_[1] = i_
	self.pid_vals_[2] = d_
        self.prev_err = err
        self.status = 1
        
def main():
    rospy.init_node('cuckoo_search')
    try:
    	cs_pub = rospy.Publisher('/pid_genetic_params', PID, queue_size = 10)
     


     	gen_params = PID()
     	gen_params.kp = [0,0,0,0,0,0]
     	gen_params.ki = [0,0,0,0,0,0]
     	gen_params.kd = [0,0,0,0,0,0]
     

     	cso_obj = cs_node(PopulationSize = 25, MaxDomain = 1, MinDomain = 0, Lambda = 1.5, Pa = 0.25, Step_Size = 0.01, Dimension = 3, Trial = 10, Iteration = 500)
     	while not rospy.is_shutdown():
      
      		if(cso_obj.status == 1):
   
      			gen_params.kp[5] = cso_obj.pid_vals_[0]
      			gen_params.ki[5] = cso_obj.pid_vals_[1]
      			gen_params.kd[5] = cso_obj.pid_vals_[2]
   
      	 		cso_obj.status = 0
      			cs_pub.publish(gen_params)
    except rospy.ROSInterruptException: pass
       

if __name__ == '__main__':
    main()
    results.close()
