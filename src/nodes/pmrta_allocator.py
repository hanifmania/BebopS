#!/usr/bin/env python

from cmath import inf
from distutils import text_file
from socket import TCP_QUICKACK
from turtle import end_fill
from unittest import result
from matplotlib.pyplot import axis
import timeit
from scipy.optimize import linprog

import sys
import itertools
import scipy
import rospy
import math
import numpy as np
from numpy import dtype, random, zeros
from numpy import linalg as LA
#import random
from std_msgs.msg import Int64
from std_msgs.msg import Int64MultiArray
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from task_switch.msg import MRTAProblem
from task_switch.msg import PMRTAResult


#from mrta_msgs.msg import MRTAProblem

class PMRTA_Allocator():

    def __init__(self):
        # ROS Initialize
        rospy.init_node('pmrta_allocator', anonymous=True)
        self.rate = rospy.Rate(10) # 10Hz
        #self.allocation_result = 10
        #self.interception_point = np.zeros([self.robot_number,self.task_number],dtype=float)
        
        # Initialize Publisher & Subscriber
        #self.pub_mrta_problem = rospy.Publisher("/mrta_problem_generator", MRTAProblem, queue_size=10)
        self.pub_pmrta_allocation_result = rospy.Publisher("/pmrta_allocation_result", PMRTAResult, queue_size=10)
        self.allocation_result = PMRTAResult()
        
        self.update_mrta_problem = rospy.Subscriber("/mrta_problem_set", MRTAProblem, self.callback_problem)
        self.update_robot_data = rospy.Subscriber("/robot_data_update", Int64, self.callback_robot)
        self.update_task_data = rospy.Subscriber("/task_data_update", Int64, self.callback_task)
        self.update_allocation = rospy.Subscriber("/allocation_update", Int64, self.callback_allocation)
        self.update_mission_status = rospy.Subscriber("/is_mission_finished", Bool, self.callback_mission_status)
        
        #self.reset_service = rospy.Service("/reset_counter", SetBool, self.callback_reset_counter)


    def callback_problem(self, msg):
        # Subscriber to get the set of MRTA problem parameter published by mrta_problem_generator node

        # Update Problem Parameter
        self.robot_number = msg.robot_number
        self.robot_position_x = msg.robot_position_x
        self.robot_position_y = msg.robot_position_y
        self.robot_velocity = msg.robot_velocity
        self.robot_battery_level = msg.robot_battery_level
        self.robot_battery_discharge_rate = msg.robot_battery_discharge_rate
        self.robot_lambda = msg.robot_lambda
        
        self.task_number = msg.task_number
        print('Number of Task is : ', self.task_number)
        self.task_position_x = msg.task_position_x
        self.task_position_y = msg.task_position_y
        self.task_velocity_x = msg.task_velocity_x
        self.task_velocity_y = msg.task_velocity_y
        self.task_priority = msg.task_priority
        self.task_operation_time = msg.task_operation_time

        self.time_horizon = msg.time_horizon

        # Load Initial Robot Position (From Problem)
        self.robot_position = np.array([(self.robot_position_x),(self.robot_position_y)])
       
        # Load Initial Task Position (From Problem)
        self.task_position = np.array([(self.task_position_x),(self.task_position_y)])
        self.task_velocity = np.array([(self.task_velocity_x),(self.task_velocity_y)])
        

        # print('R : ', self.robot_number)
        # print('T : ', self.task_number)
        # print('p_o : ', self.robot_position)
        # print('q_o : ', self.task_position)
        # print('q_dot : ', self.task_velocity)
        # print('v : ', self.robot_velocity)
        # print('lambda : ', self.robot_lambda)
        # print('b : ', self.robot_battery_level)
        # print('w : ', self.robot_battery_discharge_rate)
        # print('phi : ', self.task_priority)
        # print('psi : ', self.task_operation_time)

        # Initialize Variable Array for Evolution Estimation Calculation 
        self.d = np.zeros([self.robot_number,self.task_number,self.time_horizon],dtype=float) # Distance Matrix from robot i to interception point with task j in time horizon k
        self.t = np.zeros([self.robot_number,self.task_number,self.time_horizon],dtype=float) # Time Matrix (t_ijk) required by robot i to achieve the interception point with task j in time horizon k
        self.t_k = np.zeros((self.time_horizon),dtype=float) # Compute t^k (estimated time to accomplish tasks in certain k)
        self.t_E = np.zeros([self.robot_number,self.task_number,self.time_horizon],dtype=float)
        self.t_ak = np.zeros((self.time_horizon),dtype=float) # Estimated accumulated time in k
        self.t_ak[0] = 0 # Initial Time before the mission

        # Evolution Estimation
        self.d, self.t_E, self.t_k, self.t_ak = self.evolution_estimation(self.robot_position, self.robot_velocity, self.task_position, self.task_velocity, self.time_horizon, self.task_operation_time)
        
        
        # print('d matrix is : ', self.d)
        # print('t_E matrix is : ', self.t_E)
        # print('t_k matrix is : ', self.t_k)
        # print('t_ak matrix is : ', self.t_ak)
        
        # Compute Linprog
        self.allocation_result.allocation_result, self.cost_value, self.computational_burden = self.LP_solver_KR_geq_T(self.robot_number,self.task_number,self.time_horizon,self.robot_lambda,self.d,self.task_priority,self.t_E,self.robot_battery_level,self.robot_battery_discharge_rate)
        self.allocation_result.robot_number = self.robot_number 
        self.allocation_result.task_number = self.task_number 
        self.allocation_result.time_horizon = self.time_horizon
       
       
        self.pub_pmrta_allocation_result.publish(self.allocation_result)
        # Transform the allocation result into understandable form

        # self.allocation_result = np.zeros([self.robot_number,self.task_number,self.time_horizon],dtype=float)
        # self.target_waypoint = np.zeros([self.robot_number,self.time_horizon],dtype=int)#,dtype= int)

        # count = 0

        # for i in range(0,self.robot_number):
        #     for j in range(0, self.task_number):
        #         for k in range(0,self.time_horizon):
        #             self.allocation_result[i,j,k] = np.round(self.allocation_raw_result[count],decimals=0).astype(int)
        #             count = count + 1
        #             if (self.allocation_result[i,j,k] == 1):
        #                 self.target_waypoint[i,k] = j+1
         
        # print(self.target_waypoint)
        

        
    def compute_interception(self,p_i,q_j,vq_j,vp_i):
        # p_i is a robot position in one-dimensional vector of 2 values (x,y)
        # q_j is a task position in one-dimensional vector of 2 values (x,y)
        # vq_j is a task velocity in one-dimensional vector of 2 values (vx,vy)
        # v_i is a robot velocity in one-dimensional vector of 2 values (vx,vy)
       
        delta_x = q_j[0] - p_i[0]
        delta_y = q_j[1] - p_i[1]

        a = vq_j[0]**2 + vq_j[1]**2 - vp_i**2
        b = 2*(vq_j[0]*delta_x + vq_j[1]*delta_y)
        c = delta_x**2 + delta_y**2
        D = b**2 - 4*a*c

        t1 = (-b + math.sqrt(D))/(2*a)
        t2 = (-b - math.sqrt(D))/(2*a)

        if (D >= 0):
            if (t1*t2 <= 0):
                t = max(t1,t2)
            else:
                if(t1>0 and t2>0):
                    t = min(t1,t2)
                else:
                    t = inf
        else:
            t = inf
        
 
        result_interception = q_j + vq_j*t
        
        return result_interception
        

    def evolution_estimation(self, p_i, vp_i, q_j, vq_j, K, psi):

        R = np.shape(p_i)[1] # Number of Robots
        T = np.shape(q_j)[1] # Number of Tasks

        d = np.zeros([R,T,K],dtype=float) # Distance Matrix from robot i to interception point with task j in time horizon k
        d_E = np.zeros([R,T,T],dtype=float) # Distance Estimation in time frame k+1
        t = np.zeros([R,T,K],dtype=float) # Time Matrix (t_ijk) required by robot i to achieve the interception point with task j in time horizon k
        t_k = np.zeros((K),dtype=float) # Compute t^k (estimated time to accomplish tasks in certain k)
        t_E = np.zeros([R,T,K],dtype=float)
        t_ak = np.zeros((K),dtype=float) # Estimated accumulated time in k
        t_ak[0] = 0 # Initial Time before the mission
       
        q_k = np.zeros((2,T,K),dtype=float) # Estimated position of task in k
        q_k[:,:,0] = q_j # Initial Position of Task during k = 0


        for k in range(0,K):
            
            if (k == 0): # Compute Initial interception & d matrix
                for i in range(0,R):
                    for j in range(0,T):
                        # Compute Intersection
                        result_interception = self.compute_interception(p_i[:,i],q_k[:,j,k],vq_j[:,j],vp_i[i])
                        d[i,j,k] = np.linalg.norm(result_interception-p_i[:,i])
                        
            # Compute t_ijk
            for i in range(0,R):
                t[i,:,k] = d[i,:,k]/vp_i[i]

            # if (k==0):
            #     print('tijk = ',t[:,:,0])
            #     print('psi = ',psi)
            #     print('tijk + psi= ',t[:,:,0] + psi)
            #     print('sum pertama = ',np.sum(t[:,:,0] + psi,axis=0))
            #     print('sum kedua = ',np.sum(np.sum(t[:,:,0] + psi,axis=0)))
                
                
                

            # Compute t^k
            
            t_k[k] = (np.sum(np.sum(t[:,:,k]+psi,axis=0)))/(T*R)
            
            # for i in range(0,R):
            #     for j in range(0,T):
            #         t_k[k] = (t[i,j,k] + psi[j])/(T+R)
            
            # Update Estimated Accumulated Time
            if (k+1 < K):
                t_ak[k+1] = t_ak[k] + t_k[k]
            
            # Compute t^E in k
            for i in range(0,R):
                for j in range(0,T):
                    t_E[i,j,k] =  t_ak[k] + t[i,j,k] + psi[j]
            
            # Compute Estimated Position of Task in K + 1
            if(k+1 < K):
                q_k[:,:,k+1] = q_k[:,:,k] + t_k[k]*vq_j

            # Compute d matrix for K > 1
            if(k+1 < K):
                if (T > 1):
                    for i in range(0,R):
                        for j in range(0,T):
                            task_number_modified = np.concatenate((np.arange(0,j),np.arange(j+1,T)))
                            for m,n in enumerate(task_number_modified):
                                result_interception = self.compute_interception(q_k[:,n,k],q_k[:,j,k+1],vq_j[:,j],vp_i[i])
                                d_E[i,j,n] = np.linalg.norm(result_interception-q_k[:,j,k])
                

                    d[:,:,k+1] = d_E.mean(axis=2)

        return  d, t_E, t_k, t_ak 


    def LP_solver_KR_geq_T(self,R,T,K,lamda,d,phi,t_E,battery_levels,w):
        
        # Create c
        c = np.zeros((1,R*T*K))

        counter = 0

        for i in range(0,R):
            for j in range(0,T):
                for k in range(0,K):
                    c[0,counter] = (k+1)*(lamda[i]*d[i,j,k] + phi[j]*t_E[i,j,k])
                    counter = counter + 1
                    
        
        # Create Beta
        Beta = np.zeros((R,T,K))

        for i in range(0,R):
            for j in range(0,T):
                for k in range(0,K):
                    if ((battery_levels[i] - w[i]*t_E[i,j,k]) >= 0):
                        Beta[i,j,k] = 1
                    elif ((battery_levels[i] - w[i]*t_E[i,j,k]) < 0):
                        Beta[i,j,k] = -1
        
        # Create Tetha
        Theta = np.zeros((R,K))

        for i in range(0,R):
            for k in range(0,K):
                sumBeta = Beta.sum(axis = 1) + T
                sumBeta = np.reshape(sumBeta,(R,1,K))
                if (sumBeta[i,0,k] > 0):
                    Theta[i,k] = 1
                elif (sumBeta[i,0,k] + T <= 0):
                    Theta[i,k] = 0
        
        #Create b 
        b = np.ones((T,1))
        Theta_Array = np.zeros((R*K,1))

        counter = 0

        for i in range(0,R):
            for k in range(0,K):
                Theta_Array[counter,0] = Theta[i,k]
                counter = counter + 1
        
        b = np.vstack((b,Theta_Array))

        # Create A

        # Matrix M
        M_submatrix = np.zeros((T,T*K))        

        for j1 in range(0,T):
            for j2 in range(0,T):
                if (j1 == j2):
                    M_submatrix[j1,(j2)*K:(j2+1)*K] = np.ones((1,K))
                else:
                    M_submatrix[j1,(j2)*K:(j2+1)*K] = np.zeros((1,K))

        
        M = M_submatrix
        
        for i in range(1,R):
            M = np.hstack((M,M_submatrix))

        # Matrix N
        N_submatrix = np.zeros((R*K,T*K,R))
        # print(N_submatrix.shape)
        # print(N_submatrix[:,:,0].shape)
        # print(np.eye(K).shape)
        for i1 in range(0,R):
            for i2 in range(0,R):
                for j1 in range(0,T):
                    if (i2 == i1):
                        N_submatrix[i2*K:(i2+1)*K,j1*K:(j1+1)*K,i1] =  np.eye(K)
                    else:
                        N_submatrix[i2*K:(i2+1)*K,j1*K:(j1+1)*K,i1] = np.zeros((K,K))

        N = N_submatrix[:,:,0]
        
        for i1 in range(1,R):
            N = np.hstack((N,N_submatrix[:,:,i1]))

        # Matrix A
        A = np.hstack((M,np.zeros((T,R*K))))
        A_submatrix = np.hstack((N,np.eye(R*K)))
        

        A = np.vstack((A,A_submatrix))

        # Hanif = np.arange(1,10,1,dtype=int)
        # print('Hanif : ',Hanif)
        # print('Hanif 3 to end : ',Hanif[2:])
        # print('Hanif start to  3 : ',Hanif[:2])
        
        # Linprog Code
        # c = c.flatten()

        # print('c shape : ', c.shape)
        # print('A_ub shape : ', N.shape)
        # print('b_ub shape : ', b[T:].shape)
        # print('A_eq shape : ', M.shape)
        # print('b_eq shape : ', b[:T].shape)

        N = N.astype(int)
        b = b.astype(int)
        M = M.astype(int)

        # print('c shape : ', c.dtype)
        # print('A_ub shape : ', N.dtype)
        # print('b_ub shape : ', b[T:].dtype)
        # print('A_eq shape : ', M.dtype)
        # print('b_eq shape : ', b[:T].dtype)
        
        start = timeit.default_timer()
        res =  linprog(c,A_ub=N,b_ub=b[T:],A_eq=M,b_eq=b[:T],bounds=(0,1))
        
        stop = timeit.default_timer()
        # print('Time : ', stop-start)
        # print(np.round(res.x,decimals=2))
        # print('Sum of SOlution : ', np.sum(res.x))
        # print(np.round(res.x,decimals=0).astype(int))

        allocation_result = np.round(res.x,decimals=0).astype(int)
        cost_value = res.fun
        computational_burden = stop - start

        return allocation_result, cost_value, computational_burden
        
        
        #self.pub_pmrta_allocation_result.publish(self.allocation_result)

    def callback_robot(self, msg):
        # subscriber to get status of mission success whether it is already finished or not
        self.robot_data = msg.data

        # Update Robot Position everytime new data is coming (pi' = pi)

    def callback_task(self, msg):
        # subscriber to get status of mission success whether it is already finished or not
        self.task_data = msg.data

        # Update Task Position everytime new data is coming (qi' = qi)

    def callback_allocation(self, msg):
        # subscriber to get status of mission success whether it is already finished or not
        self.task_data = msg.data

        # Update Allocat

    def callback_mission_status(self, msg):
        # subscriber to get status of mission success whether it is already finished or not
        self.mission_status = msg.data
        self.allocation_result = self.mission_status

       
    def spin(self):
        while not rospy.is_shutdown():
            #rospy.loginfo("Problem Deployed")
            # self.pub_pmrta_allocation_result.publish(self.allocation_result)
            self.rate.sleep()

    # def checkRule1(A):
    #     n,m = A.shape
    #     '''check if every element in A is 0,1 or -1'''
    #     for i in range(m):
    #         for j in range(n):
    #             if A[i][j] != 0 and A[i][j] !=1 and  A[i][j] != -1:
    #                 return False
    #     else:
    #         return True

    # def checkRule2(A,r,c):
    #     '''Check if one sub matriz have determinant 0,1 or -1'''
    #     det=A[r[0]][c[0]]*A[r[1]][c[1]]- A[r[1]][c[0]]*A[r[0]][c[1]]
    #     if det == 1 or det == -1 or det==0:
    #         return True
    #     else:
    #         return False
    
    # def theoremNo2det(A):
    #     if (self.checkRule1(A)):
    #         n,m=A.shape
    #         N=range(n)
    #         M=range(m)
    #         rows=itertools.permutations(N,2)
    #         columns=itertools.permutations(M,2)
    #         for r in rows:
    #             for c in columns:
    #                 if not self.checkRule2(A,r,c):
    #                     return False 
    #         return True

if __name__ == '__main__':
    try:
        pmrta_allocator = PMRTA_Allocator()
        pmrta_allocator.spin()

    except rospy.ROSInterruptException:
        pass