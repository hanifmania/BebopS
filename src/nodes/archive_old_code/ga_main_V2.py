####################### LIBRARIES #######################

import numpy as np
import random
import math 
import statistics

from numpy.random import randint
from numpy import linalg as LA

####################### GENETIC ALGORITHM FUNCTIONS #######################

def ga(Nr, Nt, p, q, v, q_p, tau, lambda_penalty, phi_penalty, n_gen, pop_size, r_cross, r_mut, elite_frac, crossover_frac):
    # start min and mean
    meanJ = np.zeros(n_gen)
    bestJ = np.zeros(n_gen)
    bestsol = list()
    # elite pop number
    n_elite_pop = int(pop_size*elite_frac)
    # crossover pop number
    n_crossover_pop = int((pop_size-n_elite_pop)*crossover_frac)
    # mutation pop
    n_mutation_pop = pop_size - n_crossover_pop - n_elite_pop
    # Initial pop
    ini_pop = ga_initial_pop(Nr, Nt, pop_size)
    # print(ini_pop) 
    pop_gen = list()
    pop_gen.append(ini_pop)
    for gen in range(n_gen):
        J = np.zeros(pop_size)
        actual_gen = pop_gen[gen]
        for sol in range(pop_size):
            # Compute J for all population
            solution = actual_gen[sol]
            # print(solution)
            J[sol] = objective_function(Nr, Nt, p, q, v, q_p, tau, lambda_penalty, phi_penalty, solution)
        # Sort population by J
        index_J = J.argsort()
        sorted_J = J.copy()
        sorted_J.sort()
        meanJ[gen] = statistics.mean(sorted_J)
        bestJ[gen] = sorted_J[0]
        sorted_actual_gen = sort_gen(actual_gen, index_J)
        bestsol.append(sorted_actual_gen[0]) 
        # Split pop in elite, crossover and mutation
        elite_pop = sorted_actual_gen[:n_elite_pop]
        # Create crossover sons
        crossover_pop = list()
        for crossover_index in range(math.ceil(n_crossover_pop/2)):
            p1_index = randint(0, n_elite_pop)
            p2_index = randint(0, n_elite_pop)
            while p2_index == p1_index:
                p2_index = randint(0, n_elite_pop)    
            p1 = elite_pop[p1_index]
            p2 = elite_pop[p2_index]
            s1, s2 = ga_crossover(p1, p2, r_cross, Nr, Nt)
            crossover_pop.append(s1)
            crossover_pop.append(s2)
        # Create mutation sons
        mutation_pop = list()
        for mutation_index in range(n_mutation_pop):
            p1_index = randint(0, n_elite_pop)
            p1 = elite_pop[p1_index]
            s1 = ga_mutation(p1, r_mut, Nr, Nt)
            mutation_pop.append(s1)
        # New gen
        new_gen = elite_pop + crossover_pop + mutation_pop        
        pop_gen.append(new_gen)
    best = bestsol[n_gen-1]
    score = bestJ[n_gen-1]
    return best, score

# Initial population
def ga_initial_pop(Nr, Nt, pop_size):
    ini_pop = list()
    tasks = list(range(Nt))
    tasks = incr(tasks, 1)
    for gen in range(pop_size):
        random.shuffle(tasks)
        performing_robot = list()
        for task in range(Nt):
            performing_robot.append(randint(1, Nr+1))
        ini_solution = np.zeros((Nr*Nt,), dtype=int)
        count = list()
        for robot in range(Nr):
            count.append(1)
        for task in range(Nt):
            robot_allocated = performing_robot[task]
            index_sol = (robot_allocated-1)*Nt+count[robot_allocated-1]-1
            ini_solution[index_sol] = tasks[task]
            count[robot_allocated-1] += 1
        ini_pop.append(ini_solution)
    return ini_pop

# J, objective function
def objective_function(Nr, Nt, p, q, v, q_p, tau, lambda_penalty, phi_penalty, solution):
    J = 0
    J_dist = 0
    J_time = 0
    moving_q = q.copy()
    solution_splitted = np.array_split(solution, Nr)
    # print(solution_splitted)
    distance_robot = np.zeros(Nr)
    time_robot = np.zeros(Nr)
    completion_time_task = np.zeros(Nt)
    for robot in range(Nr):
        tasks_of_robot = solution_splitted[robot]
        first_task = tasks_of_robot[0]
        t_ell = 0
        if first_task != 0:
            ell, t_ell, p_p = interception(p[robot,:2], moving_q[first_task-1,:2], v[robot], q_p[first_task-1,:2])
            route_to_task = ell - p[robot,:2]
            distance_robot[robot] = LA.norm(route_to_task,2)
        else:
            distance_robot[robot] = 0   
            ell = p[robot]
        time_robot[robot] = t_ell+tau[first_task-1]
        completion_time_task[first_task-1] = time_robot[robot]
        prev_task = first_task
        prev_ell = ell.copy()
        for task in range(Nt-1): 
            next_task = tasks_of_robot[task+1]
            if next_task != 0:
                moving_q[next_task-1,:2] = moving_q[next_task-1,:2] + q_p[next_task-1,:2]*time_robot[robot]
                ell, t_ell, p_p = interception(prev_ell, moving_q[next_task-1,:2], v[robot], q_p[next_task-1,:2])
                route_to_task = ell - prev_ell
                distance_robot[robot] += LA.norm(route_to_task,2) 
                time_robot[robot] += t_ell + tau[next_task-1]
                completion_time_task[next_task-1] = time_robot[robot]
            prev_task = next_task
    weighted_distances = np.multiply(lambda_penalty,distance_robot)
    J_dist = np.sum(weighted_distances)
    weighted_completion_time_task = np.multiply(phi_penalty,completion_time_task)
    J_time = np.sum(weighted_completion_time_task)
    J = J_dist + J_time
    # print(distance_robot)
    # print(completion_time_task)
    # print(J)
    return J

def ga_crossover(p1_input, p2_input, r_cross, Nr, Nt): 
    p1 = p1_input.copy()
    p2 = p2_input.copy()
    # Split parents
    splitted_p1 = np.array_split(p1, Nr) 
    splitted_p2 = np.array_split(p2, Nr) 
    for robot in range(Nr):
        splitted_p1[robot] = splitted_p1[robot][splitted_p1[robot] != 0] 
        splitted_p2[robot] = splitted_p2[robot][splitted_p2[robot] != 0] 
    # Number of changes from r_cross
    Nmant = max([1,math.floor(Nt*r_cross)]) # number of tasks to keep
    Ncamb = Nt-Nmant; # number of changes
    TarCamb = np.zeros(Ncamb);
    # Select the task that will be crossed
    for i in range (Ncamb):
        aux = 0
        while aux == 0:
            A = math.ceil((randint(0, 101)/100)*Nt)
            aux2 = 0
            for j in range (Ncamb):
                if A == TarCamb[j]:
                    aux2 += 1
            if aux2 == 0:
                TarCamb[i] = int(A)
                aux = 1
    for i in range(Ncamb):
        for robot in range(Nr):  
            if TarCamb[i] in splitted_p1[robot]:                          
                # position in which the tasks is allocated in individual 1
                Positions1 = np.where(splitted_p1[robot] == TarCamb[i])[0] 
                robot_doing_task_in_p1 = robot
            if TarCamb[i] in splitted_p2[robot]:  
                # position in which the tasks is allocated in individual 2
                Positions2 = np.where(splitted_p2[robot] == TarCamb[i])[0] 
                robot_doing_task_in_p2 = robot
            # Remove the allocation that is going to be crossed
        splitted_p1[robot_doing_task_in_p1] = np.delete(splitted_p1[robot_doing_task_in_p1], Positions1[0])
        splitted_p2[robot_doing_task_in_p2] = np.delete(splitted_p2[robot_doing_task_in_p2], Positions2[0])
        if splitted_p1[robot_doing_task_in_p2].size >= Positions2[0]:
            splitted_p1[robot_doing_task_in_p2] = np.insert(splitted_p1[robot_doing_task_in_p2], Positions2[0], TarCamb[i] )
        else: 
            splitted_p1[robot_doing_task_in_p2] = np.insert(splitted_p1[robot_doing_task_in_p2], splitted_p1[robot_doing_task_in_p2].size, TarCamb[i] )    
        if splitted_p2[robot_doing_task_in_p1].size >= Positions1[0]:
            splitted_p2[robot_doing_task_in_p1] = np.insert(splitted_p2[robot_doing_task_in_p1], Positions1[0], TarCamb[i] )
        else: 
            splitted_p2[robot_doing_task_in_p1] = np.insert(splitted_p2[robot_doing_task_in_p1], splitted_p2[robot_doing_task_in_p1].size, TarCamb[i] )                  
    s1 = np.concatenate((splitted_p1[0],np.zeros(Nt-splitted_p1[0].size, dtype=int)))
    s2 = np.concatenate((splitted_p2[0],np.zeros(Nt-splitted_p2[0].size, dtype=int)))
    for robot in range(Nr-1):
        aux1 = np.concatenate((splitted_p1[robot+1],np.zeros(Nt-splitted_p1[robot+1].size, dtype=int)))
        aux2 = np.concatenate((splitted_p2[robot+1],np.zeros(Nt-splitted_p2[robot+1].size, dtype=int)))
        s1 = np.concatenate((s1,aux1))  
        s2 = np.concatenate((s2,aux2))
    return s1, s2
    
# mutation operator
def ga_mutation(p1_input, r_mut, Nr, Nt):
    p1 = p1_input.copy()
    # Number of changes from r_mut 
    Nmant = max([1,math.floor(Nt*r_mut)]) # number of tasks to keep
    Ncamb = Nt-Nmant; # number of changes
    # Split parents
    splitted_p1 = np.array_split(p1, Nr) 
    for robot in range(Nr):
        splitted_p1[robot] = splitted_p1[robot][splitted_p1[robot] != 0]         
    random_robot_1 = 0
    random_robot_2 = 0
    pos1 = 0
    pos2 = 0
    flag = 0
    while flag == 0:
        random_robot_1 = randint(0, Nr)
        random_robot_2 = randint(0, Nr)
        if splitted_p1[random_robot_1].size != 0 and splitted_p1[random_robot_2].size != 0:
            pos1 = randint(0,splitted_p1[random_robot_1].size)
            pos2 = randint(0,splitted_p1[random_robot_2].size)
            flag = 1
        else: 
            flag = 0
        if pos1 == pos2 and random_robot_1 == random_robot_2:
            flag = 0
    task1 = splitted_p1[random_robot_1][pos1]
    task2 = splitted_p1[random_robot_2][pos2]
    splitted_p1[random_robot_1][pos1] = task2
    splitted_p1[random_robot_2][pos2] = task1
    s1 = np.concatenate((splitted_p1[0],np.zeros(Nt-splitted_p1[0].size, dtype=int)))
    for robot in range(Nr-1):
        aux1 = np.concatenate((splitted_p1[robot+1],np.zeros(Nt-splitted_p1[robot+1].size, dtype=int)))
        s1 = np.concatenate((s1,aux1)) 
    return s1 

####################### AUXILIARY FUNCTIONS #######################

# increment list
def incr(list, i):
    return [x+i for x in list]

# Interception point
def interception(p, q, v, q_p):
    deltax = q[0] - p[0]
    deltay = q[1] - p[1]
    t_ell = float('inf')
    a = q_p[0]**2 + q_p[1]**2 - v**2
    b = 2*(q_p[0]*deltax + q_p[1]*deltay)
    c = (deltax**2)+(deltay**2)
    t1 = (-b + np.sqrt(b**2 - 4*a*c))/(2*a)
    t2 = (-b - np.sqrt(b**2 - 4*a*c))/(2*a)   
    if b**2 - 4*a*c >= 0:
        if  t1 > 0 and t2 <= 0:        
            t_ell = t1        
        elif t2 >= 0 and t1 <= 0:        
            t_ell = t2            
        elif t1 > 0 and t2 > 0:
            if t1 < t2:
                t_ell = t1
            elif t1 >= t2:                
                t_ell = t2              
        else:
            print('Robot cannot reach task')
    ell = q + q_p*t_ell
    p_p = ((ell-p)/LA.norm(ell-p))*v
    return ell, t_ell, p_p

# Reorder generation
def sort_gen(gen, order):
    sorted_gen = list()
    for index in range(order.size):
        sorted_gen.append(gen[order[index]])
    return sorted_gen
        


####################### CODE START HERE #######################

# Problem parameters (this must be read from a topic)

Nr = 3
Nt = 5

p = np.array([[1, 2], [4, 5], [3, 1]]) 

q =  np.array([[0, 0], [1, 0], [1, 1], [3, 3], [2, 0]]) 

v = np.array([10, 14, 16])  

q_p = np.array([[1, 1], [1, 0.5], [2, 1], [1, 2], [0.5, 1]]) 

tau = np.array([3, 2, 2, 1, 2]) 

lambda_penalty = np.array([5, 4, 2])  

phi_penalty = np.array([5, 1, 3, 2, 2])   

# Hyperparameters

n_gen = 100
r_mut = 0.8 # close to 1 but not 1
r_cross = 0.8 # close to 1 but not 1
pop_size = 100

elite_frac = 0.1
crossover_frac = 0.8

# Call function

best, score = ga(Nr, Nt, p, q, v, q_p, tau, lambda_penalty, phi_penalty, n_gen, pop_size, r_cross, r_mut, elite_frac, crossover_frac)

print('Best solution')
print(best)
print('J')
print(score)

# To send to a topic

ga_allocation = best


        
        
        
        
        
        
        
        







        



