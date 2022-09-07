# %%
from scipy.optimize import linprog

# %%
import numpy as np
import math

# %% [markdown]
# # Environment Setting

# %%

# Horizon of Prediction
max_horizon = 8
#print('Maximum Hoirzon ID is :', max_horizon)

# Number of Robot
nr = 4

# %%
# Number of Target
nt = 8

# %%
# Matrix of Robot Velocity
v = np.array([0.2,0.4,0.4])
#print(v)

# %%
# Matrix of Robot Battery Level
b = np.array([3,1,1])
#print(b)

# %%
# Matrix of Robot Battery Consumption
w = np.array([1,2,2])
#print(w)

# %%
# Robot X - Y Axis Location

# px = np.zeros(nr,dtype=int)
px= np.array([0,-2,-2])
#print(px)


# py = np.zeros(nr,dtype=int)
py= np.array([0,0,2])
#print(py)

# %%
# Target X - Y Axis Location

qx = np.array([-5,-9,-5,-9,5,9,5,9]) 

qy = np.array([-5,-6,5,6,-5,-6,5,6])

#print(qx)
#print(qy)

# %%
# Matrix d represents distance from robot-i to target-j
d =  np.zeros([nr,nt],dtype=float)
#print(d)

# %%
# Manhattan Distance
""" 
for i in range (1,nr+1):
    for j in range (1,nt+1):
        d[i-1,j-1] = abs(qy[j-1]-py[i-1]) + abs(qx[j-1]-px[i-1])
"""

# Euclidian Distance

for i in range (1,nr+1):
    for j in range (1,nt+1):
        d[i-1,j-1] = math.sqrt(pow(qy[j-1]-py[i-1],2)+pow(qx[j-1]-px[i-1],2))


#print(d)



# %%
# Matrix t which represents time required for robot i to travel to target j

t = d / v.transpose()[:,None]
#print(t)

# %%
# Matrix E (Represents energy used related to distance)

e  = d * w.transpose()[:,None]
#print(e)

# %%
# Matrix Importance Phi

phi_j = np.array([1,1,1,1,1,1,1,1])
#print(phi_j)

# %% [markdown]
# # Objective Function

# %%
# Define Matrix g as Matrix for Cost Function
g = np.zeros([nr,nt,nt],dtype=float)
#print(g)

# %%
for i in range (1,nr+1):
    for j in range (1,nt+1):
        for k in range (1,nt+1):
            #[i-1k] = 50*(phi_j[j]*t[i,j]*(k+1)) #- (phi_j[j]*(k+1)*(b[i]-e[i,j]))
            #[i-1,j-1,k-1] = 100*i + 10*j + k
            g[i-1,j-1,k-1] = 50*(phi_j[j-1]*t[i-1,j-1]*k)
#print(g)

f = g.flatten()

#print(f)
      

# %% [markdown]
# # Equality Constraint 

# %%
# Equality Constraint

a1 = np.hstack((np.ones(nt,dtype=int),np.zeros(nt*(nt-1),dtype=int)))

a2 = np.tile(a1,nr)

a3 = np.roll(a2,nt)



temp1 = np.roll(a2,-nt)
Aeq = np.array([]).reshape(0,(nr*nt*nt))

for j in range (1,nt+1):
    temp1 = np.roll(temp1,nt)
    Aeq = np.vstack([Aeq,temp1])
    
#print(Aeq)

beq = np.ones(nt)
#print(beq)

# %% [markdown]
# # Inequality Constraint

# %%
# Inequality Constraint

b1 = np.identity(nt)

b2 = np.tile(b1,nt)

b3 = np.hstack((b2,np.zeros((nt,(nt*nt*(nr-1))))))

b4 = np.roll(b3,nt*nt)



temp2 = np.roll(b3,-nt*nt)
A = np.array([]).reshape(0,(nr*nt*nt))

for i in range (1,nr+1):
    temp2 = np.roll(temp2,nt*nt)
    A = np.vstack([A,temp2])
    
#print(A)

b = np.ones(nr*nt,dtype=int)
#print(b)

# %% [markdown]
# # Lower & Upper Bound

# %%
lb = np.zeros(nr*nt*nt,dtype=int)
ub = np.array([]).reshape(0,(nr*nt*nt))

# %%
res =  linprog(f,A_ub = A, b_ub = b, A_eq = Aeq, b_eq = beq)
#print(res)
#print(res.x)

# %%
results = np.reshape(res.x,(nr,nt,nt))
results = np.around(results, decimals = 0)
#results = results.astype(int)
#print(results)


# %%
rfl = np.zeros((nr,nt))

for i in range (1,nr+1):
    for j in range (1,nt+1):
        for k in range (1,nt+1):
            if results[i-1,j-1,k-1] == 1:
                rfl[i-1,k-1] = j

#print(rfl)


