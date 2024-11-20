from gurobipy import *

model = Model()

# 决策变量
x = model.addVars(10, obj=[1 + i * 0.01 for i in range(1, 11)], vtype=GRB.CONTINUOUS, name='x')
y = model.addVar(lb=0, ub=GRB.INFINITY, obj=1.045, vtype=GRB.INTEGER, name='y')
# 目标函数
model.setAttr(GRB.Attr.ModelSense, -1)  # -1是最大化, 1是最小化.

# 约束条件
model.addConstrs((x[i] <= 100 for i in range(10)), name='c')
model.addConstr(y + quicksum(x) <= 1000)
# 优化输出
model.write('MIPmodel.lp')
model.optimize()

print(model.objval)
print(model.x)
# 最优解为x5,x6,x7,x8,x9=100,y=400. 最优值为1063.