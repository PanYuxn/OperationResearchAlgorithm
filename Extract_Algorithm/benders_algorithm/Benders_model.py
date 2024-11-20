import numpy as np
from gurobipy import *
import random


class Benders_Decomposition(object):
    def __init__(self):
        self.Original_MIP_model = None
        self.MP = None
        self.SP = None
        self.Dual_SP = None
        self.opt_sol = {}
        self.opt_obj_val = None
        self.LB = 0
        self.UB = np.inf
        self.Gap = np.inf
        self.MP_y_sol = 0
        self.benders_iter = 0

    def build_and_solve_original_MIP(self, solve=False, print_sol=False):
        self.Original_MIP_model = Model('Benders decompostion')

        ''' variable '''
        y = self.Original_MIP_model.addVar(lb=0, ub=1000, vtype=GRB.INTEGER, name='y')
        x = {}

        for i in range(10):
            x[i] = self.Original_MIP_model.addVar(lb=0, ub=100, vtype=GRB.INTEGER, name='x_' + str(i))

        ''' objective func'''

        obj = LinExpr()
        obj.addTerms(1.045, y)
        for i in range(10):
            obj.addTerms(1 + 0.01 * (i + 1), x[i])
        self.Original_MIP_model.setObjective(obj, GRB.MAXIMIZE)

        ''' add constrain'''
        lhs = LinExpr()
        lhs.addTerms(1, y)
        for i in range(10):
            lhs.addTerms(1, x[i])
        self.Original_MIP_model.addConstr(lhs <= 1000, name='budget')

        self.Original_MIP_model.update()

        if solve == True:
            self.Original_MIP_model.optimize()
            if solve == True and print_sol == True:
                print('\n\n\n')
                print('Obj:', self.Original_MIP_model.objval)
                print('Saving Account:', y.x)
                for i in range(10):
                    if x[i].x > 0:
                        print('Fund ID {}: amount:{}'.format(i + 1, x[i].x))

    def build_MP(self):
        self.MP = Model('Benders decompostion-MP')

        ''' variable'''
        y = self.MP.addVar(lb=0, ub=1500, vtype=GRB.INTEGER, name='y')
        z = self.MP.addVar(lb=0, ub=1500, vtype=GRB.INTEGER, name='z')

        self.MP.setObjective(z, GRB.MAXIMIZE)

        self.MP.update()

    def solve_MP(self, print_sol=False):
        self.MP.optimze()
        if print_sol == True:
            print('\n\n\n')
            print('Obj:', self.MP.objval)
            print('z=%4.1f' % (self.getVarByName('z').x))
            print('y=%4.1f' % (self.getVarByName('y').x))

    def build_SP(self, y_var=0, solve=False, print_sol=False):
        self.SP = Model('Sp for Benders decomposition')

        '''variable'''
        x = {}
        for i in range(10):
            x[i] = self.Original_MIP_model.addVar(lb=0, ub=100, vtype=GRB.CONTINUOUS, name='x_' + str(i))

        ''' objective func'''
        obj = LinExpr()
        for i in range(10):
            obj.addTerms(1 + 0.01 * (i + 1), x[i])
        self.SP.setObjective(obj, GRB.MAXIMIZE)

        ''' add constrain'''
        lhs = LinExpr()
        for i in range(10):
            lhs.addTerms(1, x[i])
        self.SP.addConstr(lhs <= 1000 - y_var, name='budget')
        self.SP.update()

        if solve == True:
            self.SP.optimize()

    def update_SP(self, y_bar=0, sovel=False, print_sol=False):
        new_budget_constr_rhs = 1000 - y_bar
        budget_con = self.SP.getConstrByName('budget')
        budget_con.RHS = new_budget_constr_rhs
        self.SP.update()

    def build_Dual_SP(self, y_var=0):
        y_bar = 1500

        self.Dual_SP = Model('Dual SP')

        '''variable '''
        alpha_0 = self.Dual_SP.addVar(lb=0, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name='alpha_0')
        alpha = {}
        for i in range(10):
            alpha[i] = self.Dual_SP.addVar(lb=0, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name='alpha_' + str(i))

        '''objective func'''
        obj = LinExpr()
        obj.addTerms(1000 - y_bar, alpha_0)
        for i in range(10):
            obj.addTerms(100, alpha[i])

        self.Dual_SP.setObjective(obj, GRB.MINIMIZE)

        for i in range(10):
            self.Dual_SP.addConstr(alpha_0 + alpha[i] >= 1 + 0.01 * (i + 1))
        # self.Dual_SP.setParam('InfUbdInfo', 1)
        self.Dual_SP.update()

    def update_Dual_SP(self, y_bar=0, solve=True):
        new_alpha_0_coef = 1000 - y_bar
        self.Dual_SP.getVarByName('alpha_0').Obj = new_alpha_0_coef
        self.Dual_SP.update()

    def solve_Dual_SP(self, print_sol=True):
        self.Dual_SP.optimize()
        if print_sol == True:
            print('\n\n\n')
            print('Model Statues:', self.Dual_SP.status)
            if self.Dual_SP.status == 2:
                print('Obj:', self.Dual_SP.objval)
                for var in self.Dual_SP.getVars():
                    if var.x > 0:
                        print('{}={}'.format(var.varName, var.x))
            else:
                print('========Unbound=========')
                var_alpha_0 = self.Dual_SP.getVarByName('alpha_0')
                print('extreme ray:{}={}'.format(var_alpha_0.varName, var_alpha_0.UnbdRay))
                for i in range(10):
                    var_alpha = self.Dual_SP.getVarByName('alpha_' + str(i))
                    print('extreme ray:{}={}'.format(var_alpha.varName, var_alpha.UnbdRay))

    def print_optimal_sol(self):
        print('Gap:%8.4f' % (self.Gap), end='%\n')
        print('Obj:{}'.format(self.MP.objval))
        print('=====Solutio======')
        print('{}={}'.format('y', self.MP.getVarByName('y').x))
        cnt = 1
        for con in self.Dual_SP.getConstrs():
            if con.Pi > 0:
                print('x{}={}'.format(cnt, con.Pi))
                cnt += 1

    def add_benders_cuts(self, use_Dual_SP=True, y_var=0, benders_iter=0):
        if use_Dual_SP == True:
            Cut_lhs = LinExpr()

            if self.Dual_SP.status == 2:
                var_y = self.MP.getVarByName('y')
                var_z = self.MP.getVarByName('z')
                var_y_coef = 1.045
                Cut_lhs.addTerms(1.045, var_y)
                alpha_0_value = self.Dual_SP.getVarByName('alpha_0').x
                Cut_lhs.addTerms(-alpha_0_value, var_y)
                var_y_coef = var_y_coef = alpha_0_value

                constant = 0
                for var in self.Dual_SP.getVars():
                    if var.varName == 'alpha_0':
                        constant += 1000 * var.x
                    else:
                        constant += 100 * var.x
                con_name = 'Benders optimality cut iter ' + str(benders_iter)
                Cut = self.MP.addConstr(Cut_lhs + constant >= var_z, name=con_name)
                self.MP.update()
                print('\n Iter:{},Add optimal cut : {}+{}>=z\n'.format(benders_iter, Cut_lhs, constant))

            if self.Dual_SP.status != 2:
                var_y = self.MP.getVarByName('y')
                alpha_0_ray = self.Dual_SP.getVarByName('alpha_0').UnbdRay
                Cut_lhs.addTerms(-alpha_0_ray, var_y)

                var_y_coef = -alpha_0_ray

                constant = 0
                for var in self.Dual_SP.getVars():
                    if var.varName == 'alpha_0':
                        constant += 1000 * var.UnbdRay
                    else:
                        constant += 100 * var.UnbdRay
                con_name = 'Benders feasibility cut iter ' + str(benders_iter)
                Cut = self.MP.addConstr(Cut_lhs + constant >= 0, name=con_name)
                self.MP.update()
                print('\n Iter:{},Add feasibility cut : {}+{}>=z\n'.format(benders_iter, Cut_lhs, constant))

    def Benders_Decompostion(self, eps=0):
        self.UB = 1500
        self.LB = 1000
        self.Gap = np.inf
        self.eps = 0.001
        self.benders_iter = 0
        max_no_change = 2
        no_change_cnt = 0
        y_bar_change = []

        self.build_MP()
        self.MP.setParam('OutputFlag', 0)
        self.MP.optimize()

        y_var = 1000
        y_bar_change.append(y_var)

        self.build_Dual_SP(y_var=y_var)
        self.Dual_SP.setParam('OutputFlag', 0)

        print('\n\n ========================')
        print('       Benders Decompostion Strat ')
        print('=====================')
        while self.UB - self.LB > self.eps:
            self.benders_iter += 1

            # update SP by y_bar
            self.update_Dual_SP(y_bar=y_var)

            # solve dual SP
            self.Dual_SP.optimize()

            # update global LB
            if self.Dual_SP.status == 2:
                self.LB = max(self.LB, self.MP_y_sol * 1.045 + self.Dual_SP.objval)

            # generate cut
            self.add_benders_cuts(use_Dual_SP=True, y_var=y_var, benders_iter=self.benders_iter)

            # solve updated MP
            self.MP.optimize()
            self.MP_y_sol = self.MP.getVarByName('y').x

            # update global UB
            self.UB = min(self.UB, self.MP.objval)

            self.Gap = round(100 * (self.UB - self.LB) / self.LB, 4)

            # update y_bra
            y_var = self.MP.getVarByName('y').x
            y_bar_change.append(y_var)

            if y_bar_change[-1] == y_bar_change[-2]:
                no_change_cnt += 1
            else:
                no_change_cnt = 0
            if no_change_cnt > max_no_change:
                y_var = random.randint(0, 1000)
                no_change_cnt = 0

            print('    %7.2f    ' % self.UB, end='')
            print('    %7.2f    ' % self.LB, end='')
            print('  %8.4f  ' % self.Gap, end='%')

        print('\n\n ===========Optimal Solution Found ! ======')
        self.print_optimal_sol()


if __name__ == '__main__':
    benders_solver = Benders_Decomposition()
    benders_solver.Benders_Decompostion(eps=0)
