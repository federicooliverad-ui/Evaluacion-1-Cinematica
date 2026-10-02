import numpy as np
import math
from sympy import Symbol, Matrix, cos, sin, diff

MAX_STEP = 10

class JacobianController:
    def __init__(self, arm):
        # the controller should save a reference to the arm it's controlling
        self.arm = arm

        theta1 = Symbol("theta1")
        theta2 = Symbol("theta2")
        theta3 = Symbol("theta3")
        theta4 = Symbol("theta4")
        theta5 = Symbol("theta5")
        theta6 = Symbol("theta6")

        l = 60

        T1 = Matrix([[cos(theta1), -sin(theta1), 0, l*cos(theta1)],
                    [sin(theta1), cos(theta1), 0, l*sin(theta1)],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]
        ])

        T2 = Matrix([[cos(theta2), -sin(theta2), 0, l*cos(theta2)],
                    [sin(theta2), cos(theta2), 0, l*sin(theta2)],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]
        ])

        T3 = Matrix([[cos(theta3), -sin(theta3), 0, l*cos(theta3)],
                    [sin(theta3), cos(theta3), 0, l*sin(theta3)],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]
        ])

        T4 = Matrix([[cos(theta4), -sin(theta4), 0, l*cos(theta4)],
                    [sin(theta4), cos(theta4), 0, l*sin(theta4)],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]
        ])

        T5 = Matrix([[cos(theta5), -sin(theta5), 0, l*cos(theta5)],
                    [sin(theta5), cos(theta5), 0, l*sin(theta5)],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]
        ])

        T6 = Matrix([[cos(theta6), -sin(theta6), 0, l*cos(theta6)],
                    [sin(theta6), cos(theta6), 0, l*sin(theta6)],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]
        ])

        T = T1*T2*T3*T4*T5*T6

        x = T[3]
        y = T[7]

        a00 = diff(x, theta1)
        a01 = diff(x, theta2)
        a02 = diff(x, theta3)
        a03 = diff(x, theta4)
        a04 = diff(x, theta5)
        a05 = diff(x, theta6)
        a10 = diff(y, theta1)
        a11 = diff(y, theta2)
        a12 = diff(y, theta3)
        a13 = diff(y, theta4)
        a14 = diff(y, theta5)
        a15 = diff(y, theta6)

        J = [[a00, a01, a02, a03, a04, a05],
             [a10, a11, a12, a13, a14, a15]]

        #Conservo las expresiones para evaluarlas con cada postura
        self.J_simbolico = Matrix(J)
        self.simbolos = [theta1, theta2, theta3,
                         theta4, theta5, theta6]

    def control(self, target):
        if self.arm.get_num_joints() == 2:
            # if arm has 2 joints
            self.control2J2D(target)
        elif self.arm.get_num_joints() == 3:
            # if arm has 3 joints
            self.control3J2D(target)
        elif self.arm.get_num_joints() == 6:
            self.control6J2D(target)
        else:
            raise Exception("JacobianController.control(target): Can't control an arm with this amount joints.")

    def control2J2D(self, target):
        # the control method receives a target
        pass


    def control3J2D(self, target):
        # the control method receives a target
        pass
    
    def control6J2D(self, target):
        #Sustituyo los símbolos del Jacobiano por los ángulos actuales del brazo
        sustituciones = list(zip(
            self.simbolos, self.arm.thetas
        ))

        J = np.array(
            self.J_simbolico.subs(sustituciones),
            dtype=float
        )

        #El desplazamiento se obtiene de la posicion del mouse
        error = target - self.arm.endeffector()

        #Se limita el paso porque el Jacobiano aproxima movimientos pequeños
        paso = min(error.r, MAX_STEP)

        delta_c = np.array([
            paso * math.cos(error.a),
            paso * math.sin(error.a)
        ])

        J_inv = np.linalg.pinv(J)
        delta_ang = J_inv.dot(delta_c)

        #Se aplica los incrementos calculados a las seis articulaciones
        self.arm.move(delta_ang)
