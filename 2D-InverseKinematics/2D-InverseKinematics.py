import numpy as np
import sympy as sym
import sys
import matplotlib.pyplot as plt

sys.path.insert(0, 'c:\\Development\\TheoryApplied\\KinematicsHelpers')
import Frames as frame

def main():
    global q0
    global q1
    global q2
    global q3
    global l0
    global l1
    global l2
    global l3

    q0 = sym.Symbol('q0')
    q1 = sym.Symbol('q1')
    q2 = sym.Symbol('q2')
    q3 = sym.Symbol('q3')
    l0 = sym.Symbol('l0')
    l1 = sym.Symbol('l1')
    l2 = sym.Symbol('l2')
    l3 = sym.Symbol('l3')

    b_E_s0 = frame.create2DFrame(q0, np.array([0, 0]))
    s0_E_s1 = frame.create2DFrame(q1, np.array([l0, 0]))
    s1_E_s2 = frame.create2DFrame(q2, np.array([l1, 0]))
    s2_E_s3 = frame.create2DFrame(q3, np.array([l2, 0]))
    s3_E_t = frame.create2DFrame(0, np.array([l3, 0]))

    b_E_s1 = sym.simplify(np.matmul(b_E_s0, s0_E_s1))
    b_E_s2 = sym.simplify(np.matmul(b_E_s1, s1_E_s2))
    b_E_s3 = sym.simplify(np.matmul(b_E_s2, s2_E_s3))
    b_E_t = sym.simplify(np.matmul(b_E_s3, s3_E_t))

    lengths = [1, 1, 1, 1]
    qs_symbolic = [q0, q1, q2, q3]
    lengths_symbolic = [l0, l1, l2, l3]

#     plotRobot(b_E_s1, b_E_s2, b_E_s3, b_E_t, qs, lengths)

    Ja = frame.compute2DAnalyticJacobian(b_E_t, qs_symbolic)

    qInit = [0.1, 0.1, 0.1, 0.1]
    xInit = frame.get2DFKPosition(b_E_t, qs_symbolic, qInit, lengths_symbolic, lengths).astype(float)

    xDes = np.array([2, 1])

    qCur = qInit
    xCur = xInit

    alpha = 0.01

    i = 1

    while np.linalg.norm(xDes - xCur) > 0.01:
       Ja_num = np.array(Ja.subs({q0:qCur[0], q1:qCur[1], q2:qCur[2],\
                                  q3:qCur[3], l0:lengths[0], l1:lengths[1], \
                                   l2:lengths[2], l3:lengths[3]})).astype(float)
       
       qCur = qCur + alpha * np.matmul(frame.computeRightPseudoInverse(Ja_num), (xInit - xDes))
       xCur = frame.get2DFKPosition(b_E_t, qs_symbolic, qCur, lengths_symbolic, lengths).astype(float)

       print(i)
       print(np.linalg.norm(xDes - xCur))


    plotRobot(b_E_s1, b_E_s2, b_E_s3, b_E_t, qCur, lengths)
    


def plotRobot(b_E_s1, b_E_s2, b_E_s3, b_E_t, qs, lengths):
    s1 = sym.Array([row[2] for row in b_E_s1]).subs({q0:qs[0], q1:qs[1], q2:qs[2],\
         q3:qs[3], l0:lengths[0], l1:lengths[1] ,l2:lengths[2], l3:lengths[3]})
    s2 = sym.Array([row[2] for row in b_E_s2]).subs({q0:qs[0], q1:qs[1], q2:qs[2],\
         q3:qs[3], l0:lengths[0], l1:lengths[1] ,l2:lengths[2], l3:lengths[3]})
    s3 = sym.Array([row[2] for row in b_E_s3]).subs({q0:qs[0], q1:qs[1], q2:qs[2],\
         q3:qs[3], l0:lengths[0], l1:lengths[1] ,l2:lengths[2], l3:lengths[3]})
    t = sym.Array([row[2] for row in b_E_t]).subs({q0:qs[0], q1:qs[1], q2:qs[2],\
         q3:qs[3], l0:lengths[0], l1:lengths[1] ,l2:lengths[2], l3:lengths[3]})
    
    xVals = [0, s1[0], s2[0], s3[0], t[0]]
    yVals = [0, s1[1], s2[1], s3[1], t[1]]
    
    plt.figure(figsize=(6,6))
    plt.plot(xVals, yVals, linestyle="-")
    plt.plot(xVals[0], yVals[0], marker="o", markersize=4, markeredgecolor="black", markerfacecolor="black")
    plt.plot(xVals[1], yVals[1], marker="o", markersize=4, markeredgecolor="red", markerfacecolor="red")
    plt.plot(xVals[2], yVals[2], marker="o", markersize=4, markeredgecolor="red", markerfacecolor="red")
    plt.plot(xVals[3], yVals[3], marker="o", markersize=4, markeredgecolor="red", markerfacecolor="red")
    plt.plot(xVals[4], yVals[4], marker="o", markersize=4, markeredgecolor="green", markerfacecolor="green")
    plt.xlim((-4, 4))
    plt.ylim((-4, 4))
    plt.show()
    
if __name__ == "__main__":
    main()