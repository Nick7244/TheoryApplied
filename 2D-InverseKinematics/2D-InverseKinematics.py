import numpy as np
import sympy as sym
import sys
import matplotlib.pyplot as plt
import matplotlib.animation as ani
import threading

sys.path.insert(0, 'c:\\Development\\TheoryApplied\\KinematicsHelpers')
import Frames as frame
import os
import dill

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
    
    global qs_symbolic
    global lengths_symbolic

    qs_symbolic = [q0, q1, q2, q3]
    lengths_symbolic = [l0, l1, l2, l3]

    global b_E_s0
    global b_E_s1
    global b_E_s2
    global b_E_s3
    global b_E_t
    global Ja

    b_E_s0, b_E_s1, b_E_s2, b_E_s3, b_E_t = computeKinematicChain()
    Ja = computeAnalyticJacobian(b_E_t, qs_symbolic)

    global lengths
    lengths = [1, 1, 1, 1]
    qInit = [sym.pi/2, -sym.pi/4, -sym.pi/4, -sym.pi/4]
#     plotRobot(b_E_s1, b_E_s2, b_E_s3, b_E_t, qInit, lengths)
    
#     qDes = [0.2, 0.1, 0.1, 0.1]

#     x_init = frame.get2DFKPosition(b_E_s1, qs_symbolic, qInit, lengths_symbolic, lengths).astype(float)
#     x_des = frame.get2DFKPosition(b_E_s1, qs_symbolic, qDes, lengths_symbolic, lengths).astype(float)
    
#     xCur = x_init
#     qCur = qInit
#     while np.linalg.norm(x_des - xCur) > 0.001:
#         Ja_num = np.array(Ja.subs({q0:qCur[0], q1:qCur[1], q2:qCur[2],\
#                                   q3:qCur[3], l0:lengths[0], l1:lengths[1], \
#                                    l2:lengths[2], l3:lengths[3]})).astype(float)
#         qDiff = 0.1*np.matmul(frame.computeRightPseudoInverse(Ja_num), (x_des - x_init))
#         qCur = qCur + qDiff
#         xCur = frame.get2DFKPosition(b_E_s1, qs_symbolic, qCur, lengths_symbolic, lengths).astype(float)

    global xDes
    xInit = frame.get2DFKPosition(b_E_t, qs_symbolic, qInit, lengths_symbolic, lengths).astype(float)
    xDes = np.array([2, 1])

#     data_thread = threading.Thread(target=runIK, args=(qInit, xInit))
#     data_thread.daemon = True  # Allow the thread to be terminated when the main program exits
#     data_thread.start()
    plt.figure(figsize=(6,6))
    animation = ani.FuncAnimation(plt.gcf(), runIK, fargs=(qInit, xInit), interval = 100)
    plt.show()

def runIK(i, qInit, xInit):
    global qCur
    global xCur

    if i == 0:
        qCur = qInit
        xCur = xInit
    
    alpha = 0.01

    if np.linalg.norm(xDes - xCur) > 0.01:
       Ja_num = np.array(Ja.subs({q0:qCur[0], q1:qCur[1], q2:qCur[2],\
                                  q3:qCur[3], l0:lengths[0], l1:lengths[1], \
                                   l2:lengths[2], l3:lengths[3]})).astype(float)
       
       qCur = qCur + alpha * np.matmul(frame.computeRightPseudoInverse(Ja_num), (xDes - xCur))

       xCur = frame.get2DFKPosition(b_E_t, qs_symbolic, qCur, lengths_symbolic, lengths).astype(float)
       print(i)
       print(np.linalg.norm(xDes - xCur))
       plotRobot(qCur)

def computeKinematicChain():
     if not os.path.isfile("b_E_t.pkl"):
          b_E_s0 = frame.create2DFrame(q0, np.array([0, 0]))
          s0_E_s1 = frame.create2DFrame(q1, np.array([l0, 0]))
          s1_E_s2 = frame.create2DFrame(q2, np.array([l1, 0]))
          s2_E_s3 = frame.create2DFrame(q3, np.array([l2, 0]))
          s3_E_t = frame.create2DFrame(0, np.array([l3, 0]))

          b_E_s1 = sym.simplify(np.matmul(b_E_s0, s0_E_s1))
          b_E_s2 = sym.simplify(np.matmul(b_E_s1, s1_E_s2))
          b_E_s3 = sym.simplify(np.matmul(b_E_s2, s2_E_s3))
          b_E_t = sym.simplify(np.matmul(b_E_s3, s3_E_t))
          
          with open('b_E_s0.pkl', 'wb') as file:
               dill.dump(b_E_s0, file)
          with open('b_E_s1.pkl', 'wb') as file:
               dill.dump(b_E_s1, file)
          with open('b_E_s2.pkl', 'wb') as file:
               dill.dump(b_E_s2, file)
          with open('b_E_s3.pkl', 'wb') as file:
               dill.dump(b_E_s3, file)
          with open('b_E_t.pkl', 'wb') as file:
               dill.dump(b_E_t, file)
     else:
          with open('b_E_s0.pkl', 'rb') as file:
               b_E_s0 = dill.load(file)
          with open('b_E_s1.pkl', 'rb') as file:
               b_E_s1 = dill.load(file)
          with open('b_E_s2.pkl', 'rb') as file:
               b_E_s2 = dill.load(file)
          with open('b_E_s3.pkl', 'rb') as file:
               b_E_s3 = dill.load(file)
          with open('b_E_t.pkl', 'rb') as file:
               b_E_t = dill.load(file)
               
     return [b_E_s0, b_E_s1, b_E_s2, b_E_s3, b_E_t]

def computeAnalyticJacobian(b_E_t, qs_symbolic):
     if not os.path.isfile("Ja.pkl"):
        Ja = frame.compute2DAnalyticJacobian(b_E_t, qs_symbolic)
        with open('Ja.pkl', 'wb') as file:
           dill.dump(Ja, file)
     else:
        with open('Ja.pkl', 'rb') as file:
           Ja = dill.load(file)
     
     return Ja

def plotRobot(qs):
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
    
    plt.clf()
    plt.plot(xVals, yVals, linestyle="-")
    plt.plot(xVals[0], yVals[0], marker="o", markersize=4, markeredgecolor="black", markerfacecolor="black")
    plt.plot(xVals[1], yVals[1], marker="o", markersize=4, markeredgecolor="red", markerfacecolor="red")
    plt.plot(xVals[2], yVals[2], marker="o", markersize=4, markeredgecolor="red", markerfacecolor="red")
    plt.plot(xVals[3], yVals[3], marker="o", markersize=4, markeredgecolor="red", markerfacecolor="red")
    plt.plot(xVals[4], yVals[4], marker="o", markersize=4, markeredgecolor="green", markerfacecolor="green")

    plt.plot(xDes[0], xDes[1], marker="o", markersize=4, markeredgecolor="black", markerfacecolor="black")

    plt.xlim((-4, 4))
    plt.ylim((-4, 4))
    
if __name__ == "__main__":
    main()