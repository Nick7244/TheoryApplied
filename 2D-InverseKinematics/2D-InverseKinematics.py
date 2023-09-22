import numpy as np
import sympy as sym
import sys
import matplotlib.pyplot as plt
import matplotlib.animation as ani
import os
import dill

sys.path.insert(0, 'c:\\Development\\TheoryApplied\\KinematicsHelpers')
import FrameHelpers as frame

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
    global b_E_leftFingerBase
    global b_E_leftFingerTip
    global b_E_rightFingerBase
    global b_E_rightFingerTip
    global Ja

    b_E_s0, b_E_s1, b_E_s2, b_E_s3, b_E_t, b_E_leftFingerBase, b_E_leftFingerTip,\
     b_E_rightFingerBase, b_E_rightFingerTip = computeKinematicChain()
    Ja = computeAnalyticJacobian(b_E_t, qs_symbolic)

    global lengths
    lengths = [1, 1, 1, 1]
    qInit = [sym.pi/2, -sym.pi/4, -sym.pi/4, -sym.pi/4]

    global xDes
    xInit = frame.get2DFKPosition(b_E_t, qs_symbolic, qInit, lengths_symbolic, lengths).astype(float)
    xDes = np.array([3, -0.5])

    plt.figure(figsize=(6,6))
    animation = ani.FuncAnimation(plt.gcf(), runIK, fargs=(qInit, xInit), interval = 100, cache_frame_data=False)
    plt.show()


def runIK(i, qInit, xInit):
    global qCur
    global xCur

    if i == 0:
        qCur = qInit
        xCur = xInit
    
    alpha = 0.1

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
     if not os.path.isfile("2D-InverseKinematics/Frames/b_E_t.pkl"):
          b_E_s0 = frame.create2DFrame(q0, np.array([0, 0]))
          s0_E_s1 = frame.create2DFrame(q1, np.array([l0, 0]))
          s1_E_s2 = frame.create2DFrame(q2, np.array([l1, 0]))
          s2_E_s3 = frame.create2DFrame(q3, np.array([l2, 0]))
          s3_E_t = frame.create2DFrame(0, np.array([l3, 0]))

          b_E_s1 = sym.simplify(np.matmul(b_E_s0, s0_E_s1))
          b_E_s2 = sym.simplify(np.matmul(b_E_s1, s1_E_s2))
          b_E_s3 = sym.simplify(np.matmul(b_E_s2, s2_E_s3))
          b_E_t = sym.simplify(np.matmul(b_E_s3, s3_E_t))

          t_E_leftFingerBase = frame.create2DFrame(0, np.array([0, 0.25]))
          t_E_leftFingerTip = frame.create2DFrame(0, np.array([0.25, 0.25]))
          t_E_rightFingerBase = frame.create2DFrame(0, np.array([0, -0.25]))
          t_E_rightFingerTip = frame.create2DFrame(0, np.array([0.25, -0.25]))

          b_E_leftFingerBase = sym.simplify(np.matmul(b_E_t, t_E_leftFingerBase))
          b_E_leftFingerTip = sym.simplify(np.matmul(b_E_t, t_E_leftFingerTip))
          b_E_rightFingerBase = sym.simplify(np.matmul(b_E_t, t_E_rightFingerBase))
          b_E_rightFingerTip = sym.simplify(np.matmul(b_E_t, t_E_rightFingerTip))
          
          with open('2D-InverseKinematics/Frames/b_E_s0.pkl', 'wb') as file:
               dill.dump(b_E_s0, file)
          with open('2D-InverseKinematics/Frames/b_E_s1.pkl', 'wb') as file:
               dill.dump(b_E_s1, file)
          with open('2D-InverseKinematics/Frames/b_E_s2.pkl', 'wb') as file:
               dill.dump(b_E_s2, file)
          with open('2D-InverseKinematics/Frames/b_E_s3.pkl', 'wb') as file:
               dill.dump(b_E_s3, file)
          with open('2D-InverseKinematics/Frames/b_E_t.pkl', 'wb') as file:
               dill.dump(b_E_t, file)
          with open('2D-InverseKinematics/Frames/b_E_leftFingerBase.pkl', 'wb') as file:
               dill.dump(b_E_leftFingerBase, file)
          with open('2D-InverseKinematics/Frames/b_E_leftFingerTip.pkl', 'wb') as file:
               dill.dump(b_E_leftFingerTip, file)
          with open('2D-InverseKinematics/Frames/b_E_rightFingerBase.pkl', 'wb') as file:
               dill.dump(b_E_rightFingerBase, file)
          with open('2D-InverseKinematics/Frames/b_E_rightFingerTip.pkl', 'wb') as file:
               dill.dump(b_E_rightFingerTip, file)
     else:
          with open('2D-InverseKinematics/Frames/b_E_s0.pkl', 'rb') as file:
               b_E_s0 = dill.load(file)
          with open('2D-InverseKinematics/Frames/b_E_s1.pkl', 'rb') as file:
               b_E_s1 = dill.load(file)
          with open('2D-InverseKinematics/Frames/b_E_s2.pkl', 'rb') as file:
               b_E_s2 = dill.load(file)
          with open('2D-InverseKinematics/Frames/b_E_s3.pkl', 'rb') as file:
               b_E_s3 = dill.load(file)
          with open('2D-InverseKinematics/Frames/b_E_t.pkl', 'rb') as file:
               b_E_t = dill.load(file)
          with open('2D-InverseKinematics/Frames/b_E_leftFingerBase.pkl', 'rb') as file:
               b_E_leftFingerBase = dill.load(file)
          with open('2D-InverseKinematics/Frames/b_E_leftFingerTip.pkl', 'rb') as file:
               b_E_leftFingerTip = dill.load(file)
          with open('2D-InverseKinematics/Frames/b_E_rightFingerBase.pkl', 'rb') as file:
               b_E_rightFingerBase = dill.load(file)
          with open('2D-InverseKinematics/Frames/b_E_rightFingerTip.pkl', 'rb') as file:
               b_E_rightFingerTip = dill.load(file)
               
     return [b_E_s0, b_E_s1, b_E_s2, b_E_s3, b_E_t, b_E_leftFingerBase, b_E_leftFingerTip,\
             b_E_rightFingerBase, b_E_rightFingerTip]


def computeAnalyticJacobian(b_E_t, qs_symbolic):
     if not os.path.isfile("2D-InverseKinematics/Frames/Ja.pkl"):
        Ja = frame.compute2DAnalyticJacobian(b_E_t, qs_symbolic)
        with open('2D-InverseKinematics/Frames/Ja.pkl', 'wb') as file:
           dill.dump(Ja, file)
     else:
        with open('2D-InverseKinematics/Frames/Ja.pkl', 'rb') as file:
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
    
    jointXVals = [0, s1[0], s2[0], s3[0], t[0]]
    jointYVals = [0, s1[1], s2[1], s3[1], t[1]]

    leftFingerBase = sym.Array([row[2] for row in b_E_leftFingerBase]).subs({q0:qs[0], q1:qs[1], q2:qs[2],\
         q3:qs[3], l0:lengths[0], l1:lengths[1] ,l2:lengths[2], l3:lengths[3]})
    leftFingerTip = sym.Array([row[2] for row in b_E_leftFingerTip]).subs({q0:qs[0], q1:qs[1], q2:qs[2],\
         q3:qs[3], l0:lengths[0], l1:lengths[1] ,l2:lengths[2], l3:lengths[3]})
    rightFingerBase = sym.Array([row[2] for row in b_E_rightFingerBase]).subs({q0:qs[0], q1:qs[1], q2:qs[2],\
         q3:qs[3], l0:lengths[0], l1:lengths[1] ,l2:lengths[2], l3:lengths[3]})
    rightFingerTip = sym.Array([row[2] for row in b_E_rightFingerTip]).subs({q0:qs[0], q1:qs[1], q2:qs[2],\
         q3:qs[3], l0:lengths[0], l1:lengths[1] ,l2:lengths[2], l3:lengths[3]})
    
    plt.clf()

    plt.plot(jointXVals, jointYVals, linestyle="-", color="#136891")
    plt.plot([leftFingerBase[0], rightFingerBase[0]], [leftFingerBase[1], rightFingerBase[1]], linestyle="-", color="#136891")
    plt.plot([leftFingerBase[0], leftFingerTip[0]], [leftFingerBase[1], leftFingerTip[1]], linestyle="-", color="#136891")
    plt.plot([rightFingerBase[0], rightFingerTip[0]], [rightFingerBase[1], rightFingerTip[1]], linestyle="-", color="#136891")
    
    plt.plot(xDes[0], xDes[1], marker="o", markersize=4, markeredgecolor="black", markerfacecolor="black")

    plt.plot(jointXVals[0], jointYVals[0], marker="o", markersize=4, markeredgecolor="black", markerfacecolor="black")
    plt.plot(jointXVals[1], jointYVals[1], marker="o", markersize=4, markeredgecolor="red", markerfacecolor="red")
    plt.plot(jointXVals[2], jointYVals[2], marker="o", markersize=4, markeredgecolor="red", markerfacecolor="red")
    plt.plot(jointXVals[3], jointYVals[3], marker="o", markersize=4, markeredgecolor="red", markerfacecolor="red")
    plt.plot(jointXVals[4], jointYVals[4], marker="o", markersize=4, markeredgecolor="green", markerfacecolor="green")

    plt.xlim((-4, 4))
    plt.ylim((-4, 4))


if __name__ == "__main__":
    main()