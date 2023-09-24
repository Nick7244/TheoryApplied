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
    global qWrist
    global l0
    global l1
    global l2
    global l3

    q0 = sym.Symbol('q0')
    q1 = sym.Symbol('q1')
    q2 = sym.Symbol('q2')
    q3 = sym.Symbol('q3')
    qWrist = sym.Symbol('qWrist')
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

    b_E_s0, b_E_s1, b_E_s2, b_E_s3, b_E_t, b_E_leftFingerBase, b_E_leftFingerTip,\
     b_E_rightFingerBase, b_E_rightFingerTip = computeKinematicChain()
    
    global Ja
    Ja = computeAnalyticJacobian(b_E_t, qs_symbolic)

    global Jb
    Jb = computeManipulatorJacobian(b_E_t, qs_symbolic)

    global lengths
    lengths = [1, 1, 1, 1]
    qInit = [np.pi/2, -np.pi/4, np.pi/4, np.pi/2]

    global xDes
    xInit = frame.get2DFKPosition(b_E_t, qs_symbolic, qInit, lengths_symbolic, lengths).astype(float)
#     xDes = np.array([-2, 2])
#     xDes = np.array([2, -0.5])
#     xDes = np.array([2, -0.5])
#     xDes = np.array([-0.75, -0.25])
    xDes = np.array([0, -1])

#     global thetaDes
#     global b_E_d
#     thetaDes = 3*np.pi/4
#     thetaDes = -np.pi/2
#     thetaDes = -np.pi/4
#     thetaDes = np.pi/4
#     thetaDes = 0
#     b_E_d = frame.create2DFrame(thetaDes, xDes)

    pickUpOffsetLocation = (-np.pi/2, np.array([-2, 0.5]))
    pickUpLocation = (-np.pi/2, np.array([-2, 0]))
    intermediate1 = (np.pi, np.array([-2.5, 2.5]))
    intermediate2 = (np.pi/2, np.array([0.1, 3.8]))
    intermediate3 = (0, np.array([2.5, 2.5]))
    dropOffOffsetLocation = (-np.pi/2, np.array([2, 0.5]))
    dropOffLocation = (-np.pi/2, np.array([2, 0]))

    poses = [pickUpOffsetLocation, pickUpLocation, "wrist close", pickUpOffsetLocation,\
             intermediate1, intermediate2, intermediate3, dropOffOffsetLocation,\
               dropOffLocation, "wrist open", dropOffOffsetLocation]

    plt.figure(figsize=(6,6))
#     animation = ani.FuncAnimation(plt.gcf(), runIK, fargs=(qInit, xInit), interval = 100, cache_frame_data=False)
    animation = ani.FuncAnimation(plt.gcf(), runFrameIK, fargs=(qInit, poses), interval = 100, cache_frame_data=False)
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
       print(" ")
       plotRobot(qCur, 0)


def runFrameIK(i, qInit, poses):
     global qCur
     global finalIterations
     global poseFSM
     global qWristVal
     global thetaDes
     global xDes
     global finishWristIter
     global wristFinished

     if i == 0:
          qCur = qInit
          finalIterations = 0
          poseFSM = 0
          qWristVal = 0
          finishWristIter = 0
          wristFinished = False

     pose = poses[poseFSM]

     if type(pose) == str:
          if pose == "wrist close":
               if not wristFinished:
                    qWristVal = qWristVal + (i - finalIterations) * np.pi/32 # pi/32 rads per loop iteration

                    if qWristVal >= np.pi*(3/4):
                         qWristVal = np.pi*(3/4)
                         print("Wrist closed!")
                         finishWristIter = i
                         wristFinished = True
               
               elif wristFinished and i > finishWristIter + 5:
                    finalIterations = i
                    poseFSM = poseFSM + 1 # iterate poseFSM
                    wristFinished = False

               plotRobot(qCur, qWristVal)

          elif pose == "wrist open":
               if not wristFinished:
                    qWristVal = qWristVal + (i - finalIterations) * -np.pi/32 # -pi/32 rads per loop iteration
                    
                    if qWristVal <= 0:
                         qWristVal = 0
                         print("Wrist closed!")
                         finishWristIter = i
                         wristFinished = True
               
               elif wristFinished and i > finishWristIter + 5:
                    finalIterations = i
                    poseFSM = poseFSM + 1 # iterate poseFSM
                    wristFinished = False

               plotRobot(qCur, qWristVal)
     
     else:
          thetaDes = pose[0]
          xDes = pose[1]

          # wait a few iterations to begin to plot initial pose
          if i < 10:
               plotRobot(qCur, qWristVal)
               return
     
          b_E_d = frame.create2DFrame(thetaDes, xDes) # pose = (thetaDes, xDes)

          alpha = 0.25
          v_thresh = 0.01 # 1 cm
          w_thresh = 5*np.pi/180 # 5 degrees in radians ~= 0.0872664626111

          [v, w, theta] = frame.computeDesiredTwistCoordinates(b_E_t, b_E_d, qs_symbolic, qCur, lengths_symbolic, lengths)
          twist_Vec = np.append(v, w) * theta

          if np.linalg.norm(v*theta) > v_thresh or np.linalg.norm(w*theta) > w_thresh:
               Jb_num = np.array(Jb.subs({q0:qCur[0], q1:qCur[1], q2:qCur[2],\
                                   q3:qCur[3], l0:lengths[0], l1:lengths[1], \
                                        l2:lengths[2], l3:lengths[3]})).astype(float)
               
               qCur = qCur + alpha * np.matmul(frame.computeRightPseudoInverse(Jb_num), twist_Vec)
               plotRobot(qCur, qWristVal)
          else:
               finalIterations = i
               poseFSM = poseFSM + 1 # iterate poseFSM
               print(f"Total iterations = {finalIterations}")
               print(f"Final ||v|| = {np.linalg.norm(v*theta):.3} mm")
               print(f"Final ||w|| = {np.linalg.norm(w*theta):.3} radians")
               print(" ")


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

          qWristInMM = qWrist * (0.25/np.pi) # pi rad = 0.25 mm travel of fingers
          t_E_leftFingerBase = frame.create2DFrame(0, np.array([0, 0.25 - qWristInMM]))
          t_E_leftFingerTip = frame.create2DFrame(0, np.array([0.25, 0.25 - qWristInMM]))
          t_E_rightFingerBase = frame.create2DFrame(0, np.array([0, -0.25 + qWristInMM]))
          t_E_rightFingerTip = frame.create2DFrame(0, np.array([0.25, -0.25 + qWristInMM]))

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


def computeManipulatorJacobian(b_E_t, qs_symbolic):
     if not os.path.isfile("2D-InverseKinematics/Frames/Jb.pkl"):
        Jb = frame.compute2DManipulatorJacobian(b_E_t, qs_symbolic)
        with open('2D-InverseKinematics/Frames/Jb.pkl', 'wb') as file:
           dill.dump(Jb, file)
     else:
        with open('2D-InverseKinematics/Frames/Jb.pkl', 'rb') as file:
           Jb = dill.load(file)
     
     return Jb


def plotRobot(qs, qWristVal):
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
         q3:qs[3], qWrist:qWristVal, l0:lengths[0], l1:lengths[1] ,l2:lengths[2], l3:lengths[3]})
    leftFingerTip = sym.Array([row[2] for row in b_E_leftFingerTip]).subs({q0:qs[0], q1:qs[1], q2:qs[2],\
         q3:qs[3], qWrist:qWristVal, l0:lengths[0], l1:lengths[1] ,l2:lengths[2], l3:lengths[3]})
    rightFingerBase = sym.Array([row[2] for row in b_E_rightFingerBase]).subs({q0:qs[0], q1:qs[1], q2:qs[2],\
         q3:qs[3], qWrist:qWristVal, l0:lengths[0], l1:lengths[1] ,l2:lengths[2], l3:lengths[3]})
    rightFingerTip = sym.Array([row[2] for row in b_E_rightFingerTip]).subs({q0:qs[0], q1:qs[1], q2:qs[2],\
         q3:qs[3], qWrist:qWristVal, l0:lengths[0], l1:lengths[1] ,l2:lengths[2], l3:lengths[3]})
    
    plt.clf()

    plt.plot(jointXVals, jointYVals, linestyle="-", color="#136891")
    plt.plot([leftFingerBase[0], rightFingerBase[0]], [leftFingerBase[1], rightFingerBase[1]], linestyle="-", color="#136891")
    plt.plot([leftFingerBase[0], leftFingerTip[0]], [leftFingerBase[1], leftFingerTip[1]], linestyle="-", color="#136891")
    plt.plot([rightFingerBase[0], rightFingerTip[0]], [rightFingerBase[1], rightFingerTip[1]], linestyle="-", color="#136891")
    
    plt.plot(xDes[0], xDes[1], marker="o", markersize=4, markeredgecolor="black", markerfacecolor="black")
    plt.arrow(xDes[0], xDes[1], 0.3*np.cos(thetaDes), 0.3*np.sin(thetaDes), head_width=0.1, head_length=0.1, fc='k', ec='k')

    plt.plot(jointXVals[0], jointYVals[0], marker="o", markersize=4, markeredgecolor="black", markerfacecolor="black")
    plt.plot(jointXVals[1], jointYVals[1], marker="o", markersize=4, markeredgecolor="red", markerfacecolor="red")
    plt.plot(jointXVals[2], jointYVals[2], marker="o", markersize=4, markeredgecolor="red", markerfacecolor="red")
    plt.plot(jointXVals[3], jointYVals[3], marker="o", markersize=4, markeredgecolor="red", markerfacecolor="red")
    plt.plot(jointXVals[4], jointYVals[4], marker="o", markersize=4, markeredgecolor="green", markerfacecolor="green")

    plt.xlim((-4, 4))
    plt.ylim((-4, 4))


if __name__ == "__main__":
    main()