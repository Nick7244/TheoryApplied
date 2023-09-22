import numpy as np
import sympy as sym

# Creates a 2D homogenous frame from an angle and translation vector
def create2DFrame(theta, translationVec):
    if not isinstance(translationVec, np.ndarray):
        raise TypeError("Translation vectory is not a numpy array." + \
            " Ensure only numpy arrays are passed for the translation vector." + \
                f" Type is {type(translationVec)}.")
    if not translationVec.shape == (2,):
        raise TypeError("Translation vector must be a 2x1 vector." + \
            f" Shape is {translationVec.shape}.")
    
    return np.array([[sym.cos(theta), -sym.sin(theta), translationVec[0]],\
        [sym.sin(theta), sym.cos(theta), translationVec[1]],\
            [0, 0, 1]])


# Computes the symbolic form of the analytic jacobian of a 2D robot
def compute2DAnalyticJacobian(b_E_t, qs_symbolic):
    translationVec = np.array([rows[2] for rows in b_E_t])

    Ja = np.empty((0,2))

    for q in qs_symbolic:
        dx_dqi = sym.diff(translationVec[0], q)
        dy_dqi = sym.diff(translationVec[1], q)
        Ja = np.append(Ja, np.array([[dx_dqi, dy_dqi]]), axis=0)

    return sym.simplify(np.transpose(Ja))


# Computes the position component of the forward kinematics of a 2D robot
def get2DFKPosition(b_E_t, qs_symbolic, qs, lengths_symbolic, lengths):
    q0 = qs_symbolic[0]
    q1 = qs_symbolic[1]
    q2 = qs_symbolic[2]
    q3 = qs_symbolic[3]
    l0 = lengths_symbolic[0]
    l1 = lengths_symbolic[1]
    l2 = lengths_symbolic[2]
    l3 = lengths_symbolic[3]

    t = sym.Array([row[2] for row in b_E_t]).subs({q0:qs[0], q1:qs[1], q2:qs[2],\
         q3:qs[3], l0:lengths[0], l1:lengths[1] ,l2:lengths[2], l3:lengths[3]})
    
    return np.array([t[0], t[1]])


# Computes the right pseudo-inverse of a matrix
def computeRightPseudoInverse(J):
    JJ_T = np.matmul(J, np.transpose(J))

    if abs(np.linalg.det(JJ_T)) < 0.001:
        print("Singularity detected! Cannot compute right pseudo-inverse.")
        JJ_T = JJ_T + 0.01*np.identity(JJ_T.shape[0])
    
    return np.matmul(np.transpose(J), np.linalg.inv(JJ_T))


# Extracts the 2D rotation matrix from a 2D homogenous transform
def extractRotationMatrix2D(b_E_t):
    b_R_t = np.array([[b_E_t[0][0], b_E_t[0][1]],\
                      [b_E_t[1][0], b_E_t[1][1]]])
    return b_R_t


# Extracts the 2D translation vector from a 2D homogenous transform
def extractTranslationVector2D(b_E_t):
    b_T_t = np.array([[b_E_t[0][2]],[b_E_t[1][2]]])
    return b_T_t


# Computes the inverse of a 2D homogenous transform
def computeInverseTransform2D(b_E_t):
    b_R_t = extractRotationMatrix2D(b_E_t)
    b_T_t = extractTranslationVector2D(b_E_t)

    t_E_b = np.vstack((np.hstack((np.transpose(b_R_t), -1*np.matmul(np.transpose(b_R_t), b_T_t))), [0, 0, 1]))
    return t_E_b


# Computes the symbolic form of the body/geometric jacobian of a 2D robot
def compute2DManipulatorJacobian(b_E_t, qs_symbolic):
    Jb = np.empty((0,3))

    for q in qs_symbolic:
        twist_MAT = np.matmul(computeInverseTransform2D(b_E_t), sym.diff(b_E_t, q))
        w = twist_MAT[1][0]
        v = np.array([twist_MAT[0][2], twist_MAT[1][2]])

        twist_VEC = np.hstack((v, w))
        Jb = np.append(Jb, np.array([twist_VEC]), axis=0)
    
    return sym.simplify(np.transpose(Jb))


# Computes the twist coordiates from current tool position to desired tool position
# b_E_t = tool w.r.t. base
# b_E_d = desired w.r.t. base
def computeDesiredTwistCoordinates(b_E_t, b_E_d, qs, lengths):
    t_E_d = np.matmul(computeInverseTransform2D(b_E_t), b_E_d).subs({q0:qs[0], q1:qs[1], q2:qs[2],\
         q3:qs[3], l0:lengths[0], l1:lengths[1] ,l2:lengths[2], l3:lengths[3]})
    
    t_R_d = extractRotationMatrix2D(t_E_d)
    t_T_d = extractTranslationVector2D(t_E_d)

    theta = np.arccos((np.trace(t_R_d) - 1) / 2)

    if theta == 0:
        w = 0
        t_T_d_norm = t_T_d / np.linalg.norm(t_T_d)
        v = np.array([t_T_d_norm[0], t_T_d_norm[1]])
        twist_VEC = np.transpose(np.hstack((v, w)))
    else:
        