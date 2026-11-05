import numpy as np

def cos_beta(R1, R2, norm_t):

    return (R1**2+R2**2-norm_t**2)/(2*R1*R2)

def cos_alpha(R1,R2,norm_t):

    return (norm_t**2+R1**2-R2**2)/(2*norm_t*R1)

def alpha_arccos(R1, R2, norm_t):

    return np.arccos(cos_alpha(R1=R1,R2=R2,norm_t=norm_t))

def beta_arccos(R1, R2, norm_t):

    return np.arccos(cos_beta(R1=R1,R2=R2,norm_t=norm_t))

def beta_arcsin_nfs(R1, R2, norm_t):

    delta = R1 + R2 - norm_t
    term = delta**4/4
    term += norm_t*delta**3
    term += norm_t**2*delta**2
    term = -term/(R1*R2)
    term += delta**2
    term += 2*norm_t*delta
    term = np.sqrt(term)/np.sqrt(R1*R2)

    return np.arcsin(term)

def beta_arcsin_nff(R1, R2, norm_t):

    delta = norm_t + R2 - R1
    term = delta**4/4
    term -= norm_t*delta**3
    term += norm_t**2*delta**2
    term = -term/(R1*R2)
    term -= delta**2
    term += 2*norm_t*delta
    term = np.sqrt(term)/np.sqrt(R1*R2)

    return np.arcsin(term)

def alpha_arcsin(R2, norm_t, beta):
    '''
    Formula should only be used if cos(alpha) is close to 1.
    Because formula can become numerically unstable because sin(beta)
    becomes unstable, which happens earlier than sin(alpha) becoming unstable.
    '''

    return np.arcsin(R2*np.sin(beta)/norm_t)

def inverse_kinematics(t, R1, R2):

    norm_t = np.linalg.norm(t)
    delta1 = R1+R2-norm_t
    delta2 = norm_t+R2-R1
    if delta1 <= 0 or delta2 <= 0:
        print("Target can not be reached!")
        return np.NaN, np.NaN
    
    cos_b = cos_beta(R1=R1, R2=R2, norm_t=norm_t)
    beta = np.NaN

    # 2/3 is chosen, because of conditional overhead
    if cos_b**2 > 2/3:
        if delta1 <= delta2:
            beta = beta_arcsin_nfs(R1=R1,R2=R2,norm_t=norm_t)
            beta = np.pi - beta
        else:
            beta = beta_arcsin_nff(R1=R1,R2=R2,norm_t=norm_t)
    else:
        beta = beta_arccos(R1=R1,R2=R2,norm_t=norm_t)

    cos_a = cos_alpha(R1=R1, R2=R2, norm_t=norm_t)
    alpha = np.NaN
    
    if cos_a**2 > 3/4:
        alpha = alpha_arcsin(R2=R2,norm_t=norm_t,beta=beta)
    else:
        alpha = alpha_arccos(R1=R1,R2=R2,norm_t=norm_t)

    theta = np.arctan2(t[1],t[0])

    phi1 = theta+alpha-np.pi/2
    phi2 = -np.pi+beta

    return phi1, phi2
