import math
import logging
import sys

# Simple vector operations that work with Python lists
# Very much work in progress
# Use at you own risk
#
#
# add(a, b)
# subtract(a, b)
# scale(a, scale)
# length(a)
# cross(a, b)
# dot(a, b)
# unit(a)
# tolength(a, size)
# angle(a,b)
# rotate(v, r, alpha)
# orivecs2quat(a,b)
#
#
#
#
#
#
#
#
#


# logging.basicConfig( filename='example.log', level=logging.DEBUG, filemode='w' )
logging.basicConfig( stream = sys.stdout, level=logging.INFO )

#______________________________________________________
# vector a plus vector b

def add(a, b):

    if len(a) != len(b):
        logging.error(" vadd: Vectors must have equal dimensions.")

    return [x + y for x, y in zip(a, b)]


#______________________________________________________
# vector a minus vector b

def subtract(a, b):

    if len(a) != len(b):
        logging.error(" vsubtract: Vectors must have equal dimensions.")

    return [x - y for x, y in zip(a, b)]

#______________________________________________________
# cross product a x b

def cross(a, b):

    if len(a) != len(b):
        logging.error(" cross: Vectors must have equal dimensions.")

    c = [a[1]*b[2] - a[2]*b[1],
            a[2]*b[0] - a[0]*b[2],
            a[0]*b[1] - a[1]*b[0]]

    return(c)

#______________________________________________________  
# dot product a dot b  

def dot(a, b):

    if len(a) != len(b):
        logging.error(" dot: Vectors must have equal dimensions.")

    return sum(x * y for x, y in zip(a, b))

#______________________________________________________
# scale vector a to size a

def scale(a, scale):

    return([x * scale for x in a])

#______________________________________________________
# scale vector a to size a

def tolength(a, size):

    b = unit(a)

    return([x * size for x in b])

#______________________________________________________
# scale vector a to unit size

def unit(a):

    l = length(a)
    b=[]
    if l > 0:
        for x in a:
            b.append(x/l) 
        return(b)     
    else:
        logging.error(" v2unit: vector length zero, cannot divide. " )  
        return()

#______________________________________________________
# return vector length

def length(a):

    sum_of_squares = 0
    for x in a:
        sum_of_squares = sum_of_squares + x**2
    l = math.sqrt(sum_of_squares) 

    return(l)
    

#______________________________________________________
# rotate vector v around vector r by angle alpha (radians)

def rot(v, r, alpha):

    # remove 2pi (360 degree) rotations from alpha
    sign = math.copysign(1, alpha)
    alpha = abs(alpha)
    if alpha > 2*math.pi: 
        n = int(alpha/(2*math.pi))
        alpha = alpha - n*(2*math.pi)
    alpha = alpha * sign


    # project v onto r
    size_v_proj_r = dot(v, unit(r))
    #print("size_v_proj_r=",size_v_proj_r)
    rscaled = scale(unit(r), size_v_proj_r)
    #print("rscaled=",rscaled)
    # radius_vec goes from rscaled to v
    radius_vec = subtract(v, rscaled)
    #print("radius_vec=",radius_vec)

    # rotate by incremens beta
    beta = alpha / 4
    # this is to avoid alphas when tan(alpha) goes to infinity

    for i in range(1, 5):
        
        # make a vector t perpendicular to r and radius_vec
        # such that tan(beta)=|t|/|radius_vec|
        # do not use rscaled here as it could be a zero vector
        t = tolength( cross(unit(r), unit(radius_vec))  , math.tan(beta) )
        #print("t=",t)
        new_radius_vec = tolength(add(unit(radius_vec), t), length(radius_vec) )
        radius_vec = new_radius_vec
        #print("radius_vec=",radius_vec)

    return(add(radius_vec, rscaled))



#______________________________________________________
# calculate angle between vecors a and b

def angle(a,b, *args):

    if length(a) == 0 or length(b) == 0:
        logging.warning(" angle: one of the vectors has zero length")
        return(0.0) 

    a = unit(a)
    b = unit(b)

    cos_theta = sum(x * y for x, y in zip(a, b))

    # if there is a 3rd argument indicating positive direction
    if args:
        try:
            direction_vec = args[0]
            c_pos = cross(a,b)
            c_neg = cross(b,a)
            if length(subtract(direction_vec, c_pos)) <= length(subtract(direction_vec, c_neg)):
                return(math.acos(cos_theta)) 
            else:
                return(-1*math.acos(cos_theta))
        except:
            logging.warning(" angle: the direction vector seems incorrectly formatted.")
            logging.warning(" angle: the calculated angle orientation may be wrong.")
            return(math.acos(cos_theta)) 

#______________________________________________________
# convert orientation given by 2 perpendicular vectors to a quaternion

def orivecs2quat(a,b,c):

    ux = [1,0,0]
    uy = [0,1,0]
    uz = [0,0,1]

    u = [ux, uy, uz]

    a = unit(a)
    b = unit(b)

    # check that a is perpendicular to b
    perpend_check = math.degrees(math.acos(dot(a,b)))
    #print(" perpend_check (should be close to 90): ", perpend_check)

    # make things perfectly rectangular by replacing the original b with a new b
    c = cross(a,b)
    b = cross(c,a)
    #print("b = ",b)
    abc = [a,b,c]

    #print(" abc = ", abc)

    # from reference base vectors u* to rotated ones
    dx = subtract(a, ux)
    dy = subtract(b, uy)
    dz = subtract(c, uz)

    d = [dx,dy,dz]

    #print("d = ", d)

    d_lengths = [length(dx),length(dy),length(dz)]

    if max(d_lengths) < 1E-6:
       #print(" Rotated system practically coincides with reference coordinate system. ") 
       return([0,1,0.0])
    
    # find the shortest d
    d_min_value = min(d_lengths)
    d_min_index = d_lengths.index(d_min_value)
    d.pop(d_min_index)
    abc.pop(d_min_index)
    u.pop(d_min_index)

    # let's not use this one to calculate rotation axis, let's use the other 2, the larger ones

    #print("abc = ", abc)
    #print("d = ", d)
    #print("u = ", u)

    rot_axis = unit(cross(d[0], d[1]))

    #print("rot axis = ", rot_axis)

    # select abc and u to calculate alpha
    abc_select = abc[0]
    u_select   = u[0]
    # the same thing can be done with abc[1] and u[1]
    # tested and gives identical results

    # project abc_select and u_select onto rot_axis pane
    # (don't worry about the lenth of the projected vector)
    tmp   = scale(unit(rot_axis), dot(abc_select, rot_axis))
    abc_select_proj = unit(subtract(abc_select, tmp))
    tmp   = scale(unit(rot_axis), dot(u_select, rot_axis))
    u_select_proj = unit(subtract(u_select, tmp))
    alpha = angle(u_select_proj, abc_select_proj, rot_axis)


    #print(" alpha = ", alpha)
    #print(" abc_select_proj, u_select+proj ", abc_select_proj, u_select_proj)

    vu = unit(rot_axis)
    q = math.cos(alpha/2)
    qx = vu[0]*math.sin(alpha/2)
    qy = vu[1]*math.sin(alpha/2)
    qz = vu[2]*math.sin(alpha/2)

    return([q, qx, qy, qz])
    







