import math
import logging
import sys

# Simple vector operations that work with Python lists
# 
# https://github.com/troymius/sivoli
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
# rot(v, r, alpha)
# vecs2quat(a,b)
# vcheck(a,b,c,...)
#
#
#
#
#
#
#
#
#
# DEBUG level prints all levels
# use INFO for normal runs
# logging.basicConfig( filename='example.log', level=logging.debug, filemode='w', format="[%(filename)s:%(lineno)s - %(funcName)s() ] %(message)s" )
logging.basicConfig( stream = sys.stdout, level=logging.INFO, format="[%(filename)s:%(lineno)s - %(funcName)s() ] %(message)s" )

#______________________________________________________
# vector a plus vector b

def add(a, b):

    if vcheck(a,b):
        logging.debug(" supplied with malformatted vectors. ")

    return [x + y for x, y in zip(a, b)]


#______________________________________________________
# vector a minus vector b

def subtract(a, b):

    if vcheck(a,b):
        logging.debug(" supplied with malformatted vectors. ")

    return [x - y for x, y in zip(a, b)]

#______________________________________________________
# cross product a x b

def cross(a, b):

    if vcheck(a,b):
        logging.debug(" supplied with malformatted vectors. ")

    c = [a[1]*b[2] - a[2]*b[1],
            a[2]*b[0] - a[0]*b[2],
            a[0]*b[1] - a[1]*b[0]]

    return(c)

#______________________________________________________  
# dot product a dot b  

def dot(a, b):

    if vcheck(a,b):
        logging.debug(" supplied with malformatted vectors. ")

    return sum(x * y for x, y in zip(a, b))

#______________________________________________________
# scale vector a to size a

def scale(a, scale):

    if vcheck(a):
        logging.debug(" supplied with malformatted vector. ")
    if not isinstance(scale, (int, float)):
        logging.debug(" supplied with non-number scale value. ") 

    return([x * scale for x in a])

#______________________________________________________
# scale vector a to size a

def tolength(a, size):

    if vcheck(a):
        logging.debug(" supplied with malformatted vector. ")
    if not isinstance(size, (int, float)):
        logging.debug(" supplied with non-number scale value. ") 

    b = unit(a)

    return([x * size for x in b])

#______________________________________________________
# scale vector a to unit size

def unit(a):

    if vcheck(a):
        logging.debug(" supplied with malformatted vector. ")

    l = length(a)

    b=[]
    if l > 0:
        for x in a:
            b.append(x/l) 
        return(b)     
    else:
        logging.error(" supplied with vector length zero, cannot divide. " )  
        return()

#______________________________________________________
# return vector length

def length(a):

    if vcheck(a):
        logging.debug(" supplied with malformatted vector. ")

    sum_of_squares = 0
    for x in a:
        sum_of_squares = sum_of_squares + x**2
    l = math.sqrt(sum_of_squares) 

    return(l)
    

#______________________________________________________
# rotate vector v around vector r by angle alpha (radians)

def rot(v, r, alpha):

    if vcheck(v,r):
        logging.debug(" supplied with malformatted vector. ")
    if not isinstance(alpha, (int, float)):
        logging.debug(" supplied with non-number angle value. ") 


    # remove 2pi (360 degree) rotations from alpha
    sign = math.copysign(1, alpha)
    alpha = abs(alpha)
    if alpha > 2*math.pi: 
        n = int(alpha/(2*math.pi))
        alpha = alpha - n*(2*math.pi)
    alpha = alpha * sign


    # project v onto r
    size_v_proj_r = dot(v, unit(r))

    rscaled = scale(unit(r), size_v_proj_r)

    # radius_vec goes from rscaled to v
    radius_vec = subtract(v, rscaled)


    # rotate by incremens beta
    beta = alpha / 4
    # this is to avoid alphas when tan(alpha) goes to infinity

    for i in range(1, 5):
        
        # make a vector t perpendicular to r and radius_vec
        # such that tan(beta)=|t|/|radius_vec|
        # do not use rscaled here as it could be a zero vector
        t = tolength( cross(unit(r), unit(radius_vec))  , math.tan(beta) )

        new_radius_vec = tolength(add(unit(radius_vec), t), length(radius_vec) )
        radius_vec = new_radius_vec


    return(add(radius_vec, rscaled))



#______________________________________________________
# calculate angle between vecors a and b

def angle(a,b, *args):

    if vcheck(a,b):
        logging.debug(" function supplied with malformatted vector. ")

    if length(a) == 0 or length(b) == 0:
        logging.debug(" one of the vectors has zero length")
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
            logging.debug(" direction vector seems unusable.")
            logging.debug(" calculated angle orientation may be wrong.")
            return(math.acos(cos_theta)) 

#______________________________________________________
# convert orientation given by 2 perpendicular vectors to a quaternion

def vecs2quat(a,b,c):

    if vcheck(a,b,c):
        logging.debug(" function supplied with malformatted vector. ")

    ux = [1,0,0]
    uy = [0,1,0]
    uz = [0,0,1]

    u = [ux, uy, uz]

    a = unit(a)
    b = unit(b)

    # check that a is perpendicular to b
    perpend_check = math.degrees(math.acos(dot(a,b)))


    # make things perfectly rectangular by replacing the original b with a new b
    c = cross(a,b)
    b = cross(c,a)

    logging.debug(" b = %s", b)

    abc = [a,b,c]

    logging.debug(" abc = %s", abc)

    # from reference base vectors u* to rotated ones
    dx = subtract(a, ux)
    dy = subtract(b, uy)
    dz = subtract(c, uz)

    d = [dx,dy,dz]

    logging.debug(" d = %s", d)

    d_lengths = [length(dx),length(dy),length(dz)]

    if max(d_lengths) < 1E-6:
       logging.info(" Rotated system practically coincides with reference coordinate system. ") 
       return([0,1,0.0])
    
    # find the shortest d
    d_min_value = min(d_lengths)
    d_min_index = d_lengths.index(d_min_value)
    d.pop(d_min_index)
    abc.pop(d_min_index)
    u.pop(d_min_index)

    # let's not use this one to calculate rotation axis, let's use the other 2, the larger ones

    logging.debug(" after sorting abc = %s", abc)
    logging.debug(" after sorting  d = %s", d)
    logging.debug(" after sorting  u = %s", u)

    rot_axis = unit(cross(d[0], d[1]))

    logging.debug(" after sorting rot_axis = %s", rot_axis)

    # select abc and u to calculate alpha
    abc_select = abc[0]
    u_select   = u[0]
    # the same thing can be done with abc[1] and u[1] ...
    # ... tested, gives identical results

    # project abc_select and u_select onto rot_axis pane
    # (don't worry about the lenth of the projected vector)
    tmp   = scale(unit(rot_axis), dot(abc_select, rot_axis))
    abc_select_proj = unit(subtract(abc_select, tmp))
    tmp   = scale(unit(rot_axis), dot(u_select, rot_axis))
    u_select_proj = unit(subtract(u_select, tmp))
    alpha = angle(u_select_proj, abc_select_proj, rot_axis)

    logging.debug(" alpha = %s", alpha)
    logging.debug(" abc_select_proj, u_select_proj = %s %s", abc_select_proj, u_select_proj)

    vu = unit(rot_axis)
    q = math.cos(alpha/2)
    qx = vu[0]*math.sin(alpha/2)
    qy = vu[1]*math.sin(alpha/2)
    qz = vu[2]*math.sin(alpha/2)

    return([q, qx, qy, qz])
    


#______________________________________________________

def vcheck(*args):

    # returns True if there is an issue

    status = False

    for v in args:

        if len(v) == 3: 
            for x in v:
                if not isinstance(x, (int, float)):
                   logging.info(" vector format check - not made of only floats or integers %s ", v)  
                   status = True 

        else:
            logging.info(" vector format check - not 3-dimensional %s ", v)
            status = True 

    return(status)        





