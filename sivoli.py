import math
import logging
import sys

# SImple Vector Operations that work with Python LIsts (sivoli)
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
# sys2sys2quat(a1, b1, c1, a2, b2, c2) 
# rayplanex(normal, planepoint, ray, raypoint)
#
# 
#
# to do maybe:
# shortest connection between 2 lines in space
#
#
#
#
# DEBUG level prints all levels
# use INFO for normal runs
#
# logging.basicConfig( level=logging.info, format="[%(filename)s:%(lineno)s - %(funcName)s() ] %(message)s", filename='example.log', filemode='w' )
logging.basicConfig( level=logging.INFO,  format="[%(filename)s:%(lineno)s - %(funcName)s() ] %(message)s", stream = sys.stdout )

# global tolerance for floating point comparisons
TOL = 1e-9


#______________________________________________________
# vector a plus vector b

def add(a, b):

    vcheck(a, 3)
    vcheck(b, 3)


    return [x + y for x, y in zip(a, b)]


#______________________________________________________
# vector a minus vector b

def subtract(a, b):

    vcheck(a, 3)
    vcheck(b, 3)

    return [x - y for x, y in zip(a, b)]

#______________________________________________________
# cross product a x b

def cross(a, b):

    vcheck(a, 3)
    vcheck(b, 3)


    c = [a[1]*b[2] - a[2]*b[1],
            a[2]*b[0] - a[0]*b[2],
            a[0]*b[1] - a[1]*b[0]]

    return(c)

#______________________________________________________  
# dot product a dot b  

def dot(a, b):

    vcheck(a, 3)


    return sum(x * y for x, y in zip(a, b))

#______________________________________________________
# scale vector a by a factor scale

def scale(a, scale):

    vcheck(a, 3)

    return([x * scale for x in a])

#______________________________________________________
# scale vector a to size a

def tolength(a, size):

    vcheck(a, 3)

    if length(a) < TOL:
        logging.error(" Error: tolength() was passed argument %s that is a zero length vector.", a)
        return None

    b = unit(a)

    return([x * size for x in b])

#______________________________________________________
# scale vector a to unit size

def unit(a):

    vcheck(a, 3)

    l = length(a)

    if l < TOL:
        logging.error(" Error: unit() was passed argument %s that is a zero length vector.", a)
        return None

    b=[]

    for x in a:
        b.append(x/l) 
    return(b)     


#______________________________________________________
# return vector length

def length(a):

    vcheck(a, 3)

    sum_of_squares = 0
    for x in a:
        sum_of_squares = sum_of_squares + x**2
    l = math.sqrt(sum_of_squares) 

    return(l)
    

#______________________________________________________
# rotate vector v around vector r by angle alpha (radians)

def rot(v, r, alpha):

    vcheck(v, 3)
    vcheck(r, 3)

    # check that v and r are not parallel, if so, give a warning
    if length(cross(v, r)) < TOL:
        logging.info(" Warning: rot() v and r are parallel, rotation will not change v. ")
        return(v)

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


    # rotate by increments beta
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

def angle(a, b, *args):

    vcheck(a, 3)
    vcheck(b, 3)

    a = unit(a)
    b = unit(b)
    
    # treat situations when unitv dotprod is outside (-1,1) due to numerical error
    dotprod = dot(a, b)
    if dotprod < -1.0:
        dotprod = -1.0
    if dotprod >  1.0:
        dotprod = 1.0    

    # angle = math.acos( sum(x * y for x, y in zip(a, b)) )
    angle = math.acos(dotprod)

    # if there is a 3rd argument indicating positive direction
    if args:
        try:
            direction_vec = args[0]
            c_pos = cross(a,b)
            sign = dirsign(c_pos, direction_vec)
            return(sign*angle)
        except:
            logging.info(" direction vector seems unusable.")
            logging.info(" calculated angle orientation may be wrong.")

    # need to add something here to indicate how the angle sign relates to cross(a, b)

    return(angle)    

#______________________________________________________
# convert orientation given by 2 perpendicular vectors to a quaternion
# as quaternion and angle-axis lists

def vecs2quat(a,b):

    vcheck(a, 3)
    vcheck(b, 3)

    ux = [1,0,0]
    uy = [0,1,0]
    uz = [0,0,1]

    u = [ux, uy, uz]

    a = unit(a)
    b = unit(b)


    # make things perfectly rectangular by replacing the original b with a new b
    c = cross(a,b)
    b = cross(c,a)

    logging.info(" b = %s", b)

    abc = [a,b,c]

    logging.info(" abc = %s", abc)

    # from reference base vectors u* to rotated ones
    dx = subtract(a, ux)
    dy = subtract(b, uy)
    dz = subtract(c, uz)

    d = [dx,dy,dz]

    logging.info(" d = %s", d)

    d_lengths = [length(dx),length(dy),length(dz)]

    if max(d_lengths) < TOL:
       logging.info(" Rotated system practically coincides with reference coordinate system. ") 
       return([1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0])
    
    # find the shortest d
    d_min_value = min(d_lengths)
    d_min_index = d_lengths.index(d_min_value)
    d.pop(d_min_index)
    abc.pop(d_min_index)
    u.pop(d_min_index)

    # let's not use this one to calculate rotation axis, let's use the other 2, the larger ones

    logging.info(" after sorting abc = %s", abc)
    logging.info(" after sorting  d = %s", d)
    logging.info(" after sorting  u = %s", u)

    rot_axis = unit(cross(d[0], d[1]))

    logging.info(" after sorting rot_axis = %s", rot_axis)

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

    logging.info(" alpha = %s", alpha)
    logging.info(" abc_select_proj, u_select_proj = %s %s", abc_select_proj, u_select_proj)

    vu = unit(rot_axis)
    q = math.cos(alpha/2)
    qx = vu[0]*math.sin(alpha/2)
    qy = vu[1]*math.sin(alpha/2)
    qz = vu[2]*math.sin(alpha/2)

    return([q, qx, qy, qz], [alpha, vu[0], vu[1], vu[2]])
    


#______________________________________________________
# mutual orientation of 2 systems, each defined by 2 orthogonal vectors
# returns transform from sys 1 to sys 2  
# as quaternion and angle-axis lists

def sys2sys2quat(a1, b1, a2, b2):

    vcheck(a1, 3)
    vcheck(b1, 3)
    vcheck(a2, 3)
    vcheck(b2, 3)

    a1 = unit(a1)
    b1 = unit(b1)
    # make things perfectly rectangular by replacing the original b with a new b
    c1 = cross(a1,b1)
    b1 = cross(c1,a1)
    abc1 = [a1,b1,c1]
    logging.info(" abc1 = %s", abc1)

    a2 = unit(a2)
    b2 = unit(b2)
    # make things perfectly rectangular by replacing the original b with a new b
    c2 = cross(a2,b2)
    b2 = cross(c2,a2)
    abc2 = [a2,b2,c2]
    logging.info(" abc2 = %s", abc2)



    # from sys 1 to sys 2
    dx = subtract(a2, a1)
    dy = subtract(b2, b1)
    dz = subtract(c2, c1)

    d = [dx,dy,dz]

    logging.info(" d = %s", d)

    d_lengths = [length(dx),length(dy),length(dz)]

    if max(d_lengths) < TOL:
       logging.info(" Rotated system practically coincides with reference coordinate system. ") 
       return([1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0])
    
    # find the shortest d
    d_min_value = min(d_lengths)
    d_min_index = d_lengths.index(d_min_value)
    d.pop(d_min_index)
    abc1.pop(d_min_index)
    abc2.pop(d_min_index)    

    # ... let's not use this one to calculate rotation axis, let's use the other 2, the larger ones

    logging.info(" after sorting abc1 = %s", abc1)
    logging.info(" after sorting abc2 = %s", abc2)
    logging.info(" after sorting    d = %s", d)


    rot_axis = unit(cross(d[0], d[1]))

    logging.info(" after sorting rot_axis = %s", rot_axis)

    # select abc and u to calculate alpha
    abc1_select = abc1[0]
    abc2_select = abc2[0]
    # the same thing can be done with abc[1] and u[1] ...
    # ... tested, gives identical results

    # project abc1_select and abc2_select onto rot_axis plane
    # (don't worry about the lenth of the projected vector)
    tmp   = scale(unit(rot_axis), dot(abc1_select, rot_axis))
    abc1_select_proj = unit(subtract(abc1_select, tmp))
    tmp   = scale(unit(rot_axis), dot(abc2_select, rot_axis))
    abc2_select_proj = unit(subtract(abc2_select, tmp))

    alpha = angle(abc1_select_proj, abc2_select_proj, rot_axis)

    logging.info(" alpha = %s", alpha)
    logging.info(" abc1_select_proj, abc2_select_proj = %s %s", abc1_select_proj, abc2_select_proj)

    vu = unit(rot_axis)
    q = math.cos(alpha/2)
    qx = vu[0]*math.sin(alpha/2)
    qy = vu[1]*math.sin(alpha/2)
    qz = vu[2]*math.sin(alpha/2)

    return([q, qx, qy, qz], [alpha, vu[0], vu[1], vu[2]])
    
#______________________________________________________
# find intersection point of a ray and a plane

def rayplanex(normal, planepoint, ray, raypoint):

    vcheck(normal, 3)
    vcheck(planepoint, 3)
    vcheck(ray, 3)
    vcheck(raypoint, 3)

    # imagine the raypoint is a meteorite passing by the top of a lighthouse
    # ray ... is meteorite velocity vector
    # plane ... is the ground
    # plane point ... is a dog nearby
    # intersect ... is where the meteorite hits the ground

    normal = unit(normal)
    ray = unit(ray)

    # vector from ray point to planepoint (from meteorite or top of lighthouse to the dog)
    rppp = subtract(planepoint, raypoint)
    logging.info(" rppp %s ", rppp)


    if length(rppp) < TOL:
        logging.info(" the raypoint is (almost or completely) on the plane, try a new raypoint. ")  
        raypoint = add(raypoint, scale(ray, 1.0))
        rppp = subtract(planepoint, raypoint)
        logging.info(" new raypoint %s ", raypoint)

    if abs(dot(rppp, normal)) < TOL:
        logging.info(" the raypoint is (almost or completely) on the plane, try a new raypoint. ")  
        raypoint = add(raypoint, scale(ray, 1.0))
        rppp = subtract(planepoint, raypoint)
        logging.info(" new raypoint %s ", raypoint)


    # project vrppp onto normal (vector from top to bottom of the lighthouse)
    rppp_proj_n = scale(normal, dot(rppp, normal))
    logging.info(" rppp_proj_n %s ", rppp_proj_n)

    # cosine of angle between normal and ray (between lighthouse and meteorive velocity vector)
    cosi = dot(unit(rppp_proj_n), ray)
    logging.info(" cosi %s ", cosi)


    if abs(cosi) < TOL:
       logging.error(" Error: the ray is (almost or completely) parallel to plane, cannot calculate intersect. ")  
       return None
                         
    # vector from raypoint to intersect (from meteorite to earth intersect)
    raypoint2intersect = scale(ray, length(rppp_proj_n)/cosi)

    logging.info(" raypoint2intersect %s ", raypoint2intersect)

    intersect = add(raypoint, raypoint2intersect)

    return(intersect)


#______________________________________________________
# return +1 if a and b form a sharp angle, else return -1

def dirsign(a, b):

    vcheck(a, 3)
    vcheck(b, 3)

    cosi = dot(a, b)
    
    if cosi < 0:
        return(-1)
    else:
        return(1)


#_______________________________________________________
# vcheck if supplier vectors are healthy and non zero length

def vcheck(v, dim=3):

    if len(v) != dim:
            logging.error(" Error: vcheck() was passed an argument %s that is not a %dD vector.", v, dim)
            raise ValueError("vcheck() was passed an argument that is not a vector of requested dimension %d." % dim)
            
    for vec_item in v:
        # is instance of float or int?
        if not isinstance(vec_item, (float, int)):
            logging.error(" Error: vcheck() was passed an argument %s that could not convert to a list of numbers.", v)
            raise ValueError("vcheck() was passed an argument that could not convert to a list of numbers.")

    # check for zero length vector
    # do not use the lengths function (that would create a circular dependency)
    if v[0]**2 + v[1]**2 + v[2]**2 < TOL:
        logging.info(" Warning: vcheck() was passed an argument %s that is a zero length vector.", v)


#_______________________________________________________
# vec2yawpitchroll

def vecs2ypr(vx2, vy2):

    # think of it as a tank turret with a gun barrel and a bullet. 
    # The turret yaws, the barrel pitches, and the bullet rolls.
    # 
    # vz0 is attached to the bullet, aims up before any rotation
    # vy0 is attached to the bullet, aims left before any rotation
    # vx0 is attached to the bullet, aims forward before any rotation
    #
    # yaw is rotation around initial vz (vz0 = [0,0,1])
    # pitch is rotation around new vy (vy1)
    # roll is rotation around new vx (vx2)
    #
    # 

    vcheck(vx2, 3)
    vcheck(vy2, 3)


    if min(length(vx2), length(vy2)) < TOL:
        logging.error(" Error: vec2yawpitchroll() encountered a zero length input vector.")
        return None

    # first project vx into xy plane, then calculate yaw and pitch
    vx_proj_xy = [vx2[0], vx2[1], 0.0]

    # if the projected vector is tiny, the camera is pointing straight up or down
    if length(vx_proj_xy) < TOL:
        logging.info(" Warning: vec2yawpitchroll() it seems the camera is pointing straight up or down.")
        pitch = angle([1,0,0], vx2, [0,1,0])
        yaw   = angle([0,1,0], vy2, [0,0,1])
        roll  = 0.0
        return([yaw, pitch, roll])

    yaw = angle([1,0,0], vx_proj_xy, [0,0,1])
    vy1 = rot([0,1,0], [0,0,1], yaw)
    pitch = angle(vx_proj_xy, vx2, vy1)
    roll = angle(vy1, vy2, vx2)  

    return([yaw, pitch, roll])






          





