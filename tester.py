import sivoli as sv
import math
import random


#___________________________________________________
#  quaternion calculation test 

print(" ============== quaternion calculation  test 1:") 


for i in range(0,10):

    print(" i = ", i)
      
    # generate a random vector
    v = sv.tolength([random.random()*10-5, random.random()*10-5, random.random()*10-5], random.random()*5000)
    # generate a random rotattion
    alpha = (random.random()-0.5)*math.pi*77   

    # create corresponding quaternion
    vu = sv.unit(v)
    q = math.cos(alpha/2)
    qx = vu[0]*math.sin(alpha/2)
    qy = vu[1]*math.sin(alpha/2)
    qz = vu[2]*math.sin(alpha/2)
    quaternion_in = [ q, qx, qy, qz ]
    print(" quaternion_in =", quaternion_in)

    # create 2 perpendicular vectors
    ux = [  444.9  ,    0      ,   0]
    uy = [  0      ,   1.423   ,   0]
    uz = sv.cross(ux,uy)

    urx = sv.rot(ux, v, alpha)
    ury = sv.rot(uy, v, alpha)
    urz = sv.rot(uz, v, alpha)

    # print("urx, ury: ", urx, ury)

    quaternion_out = sv.vecs2quat(urx,ury,urz)
    print(" quaternion_out =", quaternion_out)

    for x, y in zip(quaternion_in, quaternion_out):
        try:
           if abs(abs(x)-abs(y)) > 1E-8 :
                print(" === NOK quaternion test: in and out quaternions don't agree")
                print(" quternion_in  = ", quaternion_in)
                print(" quternion_out = ", quaternion_out) 
                exit

        except:
            print(" === NOK quaternion test: exception ")
            print( "quternion_in  = ", quaternion_in)
            print(" quternion_out = ", quaternion_out) 
            exit




#___________________________________________________
#  quaternion calculation test 

print(" ============== quaternion calculation test 2:") 

# create 2 perpendicular vectors
ux = [ 0.446186    ,    1.62461     ,  -1.07776    ]
uz = [ -1.94782       ,  0.324271   ,   -0.317579  ]
uy = sv.cross(uz,ux)

# print("urx, ury: ", urx, ury)

quaternion_out = sv.vecs2quat(ux,uy,uz)
print(" quaternion = ", quaternion_out)





#___________________________________________________
#  vector rotation test    

print(" ============== vector rotate test:")     

a = [-1,1,0]
r = [1.0  ,  0.0,  0.0]

alpha = .25*math.pi

print(" input = ", a,r, alpha)


rotated = sv.rot(a, r, 1*alpha)
print(" rotated =", rotated)

unrotated = sv.rot(rotated, r, -1*alpha)

print(" unrotated =", unrotated)



#___________________________________________________
#  vector check test 

print(" ============== vector check test:")

a = [1, 0]

b = ["q", 1, 2.333]

c = [1,2,3.444]

if sv.vcheck(a,b,c):
    print(" vcheck did not pass ")



#___________________________________________________
#  ray plane intersect test 

print(" ============== ray plane intersect test:")

normal = [15555,0,0]
planepoint = [ 0, 6660, -5]
ray = [-10000, 0, -5000]
raypoint = [2, 0, 0]

intersect = sv.rayplanex(normal, planepoint, ray, raypoint)

print(" intersect = ", intersect)


