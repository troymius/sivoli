import sivoli as vo
import math
import random


#___________________________________________________
#  quaternion calculation test 

for i in range(0,1):

    print(" sivoli to Blender quaternion test ", i)
      
    # generate a random vector
    v = vo.tolength([random.random()*10-5, random.random()*10-5, random.random()*10-5], random.random()*5000)
    # generate a random rotattion
    alpha = (random.random()-0.5)*math.pi*77   

    # create corresponding quaternion
    vu = vo.unit(v)
    q = math.cos(alpha/2)
    qx = vu[0]*math.sin(alpha/2)
    qy = vu[1]*math.sin(alpha/2)
    qz = vu[2]*math.sin(alpha/2)
    quaternion_in = [ q, qx, qy, qz ]
    print("quaternion_in =", quaternion_in)

    # create 2 perpendicular vectors
    ux = [  444.9  ,    0      ,   0]
    uy = [  0      ,   1.423   ,   0]
    uz = vo.cross(ux,uy)

    urx = vo.rot(ux, v, alpha)
    ury = vo.rot(uy, v, alpha)
    urz = vo.rot(uz, v, alpha)

    # print("urx, ury: ", urx, ury)

    quaternion_out = vo.vecs2quat(urx,ury,urz)
    print("quaternion_out =", quaternion_out)

    for x, y in zip(quaternion_in, quaternion_out):
        try:
           if abs(abs(x)-abs(y)) > 1E-8 :
                print(" === NOK quaternion test: in and out quaternions don't agree")
                print("quternion_in  = ", quaternion_in)
                print("quternion_out = ", quaternion_out) 
                exit

        except:
            print(" === NOK quaternion test: exception ")
            print("quternion_in  = ", quaternion_in)
            print("quternion_out = ", quaternion_out) 
            exit













#___________________________________________________
#  quaternion calculation test 



print(" ==== sivoli to blender quaternion test: ")

# create 2 perpendicular vectors
ux = [ 0.446186    ,    1.62461     ,  -1.07776    ]
uz = [ -1.94782       ,  0.324271   ,   -0.317579  ]
uy = vo.cross(uz,ux)

# print("urx, ury: ", urx, ury)

quaternion_out = vo.vecs2quat(ux,uy,uz)
print("quaternion =", quaternion_out)





        

# a = [-1,1,0]
# r = [1.0  ,  0.0,  0.0]

# alpha = .25*math.pi


# rotated = vo.rot(a, r, 1*alpha)
# print("rotated =", rotated)

# print("\n")


# unrotated = vo.rot(rotated, r, -1*alpha)

# print("unrotated =", unrotated)