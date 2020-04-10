import sympy as sym
import numpy as np


sym.init_printing()
x,y,z = sym.symbols('x,y,z')
c1 = sym.Symbol('c1')
f = sym.Eq(2*x**2+y+z,1)
g = sym.Eq(x+2*y+z,c1)
h = sym.Eq(-2*x+y,-z)

a = sym.solve([f,g,h],(x,y,z))

print(a)



def vectp(x1,y1,z1,x2,y2,z2):
    vect = []
    l = x2-x1
    m = y2-y1
    n = z2-z1
    vect.append(l)
    vect.append(m)
    vect.append(n)
    return vect

print("")
def rectEq(x1,y1,z1,x2,y2,z2,r):    #1 = p ; 2 = q                      l = vect[0] ; m = vect[1] ; n = vect[2]
    sym.init_printing()
    x,y,z = sym.symbols('x,y,z')
    vect = vectp(x1, y1, z1, x2, y2, z2)
    print(vect)
    if vect[0] !=0 and vect[1] !=0 and vect[2] !=0:#tutti diversi da 0
        print("Entro in caso l,m,n diversi da 0")
        a = sym.Eq((x-x1)/vect[0],(y-y1)/vect[1])
        b = sym.Eq((y-y1)/vect[1],(z-z1)/vect[2])
        tmp = sym.solve([a, b], (x, y, z))
        print(tmp)
        d,e,f = 0,0,0
        if tmp.get(x) is not None:
            print("x not none")
            d = sym.Eq(x,tmp.get(x))
            print(d)
        if tmp.get(y) is not None:
            print("y not none")
            e = sym.Eq(y, tmp.get(y))
            print(e)
        if tmp.get(z) is not None:
            print("z not none")
            f = sym.Eq(z, tmp.get(z))
            print(f)
        c = sym.Eq(x**2+y**2+z**2,r**2)
        result = sym.solve([d,e,f,c],(x,y,z))
        return result
    elif vect[0] ==0 and vect[1] !=0 and vect[2] !=0:#l=0
        print("Entro in caso l=0")
        a = sym.Eq(x, x1)
        b = sym.Eq((y-y1)/vect[1], (z - z1) / vect[2])
        c = sym.Eq(x ** 2 + y ** 2 + z ** 2, r ** 2)
        result = sym.solve([a,b,c], (x, y, z))
        return result
    elif vect[0] !=0 and vect[1] ==0 and vect[2] !=0:#m=0
        print("Entro in caso m=0")
        a = sym.Eq(y, y1)
        b = sym.Eq((x - x1) / vect[0], (z - z1) / vect[1])
        c = sym.Eq(x ** 2 + y ** 2 + z ** 2, r ** 2)
        result = sym.solve([a,b,c], (x, y, z))
        return result
    elif vect[0] != 0 and vect[1] !=0 and vect[2] ==0:#n=0
        print("Entro in caso n=0")
        a = sym.Eq(z,z1)
        b = sym.Eq((x-x1)/vect[0],(y-y1)/vect[1])
        c = sym.Eq(x ** 2 + y ** 2 + z ** 2, r ** 2)
        result = sym.solve([a,b,c], (x,y,z))
        return result
    elif vect[0] == 0 and vect[1] ==0 and vect[2] !=0:#l=m=0
        print("Entro in caso n=0")
        a = sym.Eq(x,x1)
        b = sym.Eq(y,y1)
        c = sym.Eq(x ** 2 + y ** 2 + z ** 2, r ** 2)
        result = sym.solve([a,b,c], (x,y,z))
        return result
    elif vect[0] == 0 and vect[1] !=0 and vect[2] ==0:#l=n=0
        print("Entro in caso n=0")
        a = sym.Eq(x, x1)
        b = sym.Eq(z,z1)
        c = sym.Eq(x ** 2 + y ** 2 + z ** 2, r ** 2)
        result = sym.solve([a,b,c], (x,y,z))
        return result
    elif vect[0] != 0 and vect[1] ==0 and vect[2] ==0:#m=n=0
        print("Entro in caso n=0")
        a = sym.Eq(y,y1)
        b = sym.Eq(z,z1)
        c = sym.Eq(x ** 2 + y ** 2 + z ** 2, r ** 2)
        result = sym.solve([a,b,c], (x,y,z))
        return result

def isVisible(x1,y1,z1,x2,y2,z2,r):
    tmp = rectEq(x1,y1,z1,x2,y2,z2,r)
    if sym.im(tmp[0][0])!=0 or sym.im(tmp[0][1])!=0 or sym.im(tmp[0][2])!=0 or sym.im(tmp[1][0])!=0 or sym.im(tmp[1][1])!=0 or sym.im(tmp[1][2])!=0:
        return True
    else:
        return False

result=rectEq(5091082.5313572,1013216.3395964877,4943045.661622145,4726826.686803615,194164.32677549776,5382534.598367236,6378137)
print("result ",result)
print("")
print("result[0] ",result[0][0])
print(sym.im(result[0][0]))


print("")
print("")
pluto = rectEq(5091082.5313572, 1013216.3395964877, 4943045.661622145, -2797274.5151061076, 5061417.945485286,
               -4202378.6294492, 6378137)
print("Pluto ", pluto)
print("Im test ", sym.im(pluto[0][0]))



if sym.im(pluto[0][0]) ==0:
    print("Nope")
else:
    print("Funge")