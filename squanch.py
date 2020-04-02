from squanch import *

qsys = QSystem(2)
qsys.state
qsys.qubits

qsys2 = QSystem(2)

for i in range(3):
        print("Loop "+str(i))
        for qubit in qsys2.qubits:
                print(qubit)


a, _ = qsys.qubits
print(a)





sub96i = ir96pos.subpoint()
print('latitudine ', sub96i.latitude)
print('longitudine ', sub96i.longitude)
print('altitudine ', sub96i.elevation)

x = str(sub96i.latitude).split(' ')
print(x)

tmp = str(x[0]).split('d')
deg = tmp[0]
print("deg", deg)

pri = str(x[1])[:2]
print("pri", pri)

tmp1 = str(x[2]).split('"')
sec = tmp1[0]
print("sec", sec)