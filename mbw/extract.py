
from os import listdir
from os.path import isfile, join
import numpy

onlyfiles = [f for f in listdir('./') if isfile(join('./', f))]

memcpy=[]
dumb=[]
mcblock=[]

for f in onlyfiles:
  if not f.endswith('.mbw'):
    continue
  if not '0.1024.' in f:
    continue
  fo = open(f, 'r')
  ls = fo.readlines()

  c = 0
  for l in ls:
    if not l.strip().startswith('AVG'):
      continue

    val = l.split(' ')[-2]
    # print(l)
    # print(val)

    if c == 0:
      memcpy.append(val)
    elif c == 1:
      dumb.append(val)
    elif c == 2:
      mcblock.append(val)
    c += 1

f = open('r.memcpy', 'w')
for v in memcpy:
  f.write(v + '\n')
 
f = open('r.dumb', 'w')
for v in dumb:
  f.write(v + '\n')
  
f = open('r.mcblock', 'w')
for v in mcblock:
  f.write(v + '\n')
   