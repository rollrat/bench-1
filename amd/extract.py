
from os import listdir
from os.path import isfile, join
import numpy

onlyfiles = [f for f in listdir('./') if isfile(join('./', f))]

for f in onlyfiles:
  if not f.endswith('.log'):
    continue
  fo = open(f, 'r')
  ls = fo.readlines()

  print(f)

  val = []
  tt = []
  for i in range(0, len(ls) - 1, 3):
    l = ls[i+1].strip()
    t = ls[i+2].strip()

    if l.endswith('Mbps'):
      val.append(float(l.split(':')[-1].split('Mbps')[0].strip()))
    elif l.endswith('trans./sec.'):
      val.append(float(l.split(':')[-1].split('trans./sec.')[0].strip()))
    
    tt.append(float(t.split(':')[-1].split('sec')[0].strip()))

  print('avg: ' + str(sum(val) / len(val)))
  print('mid: ' + str(sorted(val)[len(val)//2]))
  print('min: ' + str(min(val)))
  print('max: ' + str(max(val)))
  print('std: ' + str(numpy.std(val)))
  print('avg-tt: %f' % (sum(tt) / len(tt)))
  print()

  r = open(f + '.norm', 'w')

  
  
  for v in val:
    r.write(str(v) + '\n')

      
    
