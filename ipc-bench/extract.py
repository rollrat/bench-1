
from os import listdir
from os.path import isfile, join
import numpy

onlyfiles = [f for f in listdir('./') if isfile(join('./', f))]

msgsize = '1000'

for f in onlyfiles:
  if not f.endswith('.result'):
    continue
  fo = open(f, 'r')
  ls = fo.readlines()

  print(f)

  s = 0
  v = ''
  for i in range(5, len(ls), 11):
    if v != ls[i-3].split(' ')[-1].strip():
      v = ls[i-3].split(' ')[-1].strip()
      print('\n' + v)
    if v != msgsize:
      continue
    print(ls[i].split('\t')[0].split(' ')[-1].strip())

    # if 'RESULTS' in ls[i]:
    #   print(ls[i])