import matplotlib.pyplot as plt
import numpy as np
from matplotlib import rc
from os import listdir
from os.path import isfile, join

# rc('font',**{'family':'CMU Serif'})

csfont = {'fontname':'CMU Serif'}
# hfont = {'fontname':'CMU Serif'}
plt.rcParams["font.family"] = "CMU Serif"

# 1. 기본 스타일 설정
# plt.style.use('default')
plt.rcParams['figure.figsize'] = (4, 3)
plt.rcParams['font.size'] = 12

np.random.seed(0)
# data_a = np.random.normal(0, 2.0, 1000)
# data_b = np.random.normal(-3.0, 1.5, 500)
# data_c = np.random.normal(1.2, 1.5, 1500)

fig, ax = plt.subplots()

rows = []
row_name = []

onlyfiles = [f for f in listdir('./') if isfile(join('./', f))]
for f in onlyfiles:
  if not f.endswith('.norm'):
    continue
  if not 'stream' in f:
    continue
  print(f)
  fo = open(f, 'r')
  rows.append(list(map(lambda x: float(x), fo.readlines())))
  row_name.append(f)
  # ax.set_xlabel()

# print(rows)

plt.xticks(list(range(0, 8)), row_name)
ax.boxplot(rows) #, notch=True, whis=1.5)
ax.set_ylim(0.0, 1000000.0)
# ax.set_xlabel('Data Type', **csfont)
# ax.set_ylabel('Value', **csfont)

plt.show()