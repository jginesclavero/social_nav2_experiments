#!/usr/bin/env python
# coding: utf-8

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os

url = "/home/jgines/waf2020_extend_data/exp2_default/"
n = len(os.listdir(url)) + 1
numberofcsvs = range(1,n)
extension = ".csv"
df=pd.DataFrame(columns=["interaction","distance","dmin","psi_personal","psi_intimate"])
for i in numberofcsvs:
    url_num=url+str(i)+extension
    df_ = pd.read_csv(url_num, keep_default_na= False)
    df = pd.concat([df, df_])

df = df.drop(['distance'], axis=1)
df_line = df.drop(['psi_personal', 'psi_intimate'], axis=1)
df_g = df.groupby(by=["interaction"]).mean()
df_g_psi = df_g.drop(['dmin'], axis=1)
df_g_line = df_line.groupby(by=["interaction"]).mean()

df.head(100)
#df_g_psi.plot(kind="bar", ylabel='%')
#plt.ylim(0,1)
#plt.xticks(rotation=0)
#
#df_g_line.plot.line(ylabel='dmin (m)')
#plt.ylim(0,2)
labels = np.arange(1,6)

fig, axs = plt.subplots(2, sharex=True, sharey=False, gridspec_kw={'hspace': 0.07})
axs[0].plot(df_g_line)
axs[0].set_ylim([0, 1.3])
axs[0].set_ylabel('dmin (m)')
axs[1].plot(df_g_psi)
axs[1].set_ylim([0, 1])
axs[1].legend(['psi personal', 'psi intimate'])
axs[1].set_ylabel('occupation %')
axs[1].set_xlabel('interactions')

for ax in axs:
    ax.label_outer()


plt.show()
#plt.savefig("/home/jgines/waf2020_extend_data/figures/test.pdf")
#plt.clf()

exit()
#plt.plot(df['distance'].values) #distancia por instante
#plt.xlabel('Time (s)')
#plt.ylabel('Distance (miles)')
#plt.title('Distance traveled')
#plt.savefig("figures/distance.pdf")
#plt.clf()

#plt.plot(df['distance'].values.cumsum()) # distancia acumulada
#plt.xlabel('Time (s)')
#plt.ylabel('Distance (miles)')
#plt.title('Distance traveled')
#plt.savefig("figures/distance_cumsum.pdf")
#plt.clf()



#plt.plot(df['distance'].values.cumsum(), linewidth=2) # distancia acumulada
#plt.xlabel('Time (s)')
#plt.ylabel('Distance (miles)')
#plt.title('Distance traveled')
#plt.savefig("figures/distance_cumsum.pdf")
#
#total_dist = df['distance'].values.cumsum()
#
#test = np.empty(len(total_dist), dtype=object)
#
#i = 0
#for n in df['recovery_behavior_executed'].values:
#  if n == '1':
#    test[i] = total_dist[i]
#  i = i + 1
#
#plt.plot(test, color='black', marker='|', markersize=7) 
#plt.xlabel('Time (s)')
#plt.ylabel('Distance (miles)')
#plt.title('Recovery behavior executed on time')
#plt.savefig("figures/behaviors_vs_distance.pdf")
#plt.clf()

# In[6]:
i = 0
for n in df['psi_personal'].values:
    df['psi_personal'].values[i] = n * 100.0
    i = i + 1

print ("-------------- Extracted data ------------")
print("Experiments: "    + str(len(df['time'])))
print("Time (tau): " + str(df['time'].mean()) + "(" + str(df['time'].std()) + ")")
print("Distance (dt): " + str(df['distance'].mean()) + "(" + str(df['distance'].std()) + ")")
print("dmin: " + str(df['dmin'].min()))
print("dmin: " + str(df['dmin'].mean()) + "(" + str(df['dmin'].std()) + ")")
print("psi_personal: " + str(df['psi_personal'].mean()) + "(" + str(df['psi_personal'].std()) + ")")
print("psi_intimate: " + str(df['psi_intimate'].mean()) + "(" + str(df['psi_intimate'].std()) + ")")


#print("Total time: " + str(len(df['distance'].values)/3600))
#print("Total distance: " + str(df['distance'].values.cumsum()[-1]))
#
#total_recoveries = 0.0
#for i in df['recovery_behavior_executed'].values:
#  if (i == '1'):
#    total_recoveries = total_recoveries + 1.0
#
#print("Recoveries: " + str(total_recoveries))
#print("Recoveries per mile: " + str(total_recoveries / df['distance'].values.cumsum()[-1]))






# In[7]:


#df[df["vel_x"]!= "0.0"]


# In[8]:


#df_no = df[df['vel_x']!=0.0]

#plt.plot(df_no["distance"]) #distancia por instante
#plt.xlabel('instant')
#plt.ylabel('distance')
#plt.title('Distance traveled')


# In[9]:


#plt.plot(df_no['distance'].values.cumsum()) # distancia acumulada
#plt.xlabel('instant')
#plt.ylabel('distance')
#plt.title('Distance traveled')
#plt.savefig("figures/distance.pdf")


# In[10]:


# https://stackoverflow.com/questions/18992086/save-a-pandas-series-histogram-plot-to-file
# https://datacarpentry.org/python-ecology-lesson-es/08-putting-it-all-together/index.html

