#!/usr/bin/env python
# coding: utf-8

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os

url_lu = "/home/jgines/waf2020_data/data/exp2/lu/"
url_proposal = "/home/jgines/waf2020_data/data/exp2/proposal/"

n = len(os.listdir(url_lu)) + 1
numberofcsvs = range(1,n)
extension = ".csv"
#df = pd.DataFrame(columns=["distance","robot_cost"])
#for i in numberofcsvs:
#    url_num = url + str(i) + extension
#    df_ = pd.read_csv(url_num, keep_default_na = False)
#    df = pd.concat([df, df_])

#df_cost = pd.DataFrame()
#for i in numberofcsvs:
#    url_num = url + str(i) + extension
#    df_ = pd.read_csv(url_num, keep_default_na = False)
#    df_cost = pd.concat([df_cost, df_["robot_cost"]], axis=1)

df_dist_lu = pd.DataFrame()
for i in numberofcsvs:
    url_num = url_lu + str(i) + extension
    df_ = pd.read_csv(url_num, keep_default_na = False)
    df_dist_lu = pd.concat([df_dist_lu, df_["distance"]], axis=1)

df_dist_proposal = pd.DataFrame()
for i in numberofcsvs:
    url_num = url_proposal + str(i) + extension
    df_ = pd.read_csv(url_num, keep_default_na = False)
    df_dist_proposal = pd.concat([df_dist_proposal, df_["distance"]], axis=1)

df_dist_lu.head(100)
df_dist_proposal.head(100)
#df_cost.head(100)

#mean_cost = df_cost.mean(axis=1)
mean_distance_lu = df_dist_lu.mean(axis=1)
mean_distance_proposal = df_dist_proposal.mean(axis=1)

plt.plot(mean_distance_lu.values) # mean distance between robot and human
plt.plot(mean_distance_proposal.values)
plt.xlabel('Time (s)')
plt.ylabel('Distance (m)')
plt.yticks(np.arange(min(mean_distance_proposal.values),
           max(mean_distance_lu.values) + 0.2, 0.2))
#plt.title('Distance to human during the Escort task.')
plt.savefig("/home/jgines/waf2020_data/figures/distance.pdf")
plt.clf()


#plt.plot(mean_cost.values)
#plt.xlabel('Time (s)')
#plt.ylabel('Distance (miles)')
#plt.title('Distance traveled')
#plt.savefig("/home/jgines/waf2020_data/figures/cost.pdf")
#plt.clf()


#print ("-------------- Extracted data ------------")
#print("Experiments: "    + str(len(df['time'])))
#print("Time (tau): " + str(df['time'].mean()) + "(" + str(df['time'].std()) + ")")
#print("Distance (dt): " + str(df['distance'].mean()) + "(" + str(df['distance'].std()) + ")")
#print("dmin: " + str(df['dmin'].min()))
#print("dmin: " + str(df['dmin'].mean()) + "(" + str(df['dmin'].std()) + ")")
#print("psi_personal: " + str(df['psi_personal'].mean()) + "(" + str(df['psi_personal'].std()) + ")")
#print("psi_intimate: " + str(df['psi_intimate'].mean()) + "(" + str(df['psi_intimate'].std()) + ")")