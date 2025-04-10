# plot data from recorded argo bag file
# tobi CCNW 2023
#@@ import data
import matplotlib
matplotlib.use("tkagg")
matplotlib.rcParams.update({'font.size': 16})
import matplotlib.pyplot as plt

import bagpy
from bagpy import bagreader # https://stackoverflow.com/questions/59794328/importing-rosbag-in-python-3
from easygui import fileopenbox
from develop.analysis.prefs import MyPreferences
prefs=MyPreferences()

lastbagfile=prefs.get('lastbagfile','')

bagfilename=fileopenbox(msg='select bag file', default=lastbagfile)
if not bagfilename is None:
    prefs.put('lastbagfile', bagfilename)

bag=bagpy.bagreader(bagfilename)
from pathlib import Path
bagfilepath=Path(bagfilename)

# print topics
print(bag.topic_table)
#  dict_keys(['/acceleration', '/anem_diffpressure', '/anem_speed_angle_temp',
#  '/compass', '/diagnostics', '/fix', '/gps_data', '/joint_states',
#  '/pose', '/rosbridge_ws/client_count', '/rosbridge_ws/connected_clients',
#  '/rosbridge_wss/client_count', '/rosbridge_wss/connected_clients',
#  '/rosout', '/rosout_agg', '/rudder', '/sail', '/statistics',
#  '/tf', '/tf2_web_republisher/status', '/tf_static', '/time_reference', '/vel'])

#                               Topics  ...     Frequency
# 0                      /acceleration  ...      9.940569
# 1                 /anem_diffpressure  ...     10.000987
# 2             /anem_speed_angle_temp  ...     10.000176
# 3                           /compass  ...      9.939757
# 4                       /diagnostics  ...      0.999448
# 5                               /fix  ...      0.999851
# 6                          /gps_data  ...      7.507088
# 7                      /joint_states  ...     10.000153
# 8                              /pose  ...      9.941806
# 9         /rosbridge_ws/client_count  ...           NaN
# 10   /rosbridge_ws/connected_clients  ...           NaN
# 11       /rosbridge_wss/client_count  ...           NaN
# 12  /rosbridge_wss/connected_clients  ...           NaN
# 13                           /rosout  ...  13530.012903
# 14                       /rosout_agg  ...   9962.717340
# 15                           /rudder  ...      9.998770
# 16                             /sail  ...      9.999509
# 17                       /statistics  ...     20.887344
# 18                               /tf  ...      9.999628
# 19       /tf2_web_republisher/status  ...      4.999957
# 20                        /tf_static  ...           NaN
# 21                   /time_reference  ...      1.001502
# 22                              /vel  ...      1.805956

print(f'loaded {bagfilename}')

# load fix data from GPS

import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from numpy import min as min
from numpy import max as max
import matplotlib.animation as animation

start_time_m=10 # no data before this
end_time_m=30 # no data before this

def chopidx(t, t0=start_time_m, t1=end_time_m):
    return np.where((t>=t0) & (t<=t1))

fix_msg=bag.message_by_topic('/fix')
fix_data=pd.read_csv(fix_msg)

# fix_data.lat and fix_data.long


# compute fix arrays from GPS fix
fix_t_m=fix_data.Time.to_numpy()/60 # minutes
lats=fix_data.latitude.array.to_numpy()
lngs=fix_data.longitude.to_numpy()
fix_t_m=fix_t_m[~np.isnan(lats)]
fix_t_m=fix_t_m-fix_t_m[0]
lats=lats[~np.isnan(lats)]
lngs=lngs[~np.isnan(lngs)]


min_lat, max_lat, min_lon, max_lon = \
min(lats), max(lats), \
min(lngs), max(lngs)
print('loaded fix data')

idx=chopidx(fix_t_m)
fix_t=fix_t_m[idx]
lt_m=lats[idx]
lg_m=lngs[idx]
# compute in meters https://stackoverflow.com/questions/639695/how-to-convert-latitude-or-longitude-to-meters
lt_m=111.32e3*(lt_m-min_lat)
mean_lgn=np.mean(lngs)
lg_m=(40075e3/360)*np.cos(mean_lgn*np.pi/180)*(lg_m-min_lon)
fix_anim_data = np.row_stack((lg_m, lt_m)) # to match [],[] signature of function update_line
print('computed fix in meters')

# TODO fix the publisher for pose to publish as 3 vectors
# the pose is unfortunately returned as strings like ": roll:177.600409, pitch:-10.851735, yaw:48.976230"
pose_msg=bag.message_by_topic('/pose')
pose_data=pd.read_csv(pose_msg)
pose_t=pose_data.Time.to_numpy()/60
pose_t=pose_t-pose_t[0]
idx=chopidx(pose_t)
pose_t=pose_t[idx]
pose_arr=pose_data.to_numpy()
pose_arr=np.squeeze(pose_arr[idx,:])

import re
rpy=np.zeros((len(pose_arr),4))
for i in range(len(pose_t)):
    rpy[i,0]=pose_t[i]
    d=re.split(':|,',pose_arr[i,1])
    r = float(d[2])
    f = float(d[4])
    y = float(d[6])
    rpy[i,1]=r
    rpy[i,2]=f
    rpy[i,3]=y

print('loaded pose')

# extract wind sensor data
anem_msg=bag.message_by_topic('/anem_speed_angle_temp')
anem_data=pd.read_csv(anem_msg)
anem_t=anem_data.Time.to_numpy()/60
anem_t=anem_t-anem_t[0]
idx=chopidx(anem_t)
anem_t=anem_t[idx]

anem_speed_mps=anem_data.x.to_numpy()[idx]
anem_angle_deg=anem_data.y.to_numpy()[idx]
anem_temp_celsius=anem_data.z.to_numpy()[idx]

print('loaded anem data')

#extract IMU data for pose
compass_msg=bag.message_by_topic('/compass')
compass_data=pd.read_csv(compass_msg)
compass_t=compass_data.Time.to_numpy()
compass_t=(compass_t-compass_t[0])/60
idx=chopidx(compass_t)
compass_t=compass_t[idx]

compasss_data_0=compass_data.data_0.to_numpy()[idx]
compasss_data_1=compass_data.data_1.to_numpy()[idx]
compasss_data_2=compass_data.data_2.to_numpy()[idx]
print('loaded compass data')

# extract control input data (rudder and sail)
rudder_msg=bag.message_by_topic('/rudder')
sail_msg=bag.message_by_topic('/sail')
rudder_data=pd.read_csv(rudder_msg)
sail_data=pd.read_csv(sail_msg)
rudder_t=rudder_data.Time.to_numpy()/60
rudder_t=(rudder_t-rudder_t[0])
idx=chopidx(rudder_t)
rudder_t=rudder_t[idx]
rudder=rudder_data.data.to_numpy()[idx]
sail_t=sail_data.Time.to_numpy()/60
sail_t=(sail_t-sail_t[0])
idx=chopidx(sail_t)
sail_t=sail_t[idx]
sail=sail_data.data.to_numpy()[idx]

rudder=(rudder-1.5)/.5
sail=sail-1
print('loaded rudder and sail data')

#%% create HTML map
# Create the map plotter:
import gmplot
apikey = 'AIzaSyBLmSEqZnv2Fl8-PDCnRmC6VKPd49mfK0c' # ccnw2023
gmap = gmplot.GoogleMapPlotter(np.nanmean(lats), np.nanmean(lngs), zoom=19, apikey=apikey) # zoom is about 19 for our sailing

gmap.plot(lats,lngs,color='red')
gmap_filename='argo-path-' + bagfilepath.stem + '.html'
gmap.draw(gmap_filename)
bagfilepath=Path(gmap_filename)
print(f'wrote {bagfilepath.absolute()}')

#%%  make animation data and animation movie of fix data


# animate the path, set pycharm to show plots externally in Settings/Tools/Python Scientific

def update_line(num, data, time, line,txt):
    line.set_data(data[..., :num])
    txt.set_text(f't={time[num]:.2f}m')
    print('.', end='')
    if num%80==0:
        print(f' t={time[num]:.2f}')
    return line,txt

fig1 = plt.figure()
l, = plt.plot([], [], 'r-')
txt=plt.text(0, 0,'time')
plt.xlim(np.min(lg_m), np.max(lg_m))
plt.ylim(np.min(lt_m), np.max(lt_m))
plt.xlabel('longitude (m)')
plt.ylabel('latitude (m)')
plt.title('argo path')
print('starting animation')
line_ani = animation.FuncAnimation(fig1, update_line, len(fix_t), fargs=(fix_anim_data, fix_t, l, txt),
                                   interval=10, blit=True)

# To save the animation, use the command: line_ani.save('lines.mp4')
plt.show()
print('saving animation')
line_ani.save('argo_path-' + bagfilepath.stem + '.avi')
print('done saving, but wait for plot to show animation')

#%% plot anem

# plt.plot(anem_t/60,anem_speed_mps, anem_t/60, anem_angle_deg, anem_t/60, anem_temp_celsius)
# plt.legend(['wind speed (m/s)', 'angle (deg)', 'temp (degC)'])
plt.subplot(2,1,1)
plt.plot(anem_t,anem_speed_mps)
plt.ylabel('wind speed (m/s)')
plt.subplot(2,1,2)
plt.plot(anem_t, anem_angle_deg)
plt.xlabel('time (m)')
plt.ylabel('angle (deg)')
# fix_data.lat and fix_data.long


#%% plot the IMU compass

plt.plot(compass_t, compasss_data_0, compass_t, compasss_data_1, compass_t, compasss_data_2)
plt.xlabel('time (m)')
plt.xlim([5,25])
plt.legend(['0','1','2'])
plt.show()

#%% plot the rudder and sail servo values
plt.plot(rudder_t,rudder,sail_t, sail)
plt.legend(['rudder','sail'])
plt.xlabel('time (m)')
plt.ylabel('servo PWM')
plt.ylim(-1,1)
plt.show()

#%% plot the boat pose from acceleration sensor

plt.plot(rpy[:,0],rpy[:,1],rpy[:,0],rpy[:,2],rpy[:,0],rpy[:,3])
plt.xlabel('time (m)')
plt.ylabel('deg')
plt.legend(['roll','pitch','yaw'])
plt.show()

#%% plot all the sensor data together
matplotlib.rcParams.update({'font.size': 10})

plt.subplot(6,1,1)
plt.plot(pose_t,rpy[:,1],pose_t,rpy[:,2],pose_t,rpy[:,3])
plt.xlabel('time (m)')
plt.ylabel('deg')
plt.legend(['roll','pitch','yaw'])
plt.show()

plt.subplot(6,1,2)
plt.plot(rudder_t,rudder,sail_t, sail)
plt.legend(['rudder','sail'])
plt.xlabel('time (m)')
plt.ylabel('servo PWM')
plt.ylim(-1,1)
plt.show()

plt.subplot(6,1,3)
plt.plot(compass_t, compasss_data_0, compass_t, compasss_data_1, compass_t, compasss_data_2)
plt.xlabel('time (m)')
plt.ylabel('comp (uT)')
plt.ylim([-100,100])
plt.legend(['0','1','2'])
plt.show()

plt.subplot(6,1,4)
plt.plot(anem_t,anem_speed_mps)
plt.ylabel('wind speed (m/s)')

plt.subplot(6,1,5)
plt.plot(anem_t, anem_angle_deg)
plt.xlabel('time (m)')
plt.ylabel('angle (deg)')
# fix_data.lat and fix_data.long

plt.subplot(6,1,6)
plt.plot(fix_t,lg_m, fix_t,lt_m)
plt.ylabel('fix (m)')
plt.xlabel('time (m)')

plt.show()
