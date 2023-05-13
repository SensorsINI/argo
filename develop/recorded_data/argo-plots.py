import bagpy
from bagpy import bagreader # https://stackoverflow.com/questions/59794328/importing-rosbag-in-python-3
from easygui import fileopenbox
import matplotlib
matplotlib.rcParams.update({'font.size': 16})


bagfilename=fileopenbox(msg='select bag file')

bag=bagpy.bagreader(bagfilename)

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

import matplotlib.pyplot as plt
import seaborn as sea
import pandas as pd
import numpy as np


fix_msg=bag.message_by_topic('/fix')
fix_data=pd.read_csv(fix_msg)

print('loaded fix data')
# fix_data.lat and fix_data.long
#%% plot GPS
lats=fix_data.latitude.array.to_numpy()
lngs=fix_data.longitude.to_numpy()
lats=lats[~np.isnan(lats)]
lngs=lngs[~np.isnan(lngs)]

from numpy import min as min
from numpy import max as max

min_lat, max_lat, min_lon, max_lon = \
min(lats), max(lats), \
min(lngs), max(lngs)

# Create the map plotter:
import gmplot
apikey = 'AIzaSyBLmSEqZnv2Fl8-PDCnRmC6VKPd49mfK0c' # old key, 2020
apikey = 'AIzaSyBLmSEqZnv2Fl8-PDCnRmC6VKPd49mfK0c' # ccnw2023
gmap = gmplot.GoogleMapPlotter(np.nanmean(lats), np.nanmean(lngs), zoom=19, apikey=apikey) # zoom is about 19 for our sailing

gmap.plot(lats,lngs,color='red')
from pathlib import Path
p=Path(bagfilename)
gmap_filename='argo-path-'+p.stem+'.html'
gmap.draw(gmap_filename)
print(f'wrote {gmap_filename}')

#%% animate the path
"""
=========================
Simple animation examples
=========================

This example contains two animations. The first is a random walk plot. The
second is an image animation.
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation


def update_line(num, data, line):
    line.set_data(data[..., :num])
    return line,

fig1 = plt.figure()

data = np.row_stack((lngs,lats))
l, = plt.plot([], [], 'r-')
plt.xlim(min_lon, max_lon)
plt.ylim(min_lat, max_lat)
plt.xlabel('x')
plt.title('argo path')
print('starting animation')
line_ani = animation.FuncAnimation(fig1, update_line, len(lngs), fargs=(data, l),
                                   interval=10, blit=True)

# To save the animation, use the command: line_ani.save('lines.mp4')
plt.show()
print('saving animation')
line_ani.save('argo_path-'+p.stem+'.mp4')
print('done saving, but wait for plot to show animation')

#%% plot anem
anem_msg=bag.message_by_topic('/anem_speed_angle_temp')
anem_data=pd.read_csv(anem_msg)
anem_t=anem_data.Time.to_numpy()
anem_t=anem_t-anem_t[0]

anem_speed_mps=anem_data.x.to_numpy()
anem_angle_deg=anem_data.y.to_numpy()
anem_temp_celsius=anem_data.z.to_numpy()
print('loaded anem data')

# plt.plot(anem_t/60,anem_speed_mps, anem_t/60, anem_angle_deg, anem_t/60, anem_temp_celsius)
# plt.legend(['wind speed (m/s)', 'angle (deg)', 'temp (degC)'])
plt.subplot(2,1,1)
plt.plot(anem_t/60,anem_speed_mps)
plt.ylabel('wind speed (m/s)')
plt.subplot(2,1,2)
plt.plot(anem_t/60, anem_angle_deg)
plt.xlabel('time (m)')
plt.ylabel('angle (deg)')
# fix_data.lat and fix_data.long


#%% plot the IMU data /compass
compass_msg=bag.message_by_topic('/compass')
compass_data=pd.read_csv(compass_msg)
compass_t=compass_data.Time.to_numpy()
compass_t=compass_t-anem_t[0]

compasss_data_0=compass_data.data_0.to_numpy()
compasss_data_1=compass_data.data_1.to_numpy()
compasss_data_2=compass_data.data_2.to_numpy()
print('loaded compass data')

#%% plot the rudder and sail servo values
rudder_msg=bag.message_by_topic('/rudder')
sail_msg=bag.message_by_topic('/sail')
rudder_data=pd.read_csv(rudder_msg)
sail_data=pd.read_csv(sail_msg)
rudder_t=rudder_data.Time.to_numpy()
rudder_t=rudder_t-rudder_t[0]
sail_t=sail_data.Time.to_numpy()
sail_t=sail_t-sail_t[0]

rudder=rudder_data.data.to_numpy()
sail=sail_data.data.to_numpy()
rudder=(rudder-1.5)/.5
sail=sail-1
print('loaded rudder and sail data')

plt.plot(rudder_t/60,rudder,sail_t/60, sail)
plt.legend(['rudder','sail'])
plt.xlabel('time (m)')
plt.ylabel('servo PWM')
plt.ylim(-1,1)
plt.show()