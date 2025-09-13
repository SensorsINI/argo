import rospy
from pathlib import Path
import rosparam

# loads parameters and reloads them if the .yaml file has changed on disk since last load time
def load_or_reload_params(filepath, every=1):
    """
    Reloads a YAML params to the ROS parameter server if the yaml file was modified since runtime started or since last reloaded.
 
    :param filepath the relative path to file. Generate for call with e.g. os.path.join("Control_Toolkit_ASF", "config_cost_functions.yml")
    :param every: only check every this many times we are invoked

    :returns: True if file was modified
    """
    load_or_reload_params.counter=0
    load_or_reload_params.last_filepath=None
    if load_or_reload_params.counter%every!=0:
    	load_or_reload_params.counter+=1;
    	return
    load_or_reload_params.counter+=1
    mtime=None
    fp=Path(filepath)
    mtime=fp.stat().st_mtime # get mod time

    modified=False
    if ((filepath!=load_or_reload_params.last_filepath)) \
            or ((filepath==load_or_reload_params.last_filepath) 
            	and mtime > load_or_reload_params.last_mtime):
		modified=mtime > load_or_reload_params.last_mtime
		load_or_reload_params.last_mtime=mtime
		if modified:
			rospy.loginfo('reloading modified parameters')
		ros_params=rosparam.load_file(filepath)
		# rospy.logdebug("ros_params are \n%s\n of type %s",ros_params, type(ros_params))
		for d in ros_params[0]:
		    if isinstance(d,dict):
		        for k,v in d.items():
		            rospy.set_param(k,v)
		            rospy.logdebug('set parameter key='+ str(k)+' value='+str(v))

    load_or_reload_params.last_filepath=filepath
    return modified

load_or_reload_params.last_filepath=None
load_or_reload_params.last_mtime=None
load_or_reload_params.counter=0
