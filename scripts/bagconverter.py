import numpy as np
import rosbag
bag=rosbag.Bag('bag.bag')
odomdata=[]
rkdata=[]
for topic,msg,t in bag.read_messages(topics=['/odometry','/rk4']):
    if topic=="/odometry":
        odomdata.append([t.to_time(),msg.x,msg.y,msg.th])
    elif topic=="/rk4":
        rkdata.append([t.to_time(),msg.x,msg.y,msg.th])
bag.close()
odomdata=np.array(odomdata)
rkdata=np.array(rkdata)
np.save('odomdata',odomdata)
np.save('rkdata',rkdata)
print("Saved output, dims: ",odomdata.shape,rkdata.shape)
